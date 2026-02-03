# TODO: Clean imports, remove unused ones
import rclpy
from rclpy.time import Time
from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from cognitive_nodes.generic_model import GenericModel, Learner  # (si usas)
from simulators.scenarios_2D import SimpleScenario, EntityType    # (si usas)
from cognitive_nodes.world_model import WorldModel
from bartender_experiment_interfaces.srv import KnowClient

from cognitive_node_interfaces.msg import Perception, PerceptionStamped
from std_msgs.msg import Float32

from core.utils import perception_msg_to_dict

# -----------------------------
# Helpers
# -----------------------------
# OPT: helper para mapear id float a clave LTM sin repetir formateo
def _client_key_from_id(cid: float) -> str:
    # Mantiene tu convención "client_0_67" para 0.67
    # Evita int/round/format encadenados en cada llamada
    cid_rounded = round(cid, 2)
    # 0.67  -> "client_0_67"
    return f"client_{str(cid_rounded).replace('.', '_')}"

def _known_key_from_id_legacy(cid: float) -> str:
    # Mantiene tu convención anterior "client_0_{int(cid*100)}"
    # Úsalo solo si ya persististe así en LTM.
    return f"client_0_{int(round(cid, 2) * 100)}"


class BarEmpty(WorldModel):
    """BarEmpty class: activates when no client is present (o cliente no conocido)."""
    def __init__(self, name='world_model', actuation_config=None, perception_config=None,
                 class_name='cognitive_nodes.world_model.WorldModel', **params):

        super().__init__(name, class_name, **params)

        # LTM segura
        if not hasattr(self, 'ltm') or self.ltm is None:
            self.ltm = {}
        self.known_clients = self.ltm.setdefault('known_clients', set())

        # OPT: QoS de servicio estándar
        self.set_activation_service = self.create_service(
            KnowClient,
            f"world_model/{name}/know_client",
            self.know_client_callback,
            callback_group=self.cbgroup_server
        )

        # OPT: callback group dedicado a activación si no necesitas concurrencia
        self.cbgroup_activation = getattr(self, "cbgroup_activation", MutuallyExclusiveCallbackGroup())

        # OPT: logging throttle
        self._last_empty_warn = 0.0
        self._log_throttle_s = 2.0

    def create_activation_input(self, node: dict):
        """Añade suscripciones con QoS de sensor para baja latencia."""
        name = node['name']
        node_type = node['node_type']
        if node_type == "Perception":
            sub = self.create_subscription(
                PerceptionStamped,
                f"perception/{name}/value",
                self.read_activation_callback,
                qos_profile_sensor_data,  # OPT: QoS de sensores
                callback_group=self.cbgroup_activation
            )
            # OPT: no crear nuevos objetos por callback; usa referencias in-place
            self.activation_inputs[name] = dict(
                subscriber=sub,
                data=Perception(),             # placeholder reutilizable
                updated=False,
                timestamp=self.get_clock().now()
            )

    def read_activation_callback(self, msg: PerceptionStamped):
        """Path rápido y mínimo trabajo por callback."""
        # OPT: conversión una sola vez
        pdict = perception_msg_to_dict(msg=msg.perception)

        if not pdict:
            now = self.get_clock().now().seconds_nanoseconds()[0]
            if now - self._last_empty_warn >= self._log_throttle_s:
                self.get_logger().warn("Empty perception received in P-Node. No activation calculated")
                self._last_empty_warn = now
            return

        if len(pdict) > 1:
            self.get_logger().error(f'{self.name} -- Received perception with multiple sensors: ({pdict.keys()}).')
            # Aún así intenta despachar el primero (opcional)
            # return

        # OPT: evita listas/keys temporales
        # toma el primer par (nombre_sensor, valor)
        for node_name, value in pdict.items():
            slot = self.activation_inputs.get(node_name)
            if slot is None:
                # Sensor no esperado; ignora sin log para no inundar
                return
            slot['data'] = value     # Reemplazo de referencia (evita deepcopy)
            slot['updated'] = True
            slot['timestamp'] = Time.from_msg(msg.timestamp)
            break

    def know_client_callback(self, request, response):
        """Marca cliente como conocido en LTM (persistencia simple)."""
        # OPT: única ruta de formateo: usa la convención ya persistida
        client_key = _known_key_from_id_legacy(request.client_id)
        if client_key not in self.known_clients:
            self.known_clients.add(client_key)
            self.ltm['known_clients'] = self.known_clients  # persist
        response.success = True
        return response

    def calculate_activation(self, perception=None, activation_list=None):
        """Activa si NO hay cliente conocido activo."""
        if activation_list is not None:
            # OPT: construir dict directo sin temporales extra
            perception = {s: activation_list[s]['data'] for s in activation_list}
            for s in activation_list:
                activation_list[s]['updated'] = False

        activation_value = 1.0
        if perception:
            client_list = perception.get('client')
            if client_list:
                cid = client_list[0].get('id')
                if cid is not None and cid > 0:
                    # Si ya tienes LTM con la convención legacy, mantén esta:
                    key = _known_key_from_id_legacy(cid)
                    # Alternativa (si migras): key = _client_key_from_id(cid)
                    if key in self.known_clients:
                        activation_value = 0.0

        self.activation.activation = activation_value
        self.activation.timestamp = self.get_clock().now().to_msg()
        return self.activation


class ClientInBar(WorldModel):
    """World model por cliente."""
    def __init__(self, name='world_model', actuation_config=None, perception_config=None,
                 class_name='cognitive_nodes.world_model.WorldModel', preference=None, **params):

        self.preference = preference
        self.last_published_preference = None

        super().__init__(name, class_name, **params)

        # OPT: usa grupos adecuados
        self.timer_cbgroup = ReentrantCallbackGroup()
        self.cbgroup_activation = getattr(self, "cbgroup_activation", MutuallyExclusiveCallbackGroup())

        # OPT: timer más suave (100ms está bien) pero con QoS y sin trabajo pesado dentro
        self.preference_timer = self.create_timer(
            0.01,
            self.log_preference,
            callback_group=self.timer_cbgroup
        )

        # OPT: publisher con QoS sensor (best-effort, depth bajo)
        self.publish_last_bottle = self.create_publisher(
            Float32,
            'cognitive_node/world_model/last_bottle',
            qos_profile_sensor_data
        )
        # OPT: mensaje prealocado
        self._last_bottle_msg = Float32()

        # OPT: throttle para logs (si decides loguear)
        self._last_log_ts = 0.0
        self._log_throttle_s = 2.0

    def calculate_activation(self, perception=None, activation_list=None):
        """Activa cuando el nombre del WM coincide con client_{id} (con _ por .)."""
        self.activation.activation = 0.0

        if activation_list is not None:
            perception = {s: activation_list[s]['data'] for s in activation_list}
            for s in activation_list:
                activation_list[s]['updated'] = False

        if perception:
            activation_value = 0.0
            client_id = None

            # OPT: busca primera clave que contenga "client" sin crear estructuras extra
            for key, value in perception.items():
                if 'client' in key.lower() and value:
                    # value es lista
                    cid = value[0].get('id', None)
                    if cid is not None:
                        client_id = round(cid, 2)
                    break

            if client_id is not None:
                expected_name = _client_key_from_id(client_id)
                if self.name == expected_name:
                    activation_value = 1.0

            self.activation.activation = activation_value

        self.activation.timestamp = self.get_clock().now().to_msg()
        return self.activation

    def set_activation_callback(self, request, response):
        self.get_logger().debug('Setting activation ' + str(request.activation) + '...')
        self.activation.activation = request.activation
        self.activation.timestamp = self.get_clock().now().to_msg()
        response.set = True
        return response

    def create_activation_input(self, node: dict):
        """Suscripciones con QoS de sensores y estado in-place."""
        name = node['name']
        node_type = node['node_type']
        if node_type == "Perception":
            sub = self.create_subscription(
                PerceptionStamped,
                f"perception/{name}/value",
                self.read_activation_callback,
                qos_profile_sensor_data,
                callback_group=self.cbgroup_activation
            )
            # Reutiliza el slot
            self.activation_inputs[name] = dict(
                subscriber=sub,
                data=Perception(),
                updated=False,
                timestamp=Time()
            )
            self.get_logger().debug(f'{self.name} -- Created activation input: {name} ({node_type})')

    def read_activation_callback(self, msg: PerceptionStamped):
        """Callback de lectura con path rápido."""
        pdict = perception_msg_to_dict(msg=msg.perception)
        if not pdict:
            # throttle
            now = self.get_clock().now().seconds_nanoseconds()[0]
            if now - self._last_log_ts >= self._log_throttle_s:
                self.get_logger().warn("Empty perception received in P-Node. No activation calculated")
                self._last_log_ts = now
            return

        if len(pdict) > 1:
            self.get_logger().error(f'{self.name} -- Received perception with multiple sensors: ({pdict.keys()}).')

        for node_name, value in pdict.items():
            slot = self.activation_inputs.get(node_name)
            if slot is None:
                return
            slot['data'] = value
            slot['updated'] = True
            slot['timestamp'] = Time.from_msg(msg.timestamp)
            break

    def log_preference(self):
        """
        Timer: publica 'last_bottle' solo si está activo.
        Mantén el trabajo mínimo aquí para no bloquear el executor.
        """
        if self.preference is not None and self.activation.activation > 0.0:
            # OPT: reutiliza el mensaje
            self._last_bottle_msg.data = float(self.preference)
            self.publish_last_bottle.publish(self._last_bottle_msg)
        else:
            # OPT: publica -1.0 para indicar "ninguno"
            self._last_bottle_msg.data = -1.0
            self.publish_last_bottle.publish(self._last_bottle_msg)
