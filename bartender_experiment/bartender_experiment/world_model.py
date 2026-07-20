# TODO: Clean imports, remove unused ones
import rclpy
from rclpy.time import Time
from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from core.container import Container, consolidate_containers

from core_interfaces.msg import Container as ContainerMsg
from cognitive_nodes.world_model import WorldModel
from bartender_experiment_interfaces.srv import KnowClient

from std_msgs.msg import Float32


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
    def __init__(self, name='world_model', class_name='cognitive_nodes.world_model.WorldModel', **params):

        super().__init__(name=name, class_name=class_name, **params)

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

        self.perception = None  # placeholder for consolidated perception data

    def create_activation_input(self, node: dict):
        """Añade suscripciones con QoS de sensor para baja latencia."""
        name = node['name']
        node_type = node['node_type']
        if node_type == "Perception":
            sub = self.create_subscription(
                ContainerMsg,
                f"perception/{name}/value",
                self.read_activation_callback,
                qos_profile_sensor_data,  # OPT: QoS de sensores
                callback_group=self.cbgroup_activation
            )
            # OPT: no crear nuevos objetos por callback; usa referencias in-place
            self.activation_inputs[name] = dict(
                subscriber=sub,
                data=None,             # placeholder reutilizable
                updated=False
            )

    def read_activation_callback(self, msg: ContainerMsg):
        """
        Callback method that reads a perception and stores it in the activation inputs list.

        :param msg: PerceptionStamped message that contains the perception and its timestamp.
        :type msg: cognitive_node_interfaces.msg.PerceptionStamped
        """        
        if msg.max_size>1:
            self.get_logger().error(f'Received perception with multiple readings: ({msg.name}). Perception messages should (currently) include only one reading!')
        elif msg.max_size==1:
            node_name=msg.name
            if node_name in self.activation_inputs:
                if self.activation_inputs[node_name]['data'] is None:
                    self.activation_inputs[node_name]['data']=Container.from_msg(msg)
                else:
                    self.activation_inputs[node_name]['data'].push_from_msg(msg)
                self.activation_inputs[node_name]['updated']=True
            else:
                self.get_logger().error(
                    "Received perception not registered in local perception cache!!!"
                )
        else:
            self.get_logger().warn("Empty perception recieved in P-Node")

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
            data = [activation_list[sensor]['data'] for sensor in activation_list]
            if self.perception is None and len(data)>0:
                self.perception = consolidate_containers(data, name="perception", container_type="perception")
            elif len(data)==0: # Activation list may be empty when initializing the P-Node.
                self.activation.activation = 0.0
                self.activation.timestamp = self.get_clock().now().to_msg()
                return self.activation
            else:
                consolidate_containers(data, write_container=self.perception)
            perception = self.perception

        activation_value = 1.0 # Activate by default (no client known)
        if perception:
            client_id = float(perception.read().sel(features=["client:id"]).values[-1]) if "client:id" in perception.feature_labels else 0.0
            if client_id != 0.0:
                # Si ya tienes LTM con la convención legacy, mantén esta:
                key = _known_key_from_id_legacy(client_id)
                # Alternativa (si migras): key = _client_key_from_id(cid)
                if key in self.known_clients:
                    activation_value = 0.0

        perception_timestamp = self.perception.data.coords["timestamp"].values[-1]
        self.activation.activation = activation_value
        self.activation.timestamp = Time(nanoseconds=perception_timestamp).to_msg()
        return self.activation


class ClientInBar(WorldModel):
    """World model por cliente."""
    def __init__(self, name='world_model', actuation_config=None, perception_config=None,
                 class_name='cognitive_nodes.world_model.WorldModel', preference=None, **params):

        self.preference = preference
        self.last_published_preference = None

        super().__init__(name=name, class_name=class_name, **params)

        # OPT: usa grupos adecuados
        self.timer_cbgroup = ReentrantCallbackGroup()
        self.cbgroup_activation = getattr(self, "cbgroup_activation", MutuallyExclusiveCallbackGroup())

        # OPT: timer más suave (100ms está bien) pero con QoS y sin trabajo pesado dentro
        self.preference_timer = self.create_timer(
            0.01,
            self.log_preference,
            callback_group=self.timer_cbgroup
        )

        self.publish_last_bottle = self.create_publisher(
            Float32,
            'cognitive_node/world_model/last_bottle',
            1
        )
        # OPT: mensaje prealocado
        self._last_bottle_msg = Float32()

        # OPT: throttle para logs (si decides loguear)
        self._last_log_ts = 0.0
        self._log_throttle_s = 2.0

        self.perception = None  # placeholder for consolidated perception data

    def calculate_activation(self, perception=None, activation_list=None):
        """Activa cuando el nombre del WM coincide con client_{id} (con _ por .)."""
        self.activation.activation = 0.0

        if activation_list is not None:
            data = [activation_list[sensor]['data'] for sensor in activation_list]
            if self.perception is None and len(data)>0:
                self.perception = consolidate_containers(data, name="perception", container_type="perception")
            elif len(data)==0: # Activation list may be empty when initializing the P-Node.
                self.activation.activation = 0.0
                self.activation.timestamp = self.get_clock().now().to_msg()
                return self.activation
            else:
                consolidate_containers(data, write_container=self.perception)
            perception = self.perception

        if perception:
            activation_value = 0.0

            # OPT: busca primera clave que contenga "client" sin crear estructuras extra
            cid = float(perception.read().sel(features=["client:id"]).values[-1]) if "client:id" in perception.feature_labels else None
            client_id = round(cid, 2) if cid is not None else None

            if client_id is not None:
                expected_name = _client_key_from_id(client_id)
                if self.name == expected_name:
                    activation_value = 1.0

            self.activation.activation = activation_value

        perception_timestamp = self.perception.data.coords["timestamp"].values[-1]
        self.activation.timestamp = Time(nanoseconds=perception_timestamp).to_msg()
        return self.activation

    def set_activation_callback(self, request, response):
        self.get_logger().debug('Setting activation ' + str(request.activation) + '...')
        self.activation.activation = request.activation
        self.activation.timestamp = self.get_clock().now().to_msg()
        response.set = True
        return response

    # TODO: Refactor common methods in BarEmpty and ClientInBar to avoid code duplication
    def create_activation_input(self, node: dict):
        """Añade suscripciones con QoS de sensor para baja latencia."""
        name = node['name']
        node_type = node['node_type']
        if node_type == "Perception":
            sub = self.create_subscription(
                ContainerMsg,
                f"perception/{name}/value",
                self.read_activation_callback,
                1,
                callback_group=self.cbgroup_activation
            )
            # OPT: no crear nuevos objetos por callback; usa referencias in-place
            self.activation_inputs[name] = dict(
                subscriber=sub,
                data=None,             # placeholder reutilizable
                updated=False
            )

    def read_activation_callback(self, msg: ContainerMsg):
        """
        Callback method that reads a perception and stores it in the activation inputs list.

        :param msg: PerceptionStamped message that contains the perception and its timestamp.
        :type msg: cognitive_node_interfaces.msg.PerceptionStamped
        """        
        if msg.max_size>1:
            self.get_logger().error(f'Received perception with multiple readings: ({msg.name}). Perception messages should (currently) include only one reading!')
        elif msg.max_size==1:
            node_name=msg.name
            if node_name in self.activation_inputs:
                if self.activation_inputs[node_name]['data'] is None:
                    self.activation_inputs[node_name]['data']=Container.from_msg(msg)
                else:
                    self.activation_inputs[node_name]['data'].push_from_msg(msg)
                self.activation_inputs[node_name]['updated']=True
            else:
                self.get_logger().error(
                    "Received perception not registered in local perception cache!!!"
                )
        else:
            self.get_logger().warn("Empty perception recieved in P-Node")

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
