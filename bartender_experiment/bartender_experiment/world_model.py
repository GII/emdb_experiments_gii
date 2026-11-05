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

def _latest_value(perception, label, default=0.0):
    """
    Value of a feature in the latest sample of a perception. It reads the container with read_numpy:
    this runs in every activation update, and read() builds a new xarray object in every call.

    :param perception: The perception.
    :type perception: core.container.Container
    :param label: The feature, e.g. "client:id".
    :type label: str
    :param default: Value returned if the perception does not have the feature.
    :return: The value as a float, or default.
    """
    labels = perception.feature_labels
    if label not in labels:
        return default
    values, _ = perception.read_numpy()
    return float(values[-1, labels.index(label)])

def _known_key_from_id_legacy(cid: float) -> str:
    # Mantiene tu convención anterior "client_0_{int(cid*100)}"
    # Úsalo solo si ya persististe así en LTM.
    return f"client_0_{int(round(cid, 2) * 100)}"


class BarEmpty(WorldModel):
    """
    World Model that is active while the client in the perception is not known.

    The activation is 1.0, except when the perceived client (``client:id`` of the perception) has been registered with the
    ``world_model/<name>/know_client`` service (:class:`bartender_experiment_interfaces.srv.KnowClient`), in which case it is 0.0.
    The known clients are stored in the LTM data of the node. While it is active, it publishes its ``preference`` (0)
    as a ``std_msgs/msg/Float32`` on ``cognitive_node/world_model/last_bottle``.
    """
    def __init__(self, name='world_model', class_name='cognitive_nodes.world_model.WorldModel', **params):
        """
        Constructor for the BarEmpty class. It creates the ``know_client`` service and the publisher of the last bottle.

        :param name: The name of the World Model.
        :type name: str
        :param class_name: The name of the World Model class.
        :type class_name: str
        :param params: Additional parameters passed to :class:`cognitive_nodes.world_model.WorldModel`.
        :type params: dict
        """

        super().__init__(name=name, class_name=class_name, **params)

        # LTM segura
        if not hasattr(self, 'ltm') or self.ltm is None:
            self.ltm = {}
        self.known_clients = self.ltm.setdefault('known_clients', set())

        # OPT: QoS de servicio estándar
        self.know_client_service = self.create_service(
            KnowClient,
            f"world_model/{name}/know_client",
            self.know_client_callback,
            callback_group=self.cbgroup_server
        )

        # OPT: callback group dedicado a activación si no necesitas concurrencia
        self.cbgroup_activation = getattr(self, "cbgroup_activation", MutuallyExclusiveCallbackGroup())

        self.perception = None  # placeholder for consolidated perception data

        # Publish no preference
        self.preference = 0
        self._last_bottle_msg = Float32()
        self.publish_last_bottle = self.create_publisher(
            Float32,
            'cognitive_node/world_model/last_bottle',
            1
        )


    def create_activation_input(self, node: dict):
        """
        Create the subscription to the value of a Perception neighbor, with the sensor data QoS profile, and register it in the activation inputs.
        Neighbors that are not Perceptions are ignored.

        :param node: The neighbor, with its ``name`` and ``node_type``.
        :type node: dict
        """
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
        Callback that stores a perception in the activation inputs. Only messages with a single reading (``max_size`` of 1) are supported.

        :param msg: The perception, as a container message.
        :type msg: core_interfaces.msg.Container
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
        """
        Callback of the ``know_client`` service. It registers the client as known and stores it in the LTM data of the node.

        :param request: The request, with the ``client_id`` of the client.
        :type request: bartender_experiment_interfaces.srv.KnowClient.Request
        :param response: The response of the service.
        :type response: bartender_experiment_interfaces.srv.KnowClient.Response
        :return: The response, with ``success`` set to True.
        :rtype: bartender_experiment_interfaces.srv.KnowClient.Response
        """
        # OPT: única ruta de formateo: usa la convención ya persistida
        client_key = _known_key_from_id_legacy(request.client_id)
        if client_key not in self.known_clients:
            self.known_clients.add(client_key)
            self.ltm['known_clients'] = self.known_clients  # persist
        response.success = True
        return response

    def calculate_activation(self, perception=None, activation_list=None):
        """
        Calculate the activation of the World Model: 1.0 unless the client in the perception is known (see the class description).
        The timestamp of the activation is that of the last sample of the perception, and the preference is published with :meth:`log_preference`.

        :param perception: Perception to use if no activation list is given.
        :type perception: core.container.Container
        :param activation_list: Dictionary of the activation inputs, with the data of each input (``activation_list[name]['data']``).
        :type activation_list: dict
        :return: The ``Activation`` message of the node.
        :rtype: cognitive_node_interfaces.msg.Activation
        """
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
            client_id = _latest_value(perception, "client:id")
            if client_id != 0.0:
                # Si ya tienes LTM con la convención legacy, mantén esta:
                key = _known_key_from_id_legacy(client_id)
                # Alternativa (si migras): key = _client_key_from_id(cid)
                if key in self.known_clients:
                    activation_value = 0.0

        perception_timestamp = self.perception.data.coords["timestamp"].values[-1]
        self.activation.activation = activation_value
        self.activation.timestamp = Time(nanoseconds=perception_timestamp).to_msg()
        self.log_preference(client_id)
        return self.activation

    def log_preference(self, client_id):
        """
        Publish the preference on ``cognitive_node/world_model/last_bottle`` if the World Model is active.

        :param client_id: Normalized id of the client in the perception.
        :type client_id: float
        """
        if self.preference is not None and self.activation.activation > 0.0:
            # OPT: reutiliza el mensaje
            self._last_bottle_msg.data = float(self.preference)
            self.publish_last_bottle.publish(self._last_bottle_msg)


class BarAnyClient(BarEmpty):
    """
    World Model that is always active and publishes the preference of the perceived client.

    It extends :class:`BarEmpty`: its ``know_client`` service also stores the preference of the client. The preference
    published on ``cognitive_node/world_model/last_bottle`` is that of the client in the perception, or -1.0 if the client is not registered.
    """
    def __init__(self, name='world_model',
                 class_name='cognitive_nodes.world_model.WorldModel', **params):
        """
        Constructor for the BarAnyClient class.

        :param name: The name of the World Model.
        :type name: str
        :param class_name: The name of the World Model class.
        :type class_name: str
        :param params: Additional parameters passed to :class:`BarEmpty`.
        :type params: dict
        """
        super().__init__(name=name, class_name=class_name, **params)

        self.client_preferences = self.ltm.setdefault('client_preferences', {})
        self.publish_last_bottle = self.create_publisher(
            Float32,
            'cognitive_node/world_model/last_bottle',
            1
        )
        self._last_bottle_msg = Float32()

    def know_client_callback(self, request, response):
        """
        Callback of the ``know_client`` service. It registers the client as known, stores its preference and saves both in the LTM data of the node.

        :param request: The request, with the ``client_id`` and the ``preference`` of the client.
        :type request: bartender_experiment_interfaces.srv.KnowClient.Request
        :param response: The response of the service.
        :type response: bartender_experiment_interfaces.srv.KnowClient.Response
        :return: The response, with ``success`` set to True.
        :rtype: bartender_experiment_interfaces.srv.KnowClient.Response
        """
        client_key = _known_key_from_id_legacy(request.client_id)
        self.known_clients.add(client_key)
        self.client_preferences[client_key] = float(request.preference)
        self.ltm['known_clients'] = self.known_clients
        self.ltm['client_preferences'] = self.client_preferences
        response.success = True
        return response

    def calculate_activation(self, perception=None, activation_list=None):
        """
        Set the activation to 1.0 and publish the preference of the client in the perception on ``cognitive_node/world_model/last_bottle``.

        :param perception: Perception to use if no activation list is given.
        :type perception: core.container.Container
        :param activation_list: Dictionary of the activation inputs, with the data of each input (``activation_list[name]['data']``).
        :type activation_list: dict
        :return: The ``Activation`` message of the node.
        :rtype: cognitive_node_interfaces.msg.Activation
        """
        if activation_list is not None:
            data = [activation_list[sensor]['data'] for sensor in activation_list]
            if self.perception is None and len(data) > 0:
                self.perception = consolidate_containers(
                    data, name="perception", container_type="perception"
                )
            elif len(data) > 0:
                consolidate_containers(data, write_container=self.perception)
            perception = self.perception

        self.activation.activation = 1.0

        if perception:
            client_id = _latest_value(perception, "client:id")
            client_key = _known_key_from_id_legacy(client_id)
            preference = self.client_preferences.get(client_key, -1.0)
            perception_timestamp = self.perception.data.coords["timestamp"].values[-1]
            self.activation.timestamp = Time(
                nanoseconds=perception_timestamp
            ).to_msg()
        else:
            preference = -1.0
            self.activation.timestamp = self.get_clock().now().to_msg()

        self._last_bottle_msg.data = float(preference)
        self.publish_last_bottle.publish(self._last_bottle_msg)
        return self.activation


class ClientInBar(WorldModel):
    """
    World Model of one client.

    It is active (1.0) only while the client in the perception (``client:id``, rounded to two decimals) is the client of the
    node: the name of the node has to be ``client_<id>``, with the decimal point of the id replaced by ``_`` (for example ``client_0_67``
    for the id 0.67). While it is active, it publishes its preference as a ``std_msgs/msg/Float32`` on ``cognitive_node/world_model/last_bottle``.
    The nodes are created by :class:`bartender_experiment.policy.BartenderClientPolicy`.
    """
    def __init__(self, name='world_model', actuation_config=None, perception_config=None,
                 class_name='cognitive_nodes.world_model.WorldModel', preference=None, **params):
        """
        Constructor for the ClientInBar class.

        :param name: The name of the World Model, ``client_<id>``.
        :type name: str
        :param actuation_config: Actuation configuration. It is not used.
        :type actuation_config: dict
        :param perception_config: Perception configuration. It is not used.
        :type perception_config: dict
        :param class_name: The name of the World Model class.
        :type class_name: str
        :param preference: Preference (drink type) of the client, published while the node is active.
        :type preference: float
        :param params: Additional parameters passed to :class:`cognitive_nodes.world_model.WorldModel`.
        :type params: dict
        """

        self.preference = preference
        self.last_published_preference = None

        super().__init__(name=name, class_name=class_name, **params)

        # OPT: usa grupos adecuados
        self.timer_cbgroup = ReentrantCallbackGroup()
        self.cbgroup_activation = getattr(self, "cbgroup_activation", MutuallyExclusiveCallbackGroup())

        self.publish_last_bottle = self.create_publisher(
            Float32,
            'cognitive_node/world_model/last_bottle',
            1
        )
        # OPT: mensaje prealocado
        self._last_bottle_msg = Float32()


        self.perception = None  # placeholder for consolidated perception data

    def calculate_activation(self, perception=None, activation_list=None):
        """
        Calculate the activation of the World Model: 1.0 if the client in the perception is the client of the node, 0.0 otherwise.
        The timestamp of the activation is that of the last sample of the perception, and the preference is published with :meth:`log_preference`.

        :param perception: Perception to use if no activation list is given.
        :type perception: core.container.Container
        :param activation_list: Dictionary of the activation inputs, with the data of each input (``activation_list[name]['data']``).
        :type activation_list: dict
        :return: The ``Activation`` message of the node.
        :rtype: cognitive_node_interfaces.msg.Activation
        """
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
            cid = _latest_value(perception, "client:id", None)
            client_id = round(cid, 2) if cid is not None else None

            if client_id is not None:
                expected_name = _client_key_from_id(client_id)
                if self.name == expected_name:
                    activation_value = 1.0

            self.activation.activation = activation_value

        perception_timestamp = self.perception.data.coords["timestamp"].values[-1]
        self.activation.timestamp = Time(nanoseconds=perception_timestamp).to_msg()
        self.log_preference()
        return self.activation

    def set_activation_callback(self, request, response):
        """
        Callback of the set activation service. It sets the activation of the node to the requested value.

        :param request: The request, with the new ``activation``.
        :type request: cognitive_node_interfaces.srv.SetActivation.Request
        :param response: The response of the service.
        :type response: cognitive_node_interfaces.srv.SetActivation.Response
        :return: The response, with ``set`` set to True.
        :rtype: cognitive_node_interfaces.srv.SetActivation.Response
        """
        self.get_logger().debug('Setting activation ' + str(request.activation) + '...')
        self.activation.activation = request.activation
        self.activation.timestamp = self.get_clock().now().to_msg()
        response.set = True
        return response

    # TODO: Refactor common methods in BarEmpty and ClientInBar to avoid code duplication
    def create_activation_input(self, node: dict):
        """
        Create the subscription to the value of a Perception neighbor and register it in the activation inputs. Neighbors that are not Perceptions are ignored.

        :param node: The neighbor, with its ``name`` and ``node_type``.
        :type node: dict
        """
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
        Callback that stores a perception in the activation inputs. Only messages with a single reading (``max_size`` of 1) are supported.

        :param msg: The perception, as a container message.
        :type msg: core_interfaces.msg.Container
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
        Publish the preference of the client on ``cognitive_node/world_model/last_bottle`` if the World Model is active.
        """
        if self.preference is not None and self.activation.activation > 0.0:
            # OPT: reutiliza el mensaje
            self._last_bottle_msg.data = float(self.preference)
            self.publish_last_bottle.publish(self._last_bottle_msg)
