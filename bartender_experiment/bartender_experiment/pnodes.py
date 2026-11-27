import json
import numpy as np
from math import isclose

from rclpy.time import Time


from cognitive_nodes.pnode import PNode
from core.service_client import ServiceClient
from core_interfaces.srv import GetNodeFromLTM
from core.container import Container, consolidate_containers
from cognitive_nodes.space import PointBasedSpace
from bartender_experiment.world_model import _known_key_from_id_legacy, _latest_value

from bartender_experiment_interfaces.srv import KnowClient

class PNodeBartenderClient(PNode):
    """
    P-Node that is active when the client has told its preference.

    The activation is binary: 1.0 if the ``client:preference`` of the perception is different from 0, and 0.0 otherwise.
    """
    def __init__(self, name='bartender_client', class_name='cognitive_nodes.pnode.PNode',
                 space_class=None, space=None, history_size=100, **params):
        """
        Constructor for the PNodeBartenderClient class.

        :param name: The name of the P-Node.
        :type name: str
        :param class_name: The name of the P-Node class.
        :type class_name: str
        :param space_class: Full name of the class of the space of the P-Node. It is only used if no space is given.
        :type space_class: str
        :param space: The space of the P-Node.
        :type space: cognitive_nodes.space.PointBasedSpace
        :param history_size: The size of the history of the P-Node, defaults to 100.
        :type history_size: int
        :param params: Additional parameters passed to :class:`cognitive_nodes.pnode.PNode`.
        :type params: dict
        """
        super().__init__(name=name, class_name=class_name, space_class=space_class, space=space, history_size=history_size, **params)
        self.get_logger().info('PNodeBartenderClient: Initialized')

    def calculate_activation(self, perception=None, activation_list=None):
        """
        Calculate the activation: 1.0 if ``client:preference`` is different from 0, 0.0 otherwise (there is no decay).

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

        activation_value = 0.0
        if perception:
            preference = _latest_value(perception, "client:preference")
            if not isclose(preference, 0.0):
                activation_value = 1.0

        perception_timestamp = self.perception.data.coords["timestamp"].values[-1]
        self.activation.activation = activation_value
        self.activation.timestamp = Time(nanoseconds=perception_timestamp).to_msg()
        return self.activation

class PNodeBartenderClientMemory(PNode):
    """
    P-Node that is active when the client has told its preference and the client is not known yet.

    The activation is binary: 1.0 if the ``client:preference`` of the perception is different from 0 and the client
    (``client:id``) has not been registered, and 0.0 otherwise. Clients are registered with the ``pnode/<name>/know_client``
    service (:class:`bartender_experiment_interfaces.srv.KnowClient`), and they are kept in the memory of the node.
    """
    def __init__(self, name='bartender_client', class_name='cognitive_nodes.pnode.PNode',
                 space_class=None, space=None, history_size=100, **params):
        """
        Constructor for the PNodeBartenderClientMemory class. It also creates the ``know_client`` service.

        :param name: The name of the P-Node.
        :type name: str
        :param class_name: The name of the P-Node class.
        :type class_name: str
        :param space_class: Full name of the class of the space of the P-Node. It is only used if no space is given.
        :type space_class: str
        :param space: The space of the P-Node.
        :type space: cognitive_nodes.space.PointBasedSpace
        :param history_size: The size of the history of the P-Node, defaults to 100.
        :type history_size: int
        :param params: Additional parameters passed to :class:`cognitive_nodes.pnode.PNode`.
        :type params: dict
        """
        super().__init__(name=name, class_name=class_name, space_class=space_class, space=space, history_size=history_size, **params)

        self.know_client_service = self.create_service(
            KnowClient,
            f"pnode/{name}/know_client",
            self.know_client_callback,
            callback_group=self.cbgroup_server
        )

        self.known_clients = set()

        self.get_logger().info('PNodeBartenderClient: Initialized')

    def calculate_activation(self, perception=None, activation_list=None):
        """
        Calculate the activation: 1.0 if ``client:preference`` is different from 0 and the client is not known, 0.0 otherwise (there is no decay).

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

        activation_value = 0.0
        if perception:
            preference = _latest_value(perception, "client:preference")
            client_id = _latest_value(perception, "client:id")
            client_key = _known_key_from_id_legacy(client_id)

            # Activate only if the client is not already known and has a non-zero preference
            if not isclose(preference, 0.0) and client_key not in self.known_clients:
                activation_value = 1.0

        perception_timestamp = self.perception.data.coords["timestamp"].values[-1]
        self.activation.activation = activation_value
        self.activation.timestamp = Time(nanoseconds=perception_timestamp).to_msg()
        return self.activation

    def know_client_callback(self, request, response):
        """
        Callback of the ``pnode/<name>/know_client`` service. It registers the client as known.

        :param request: The request, with the ``client_id`` of the client.
        :type request: bartender_experiment_interfaces.srv.KnowClient.Request
        :param response: The response of the service.
        :type response: bartender_experiment_interfaces.srv.KnowClient.Response
        :return: The response, with ``success`` set to True.
        :rtype: bartender_experiment_interfaces.srv.KnowClient.Response
        """
        client_key = _known_key_from_id_legacy(request.client_id)
        if client_key not in self.known_clients:
            self.known_clients.add(client_key)
        response.success = True
        return response

class PNodeNewClientPresent(PNode):
    """
    P-Node that is active when there is a client that is not known yet and has not told its preference.

    The activation is binary: 1.0 if the ``client:preference`` of the perception is 0 and the client (``client:id``) has not been
    registered, and 0.0 otherwise. Clients are registered with the ``pnode/<name>/know_client`` service
    (:class:`bartender_experiment_interfaces.srv.KnowClient`), and they are kept in the memory of the node.
    """
    def __init__(self, name='bartender_client', class_name='cognitive_nodes.pnode.PNode',
                 space_class=None, space=None, history_size=100, **params):
        """
        Constructor for the PNodeNewClientPresent class. It also creates the ``know_client`` service.

        :param name: The name of the P-Node.
        :type name: str
        :param class_name: The name of the P-Node class.
        :type class_name: str
        :param space_class: Full name of the class of the space of the P-Node. It is only used if no space is given.
        :type space_class: str
        :param space: The space of the P-Node.
        :type space: cognitive_nodes.space.PointBasedSpace
        :param history_size: The size of the history of the P-Node, defaults to 100.
        :type history_size: int
        :param params: Additional parameters passed to :class:`cognitive_nodes.pnode.PNode`.
        :type params: dict
        """
        super().__init__(name=name, class_name=class_name, space_class=space_class, space=space, history_size=history_size, **params)

        self.know_client_service = self.create_service(
            KnowClient,
            f"pnode/{name}/know_client",
            self.know_client_callback,
            callback_group=self.cbgroup_server
        )

        self.known_clients = set()

        self.get_logger().info('PNodeBartenderClient: Initialized')

    def calculate_activation(self, perception=None, activation_list=None):
        """
        Calculate the activation: 1.0 if ``client:preference`` is 0 and the client is not known, 0.0 otherwise (there is no decay).

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

        activation_value = 0.0
        if perception:
            preference = _latest_value(perception, "client:preference")
            client_id = _latest_value(perception, "client:id")
            client_key = _known_key_from_id_legacy(client_id)

            # Activate only if the client is not already known and has a non-zero preference
            if isclose(preference, 0.0) and client_key not in self.known_clients:
                activation_value = 1.0

        perception_timestamp = self.perception.data.coords["timestamp"].values[-1]
        self.activation.activation = activation_value
        self.activation.timestamp = Time(nanoseconds=perception_timestamp).to_msg()
        return self.activation

    def know_client_callback(self, request, response):
        """
        Callback of the ``pnode/<name>/know_client`` service. It registers the client as known.

        :param request: The request, with the ``client_id`` of the client.
        :type request: bartender_experiment_interfaces.srv.KnowClient.Request
        :param response: The response of the service.
        :type response: bartender_experiment_interfaces.srv.KnowClient.Response
        :return: The response, with ``success`` set to True.
        :rtype: bartender_experiment_interfaces.srv.KnowClient.Response
        """
        client_key = _known_key_from_id_legacy(request.client_id)
        if client_key not in self.known_clients:
            self.known_clients.add(client_key)
        response.success = True
        return response

class PNodeClientPresent(PNode):
    """
    P-Node that is active when a client is present but has not told its preference yet.

    The activation is binary: 1.0 if the ``client:id`` of the perception is different from 0 and its ``client:preference`` is 0, and 0.0 otherwise.
    """
    def __init__(self, name='client_present', class_name='cognitive_nodes.pnode.PNode',
                 space_class=None, space=None, history_size=100, **params):
        """
        Constructor for the PNodeClientPresent class.

        :param name: The name of the P-Node.
        :type name: str
        :param class_name: The name of the P-Node class.
        :type class_name: str
        :param space_class: Full name of the class of the space of the P-Node. It is only used if no space is given.
        :type space_class: str
        :param space: The space of the P-Node.
        :type space: cognitive_nodes.space.PointBasedSpace
        :param history_size: The size of the history of the P-Node, defaults to 100.
        :type history_size: int
        :param params: Additional parameters passed to :class:`cognitive_nodes.pnode.PNode`.
        :type params: dict
        """
        super().__init__(name=name, class_name=class_name, space_class=space_class, space=space, history_size=history_size, **params)

    def calculate_activation(self, perception=None, activation_list=None):
        """
        Calculate the activation: 1.0 if ``client:id`` is different from 0 and ``client:preference`` is 0, 0.0 otherwise (there is no waiting or decay).

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

        activation_value = 0.0
        if perception:
            client_id = _latest_value(perception, "client:id")
            preference = _latest_value(perception, "client:preference")
            if not isclose(client_id, 0.0) and isclose(preference, 0.0):
                activation_value = 1.0

        perception_timestamp = self.perception.data.coords["timestamp"].values[-1]
        self.activation.activation = activation_value
        self.activation.timestamp = Time(nanoseconds=perception_timestamp).to_msg()

        return self.activation

class LookupTableSpace(PointBasedSpace):
    """
    Exact-match point space.

    It is active only for perceptions that match a stored point within a tolerance. A hash table gives the candidate points
    of each perception and a final ``numpy.isclose`` check with the same tolerance confirms the match. The activation is
    the confidence stored for the matching point, or 0.0 if there is none.
    """

    def __init__(self, atol=1e-8, **kwargs):
        """
        Constructor for the LookupTableSpace class.

        :param atol: Absolute tolerance used to match a perception with a stored point, defaults to 1e-8. If it is 0 or less, only identical values match.
        :type atol: float
        :param kwargs: Additional keyword arguments passed to :class:`cognitive_nodes.space.PointBasedSpace`.
        """
        self.atol = atol
        self._lookup = {}
        super().__init__(**kwargs)

    def _key(self, row):
        row = np.asarray(row, dtype=np.float64)
        if self.atol <= 0.0:
            return tuple(row.tolist())
        return tuple(np.rint(row / self.atol).astype(np.int64))

    def _rebuild_lookup(self):
        self._lookup.clear()
        if self._data is None or self.size == 0:
            return

        members = self.members
        for slot, row in enumerate(members):
            self._lookup.setdefault(self._key(row), []).append(slot)

    @classmethod
    def populate_space(cls, data_container: Container, **kwargs):
        """
        Create the space from a data container, as in :meth:`cognitive_nodes.space.PointBasedSpace.populate_space`, and build the lookup table.

        :param data_container: Container with the points of the space (features plus the confidence column).
        :type data_container: core.container.Container
        :param kwargs: Additional keyword arguments for the constructor of the space.
        :return: The space populated with the data of the container.
        :rtype: LookupTableSpace
        """
        space = super().populate_space(data_container, **kwargs)
        space._rebuild_lookup()
        return space

    def reload_members(self, members, memberships, timestamps):
        """
        Replace the points of the space as in :meth:`cognitive_nodes.space.PointBasedSpace.reload_members` and rebuild the lookup table.

        :param members: Features of the points, one row per point.
        :type members: numpy.ndarray
        :param memberships: Confidence of each point.
        :type memberships: numpy.ndarray
        :param timestamps: Timestamp of each point.
        :type timestamps: numpy.ndarray
        """
        super().reload_members(members, memberships, timestamps)
        self._rebuild_lookup()

    def prune(self, space):
        """
        Prune the features of the space as in :meth:`cognitive_nodes.space.PointBasedSpace.prune` and rebuild the lookup table.

        :param space: The space used for comparison.
        :type space: cognitive_nodes.space.PointBasedSpace
        """
        super().prune(space)
        self._rebuild_lookup()

    def add_point(self, perceptions: Container, confidences: np.ndarray):
        """
        Add points to the space with the insertion rule of :meth:`cognitive_nodes.space.PointBasedSpace.add_point`, and update the lookup table with the new points.

        :param perceptions: The perceptions to add.
        :type perceptions: core.container.Container
        :param confidences: The confidence of each perception.
        :type confidences: numpy.ndarray
        :return: The slot or slots where the points were stored, or -1 if no point was added.
        :rtype: int | numpy.ndarray
        """
        probabilities = self.get_probability(perceptions)
        indexes = (confidences > 0.0) | (probabilities > 0.0)

        data_array = perceptions.read(ordered=True)
        data = data_array.values[indexes]
        timestamps = data_array.coords["timestamp"].values[indexes]
        points = np.concatenate([data, confidences[indexes].reshape(-1, 1)], axis=1)
        labels = perceptions.feature_labels + ["confidence"]

        if points.shape[0] > 0:
            if self.parent_space:
                self.parent_space.add_point(perceptions, confidences)

            if self.size == 0:
                self.initialize_data_structure(perceptions, self.real_size)

            added_slots = self._data.push(points, labels, timestamps=timestamps)
            added_slots = np.atleast_1d(added_slots).astype(int)

            for row, slot in zip(points, added_slots):
                self._lookup.setdefault(self._key(row), []).append(int(slot))

            return added_slots[0] if added_slots.size == 1 else added_slots

        return -1

    def get_probability(self, perceptions):
        """
        Get the activation of the perceptions: the confidence stored for a point that matches each perception, or 0.0 if there is
        none. If the space has a parent space, the minimum of both activations is taken.

        :param perceptions: The perceptions to calculate the activation for.
        :type perceptions: core.container.Container
        :return: The activation values, one per perception row.
        :rtype: numpy.ndarray
        """
        if self._data is None:
            return np.zeros(perceptions.size, dtype=float)

        points = self.data_from_perception(perceptions)
        n_rows = points.shape[0]

        if self.size == 0:
            return np.zeros(n_rows, dtype=float)

        members = self.members
        memberships = self.memberships.reshape(-1)
        activation = np.zeros(n_rows, dtype=float)

        for row_idx, row in enumerate(points):
            slot_list = self._lookup.get(self._key(row))
            if not slot_list:
                continue

            candidate_slots = np.asarray(slot_list, dtype=int)
            candidate_rows = members[candidate_slots]

            close_mask = np.all(
                np.isclose(candidate_rows, row[None, :], atol=self.atol, rtol=0.0),
                axis=1,
            )

            if np.any(close_mask):
                matched_slot = candidate_slots[np.flatnonzero(close_mask)[-1]]
                activation[row_idx] = float(memberships[matched_slot])

        if self.parent_space:
            activation = np.minimum(activation, self.parent_space.get_probability(perceptions))

        return activation.reshape(-1)