import json
import numpy as np

from rclpy.time import Time


from cognitive_nodes.pnode import PNode
from core.service_client import ServiceClient
from core_interfaces.srv import GetNodeFromLTM
from core.container import Container, consolidate_containers
from cognitive_nodes.space import PointBasedSpace

class PNodeBartenderClient(PNode):
    """
    PNode that represents a bartender client.
    Activates when client preference is different from 0.
    """
    def __init__(self, name='bartender_client', class_name='cognitive_nodes.pnode.PNode',
                 space_class=None, space=None, history_size=100, **params):
        super().__init__(name=name, class_name=class_name, space_class=space_class, space=space, history_size=history_size, **params)
        self.get_logger().info('PNodeBartenderClient: Initialized')

    def calculate_activation(self, perception=None, activation_list=None):
        """
        Activates when client preference != 0. Returns 1.0 or 0.0 (sin decay).
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
            preference = float(perception.read().sel(features=["client:preference"]).values[-1]) if "client:preference" in perception.feature_labels else 0.0
            if preference != 0.0:
                activation_value = 1.0

        perception_timestamp = self.perception.data.coords["timestamp"].values[-1]
        self.activation.activation = activation_value
        self.activation.timestamp = Time(nanoseconds=perception_timestamp).to_msg()
        return self.activation



class PNodeClientPresent(PNode):
    """
    PNode that represents that a client is present (but no world model yet).
    Activates with 1.0 while WM no existe; se desactiva (0.0) cuando existe.
    """
    def __init__(self, name='client_present', class_name='cognitive_nodes.pnode.PNode',
                 space_class=None, space=None, history_size=100, **params):
        super().__init__(name=name, class_name=class_name, space_class=space_class, space=space, history_size=history_size, **params)

    def calculate_activation(self, perception=None, activation_list=None):
        """
        Binario 0/1 (sin waits ni decay). Deja que el goal/policy cree el WM.
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
            client_id = float(perception.read().sel(features=["client:id"]).values[-1]) if "client:id" in perception.feature_labels else 0.0
            if client_id != 0.0:
                activation_value = 1.0

        perception_timestamp = self.perception.data.coords["timestamp"].values[-1]
        self.activation.activation = activation_value
        self.activation.timestamp = Time(nanoseconds=perception_timestamp).to_msg()

        return self.activation

class LookupTableSpace(PointBasedSpace):
    """
    Exact-match point space.

    Activates only when a perception matches an already stored point within
    a tolerance. Uses a hash table for fast candidate lookup and a final
    isclose check for correctness.
    """

    def __init__(self, atol=1e-8, **kwargs):
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
        space = super().populate_space(data_container, **kwargs)
        space._rebuild_lookup()
        return space

    def reload_members(self, members, memberships, timestamps):
        super().reload_members(members, memberships, timestamps)
        self._rebuild_lookup()

    def prune(self, space):
        super().prune(space)
        self._rebuild_lookup()

    def add_point(self, perceptions: Container, confidences: np.ndarray):
        """
        Same insertion policy as PointBasedSpace, plus incremental index updates.
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
        Returns the stored confidence for an exact/tolerance match, otherwise 0.0.
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