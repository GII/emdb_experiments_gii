import hashlib
import json
import hashlib
from cognitive_nodes.pnode import PNode
from core.service_client import ServiceClient
from core_interfaces.srv import CreateNode, GetNodeFromLTM
from core.utils import perception_msg_to_dict, perception_dict_to_msg, separate_perceptions
from collections import deque
from core.cognitive_node import CognitiveNode
from cognitive_nodes.space import PointBasedSpace
from cognitive_node_interfaces.srv import AddPoint, SendSpace, ContainsSpace
from cognitive_node_interfaces.msg import Perception, PerceptionStamped, SuccessRate

class PNodeBartenderClient(PNode):
    """
    PNode that represents a bartender client.
    Activates when client preference is different from 0.
    """
    def __init__(self, name='bartender_client', class_name='cognitive_nodes.pnode.PNode',
                 space_class=None, space=None, history_size=100, **params):
        super().__init__(name, class_name, space_class, space, history_size, **params)
        self.get_logger().info('PNodeBartenderClient: Initialized')

    def calculate_activation(self, perception=None, activation_list=None):
        """
        Activates when client preference != 0. Returns 1.0 or 0.0 (sin decay).
        """
        if activation_list is not None:
            perception = {}
            for sensor in activation_list:
                activation_list[sensor]['updated'] = False
                perception[sensor] = activation_list[sensor]['data']

        activation_value = 0.0

        if perception:
            client_data = perception.get('client', [])
            if client_data:
                preference = client_data[0].get('preference', 0.0)
                if preference != 0.0:
                    activation_value = 1.0

        self.activation.activation = activation_value
        self.activation.timestamp = self.get_clock().now().to_msg()
        return self.activation



class PNodeClientPresent(PNode):
    """
    PNode that represents that a client is present (but no world model yet).
    Activates with 1.0 while WM no existe; se desactiva (0.0) cuando existe.
    """
    def __init__(self, name='client_present', class_name='cognitive_nodes.pnode.PNode',
                 space_class=None, space=None, history_size=100, **params):
        super().__init__(name, class_name, space_class, space, history_size, **params)
        self.ltm_client = ServiceClient(GetNodeFromLTM, "ltm_0/get_node")
        self.known_world_models = set()
        self.max_cache_size = 500  # solo mantenimiento; no altera la lógica

    def calculate_activation(self, perception=None, activation_list=None):
        """
        Binario 0/1 (sin waits ni decay). Deja que el goal/policy cree el WM.
        """
        if activation_list is not None:
            perception = {}
            for sensor in activation_list:
                activation_list[sensor]['updated'] = False
                perception[sensor] = activation_list[sensor]['data']

        activation_value = 0.0

        if perception:
            client_data = perception.get('client', [])
            if client_data:
                client = client_data[0]
                client_id = client.get('id', None)
                preference = client.get('preference', 0.0)

                if client_id is not None and client_id != 0.0 and preference == 0.0:
                    client_id_str = str(round(client_id, 2)).replace('.', '_')
                    world_model_name = f"client_{client_id_str}"

                    if world_model_name in self.known_world_models:
                        activation_value = 0.0
                    else:
                        try:
                            response = self.ltm_client.send_request(name=world_model_name)
                            if response is not None and response.data:
                                # ya existe → memoriza y desactiva
                                self.known_world_models.add(world_model_name)
                                activation_value = 0.0
                            else:
                                # no existe → activa para que el goal lo cree
                                activation_value = 1.0
                        except Exception as e:
                            # si falla la consulta, asumimos que no existe → activar
                            activation_value = 1.0
                            self.get_logger().warn(
                                f'PNodeClientPresent: Error checking LTM for {world_model_name}: {e}'
                            )

        self.activation.activation = activation_value
        self.activation.timestamp = self.get_clock().now().to_msg()

        # mantenimiento del cache (no cambia la semántica)
        if len(self.known_world_models) > self.max_cache_size:
            self.known_world_models.clear()
            self.get_logger().info("PNodeClientPresent: Cache cleared to prevent memory growth")

        return self.activation

    def mark_world_model_created(self, world_model_name):
        """
        Llamar desde el componente que crea el WM tras hacerlo efectivo.
        """
        self.known_world_models.add(world_model_name)
        self.get_logger().info(f'PNodeClientPresent: Marked {world_model_name} as known in cache')

class LookupSpaceAdapter:
    """
    Space-like adapter for exact-match point storage.

    It preserves the minimal interface expected by PNode and related services,
    but internally behaves as a deterministic lookup table rather than a
    clustering / generalization space.
    """

    def __init__(self, ident='lookup_space', random_seed=None):
        self.ident = ident
        self.random_seed = random_seed
        self.members = []
        self.memberships = []
        self.size = 0
        self._signatures = set()

    def _normalize_point(self, point):
        return json.dumps(point, sort_keys=True, separators=(',', ':'), ensure_ascii=True)

    def _flatten_point(self, point):
        row = []
        for _, values in point.items():
            row.extend(values)
        return row

    def add_point(self, point, confidence):
        signature = self._normalize_point(point)
        row = self._flatten_point(point)

        if signature in self._signatures:
            for i, existing in enumerate(self.members):
                if existing == row:
                    self.memberships[i] = max(self.memberships[i], confidence)
                    return False
            return False

        self.members.append(row)
        self.memberships.append(confidence)
        self._signatures.add(signature)
        self.size += 1
        return True

    def get_probability(self, point):
        signature = self._normalize_point(point)
        if signature not in self._signatures:
            return 0.0

        row = self._flatten_point(point)
        for i, existing in enumerate(self.members):
            if existing == row:
                return max(0.0, self.memberships[i])
        return 0.0

    def same_sensors(self, other_space):
        return True

    def contains(self, compare_space):
        own_rows = {tuple(row) for row in self.members[0:self.size]}
        compare_rows = {tuple(row) for row in compare_space.members[0:compare_space.size]}
        return compare_rows.issubset(own_rows)

    def learnable(self):
        return True


class PNodeLookup(PNode):
    """
    Ablation version of PNode.

    Same public methods and services as the real PNode, but internally uses
    exact-match lookup-table storage instead of clustering/generalization.
    """

    def __init__(self, name='pnode_lookup',
                 class_name='cognitive_nodes.pnode_lookup.PNodeLookup',
                 space_class=None, space=None, history_size=100, **params):
        super().__init__(name, class_name, space_class, space, history_size, **params)

        lookup_space = LookupSpaceAdapter(
            ident=name + " space",
            random_seed=getattr(self, 'random_seed', None)
        )
        self.spaces = [lookup_space]
        self.space = None
        self.point_table = {}

    def _normalize_point(self, point):
        if not isinstance(point, dict):
            point = perception_msg_to_dict(point)
        return json.dumps(point, sort_keys=True, separators=(',', ':'), ensure_ascii=True)

    def has_point(self, point):
        return self._normalize_point(point) in self.point_table

    def point_count(self):
        return len(self.point_table)

    def add_point_callback(self, request, response):
        """
        Callback method for adding a point (or anti-point) to this P-Node.
        """
        self.point_msg = request.point
        confidence = request.confidence
        point = perception_msg_to_dict(self.point_msg)
        response.added = self.add_point(point, confidence)
        self.get_logger().info('Adding point: ' + str(point) + 'Confidence: ' + str(confidence))
        return response

    def add_point(self, point, confidence):
        """
        Add a new point (or anti-point) to the lookup-table P-Node.

        Returns True only if at least one new exact point was inserted.
        Repeated exact points update metadata but are not counted as newly added.
        """
        points = separate_perceptions(point)
        added_any = False

        for point in points:
            self.space = self.spaces[0]
            if not self.space:
                self.space = LookupSpaceAdapter(
                    ident=self.name + " space",
                    random_seed=getattr(self, 'random_seed', None)
                )
                self.spaces = [self.space]

            signature = self._normalize_point(point)

            if signature in self.point_table:
                entry = self.point_table[signature]
                entry["count"] += 1
                entry["confidence"] = max(entry["confidence"], confidence)
            else:
                self.point_table[signature] = {
                    "point": point,
                    "confidence": confidence,
                    "count": 1
                }
                added_any = True

            self.space.add_point(point, confidence)

        self.added_point = self.added_point or added_any
        self.update_history(confidence)
        self.publish_success_rate()
        return added_any

    def calculate_activation(self, perception=None, activation_list=None):
        """
        Calculate activation using exact-match lookup instead of learned probability.
        """
        if activation_list is not None:
            perception = {}
            for sensor in activation_list:
                activation_list[sensor]['updated'] = False
                perception[sensor] = activation_list[sensor]['data']

        if perception:
            activations = []
            perceptions = separate_perceptions(perception)

            for perception_line in perceptions:
                if self.spaces[0] and self.added_point:
                    signature = self._normalize_point(perception_line)
                    if signature in self.point_table:
                        activation_value = max(0.0, self.point_table[signature]["confidence"])
                    else:
                        activation_value = 0.0
                    self.get_logger().debug(
                        f'PNODE LOOKUP DEBUG: Perception: {perception_line} Activation: {activation_value}'
                    )
                else:
                    activation_value = 0.0

                activations.append(activation_value)

            self.activation.activation = (
                activations[0] if len(activations) == 1 else float(max(activations))
            )
            self.activation.timestamp = self.get_clock().now().to_msg()

        return self.activation

    def get_space(self, perception):
        """
        Return the compatible space with perception.

        Kept for interface compatibility with the real PNode.
        """
        temp_space = LookupSpaceAdapter(random_seed=getattr(self, 'random_seed', None))
        temp_space.add_point(perception, 1.0)
        for space in self.spaces:
            if (not space.size) or space.same_sensors(temp_space):
                return space
        return None

    def send_pnode_space_callback(self, request, response):
        """
        Callback that sends the space of the P-Node in the same flattened format
        expected by external callers.
        """
        if self.space:
            if not self.data_labels and hasattr(self, 'point_msg'):
                self.configure_labels()
            response.labels = self.data_labels

            data = []
            for perception in self.space.members[0:self.space.size]:
                for value in perception:
                    data.append(value)
            response.data = data

            confidences = list(self.space.memberships[0:self.space.size])
            response.confidences = confidences

        return response

    def contains_space_callback(self, request, response):
        """
        Callback that checks if the lookup-table space contains a given space.
        """
        labels = request.labels
        data = request.data
        confidences = request.confidences

        compare_space = PointBasedSpace(len(confidences))
        compare_space.populate_space(labels, data, confidences)

        if self.space:
            response.contained = self.space.contains(compare_space)
        else:
            response.contained = False
        return response

    def update_history(self, confidence):
        """
        Updates the history of the P-Node with the new confidence value.
        Kept structurally aligned with the real PNode.
        """
        if confidence > 0 and self.spaces[0].learnable():
            self.history.appendleft(True)
        else:
            self.history.appendleft(False)
        self.success_rate = sum(self.history) / self.history.maxlen