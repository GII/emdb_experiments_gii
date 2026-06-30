import hashlib
import json
import hashlib

from cognitive_nodes.pnode import PNode
from core.service_client import ServiceClient
from core_interfaces.srv import CreateNode, GetNodeFromLTM
from cognitive_node_interfaces.srv import AddPoint
from core.utils import perception_msg_to_dict, perception_dict_to_msg


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


class PNodeSinglePoint(PNode):
    """
    PNode that keeps one point per node.
    Distinct points are routed into sibling nodes with deterministic names.
    """

    def __init__(self, name='single_point', class_name='cognitive_nodes.pnode.PNode',
                 space_class=None, space=None, history_size=100, **params):
        super().__init__(name, class_name, space_class, space, history_size, **params)
        self.stored_point_signature = None
        self.ltm_client = ServiceClient(GetNodeFromLTM, 'ltm_0/get_node')

    def _normalize_point(self, point):
        if not isinstance(point, dict):
            point = perception_msg_to_dict(point)
        return json.dumps(point, sort_keys=True, separators=(',', ':'), ensure_ascii=True)

    def _point_node_name(self, point):
        signature = self._normalize_point(point)
        digest = hashlib.sha1(signature.encode('utf-8')).hexdigest()[:10]
        return f'{self.name}_{digest}'

    def _point_exists(self, point_node_name):
        response = self.ltm_client.send_request(name=point_node_name)
        return response is not None and bool(response.data)

    def _space_class_name(self):
        if getattr(self, 'space_class', None):
            return self.space_class
        space_class = self.spaces[0].__class__
        return f'{space_class.__module__}.{space_class.__name__}'

    def _create_sibling_for_point(self, point, confidence):
        if not isinstance(point, dict):
            point = perception_msg_to_dict(point)
        point_node_name = self._point_node_name(point)
        if self._point_exists(point_node_name):
            self.get_logger().info(f'{self.name}: exact point already represented by {point_node_name}')
            return True

        # Creation of sibling PNodes has been removed from PNodeSinglePoint.
        # Higher-level logic (MainLoop) is responsible for creating new PNodes
        # when no existing PNode represents the point.
        self.get_logger().info(f'{self.name}: would create sibling {point_node_name}, but creation is delegated to MainLoop')
        return False

    def _point_to_msg(self, point):
        if isinstance(point, dict):
            return perception_dict_to_msg(point)
        return point

    def _handle_point(self, point, confidence):
        point_dict = point if isinstance(point, dict) else perception_msg_to_dict(point)
        point_signature = self._normalize_point(point_dict)

        if self.stored_point_signature is None:
            self.stored_point_signature = point_signature
            super().add_point(point_dict, confidence)
            return True

        if point_signature == self.stored_point_signature:
            self.get_logger().debug(f'{self.name}: exact point already stored in this node')
            return True

        # If this node already has a different point, ignore the new one.
        # Sibling creation is delegated to higher-level logic (MainLoop)
        # so here we simply do not add the point.
        self.get_logger().info(f'{self.name}: point differs from stored point, ignoring (no sibling creation)')
        return False

    def _has_point(self):
        return self.stored_point_signature is not None or self.added_point or (self.space is not None and getattr(self.space, 'size', 0) > 0)

    def add_point_callback(self, request, response):
        """
        Accepts one point per node and creates a sibling node for distinct points.
        """
        self.point_msg = request.point
        confidence = request.confidence
        point = request.point
        response.added = self._handle_point(point, confidence)
        self.get_logger().info('Adding point: ' + str(point) + 'Confidence: ' + str(confidence))
        return response

    def add_point(self, point, confidence):
        """
        Adds the first point locally. Distinct points create sibling nodes.
        """
        return self._handle_point(point, confidence)


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
