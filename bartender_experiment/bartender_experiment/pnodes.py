from cognitive_nodes.pnode import PNode
from core.service_client import ServiceClient
from core_interfaces.srv import GetNodeFromLTM


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
