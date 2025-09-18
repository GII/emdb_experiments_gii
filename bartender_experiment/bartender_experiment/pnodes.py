from cognitive_nodes.pnode import PNode
from core.service_client import ServiceClient
from core_interfaces.srv import GetNodeFromLTM


class PNodeBartenderClient(PNode):
    """
    PNode that represents a bartender client
    """
    def __init__(self, name='bartender_client', class_name='cognitive_nodes.pnode.PNode', space_class=None, space=None, history_size=100, **params):
        super().__init__(name, class_name, space_class, space, history_size, **params)

    def calculate_activation(self, perception=None, activation_list=None):
        """
        Calculate the new activation value for a given perception.
        Activates when client preference is different from 0.

        :param perception: The perception for which PNode activation is calculated.
        :type perception: dict
        :return: If there is space, returns the activation of the PNode. If not, returns 0
        :rtype: float
        """
        if activation_list is not None:
            perception = {}
            for sensor in activation_list:
                activation_list[sensor]['updated'] = False
                perception[sensor] = activation_list[sensor]['data']

        activation_value = 0.0

        if perception:
            # Get client data (always a list)
            client_data = perception.get('client', [])
            self.get_logger().debug(f'PNodeBartenderClient: Perception client data: {client_data}')
            
            # Check if there are clients in the list
            if client_data and len(client_data) > 0:
                # Take the first client (always a dict)
                client = client_data[0]
                preference = client.get('preference', 0.0)
                
                if preference != 0.0:
                    activation_value = 1.0
                    self.get_logger().debug(f'PNodeBartenderClient: Client has preference {preference}, ACTIVATING')
                else:
                    self.get_logger().debug(f'PNodeBartenderClient: Client preference is 0.0, staying DEACTIVATED')
            else:
                self.get_logger().debug(f'PNodeBartenderClient: No client data found')

        self.activation.activation = activation_value
        self.activation.timestamp = self.get_clock().now().to_msg()
        
        self.get_logger().debug(f'PNodeBartenderClient: Final activation value: {activation_value}')
        return self.activation


class PNodeClientPresent(PNode):
    """
    PNode that represents that a client is present
    """
    def __init__(
        self,
        name='client_present',
        class_name='cognitive_nodes.pnode.PNode',
        space_class=None,
        space=None,
        history_size=100,
        **params
    ):
        super().__init__(name, class_name, space_class, space, history_size, **params)
        self.ltm_client = ServiceClient(GetNodeFromLTM, "ltm_0/get_node")

    def calculate_activation(self, perception=None, activation_list=None):
        """
        Calculate the new activation value for a given perception.
        Activates when a client is present but NO world model exists for that client yet.
        Excludes clients with ID 0.0 (invalid clients).

        :param perception: The perception for which PNode activation is calculated.
        :type perception: dict
        :return: If there is space, returns the activation of the PNode. If not, returns 0
        :rtype: float
        """
        if activation_list is not None:
            perception = {}
            for sensor in activation_list:
                activation_list[sensor]['updated'] = False
                perception[sensor] = activation_list[sensor]['data']

        activation_value = 0.0

        if perception:
            # Get client data (always a list)
            client_data = perception.get('client', [])
            self.get_logger().debug(f'PNodeClientPresent: Perception client data: {client_data}')

            # Check if there are clients in the list
            if client_data and len(client_data) > 0:
                # Take the first client (always a dict)
                client = client_data[0]
                client_id = client.get('id', None)
                preference = client.get('preference', 0.0)

                # Check if client world model already exists or preference is 0.0
                if client_id is not None and client_id != 0.0 and preference == 0.0:
                    client_id_rounded = round(client_id, 2) # Round the ID to 2 decimal places to avoid long strings
                    client_id_str = str(client_id_rounded).replace('.', '_')
                    world_model_name = f"client_{client_id_str}"
                    
                    try:                   
                        request = GetNodeFromLTM.Request()
                        request.name = world_model_name
                        self.get_logger().debug(f'PNodeClientPresent: Checking LTM for world model: {world_model_name}')
                        response = self.ltm_client.send_request(name=world_model_name)
                        
                        if response is not None and response.data:
                            activation_value = 0.0
                            self.get_logger().debug(f'PNodeClientPresent: World model {world_model_name} found in LTM, DEACTIVATING')
                            self.get_logger().debug(f'PNodeClientPresent: Response: {response}')
                        elif response is not None and not response.data and preference == 0.0:
                            activation_value = 1.0
                            self.get_logger().debug(f'PNodeClientPresent: World model {world_model_name} NOT found in LTM, ACTIVATING')
                            self.get_logger().debug(f'PNodeClientPresent: Response: {response}')
                            
                    except Exception as e:
                        # Error accessing LTM, assume world model doesn't exist, ACTIVATE
                        activation_value = 1.0 
                        self.get_logger().error(f'PNodeClientPresent: Error checking LTM for {world_model_name}: {e}')
                        self.get_logger().debug(f'PNodeClientPresent: Assuming world model does not exist, ACTIVATING')
                else:
                    # No valid client ID found (or client ID is 0.0)
                    self.get_logger().debug('PNodeClientPresent: Invalid or missing client ID, not activating')
            else:
                self.get_logger().debug('PNodeClientPresent: No client data found in perception')

        self.activation.activation = activation_value
        self.activation.timestamp = self.get_clock().now().to_msg()
        
        self.get_logger().debug(f'PNodeClientPresent: Final activation value: {activation_value}')
        return self.activation
