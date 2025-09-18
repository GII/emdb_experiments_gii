from cognitive_nodes.policy import Policy
from core.utils import perception_dict_to_msg, class_from_classname, perception_msg_to_dict
from core.service_client import ServiceClientAsync
from bartender_experiment_interfaces.srv import KnowClient

class BartenderClientPolicy(Policy):
    """
    BartenderClientPolicy class. Represents a policy for the bartender client.
    """    
    def __init__(self, name='BartenderClientPolicy', class_name='cognitive_nodes.policy.Policy', service_msg=None, service_name=None, **params):
        """
        Constructor for the BartenderClientPolicy class.

        :param name: The name of the policy.
        :type name: str
        :param class_name: The name of the base Policy class.
        :type class_name: str
        :param service_msg: Message type of the service that executes the policy.
        :type service_msg: ROS2 message type. Typically cognitive_node_interfaces.srv.Policy
        :param service_name: Name of the service that executes the policy.
        :type service_name: str
        """        
        super().__init__(name, class_name, **params)
        self.know_client_service = None

    async def execute_callback(self, request, response):
        """
        Makes a service call to the server that handles the execution of the policy.
        """
        self.get_logger().debug('=== EXECUTING BARTENDER POLICY: ' + self.name + ' ===')
    
        try:
            # Extract client ID and preference from perception
            client_id, client_preference = self.get_client_data(request.perception)
            self.get_logger().debug(f"POLICY INFO: Extracted client_id: {client_id}, preference: {client_preference}")
            
            if client_id is not None and client_id != "0_0":
                client_name = f"client_{client_id}"
                self.get_logger().debug(f"POLICY INFO: Client name: {client_name}")
                
                # Check if world model already exists
                expected_topic = f"/cognitive_node/{client_name}/activation"
                self.get_logger().debug(f"POLICY INFO: Expected topic: {expected_topic}")
                
                topic_names_and_types = self.get_topic_names_and_types()
                existing_topics = [topic_name for topic_name, _ in topic_names_and_types]
                self.get_logger().debug(f"POLICY INFO: Found {len(existing_topics)} topics")
                
                if expected_topic not in existing_topics:
                    self.get_logger().debug(f"POLICY INFO: Creating world model for client: {client_name}")
                    
                    # Create the world model node with client's actual preference
                    success = self.create_node_client(
                        name=client_name, 
                        class_name="bartender_experiment.world_model.ClientInBar",
                        parameters={"preference": client_preference}  # Use actual client preference
                    )
                    
                    if success:
                        self.get_logger().info(f"World model {client_name} created successfully")
                        
                        # Add client to known_clients list - apply same rounding
                        client_id_float = float(client_id.replace('_', '.'))
                        client_id_rounded = round(client_id_float, 2)  # Apply same rounding
                        success = await self.add_known_client(client_id_rounded)
                        
                        if success:
                            self.get_logger().info(f"Client {client_id_rounded} added to known clients")
                        else:
                            self.get_logger().error(f"POLICY ERROR: Failed to add client {client_id_rounded} to known clients")
                    else:
                        self.get_logger().error(f"POLICY ERROR: Failed to create world model {client_name}")
                else:
                    self.get_logger().debug(f"POLICY INFO: World model {client_name} already exists")
            else:
                self.get_logger().warn("POLICY WARN: Could not extract valid client ID from perception")
                
        except Exception as e:
            self.get_logger().error(f"POLICY ERROR: Exception in execute_callback: {e}")
            import traceback
            self.get_logger().error(f"POLICY ERROR: Traceback: {traceback.format_exc()}")
    
        response.policy = self.name
        self.get_logger().debug('=== FINISHED BARTENDER POLICY: ' + self.name + ' ===')
        return response
    
    async def add_known_client(self, client_id):
        """
        Add a client to the known clients list using the know_client service.
        """
        try:
            service_name = "/world_model/BARTENDER/know_client"
            
            # Crear cliente del servicio cada vez (más simple)
            self.get_logger().info(f"Creating service client for {service_name}")
            client = ServiceClientAsync(
                self, KnowClient, service_name, self.cbgroup_client
            )
            
            # Crear y enviar request
            request = KnowClient.Request()
            request.client_id = client_id
            
            self.get_logger().debug(f"Sending request to add client {client_id} to known clients")
            response = await client.send_request_async(client_id=client_id)
            
            if response is not None:
                self.get_logger().debug(f"Add known client result: {response.success}")
                return response.success
            else:
                self.get_logger().error(f"Received None response from service")
                return False
            
        except Exception as e:
            self.get_logger().error(f"Failed to add known client {client_id}: {e}")
            return False
    
    def get_client_data(self, perception):
        """
        Extract client ID and preference from perception data.
        
        :param perception: The perception message
        :return: Tuple of (Client ID value (rounded and formatted as string), Client preference)
        """
        try:
            # Para info
            self.get_logger().debug(f"Extracting client data from: {type(perception)}")
            perception_dict = perception_msg_to_dict(perception)
            self.get_logger().debug(f"Perception dict: {perception_dict}")

            # Buscar datos del cliente
            if 'client' in perception_dict and perception_dict['client']:
                client_list = perception_dict['client']
                if len(client_list) > 0:
                    client = client_list[0]
                    client_id = client.get('id', 0.0)
                    client_preference = client.get('preference', 0.5)  # Default preference value

                    self.get_logger().debug(f"Found client ID: {client_id}, preference: {client_preference}")
                    
                    # Round client ID to 2 decimal places to match PNode behavior
                    client_id_rounded = round(client_id, 2)
                    client_id_str = str(client_id_rounded).replace('.', '_')
                    
                    self.get_logger().debug(f"Rounded client ID: {client_id_rounded} -> {client_id_str}")
                    return client_id_str, client_preference
        
            self.get_logger().info("No client found in perception")
            return "0_0", 0.5  # Return default preference value
            
        except Exception as e:
            self.get_logger().error(f"Error extracting client data: {e}")
            return "0_0", 0.5  # Return default preference value