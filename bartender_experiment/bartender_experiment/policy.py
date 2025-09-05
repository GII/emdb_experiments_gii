from cognitive_nodes.policy import Policy
from core.utils import perception_dict_to_msg, class_from_classname, perception_msg_to_dict
from core.service_client import ServiceClientAsync

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

    async def execute_callback(self, request, response):
        """
        Makes a service call to the server that handles the execution of the policy.
        Creates a client world model if needed and marks the client as known.
        """
        self.get_logger().info('Executing bartender policy: ' + self.name + '...')
    
        try:
            # Extract client ID from perception
            client_id = self.get_client_id(request.perception)
            
            if client_id is not None and client_id != "0_0":
                client_name = f"client_{client_id}"
                
                # Check if world model already exists
                expected_topic = f"/cognitive_node/{client_name}/activation"
                topic_names_and_types = self.get_topic_names_and_types()
                existing_topics = [topic_name for topic_name, _ in topic_names_and_types]
                
                if expected_topic not in existing_topics:
                    self.get_logger().info(f"Creating world model for client: {client_name}")
                    
                    # Create the world model node
                    success = await self.create_node_client(
                        name=client_name, 
                        class_name="bartender_experiment.world_model.SimBartender"
                    )
                    
                    if success:
                        self.get_logger().info(f"World model {client_name} created successfully")
                    else:
                        self.get_logger().error(f"Failed to create world model {client_name}")
                else:
                    self.get_logger().info(f"World model {client_name} already exists")
                
                # Add client to known_clients list regardless of world model creation
                # Convert client_id back to float for storage
                client_id_float = float(client_id.replace('_', '.'))
                success = await self.add_known_client(client_id_float)
                
                if success:
                    self.get_logger().info(f"Client {client_id_float} added to known clients")
                else:
                    self.get_logger().warn(f"Failed to add client {client_id_float} to known clients")
                    
            else:
                self.get_logger().warn("Could not extract valid client ID from perception")
                
        except Exception as e:
            self.get_logger().error(f"Error processing client: {e}")
        
        response.policy = self.name
        return response
    
    async def add_known_client(self, client_id):
        """
        Add a client to the known clients list using the know_client service.
        
        :param client_id: The ID of the client to add (as float)
        :return: True if successful, False otherwise
        """
        try:
            from bartender_experiment_interfaces.srv import KnowClient
            
            # Create the service client if it doesn't exist
            service_name = "/world_model/BARTENDER/know_client"  # Adjust based on your YAML configuration
            if self.know_client_service is None:
                self.know_client_service = ServiceClientAsync(
                    self, KnowClient, service_name, self.cbgroup_client
                )
            
            # Wait for service to be available
            if not await self.know_client_service.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"Service {service_name} not available")
                return False
            
            # Send the request
            request = KnowClient.Request()
            request.client_id = client_id
            
            response = await self.know_client_service.send_request_async(request)
            
            self.get_logger().info(f"Add known client result: {response.success}")
            return response.success
            
        except Exception as e:
            self.get_logger().error(f"Failed to add known client {client_id}: {e}")
            return False
    
    def get_client_id(self, perception):
        """
        Extract client ID from perception data.
        
        :param perception: The perception message
        :return: Client ID value
        """
        try:
            # Para debug
            self.get_logger().debug(f"Extracting client ID from: {type(perception)}")
            perception_dict = perception_msg_to_dict(perception)
            self.get_logger().debug(f"Perception dict: {perception_dict}")

            # Buscar datos del cliente
            if 'client' in perception_dict and perception_dict['client']:
                client_list = perception_dict['client']
                if len(client_list) > 0:
                    client = client_list[0]
                    client_id = client.get('id', 0.0)
                    self.get_logger().debug(f"Found client ID: {client_id}")
                    client_id = str(client_id).replace('.', '_')
                    return client_id
            
            self.get_logger().debug("No client found in perception")
            return "0_0"
            
        except Exception as e:
            self.get_logger().error(f"Error extracting client ID: {e}")
            return "0_0"