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
        try:
            # Extract client ID and preference from perception
            client_id, client_preference = self.get_client_data(request.perception)
            
            if client_id is not None and client_id != "0_0":
                client_name = f"client_{client_id}"
                
                # Check if world model already exists
                expected_topic = f"/cognitive_node/{client_name}/activation"
                existing_topics = [topic_name for topic_name, _ in self.get_topic_names_and_types()]
                
                if expected_topic not in existing_topics:
                    # Create the world model node with client's actual preference
                    success = await self.create_node_client(
                        name=client_name, 
                        class_name="bartender_experiment.world_model.ClientInBar",
                        parameters={"preference": client_preference}
                    )
                    
                    if success:
                        # Add client to known_clients list
                        client_id_float = float(client_id.replace('_', '.'))
                        client_id_rounded = round(client_id_float, 2)
                        await self.add_known_client(client_id_rounded)
                
        except Exception as e:
            self.get_logger().error(f"Exception in execute_callback: {e}")
    
        response.policy = self.name
        return response
    
    async def add_known_client(self, client_id):
        """
        Add a client to the known clients list using the know_client service.
        """
        try:
            service_name = "/world_model/BARTENDER/know_client"
            client = ServiceClientAsync(self, KnowClient, service_name, self.cbgroup_client)
            response = await client.send_request_async(client_id=client_id)
            return response.success if response is not None else False
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
            perception_dict = perception_msg_to_dict(perception)

            if 'client' in perception_dict and perception_dict['client']:
                client_list = perception_dict['client']
                if client_list:
                    client = client_list[0]
                    client_id = client.get('id', 0.0)
                    client_preference = client.get('preference', 0.5)
                    
                    # Round client ID to 2 decimal places to match PNode behavior
                    client_id_rounded = round(client_id, 2)
                    client_id_str = str(client_id_rounded).replace('.', '_')
                    
                    return client_id_str, client_preference
        
            return "0_0", 0.5
            
        except Exception as e:
            self.get_logger().error(f"Error extracting client data: {e}")
            return "0_0", 0.5