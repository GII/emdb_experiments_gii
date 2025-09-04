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

        :param request: The request to execute the policy.
        :type request: cognitive_node_interfaces.srv.Execute.Request
        :param response: The response indicating the executed policy.
        :type response: cognitive_node_interfaces.srv.Execute.Response
        :return: The response with the executed policy name.
        :rtype: cognitive_node_interfaces.srv.Execute.Response
        """
        self.get_logger().info('Executing bartender policy: ' + self.name + '...')
    
        try:
            # Intentar extraer el ID del cliente
            client_id = self.get_client_id(request.perception)
            
            if client_id is not None:
                client_name = f"client_{client_id}"
                self.get_logger().info(f"Creating node client for: {client_name}")
                await self.create_node_client(name=client_name, class_name="cognitive_nodes.world_model.SimBartender")
            else:
                self.get_logger().warn("Could not extract client ID from perception")
                
        except Exception as e:
            self.get_logger().error(f"Error extracting client ID: {e}")
        
        response.policy = self.name
        return response
    
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