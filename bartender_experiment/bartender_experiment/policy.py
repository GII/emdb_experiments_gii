from cognitive_nodes.policy import Policy
from core.utils import perception_dict_to_msg, class_from_classname
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
        super().__init__(name, class_name, service_msg, service_name, **params)
        self.service_msg=service_msg
        self.service_name=service_name
        self.policy_service=ServiceClientAsync(self, class_from_classname(service_msg), service_name, self.cbgroup_client)

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
        # Debug: Imprimir la estructura completa del objeto perception
        self.get_logger().info(f"Perception type: {type(request.perception)}")
        self.get_logger().info(f"Perception attributes: {dir(request.perception)}")
        self.get_logger().info(f"Perception content: {request.perception}")
        await self.policy_service.send_request_async(policy=self.name)
        response.policy = self.name
        return response