from cognitive_nodes.policy import Policy, PolicyBlocking
from core.service_client import ServiceClientAsync
from core.container import Container
from bartender_experiment.world_model import _latest_value

from bartender_experiment_interfaces.srv import KnowClient



class BartenderClientPolicy(Policy):
    """
    BartenderClientPolicy class. Represents the policy that registers a new client in the bartender experiment.
    It does not call a server to execute anything: it creates the World Model node of the client and registers the client in the BARTENDER World Model.
    """    
    def __init__(self, name='BartenderClientPolicy', class_name='cognitive_nodes.policy.Policy', **params):
        """
        Constructor for the BartenderClientPolicy class.

        :param name: The name of the policy.
        :type name: str
        :param class_name: The name of the base Policy class.
        :type class_name: str
        """        
        super().__init__(name=name, class_name=class_name, **params)
        self.know_client_service = None

    async def execute_callback(self, request, response):
        """
        Executes the policy. Reads the client id and preference from the perception. If the client is valid
        (id different from ``0_0``) and its World Model node (``client_<id>``) does not exist yet, creates it with the
        class ``bartender_experiment.world_model.ClientInBar`` and the preference of the client, and registers the
        client in the BARTENDER World Model with the ``know_client`` service. No server is called to execute the policy.

        :param request: The request to execute the policy, with the current perception.
        :type request: cognitive_node_interfaces.srv.Execute.Request
        :param response: The response indicating the executed policy.
        :type response: cognitive_node_interfaces.srv.Execute.Response
        :return: The response with the executed policy name.
        :rtype: cognitive_node_interfaces.srv.Execute.Response
        """
        try:
            # Extract client ID and preference from perception
            perception = Container.from_msg(request.perception)
            client_id, client_preference = self.get_client_data(perception)
            
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
        Add a client to the known clients list of the BARTENDER World Model using its ``know_client`` service.

        :param client_id: Id of the client (rounded to two decimals).
        :type client_id: float
        :return: True if the service call succeeded, False otherwise.
        :rtype: bool
        """
        try:
            service_name = "/world_model/BARTENDER/know_client"
            # The client is created once: a new client in every execution means a new ROS client, which has to
            # discover the service again
            if service_name not in self.node_clients:
                self.node_clients[service_name] = ServiceClientAsync(self, KnowClient, service_name, self.cbgroup_client)
            response = await self.node_clients[service_name].send_request_async(client_id=client_id)
            return response.success if response is not None else False
        except Exception as e:
            self.get_logger().error(f"Failed to add known client {client_id}: {e}")
            return False
    
    def get_client_data(self, perception):
        """
        Extract client ID and preference from perception data.
        
        :param perception: The current perception, with the features ``client:id`` and ``client:preference``.
        :type perception: core.container.Container
        :return: Tuple of (Client ID value (rounded to two decimals, with the dot replaced by an underscore, as a string), Client preference). If the data cannot be read, ``("0_0", 0.5)``.
        :rtype: tuple
        """
        try:
            client_preference = _latest_value(perception, "client:preference")
            client_id = _latest_value(perception, "client:id")
            client_id_rounded = round(client_id, 2)
            client_id_str = str(client_id_rounded).replace('.', '_')
            return client_id_str, client_preference
            
        except Exception as e:
            self.get_logger().error(f"Error extracting client data: {e}")
            return "0_0", 0.5

class BartenderClientNoWorldModelPolicy(Policy):
    """
    BartenderClientNoWorldModelPolicy class. Similar to BartenderClientPolicy but does not create a World Model node for the client.
    The client and its preference are stored in the BARTENDER World Model and in two P-Nodes instead.
    """    
    def __init__(self, name='BartenderClientPolicy', class_name='cognitive_nodes.policy.Policy', **params):
        """
        Constructor for the BartenderClientNoWorldModelPolicy class.

        :param name: The name of the policy.
        :type name: str
        :param class_name: The name of the base Policy class.
        :type class_name: str
        """        
        super().__init__(name=name, class_name=class_name, **params)
        self.know_client_service = None

    async def execute_callback(self, request, response):
        """
        Executes the policy. Reads the client id and preference from the perception and, if they can be read,
        registers the client with the ``know_client`` services (see :meth:`add_known_client`). No World Model node is
        created for the client and no server is called to execute the policy.

        :param request: The request to execute the policy, with the current perception.
        :type request: cognitive_node_interfaces.srv.Execute.Request
        :param response: The response indicating the executed policy.
        :type response: cognitive_node_interfaces.srv.Execute.Response
        :return: The response with the executed policy name.
        :rtype: cognitive_node_interfaces.srv.Execute.Response
        """
        try:
            # Extract client ID and preference from perception
            perception = Container.from_msg(request.perception)
            client_id, client_preference = self.get_client_data(perception)
            
            if client_id is not None:
                # Add client to known_clients list
                await self.add_known_client(client_id, client_preference)
                
        except Exception as e:
            self.get_logger().error(f"Exception in execute_callback: {e}")
    
        response.policy = self.name
        return response
    
    async def add_known_client(self, client_id, client_preference):
        """
        Add a client and its preference using the ``know_client`` service of three nodes: the BARTENDER World Model
        and the P-Nodes ``get_response_pnode`` and ``client_present_pnode``.

        :param client_id: Id of the client (rounded to two decimals).
        :type client_id: float
        :param client_preference: Preference of the client.
        :type client_preference: float
        :return: True if the three services succeeded, False otherwise.
        :rtype: bool
        """
        try:
            world_model_service = "/world_model/BARTENDER/know_client"
            pnode_response_service = "/pnode/get_response_pnode/know_client"
            pnode_ask_service = "/pnode/client_present_pnode/know_client"
            if world_model_service not in self.node_clients:
                client = ServiceClientAsync(self, KnowClient, world_model_service, self.cbgroup_client)
                self.node_clients[world_model_service] = client
            if pnode_response_service not in self.node_clients:
                client = ServiceClientAsync(self, KnowClient, pnode_response_service, self.cbgroup_client)
                self.node_clients[pnode_response_service] = client
            if pnode_ask_service not in self.node_clients:
                client = ServiceClientAsync(self, KnowClient, pnode_ask_service, self.cbgroup_client)
                self.node_clients[pnode_ask_service] = client
            response_wm = await self.node_clients[world_model_service].send_request_async(client_id=client_id, preference=client_preference)
            response_pnode = await self.node_clients[pnode_response_service].send_request_async(client_id=client_id, preference=client_preference)
            response_pnode_ask = await self.node_clients[pnode_ask_service].send_request_async(client_id=client_id, preference=client_preference)
            success = response_wm.success and response_pnode.success and response_pnode_ask.success
            return success
        
        except Exception as e:
            self.get_logger().error(f"Failed to add known client {client_id}: {e}")
            return False
    
    def get_client_data(self, perception):
        """
        Extract client ID and preference from perception data.
        
        :param perception: The current perception, with the features ``client:id`` and ``client:preference``.
        :type perception: core.container.Container
        :return: Tuple of (Client ID value (rounded to two decimals), Client preference). If the data cannot be read, ``(None, 0.5)``.
        :rtype: tuple
        """
        try:
            client_preference = _latest_value(perception, "client:preference")
            client_id = _latest_value(perception, "client:id")
            client_id_rounded = round(client_id, 2)
            return client_id_rounded, client_preference
            
        except Exception as e:
            self.get_logger().error(f"Error extracting client data: {e}")
            return None, 0.5

class PolicyPerception(PolicyBlocking):
    """
    PolicyPerception class. Represents a policy that passes the current perception to the execution service.
    """    
    def __init__(self, name='PolicyPerception', class_name='cognitive_nodes.policy.Policy', service_msg=None, service_name=None, **params):
        """
        Constructor for the PolicyPerception class.

        :param name: The name of the policy.
        :type name: str
        :param class_name: The name of the base Policy class.
        :type class_name: str
        """        
        super().__init__(name=name, class_name=class_name, service_msg=service_msg, service_name=service_name, **params)

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
        self.get_logger().info('Executing policy: ' + self.name + '...')
        await self.policy_service.send_request_async(policy=self.name, perception=request.perception)
        response.policy = self.name
        return response