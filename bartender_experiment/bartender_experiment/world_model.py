import rclpy
from copy import deepcopy

from cognitive_nodes.generic_model import GenericModel, Learner
from simulators.scenarios_2D import SimpleScenario, EntityType
from cognitive_node_interfaces.msg import Perception, Actuation
from core.utils import actuation_dict_to_msg, actuation_msg_to_dict, perception_dict_to_msg, perception_msg_to_dict
from rclpy.impl.rcutils_logger import RcutilsLogger
from cognitive_node_interfaces.msg import Perception, PerceptionStamped, SuccessRate
from rclpy.time import Time
from cognitive_nodes.world_model import WorldModel
from bartender_experiment_interfaces.srv import KnowClient

class SimBartenderEmpty(WorldModel):
    """SimBartenderEmpty class: A fixed world model of a bartender simulation that activates when no client is present."""
    def __init__(self, name='world_model', actuation_config=None, perception_config=None, class_name='cognitive_nodes.world_model.WorldModel', **params):
        """
        Constructor of the SimBartenderEmpty class.

        :param name: The name of the World Model instance.
        :type name: str
        :param actuation_config: Dictionary with the parameters of the actuation.
        :type actuation_config: dict
        :param perception_config: Dictionary with the parameters of the perception.
        :type perception_config: dict
        :param class_name: Name of the base WorldModel class, defaults to 'cognitive_nodes.world_model.WorldModel'.
        :type class_name: str
        """

        super().__init__(name, class_name, **params)
        # Create a service to add know clients to the bartender's memory
        self.set_activation_service = self.create_service(
            KnowClient,
            "world_model/" + str(name) + '/know_client',
            self.know_client_callback,
            callback_group=self.cbgroup_server
        )


    def know_client_callback(self, request, response):
        """
        Service callback to add a known client to the bartender's memory.

        :param request: The request that contains the client ID to be added.
        :type request: bartender_experiment_interfaces.srv.KnowClient.Request
        :param response: The response indicating if the client was successfully added.
        :type response: bartender_experiment_interfaces.srv.KnowClient.Response
        :return: The response indicating if the client was successfully added.
        :rtype: bartender_experiment_interfaces.srv.KnowClient.Response
        """
        client_id = request.client_id
        self.get_logger().info(f'Adding known client with ID {client_id} to memory...')
        
        # Add the client to the long-term memory (LTM)
        if 'known_clients' not in self.ltm:
            self.ltm['known_clients'] = set()
        
        self.ltm['known_clients'].add(client_id)
        self.get_logger().info(f'Known clients in memory: {self.ltm["known_clients"]}')
        
        response.success = True
        return response
    
    def calculate_activation(self, perception = None, activation_list=None):
        """
        Returns the activation value of the Model.
        Activates when client is present but NOT in known_clients list.
        Deactivates when client is in known_clients list.

        :param perception: Perception that influences the activation.
        :type perception: dict
        :param activation_list: List of activation values from other sources, defaults to None.
        :type activation_list: list
        :return: The activation of the instance and its timestamp.
        :rtype: cognitive_node_interfaces.msg.Activation
        """
        # Default activation is 1.0 (active by default when no client present)
        activation_value = 1.0
        
        if activation_list != None:
            perception = {}
            for sensor in activation_list:
                activation_list[sensor]['updated'] = False
                perception[sensor] = activation_list[sensor]['data']
            self.get_logger().debug(f'SimBartenderEmpty DEBUG: perception after activation_list processing: {perception}')
        
        # Check if there's a client in perception
        if perception:
            # Search for client data in perceptions
            client_data = None
            client_id = None
            
            # Check if there is client data in perceptions
            for key, value in perception.items():
                self.get_logger().debug(f'SimBartenderEmpty DEBUG: Checking key: {key}, value type: {type(value)}, value: {value}')
                if 'client' in key.lower() and value:
                    client_data = value
                    self.get_logger().debug(f'SimBartenderEmpty DEBUG: Found client data in key: {key}')
                    break
            
            # If there's client data, check if client is known
            if client_data:
                # Extract client ID from different data formats
                if isinstance(client_data, list) and len(client_data) > 0:
                    client = client_data[0]
                    if isinstance(client, dict) and client.get('id') is not None:
                        client_id = client.get('id')
                    elif hasattr(client, 'id') and client.id is not None:
                        client_id = client.id
                elif hasattr(client_data, 'id') and client_data.id is not None:
                    client_id = client_data.id
                elif isinstance(client_data, dict) and client_data.get('id') is not None:
                    client_id = client_data.get('id')
                
                self.get_logger().debug(f'SimBartenderEmpty DEBUG: Extracted client ID: {client_id}')
                
                if client_id is not None:
                    # Check if client is in known_clients list
                    known_clients = self.ltm.get('known_clients', set())
                    self.get_logger().debug(f'SimBartenderEmpty DEBUG: Known clients: {known_clients}')
                    
                    if client_id in known_clients:
                        # Client is known, DEACTIVATE
                        activation_value = 0.0
                        self.get_logger().info(f'SimBartenderEmpty: Client {client_id} is KNOWN, DEACTIVATING')
                    else:
                        # Client is unknown, ACTIVATE
                        activation_value = 1.0
                        self.get_logger().info(f'SimBartenderEmpty: Client {client_id} is UNKNOWN, ACTIVATING')
                else:
                    # No valid client ID found, stay active (default behavior)
                    activation_value = 1.0
                    self.get_logger().debug(f'SimBartenderEmpty DEBUG: No valid client ID found, staying ACTIVE')
            else:
                # No client data found in perception, stay active
                activation_value = 1.0
                self.get_logger().debug(f'SimBartenderEmpty DEBUG: No client data found in perception, staying ACTIVE')
        
        self.activation.activation = activation_value
        self.activation.timestamp = self.get_clock().now().to_msg()
        
        self.get_logger().debug(f'SimBartenderEmpty DEBUG: Final activation value: {activation_value}')
        return self.activation

class SimBartender(WorldModel):
    """SimBartender class: A fixed world model of a bartender simulation."""
    def __init__(self, name='world_model', actuation_config=None, perception_config=None, class_name='cognitive_nodes.world_model.WorldModel', preference=None, **params):
        """
        Constructor of the SimBartender class.

        :param name: The name of the World Model instance.
        :type name: str
        :param actuation_config: Dictionary with the parameters of the actuation.
        :type actuation_config: dict
        :param perception_config: Dictionary with the parameters of the perception.
        :type perception_config: dict
        :param class_name: Name of the base WorldModel class, defaults to 'cognitive_nodes.world_model.WorldModel'.
        :type class_name: str
        """
        self.preference=preference
        super().__init__(name, class_name, **params)
    
    def calculate_activation(self, perception = None, activation_list=None):
        """
        Returns the activation value of the Model.
        Activates when the world model name matches client_{id} pattern.

        :param perception: Perception that influences the activation.
        :type perception: dict
        :param activation_list: List of activation values from other sources, defaults to None.
        :type activation_list: list
        :return: The activation of the instance and its timestamp.
        :rtype: cognitive_node_interfaces.msg.Activation
        """
        self.activation.activation = 0.0
        
        if activation_list != None:
            perception = {}
            for sensor in activation_list:
                activation_list[sensor]['updated'] = False
                perception[sensor] = activation_list[sensor]['data']
            self.get_logger().debug(f'World Model DEBUG: perception after activation_list processing: {perception}')
        
        if perception:
            activation_value = 0.0
            
            # Search for client data in perceptions
            client_data = None
            client_id = None
            
            # Check if there is client data in perceptions
            for key, value in perception.items():
                self.get_logger().debug(f'World Model DEBUG: Checking key: {key}, value type: {type(value)}, value: {value}')
                if 'client' in key.lower() and value:
                    client_data = value
                    self.get_logger().debug(f'World Model DEBUG: Found client data in key: {key}')
                    break
            
            self.get_logger().debug(f'World Model DEBUG: Client data: {client_data}')
            
            if client_data:
                # If client_data is a list of clients
                if isinstance(client_data, list) and len(client_data) > 0:
                    client = client_data[0]  # Take the first client
                    self.get_logger().debug(f'World Model DEBUG: Client is list, first client: {client}')
                    self.get_logger().debug(f'World Model DEBUG: Client type: {type(client)}')
                    
                    # The client is a dictionary, access it as such
                    if isinstance(client, dict):
                        client_id = client.get('id', None)
                        self.get_logger().debug(f'World Model DEBUG: Client ID from dict: {client_id}')
                    
                    # If the client has attributes (ROS message object)
                    elif hasattr(client, 'id'):
                        client_id = client.id
                        self.get_logger().debug(f'World Model DEBUG: Client ID from attribute: {client_id}')
                
                # If client_data is a direct object with ID
                elif hasattr(client_data, 'id'):
                    client_id = client_data.id
                    self.get_logger().debug(f'World Model DEBUG: Client ID from direct object: {client_id}')
                
                # If client_data is a dictionary
                elif isinstance(client_data, dict):
                    client_id = client_data.get('id', None)
                    self.get_logger().debug(f'World Model DEBUG: Client ID from direct dict: {client_id}')
            
            # Check if world model name matches client_{id} pattern
            if client_id is not None:
                expected_name = f"client_{int(client_id)}"
                self.get_logger().debug(f'World Model DEBUG: Expected name: {expected_name}, Actual name: {self.name}')
                
                if self.name == expected_name:
                    activation_value = 1.0
                    self.get_logger().info(f'World Model: {self.name} matches client ID {client_id}, activating')
                else:
                    self.get_logger().debug(f'World Model DEBUG: {self.name} does not match client ID {client_id}')
            else:
                self.get_logger().debug(f'World Model DEBUG: No client ID found in perception')
            
            self.activation.activation = activation_value
            self.get_logger().debug(f'World Model DEBUG: Final activation value: {activation_value}')
        
        self.activation.timestamp = self.get_clock().now().to_msg()
        return self.activation
    
    def set_activation_callback(self, request, response):
        """
        Some processes can modify the activation of a Model.

        :param request: The request that contains the new activation value.
        :type request: cognitive_node_interfaces.srv.SetActivation.Request
        :param response: The response indicating if the activation was set.
        :type response: cognitive_node_interfaces.srv.SetActivation.Response
        :return: The response indicating if the activation was set.
        :rtype: cognitive_node_interfaces.srv.SetActivation.Response
        """
        activation = request.activation
        self.get_logger().info('Setting activation ' + str(activation) + '...')
        self.activation.activation = activation
        self.activation.timestamp = self.get_clock().now().to_msg()
        response.set = True
        return response
    
    def create_activation_input(self, node: dict): #Adds or deletes a node from the activation inputs list. By default reads activations.
        """
        Adds perceptions to the activation inputs list.

        :param node: Dictionary with the information of the node {'name': <name>, 'node_type': <node_type>}.
        :type node: dict
        """    
        name=node['name']
        node_type=node['node_type']
        if node_type == "Perception":
            subscriber=self.create_subscription(PerceptionStamped, "perception/" + str(name) + "/value", self.read_activation_callback, 1, callback_group=self.cbgroup_activation)
            data=Perception()
            updated=False
            timestamp=Time()
            new_input=dict(subscriber=subscriber, data=data, updated=updated, timestamp=timestamp)
            self.activation_inputs[name]=new_input
            self.get_logger().debug(f'{self.name} -- Created new activation input: {name} of type {node_type}')


    def read_activation_callback(self, msg: PerceptionStamped):
        """
        Callback method that reads a perception and stores it in the activation inputs list.

        :param msg: PerceptionStamped message that contains the perception and its timestamp.
        :type msg: cognitive_node_interfaces.msg.PerceptionStamped
        """        
        perception_dict=perception_msg_to_dict(msg=msg.perception)
        if len(perception_dict)>1:
            self.get_logger().error(f'{self.name} -- Received perception with multiple sensors: ({perception_dict.keys()}). Perception nodes should (currently) include only one sensor!')
        if len(perception_dict)==1:
            node_name=list(perception_dict.keys())[0]
            if node_name in self.activation_inputs:
                self.activation_inputs[node_name]['data']=perception_dict[node_name]
                self.activation_inputs[node_name]['updated']=True
                self.activation_inputs[node_name]['timestamp']=Time.from_msg(msg.timestamp)
        else:
            self.get_logger().warn("Empty perception recieved in P-Node. No activation calculated")


