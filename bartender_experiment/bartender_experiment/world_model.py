#TODO: Clean imports, remove unused ones
import rclpy
from copy import deepcopy
import inspect
from cognitive_nodes.generic_model import GenericModel, Learner
from simulators.scenarios_2D import SimpleScenario, EntityType
from cognitive_node_interfaces.msg import Perception, Actuation
from core.utils import actuation_dict_to_msg, actuation_msg_to_dict, perception_dict_to_msg, perception_msg_to_dict
from rclpy.impl.rcutils_logger import RcutilsLogger
from cognitive_node_interfaces.msg import Perception, PerceptionStamped, SuccessRate
from rclpy.time import Time
from cognitive_nodes.world_model import WorldModel
from bartender_experiment_interfaces.srv import KnowClient
from std_msgs.msg import Float32

class BarEmpty(WorldModel):
    """BarEmpty class: A fixed world model of a bartender simulation that activates when no client is present."""
    def __init__(self, name='world_model', actuation_config=None, perception_config=None, class_name='cognitive_nodes.world_model.WorldModel', **params):
        """
        Constructor of the BarEmpty class.

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
        if not hasattr(self, 'ltm') or self.ltm is None:
            self.ltm = {}

        if 'known_clients' not in self.ltm:
            self.ltm['known_clients'] = set()
        
        self.set_activation_service = self.create_service(
            KnowClient,
            "world_model/" + str(name) + '/know_client',
            self.know_client_callback,
            callback_group=self.cbgroup_server
        )

    
    def create_activation_input(self, node: dict):
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
            timestamp = self.get_clock().now()
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
        self.get_logger().debug(f'Adding known client with ID {client_id} to memory...')
        
        if 'known_clients' not in self.ltm:
            self.ltm['known_clients'] = set()
        
        client_id = round(client_id, 2)  # Round to 2 decimal places to avoid long strings
        client_id = f"client_0_{int(round(client_id * 100))}"
        self.ltm['known_clients'].add(client_id)
        self.get_logger().debug(f'Known clients in memory: {self.ltm["known_clients"]}')
        
        response.success = True
        return response
    
    def calculate_activation(self, perception = None, activation_list=None):
        """
        Returns the activation value of the Model.
        This world model activates when NO client is present OR when the current client is unknown.
        Deactivates when the current client in perception matches a known client.

        :param perception: Perception that influences the activation.
        :type perception: dict
        :param activation_list: List of activation values from other sources, defaults to None.
        :type activation_list: list
        :return: The activation of the instance and its timestamp.
        :rtype: cognitive_node_interfaces.msg.Activation
        """
        if activation_list is not None:
            perception = {}
            for sensor in activation_list:
                activation_list[sensor]['updated'] = False
                perception[sensor] = activation_list[sensor]['data']
            self.get_logger().debug(f'BarEmpty: perception after activation_list processing: {perception}')
        
        activation_value = 1.0
        
        if not hasattr(self, 'ltm') or self.ltm is None:
            self.ltm = {}
            self.get_logger().warn(f'BarEmpty WARN: LTM was not initialized, creating empty LTM')
    
        if 'known_clients' not in self.ltm:
            self.ltm['known_clients'] = set()
            self.get_logger().debug(f'BarEmpty: Initializing empty known_clients set')
        
        known_clients = self.ltm.get('known_clients', set())
        self.get_logger().debug(f'BarEmpty: Known clients in memory: {known_clients}')
        
        if perception:
    
            client_data = perception.get('client', [])
            
            if client_data and len(client_data) > 0:
                client = client_data[0]
                client_id = client.get('id', None)
                
                self.get_logger().debug(f'BarEmpty: Current client ID: {client_id}')
                
                if client_id is not None and client_id > 0:
                    # Round the ID to 2 decimal places and format as string
                    client_id_rounded = round(client_id, 2)
                    client_id_formatted = f"client_0_{int(round(client_id_rounded * 100))}"
                    
                    self.get_logger().debug(f'BarEmpty: Formatted client ID: {client_id_formatted}')
                    
                   
                    if client_id_formatted in known_clients:
                       
                        activation_value = 0.0
                        self.get_logger().debug(f'BarEmpty: Current client {client_id_formatted} is KNOWN, DEACTIVATING')
                    else:
                       
                        activation_value = 1.0
                        self.get_logger().debug(f'BarEmpty: Current client {client_id_formatted} is UNKNOWN, ACTIVATING')
                else:
                    # No valid client present (ID 0 or None)
                    activation_value = 1.0
                    if client_id == 0:
                        self.get_logger().debug(f'BarEmpty: No client present (ID 0), staying ACTIVE')
                    else:
                        self.get_logger().debug(f'BarEmpty: Invalid client ID, staying ACTIVE')
            else:
                
                activation_value = 1.0
                self.get_logger().debug(f'BarEmpty: No client data found in perception, staying ACTIVE')
        else:
        
            activation_value = 1.0
            self.get_logger().debug(f'BarEmpty: No perception available, staying ACTIVE')
        
        self.get_logger().debug(f'BarEmpty: Final activation value: {activation_value}')
        self.activation.activation = activation_value
        self.activation.timestamp = self.get_clock().now().to_msg()
        return self.activation

class ClientInBar(WorldModel):
    """ClientInBar class: A fixed world model of a bartender simulation."""
    def __init__(self, name='world_model', actuation_config=None, perception_config=None, class_name='cognitive_nodes.world_model.WorldModel', preference=None, **params):
        """
        Constructor of the ClientInBar class.

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
        self.preference_timer = self.create_timer(0.1, self.log_preference)
        self.publish_last_bottle = self.create_publisher(
            Float32,
            'cognitive_node/world_model/last_bottle',
            0
        )

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
            self.get_logger().debug(f'World Model: perception after activation_list processing: {perception}')
        
        if perception:
            activation_value = 0.0      
            client_data = None
            client_id = None
            
            for key, value in perception.items():
                if 'client' in key.lower() and value:
                    client_data = value
                    self.get_logger().debug(f'World Model: Found client data: {client_data}')
                    break
            
            if client_data:
                if isinstance(client_data, list) and len(client_data) > 0:
                    client = client_data[0]  
                    client_id = client.get('id', None)
                    client_id = round(client_id, 2)  # Round to 2 decimal places to match naming convention
                    
                    
                    self.get_logger().debug(f'World Model: Client ID: {client_id}')
            
            
            if client_id is not None:
                expected_name = f"client_{client_id}"
                expected_name = expected_name.replace('.', '_')  # Replace dot with underscore for naming
                self.get_logger().debug(f'World Model: Expected name: {expected_name}, Actual name: {self.name}')
                
                if self.name == expected_name:
                    activation_value = 1.0
                    self.get_logger().debug(f'World Model: {self.name} matches client ID {client_id}, activating')
                else:
                    self.get_logger().debug(f'World Model: {self.name} does not match client ID {client_id}')
            else:
                self.get_logger().debug(f'World Model: No client ID found in perception')
            
            self.activation.activation = activation_value
            self.get_logger().debug(f'World Model: Final activation value: {activation_value}')
        
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
        self.get_logger().debug('Setting activation ' + str(activation) + '...')
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
    
    def log_preference(self):
        """
        Timer callback to log the current preference value.
        """
        # Publish the last bottle preference as a Float32 message
        if self.preference is not None and self.activation.activation > 0.0:
            msg = Float32()
            msg.data = float(self.preference)
            self.publish_last_bottle.publish(msg)
            self.get_logger().debug(f'World Model {self.name}: Published last bottle preference = {self.preference}')


