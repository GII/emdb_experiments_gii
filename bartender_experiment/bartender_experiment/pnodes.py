import math

from cognitive_nodes.pnode import PNode
from cognitive_nodes.perception import perception_dict_to_msg
from core.utils import separate_perceptions


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
        # DEBUG: Initial log
        self.get_logger().debug(f'PNODE DEBUG: calculate_activation called')
        self.get_logger().debug(f'PNODE DEBUG: activation_list is None: {activation_list is None}')
        self.get_logger().debug(f'PNODE DEBUG: perception: {perception}')
        
        if activation_list != None:
            perception = {}
            for sensor in activation_list:
                activation_list[sensor]['updated'] = False
                perception[sensor] = activation_list[sensor]['data']
            self.get_logger().debug(f'PNODE DEBUG: perception after activation_list processing: {perception}')

        if perception:
            self.get_logger().debug(f'PNODE DEBUG: Processing perception keys: {perception.keys()}')
            
            # Search for client data in perceptions
            client_data = None
            
            # Check if there is client data in perceptions
            for key, value in perception.items():
                self.get_logger().debug(f'PNODE DEBUG: Checking key: {key}, value type: {type(value)}, value: {value}')
                if 'client' in key.lower() and value:
                    client_data = value
                    self.get_logger().debug(f'PNODE DEBUG: Found client data in key: {key}')
                    break
            
            self.get_logger().debug(f'PNODE DEBUG: Client data: {client_data}')
            self.get_logger().debug(f'PNODE DEBUG: Client data type: {type(client_data)}')
            
            activation_value = 0.0
            
            if client_data:
                # If client_data is a list of clients
                if isinstance(client_data, list) and len(client_data) > 0:
                    client = client_data[0]  # Take the first client
                    self.get_logger().debug(f'PNODE DEBUG: Client is list, first client: {client}')
                    self.get_logger().debug(f'PNODE DEBUG: Client type: {type(client)}')
                    
                    # The client is a dictionary, access it as such
                    if isinstance(client, dict):
                        preference = client.get('preference', 0.0)
                        self.get_logger().debug(f'PNODE DEBUG: Client preference from dict: {preference}')
                        if preference != 0.0:
                            activation_value = 1.0
                            self.get_logger().info(f'PNODE: Client has preference {preference}, activating')
                        else:
                            self.get_logger().debug(f'PNODE DEBUG: Client preference is 0.0')
                    
                    # If the client has attributes (ROS message object)
                    elif hasattr(client, 'preference'):
                        self.get_logger().debug(f'PNODE DEBUG: Client preference value: {client.preference}')
                        if client.preference != 0.0:
                            activation_value = 1.0
                            self.get_logger().info(f'PNODE: Client has preference {client.preference}, activating')
                        else:
                            self.get_logger().debug(f'PNODE DEBUG: Client preference is 0.0 or not set')
                    else:
                        self.get_logger().debug(f'PNODE DEBUG: Client format not recognized')
                
                # If client_data is a direct object
                elif hasattr(client_data, 'preference'):
                    self.get_logger().debug(f'PNODE DEBUG: Client data has preference attribute: {client_data.preference}')
                    if client_data.preference != 0.0:
                        activation_value = 1.0
                        self.get_logger().info(f'PNODE: Client has preference {client_data.preference}, activating')
                    else:
                        self.get_logger().debug(f'PNODE DEBUG: Client preference is 0.0')
                
                # If client_data is a dictionary
                elif isinstance(client_data, dict):
                    self.get_logger().debug(f'PNODE DEBUG: Client data is dict: {client_data}')
                    preference = client_data.get('preference', 0.0)
                    self.get_logger().debug(f'PNODE DEBUG: Dict preference: {preference}')
                    if preference != 0.0:
                        activation_value = 1.0
                        self.get_logger().info(f'PNODE: Client has preference {preference}, activating')
                    else:
                        self.get_logger().debug(f'PNODE DEBUG: Client preference is 0.0')
                else:
                    self.get_logger().debug(f'PNODE DEBUG: Client data format not recognized')
            else:
                self.get_logger().debug(f'PNODE DEBUG: No client data found')
            
            self.get_logger().debug(f'PNODE DEBUG: Final activation value: {activation_value}')
            self.activation.activation = activation_value
            self.activation.timestamp = self.get_clock().now().to_msg()
        else:
            self.get_logger().debug(f'PNODE DEBUG: No perception provided')
        
        self.get_logger().debug(f'PNODE DEBUG: Returning activation: {self.activation.activation}')
        return self.activation
    

class PNodeClientPresent(PNode):
    """
    PNode that represents that a client is present
    """
    def __init__(self, name='client_present', class_name='cognitive_nodes.pnode.PNode', space_class=None, space=None, history_size=100, invert=False, **params):
        super().__init__(name, class_name, space_class, space, history_size, **params)
        self.invert = invert


    def calculate_activation(self, perception=None, activation_list=None):
        """
        Calculate the new activation value for a given perception.
        Activates when a client is present but NO world model exists for that client yet.

        :param perception: The perception for which PNode activation is calculated.
        :type perception: dict
        :return: If there is space, returns the activation of the PNode. If not, returns 0
        :rtype: float
        """
        if activation_list != None:
            perception = {}
            for sensor in activation_list:
                activation_list[sensor]['updated'] = False
                perception[sensor] = activation_list[sensor]['data']

        if perception:
            activation_value = 0.0
            
            # Search for client data in perceptions
            client_data = None
            client_id = None
            
            # Check if there is client data in perceptions
            for key, value in perception.items():
                self.get_logger().debug(f'PNodeClientPresent DEBUG: Checking key: {key}, value type: {type(value)}, value: {value}')
                if 'client' in key.lower() and value:
                    client_data = value
                    self.get_logger().debug(f'PNodeClientPresent DEBUG: Found client data in key: {key}')
                    break
            
            self.get_logger().debug(f'PNodeClientPresent DEBUG: Client data: {client_data}')
            
            if client_data:
                # If client_data is a list of clients
                if isinstance(client_data, list) and len(client_data) > 0:
                    client = client_data[0]  # Take the first client
                    self.get_logger().debug(f'PNodeClientPresent DEBUG: Client is list, first client: {client}')
                    
                    # The client is a dictionary, access it as such
                    if isinstance(client, dict):
                        client_id = client.get('id', None)
                        self.get_logger().debug(f'PNodeClientPresent DEBUG: Client ID from dict: {client_id}')
                    
                    # If the client has attributes (ROS message object)
                    elif hasattr(client, 'id'):
                        client_id = client.id
                        self.get_logger().debug(f'PNodeClientPresent DEBUG: Client ID from attribute: {client_id}')
                
                # If client_data is a direct object with ID
                elif hasattr(client_data, 'id'):
                    client_id = client_data.id
                    self.get_logger().debug(f'PNodeClientPresent DEBUG: Client ID from direct object: {client_id}')
                
                # If client_data is a dictionary
                elif isinstance(client_data, dict):
                    client_id = client_data.get('id', None)
                    self.get_logger().debug(f'PNodeClientPresent DEBUG: Client ID from direct dict: {client_id}')
            
            # Check if client world model already exists
            if client_id is not None:
                # Convert float ID to string format (e.g., 0.5 -> "0_5")
                client_id_str = str(client_id).replace('.', '_')
                expected_topic = f"/cognitive_node/client_{client_id_str}/activation"
                
                self.get_logger().debug(f'PNodeClientPresent DEBUG: Looking for topic: {expected_topic}')
                
                # Check if the topic exists in the ROS2 system
                topic_names_and_types = self.get_topic_names_and_types()
                existing_topics = [topic_name for topic_name, _ in topic_names_and_types]
                
                self.get_logger().debug(f'PNodeClientPresent DEBUG: Existing topics: {existing_topics}')
                
                if expected_topic in existing_topics:
                    # World model already exists, DEACTIVATE
                    activation_value = 0.0 if not self.invert else 0.95
                    self.get_logger().info(f'PNodeClientPresent: Client world model {expected_topic} already exists, DEACTIVATING')
                else:
                    # World model does not exist, ACTIVATE
                    activation_value = 1.0 if not self.invert else 0.0
                    self.get_logger().info(f'PNodeClientPresent: Client world model {expected_topic} does not exist, ACTIVATING')
            else:
                # No client ID found, treat as no client present
                activation_value = 0.0 if not self.invert else 0.95
                self.get_logger().debug(f'PNodeClientPresent DEBUG: No client ID found in perception')
        
            self.activation.activation = activation_value
            self.get_logger().debug(f'PNodeClientPresent DEBUG: Final activation value: {activation_value}')
        else:
            # No perception provided
            self.activation.activation = 0.0 if not self.invert else 0.95
            self.get_logger().debug(f'PNodeClientPresent DEBUG: No perception provided')
    
        self.activation.timestamp = self.get_clock().now().to_msg()
        return self.activation