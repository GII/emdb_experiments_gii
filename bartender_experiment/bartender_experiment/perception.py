from cognitive_nodes.perception import Perception
from math import isclose
from std_msgs.msg import Float32

from core.cognitive_node import CognitiveNode
from cognitive_node_interfaces.srv import SetActivation, SetInputs
from cognitive_node_interfaces.msg import PerceptionStamped
from core.utils import class_from_classname, perception_dict_to_msg

class BartenderPerception(Perception):
    """Bartender Perception class"""
    # Shared class variable to store the last_bottle_id
    _last_bottle_id = None
    
    def __init__(self, name='perception', class_name = 'cognitive_nodes.perception.Perception', default_msg = None, default_topic = None, normalize_data = None, **params):
        """
        Constructor for the Perception class.
        Initializes a Perception instance with the given name and registers it in the LTM.
        
        :param name: The name of the Perception instance.
        :type name: str
        :param class_name: The name of the Perception class.
        :type class_name: str
        :param default_msg: The msg of the default subscription.
        :type default_msg: str
        :param default_topic: The topic of the default subscription.
        :type default_topic: str
        :param normalize_data: Values in order to normalize values.
        :type normalize_data: dict
        """
        super().__init__(name, class_name, default_msg, default_topic, normalize_data, **params)

    def process_and_send_reading(self):
        """
        Method that processes the sensor values received.
        """
        sensor = {}
        value = []
        
        # Handle last_bottle separately since it might be an int instead of a list
        if "last_bottle" in self.name:
            if isinstance(self.reading.data, list):
                for perception in self.reading.data:
                    raw_data = perception.data
                    self.get_logger().debug(f"Processing last_bottle from list: raw_data = {raw_data}")
                    
                    # Store the raw ID in the class variable for bottle filtering
                    BartenderPerception._last_bottle_id = raw_data
                    self.get_logger().debug(f"Updated shared last_bottle_id to: {raw_data}")
                    
                    data = perception.data/(self.normalize_values["n_ids"] - 1) # Normalize 0,1,2 states between 0 and 1
                    data = 0.98 if isclose(data, 1.0) else data
                    
                    self.get_logger().debug(f"Normalized last_bottle: {raw_data} -> {data}")
                    
                    value.append(
                        dict(
                            data=data
                        )
                    )
            else:
                # Handle case where last_bottle is a direct int value
                raw_data = self.reading.data
                self.get_logger().debug(f"Processing last_bottle as int: raw_data = {raw_data}")
                
                # Store the raw ID in the class variable for bottle filtering
                BartenderPerception._last_bottle_id = raw_data
                self.get_logger().debug(f"Updated shared last_bottle_id to: {raw_data}")
                
                data = self.reading.data/(self.normalize_values["n_ids"] - 1) # Normalize 0,1,2 states between 0 and 1
                data = 0.98 if isclose(data, 1.0) else data
                
                self.get_logger().debug(f"Normalized last_bottle: {raw_data} -> {data}")
                
                value.append(
                    dict(
                        data=data
                    )
                )
        elif isinstance(self.reading.data, list):
            if "glass" in self.name:
                for perception in self.reading.data:
                    distance = (
                    perception.distance - self.normalize_values["distance_min"]
                    ) / (
                        self.normalize_values["distance_max"]
                        - self.normalize_values["distance_min"]
                    )
                    angle = (perception.angle - self.normalize_values["angle_min"]) / (
                        self.normalize_values["angle_max"]
                        - self.normalize_values["angle_min"]
                    )
                    
                    state = perception.state/(self.normalize_values["n_states"] - 1) # Normalize 0,1,2 states between 0 and 1
                    state = 0.98 if isclose(state, 1.0) else state

                    value.append(
                        dict(
                            distance = distance,
                            angle = angle,
                            state = state
                        )
                    )
            elif "client" in self.name:
                # TODO: Implement distance and angle
                for perception in self.reading.data:
                    id = perception.id/(self.normalize_values["n_ids"] - 1) # Normalize 0,1,2 states between 0 and 1
                    id = 0.98 if isclose(id, 1.0) else id
                    preference = perception.preference/(self.normalize_values["n_preferences"] - 1) # Normalize 0,1,2 states between 0 and 1
                    preference = 0.98 if isclose(preference, 1.0) else preference


                    value.append(
                        dict(
                            id=id,
                            preference=preference
                        )
                    )
        else:
            value.append(dict(data=self.reading.data))
        
        sensor[self.name] = value
        self.get_logger().debug("Publishing normalized " + self.name + " = " + str(sensor))
        sensor_msg = perception_dict_to_msg(sensor)
        self.publish_msg.perception=sensor_msg
        self.publish_msg.timestamp=self.get_clock().now().to_msg()
        self.perception_publisher.publish(self.publish_msg)


class BartenderFilterPerception(Perception):
    """Bartender Filter Perception class with bottle filtering capability"""
    def __init__(self, name='filter_perception', class_name = 'cognitive_nodes.perception.Perception', default_msg = None, default_topic = None, normalize_data = None, **params):
        """
        Constructor for the Perception class.
        Initializes a Perception instance with the given name and registers it in the LTM.
        
        :param name: The name of the Perception instance.
        :type name: str
        :param class_name: The name of the Perception class.
        :type class_name: str
        :param default_msg: The msg of the default subscription.
        :type default_msg: str
        :param default_topic: The topic of the default subscription.
        :type default_topic: str
        :param normalize_data: Values in order to normalize values.
        :type normalize_data: dict
        """
        super().__init__(name, class_name, default_msg, default_topic, normalize_data, **params)
        self.extra_subscription = self.create_subscription(
            Float32,
            'cognitive_node/world_model/last_bottle',
            self.filter_callback,
            10
        )
        self._last_bottle_id = None  # Instance variable to store the last bottle ID

    
    def process_and_send_reading(self):
        sensor = {}
        value = []
        
        # Handle last_bottle separately since it might be an int instead of a list
        if "last_bottle" in self.name:
            if isinstance(self.reading.data, list):
                for perception in self.reading.data:
                    raw_data = perception.data
                    self.get_logger().debug(f"Processing last_bottle from list: raw_data = {raw_data}")
                    
                    # Store the raw ID in the class variable for bottle filtering
                    BartenderPerception._last_bottle_id = raw_data
                    self.get_logger().debug(f"Updated shared last_bottle_id to: {raw_data}")
                    
                    data = perception.data/(self.normalize_values["n_ids"] - 1) # Normalize 0,1,2 states between 0 and 1
                    data = 0.98 if isclose(data, 1.0) else data
                    
                    self.get_logger().debug(f"Normalized last_bottle: {raw_data} -> {data}")
                    
                    value.append(
                        dict(
                            data=data
                        )
                    )
            else:
                # Handle case where last_bottle is a direct int value
                raw_data = self.reading.data
                self.get_logger().debug(f"Processing last_bottle as int: raw_data = {raw_data}")
                
                # Store the raw ID in the class variable for bottle filtering
                BartenderPerception._last_bottle_id = raw_data
                self.get_logger().debug(f"Updated shared last_bottle_id to: {raw_data}")
                
                data = self.reading.data/(self.normalize_values["n_ids"] - 1) # Normalize 0,1,2 states between 0 and 1
                data = 0.98 if isclose(data, 1.0) else data
                
                self.get_logger().debug(f"Normalized last_bottle: {raw_data} -> {data}")
                
                value.append(
                    dict(
                        data=data
                    )
                )
        elif isinstance(self.reading.data, list):
            if "bottles" in self.name:
                # BOTTLE FILTERING LOGIC - Only process one bottle based on last_bottle_id
                last_bottle_id = self._last_bottle_id
                self.get_logger().debug(f"Filtering bottles by last_bottle ID: {last_bottle_id}")
                
                bottle_found = False
                
                # First try to find the bottle that matches last_bottle_id
                if last_bottle_id is not None:
                    for perception in self.reading.data:
                        if isclose(perception.id, last_bottle_id, abs_tol=1e-3):
                            distance = (
                                perception.distance - self.normalize_values["distance_min"]
                            ) / (
                                self.normalize_values["distance_max"]
                                - self.normalize_values["distance_min"]
                            )
                            angle = (perception.angle - self.normalize_values["angle_min"]) / (
                                self.normalize_values["angle_max"]
                                - self.normalize_values["angle_min"]
                            )
                            state = perception.state/(self.normalize_values["n_states"] - 1) # Normalize 0,1,2 states between 0 and 1
                            state = 0.98 if isclose(state, 1.0) else state
                            id = perception.id/(self.normalize_values["n_ids"] - 1) # Normalize 0,1,2 states between 0 and 1
                            value.append(
                                dict(
                                    distance=distance,
                                    angle=angle,
                                    state=state,
                                    id=id
                                )
                            )
                            
                            self.get_logger().debug(f"Added bottle {perception.id} to perception (matches last_bottle)")
                            bottle_found = True
                            break
                
                # If no matching bottle found or no last_bottle_id, use the first bottle
                if not bottle_found and self.reading.data:
                    perception = self.reading.data[0]  # Take the first bottle
                    distance = (
                        perception.distance - self.normalize_values["distance_min"]
                    ) / (
                        self.normalize_values["distance_max"]
                        - self.normalize_values["distance_min"]
                    )
                    angle = (perception.angle - self.normalize_values["angle_min"]) / (
                        self.normalize_values["angle_max"]
                        - self.normalize_values["angle_min"]
                    )
                    state = perception.state/(self.normalize_values["n_states"] - 1) # Normalize 0,1,2 states between 0 and 1
                    state = 0.98 if isclose(state, 1.0) else state
                    id = perception.id/(self.normalize_values["n_ids"] - 1) # Normalize 0,1,2 states between 0 and 1
                    value.append(
                        dict(
                            distance=distance,
                            angle=angle,
                            state=state,
                            id=id
                        )
                    )
                    
                    if last_bottle_id is not None:
                        self.get_logger().debug(f"No bottle found matching last_bottle ID: {last_bottle_id}, using first bottle {perception.id}")
                    else:
                        self.get_logger().debug(f"No last_bottle ID available, using first bottle {perception.id}")
                
                # Always ensure only one bottle is published
                if len(value) > 1:
                    value = [value[0]]
                    self.get_logger().debug("Multiple bottles found, keeping only the first one")
        else:
            value.append(dict(data=self.reading.data))
        
        sensor[self.name] = value
        self.get_logger().debug("Publishing normalized " + self.name + " = " + str(sensor))
        sensor_msg = perception_dict_to_msg(sensor)
        self.publish_msg.perception=sensor_msg
        self.publish_msg.timestamp=self.get_clock().now().to_msg()
        self.perception_publisher.publish(self.publish_msg)

    def filter_callback(self, msg):
        """
        Callback for the extra subscription to filter bottles based on last_bottle ID.
        """
        self.get_logger().debug(f"Received filter message: {msg.data}")
        self._last_bottle_id = msg.data
        # Implement any additional filtering logic if needed
        # Currently, bottle filtering is handled in process_and_send_reading()
        pass

