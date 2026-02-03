from math import cos, sin, pi

from cognitive_nodes.perception import Perception
from core.utils import perception_dict_to_msg

class Sim2DPerception(Perception):
    """
    Sim2DPerception class
    """
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
        if isinstance(self.reading.data, list):
            for perception in self.reading.data:
                x = (
                    perception.x - self.normalize_values["x_min"]
                ) / (
                    self.normalize_values["x_max"]
                    - self.normalize_values["x_min"]
                )
                y = (
                    perception.y - self.normalize_values["y_min"]
                ) / (
                    self.normalize_values["y_max"]
                    - self.normalize_values["y_min"]
                )
                if self.normalize_values.get("angle_max") and self.normalize_values.get("angle_min"):
                    # Check if angle is in degrees (common ranges: 0-360, -180 to 180)
                    angle_range = self.normalize_values["angle_max"] - self.normalize_values["angle_min"]
                    if angle_range > 2 * pi:  # Likely in degrees
                        angle_rad = perception.angle * pi / 180.0
                    else:  # Already in radians
                        angle_rad = perception.angle
                    
                    angle_cos_raw = cos(angle_rad)
                    angle_sin_raw = sin(angle_rad)
                    # normalize from [-1, 1] to [0, 1] and clip to avoid tiny numerical drift
                    angle_cos = min(max((angle_cos_raw + 1.0) / 2.0, 0.0), 1.0)
                    angle_sin = min(max((angle_sin_raw + 1.0) / 2.0, 0.0), 1.0)
                    
                    value.append(
                        dict(
                            x=x,
                            y=y,
                            angle_cos=angle_cos,
                            angle_sin=angle_sin
                        )
                    )
                else:
                    value.append(
                        dict(
                            x=x,
                            y=y,
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

class Sim2DDistancesPerception(Perception):
    """
    Sim2DDistancesPerception class
    """
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
        if isinstance(self.reading.data, list):
            for perception in self.reading.data:
                distance = (
                    perception.distance - self.normalize_values["distance_min"]
                ) / (
                    self.normalize_values["distance_max"]
                    - self.normalize_values["distance_min"]
                )
                if self.normalize_values.get("angle_max") and self.normalize_values.get("angle_min"):
                    # Check if angle is in degrees (common ranges: 0-360, -180 to 180)
                    angle_range = self.normalize_values["angle_max"] - self.normalize_values["angle_min"]
                    if angle_range > 2 * pi:  # Likely in degrees
                        angle_rad = perception.angle * pi / 180.0
                    else:  # Already in radians
                        angle_rad = perception.angle
                    
                    angle_cos_raw = cos(angle_rad)
                    angle_sin_raw = sin(angle_rad)
                    # normalize from [-1, 1] to [0, 1] and clip to avoid tiny numerical drift
                    angle_cos = min(max((angle_cos_raw + 1.0) / 2.0, 0.0), 1.0)
                    angle_sin = min(max((angle_sin_raw + 1.0) / 2.0, 0.0), 1.0)
                    
                    value.append(
                        dict(
                            distance=distance,
                            angle_cos=angle_cos,
                            angle_sin=angle_sin
                        )
                    )
                else:
                    value.append(
                        dict(
                            distance=distance,
                        )
                    )
        else:
            data=self.reading.data
            normalized_value = (
                data - self.normalize_values["min_value"]
            ) / (
                self.normalize_values["max_value"]
                - self.normalize_values["min_value"]
            )
            value.append(dict(data=normalized_value))

        sensor[self.name] = value
        self.get_logger().debug("Publishing normalized " + self.name + " = " + str(sensor))
        sensor_msg = perception_dict_to_msg(sensor)
        self.publish_msg.perception=sensor_msg
        self.publish_msg.timestamp=self.get_clock().now().to_msg()
        self.perception_publisher.publish(self.publish_msg)