"""
Bartender Perceptions - Deterministic version for EMDB
Maintains synchronization across perception nodes.
"""
import numpy as np

from math import isclose
from std_msgs.msg import Float32


from cognitive_nodes.perception import Perception
from core.container import Container
from core.utils import class_from_classname



# ================================================================
# BartenderPerception
# ================================================================
class BartenderPerception(Perception):
    """Deterministic perception node (glass, client, last_bottle)."""

    def __init__(self, name='perception', class_name='cognitive_nodes.perception.Perception',
                 default_msg=None, default_topic=None, normalize_data=None, **params):
        super().__init__(name=name, class_name=class_name, default_msg=default_msg, default_topic=default_topic, normalize_data=normalize_data, **params)

        self._last_bottle_id = None

        # Precompute normalization
        if normalize_data:
            self._distance_min = normalize_data.get("distance_min", 0.0)
            self._distance_range = max(1e-6, normalize_data.get("distance_max", 1.0) - self._distance_min)
            self._angle_min = normalize_data.get("angle_min", -1.0)
            self._angle_range = max(1e-6, normalize_data.get("angle_max", 1.0) - self._angle_min)
            self._id_divisor = max(1, normalize_data.get("n_ids", 2) - 1)
            self._drinks_divisor = max(1, normalize_data.get("n_drink_types", 2) - 1)

    def _normalize_and_clamp(self, raw_value, divisor):
        n = raw_value / divisor
        return 0.98 if n >= 1.0 else max(0.0, n)

    def process_and_send_reading(self):
        reading = getattr(self.reading, "data", None)
        if "glass" in self.name and isinstance(reading, list):
            if len(reading) == 0:
                return # No reading to process, return immediatly
            for p in reading:
                distance=(p.distance - self._distance_min) / self._distance_range
                angle=(p.angle - self._angle_min) / self._angle_range
                state=p.state
                was_used=p.was_used
                drink_type=self._normalize_and_clamp(p.drink_type, self._drinks_divisor)
                is_shaken=p.is_shaken
                data=np.array([distance, angle, state, was_used, drink_type, is_shaken])
                labels = ["distance", "angle", "state", "was_used", "drink_type", "is_shaken"]
        elif "client" in self.name and isinstance(reading, list):
            if len(reading) == 0:
                return # No reading to process, return immediatly
            for p in reading:
                id=self._normalize_and_clamp(p.id, self._id_divisor)
                preference=self._normalize_and_clamp(p.preference, self._drinks_divisor)
                data=np.array([id, preference])
                labels = ["id", "preference"]
        else:
            data=np.array([reading])
            labels = ["data"]

        if self.container is None:
            self.container = Container(self.name, max_size=1, container_type="perception", labels=labels)
        self.container.push(data, labels, timestamps=self.get_clock().now().nanoseconds)

        self.get_logger().debug("Publishing normalized " + self.name + " = " + str(self.container))
        sensor_msg = self.container.to_msg()
        self.perception_publisher.publish(sensor_msg)



# ================================================================
# BartenderFilterPerception
# ================================================================
class BartenderFilterPerception(Perception):
    """Deterministic bottle filter perception node."""

    def __init__(self, name='filter_perception', class_name='cognitive_nodes.perception.Perception',
                 default_msg=None, default_topic=None, normalize_data=None, last_bottle_topic=None, last_bottle_msg=None, gripper_topic=None, gripper_msg=None, **params):
        super().__init__(name=name, class_name=class_name, default_msg=default_msg, default_topic=default_topic, normalize_data=normalize_data, **params)

        # Subscribe to the world_model's last_bottle by default (Float32)
        # while the base class already subscribes to `default_topic` for bottles.
        if last_bottle_topic and last_bottle_msg:
            self.last_bottle_subscription = self.create_subscription(
                class_from_classname(last_bottle_msg),
                last_bottle_topic,
                self.last_bottle_callback,
                1
            )
        else:
            raise ValueError("last_bottle_topic and last_bottle_msg must be provided for BartenderFilterPerception.")

        if gripper_topic and gripper_msg:
            self.gripper_subscription = self.create_subscription(
                class_from_classname(gripper_msg),
                gripper_topic,
                self.gripper_callback,
                1
            )
        else:
            raise ValueError("gripper_topic and gripper_msg must be provided for BartenderFilterPerception.")

        self._last_bottle_id = None
        self._grasped_bottle_id = None

        if normalize_data:
            self._distance_min = normalize_data.get("distance_min", 0.0)
            self._distance_range = max(1e-6, normalize_data.get("distance_max", 1.0) - self._distance_min)
            self._angle_min = normalize_data.get("angle_min", -1.0)
            self._angle_range = max(1e-6, normalize_data.get("angle_max", 1.0) - self._angle_min)
            self._drinks_divisor = max(1, normalize_data.get("n_drink_types", 2) - 1)

    def _normalize_and_clamp(self, raw_value, divisor):
        n = raw_value / divisor
        return 0.98 if n >= 1.0 else max(0.0, n)

    def _normalize_bottle(self, p):
        distance=(p.distance - self._distance_min) / self._distance_range
        angle=(p.angle - self._angle_min) / self._angle_range
        drink_type=p.drink_type / self._drinks_divisor
        data=np.array([distance, angle, drink_type])
        labels = ["distance", "angle", "drink_type"]

        return data, labels

    def _as_bottle_list(self, data):
        if data is None:
            return []
        if isinstance(data, list):
            return data
        return [data]

    def _select_bottle(self, bottles):
        selected = None
        selected_id = self._grasped_bottle_id if self._grasped_bottle_id is not None else self._last_bottle_id

        if selected_id is not None:
            for bottle in bottles:
                if isclose(bottle.drink_type, selected_id, abs_tol=1e-3):
                    selected = bottle
                    break
        if selected is None and bottles:
            selected = bottles[0]
        return selected

    def process_and_send_reading(self):
        # `BottleMsg` does not have a `.data` field, so work with the message
        # object directly and also accept list-style containers from other
        # bottle perceptions.
        reading = self.reading.data
        data = None
        labels = None

        bottles = self._as_bottle_list(reading)
        if bottles:
            selected = self._select_bottle(bottles)
            if selected is not None:
                data, labels = self._normalize_bottle(selected)
        if data is None:
            return  # No valid bottle selected, return immediately

        if self.container is None:
            self.container = Container(self.name, max_size=1, container_type="perception", labels=labels)
        self.container.push(data, labels, timestamps=self.get_clock().now().nanoseconds)

        self.get_logger().debug("Publishing normalized " + self.name + " = " + str(self.container))
        sensor_msg = self.container.to_msg()
        self.perception_publisher.publish(sensor_msg)

    def last_bottle_callback(self, msg):
        # Accept whatever message type is configured for the perception topic
        # (e.g., std_msgs.msg.Int8 from the simulator or std_msgs.msg.Float32
        # from the world_model). Extract numeric value in a tolerant way.
        try:
            val = msg.data
        except Exception:
            # Fallback: if message wraps list or nested structure
            try:
                val = float(msg)
            except Exception:
                val = None
        self._last_bottle_id = int(val * self._drinks_divisor)

    def gripper_callback(self, msg):
        if msg.data:
            if msg.contents == "bottle":
                self._grasped_bottle_id = msg.contents_id
            else:
                self.get_logger().warn("Gripper callback received unexpected contents: " + str(msg.contents))
        else:
            self._grasped_bottle_id = None
                
