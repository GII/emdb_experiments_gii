"""
Bartender Perceptions - Deterministic version for EMDB
Maintains synchronization across perception nodes.
"""
import numpy as np

from math import isclose
from std_msgs.msg import Float32
from cognitive_nodes.perception import Perception
from core.container import Container


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
            self._preference_divisor = max(1, normalize_data.get("n_preferences", 2) - 1)
            self._state_divisor = max(1, normalize_data.get("n_states", 2) - 1)

    def _normalize_and_clamp(self, raw_value, divisor):
        n = raw_value / divisor
        return 0.98 if n >= 1.0 else max(0.0, n)

    def _process_last_bottle(self, raw_data):
        self._last_bottle_id = raw_data
        return self._normalize_and_clamp(raw_data, self._id_divisor)

    def process_and_send_reading(self):
        reading = getattr(self.reading, "data", None)
        if "last_bottle" in self.name:
            raw = reading
            if isinstance(raw, list):
                raw = raw[0].data if raw else 0.0
            data=np.array([self._process_last_bottle(raw)])
            labels = ["data"]
        elif "glass" in self.name and isinstance(reading, list):
            if len(reading) == 0:
                return # No reading to process, return immediatly
            for p in reading:
                distance=(p.distance - self._distance_min) / self._distance_range
                angle=(p.angle - self._angle_min) / self._angle_range
                state=p.state
                was_used=p.was_used
                drink_type=self._normalize_and_clamp(p.drink_type, self._preference_divisor)
                is_shaken=p.is_shaken
                data=np.array([distance, angle, state, was_used, drink_type, is_shaken])
                labels = ["distance", "angle", "state", "was_used", "drink_type", "is_shaken"]
        elif "client" in self.name and isinstance(reading, list):
            if len(reading) == 0:
                return # No reading to process, return immediatly
            for p in reading:
                id=self._normalize_and_clamp(p.id, self._id_divisor)
                preference=self._normalize_and_clamp(p.preference, self._preference_divisor)
                likes_shake=self._normalize_and_clamp(p.likes_shake, self._state_divisor)
                data=np.array([id, preference, likes_shake])
                labels = ["id", "preference", "likes_shake"]
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
                 default_msg=None, default_topic=None, normalize_data=None, **params):
        super().__init__(name=name, class_name=class_name, default_msg=default_msg, default_topic=default_topic, normalize_data=normalize_data, **params)

        # Subscribe to the world_model's last_bottle by default (Float32)
        # while the base class already subscribes to `default_topic` for bottles.
        self.extra_subscription = self.create_subscription(
            Float32,
            'cognitive_node/world_model/last_bottle',
            self.filter_callback,
            1
        )

        self._last_bottle_id = None

        if normalize_data:
            self._distance_min = normalize_data.get("distance_min", 0.0)
            self._distance_range = max(1e-6, normalize_data.get("distance_max", 1.0) - self._distance_min)
            self._angle_min = normalize_data.get("angle_min", -1.0)
            self._angle_range = max(1e-6, normalize_data.get("angle_max", 1.0) - self._angle_min)
            self._x_min = normalize_data.get("x_min", 0.0)
            self._x_range = max(1e-6, normalize_data.get("x_max", 1.5) - self._x_min)
            self._y_min = normalize_data.get("y_min", 0.0)
            self._y_range = max(1e-6, normalize_data.get("y_max", 1.5) - self._y_min)
            self._id_divisor = max(1, normalize_data.get("n_ids", 2) - 1)
            self._state_divisor = max(1, normalize_data.get("n_states", 2) - 1)

    def _normalize_and_clamp(self, raw_value, divisor):
        n = raw_value / divisor
        return 0.98 if n >= 1.0 else max(0.0, n)

    def _normalize_bottle(self, p):
        distance=(p.distance - self._distance_min) / self._distance_range
        angle=(p.angle - self._angle_min) / self._angle_range
        id=p.id / self._id_divisor
        x=(getattr(p, "x", 0.0) - self._x_min) / self._x_range
        y=(getattr(p, "y", 0.0) - self._y_min) / self._y_range
        data=np.array([distance, angle, id, x, y])
        labels = ["distance", "angle", "id", "x", "y"]

        return data, labels

    def _as_bottle_list(self, data):
        if data is None:
            return []
        if isinstance(data, list):
            return data
        return [data]

    def _select_bottle(self, bottles):
        selected = None
        if self._last_bottle_id is not None:
            for bottle in bottles:
                if isclose(bottle.id, self._last_bottle_id, abs_tol=1e-3):
                    selected = bottle
                    break
        if selected is None and bottles:
            selected = bottles[0]
        return selected

    def process_and_send_reading(self):
        # `BottleMsg` does not have a `.data` field, so work with the message
        # object directly and also accept list-style containers from other
        # bottle perceptions.
        reading = self.reading
        data = None
        labels = None

        if "last_bottle" in self.name:
            bottles = self._as_bottle_list(reading)
            selected = None

            if bottles:
                selected = self._select_bottle(bottles)

            if selected is not None:
                self._last_bottle_id = getattr(selected, "id", None)
                data, labels = self._normalize_bottle(selected)
            else:
                raw = self._last_bottle_id
                if raw is None:
                    raw = getattr(reading, "id", None)
                if raw is None:
                    raw = -1
                data = np.array([self._normalize_and_clamp(raw, self._id_divisor)])
                labels = ["data"]

        else:
            bottles = self._as_bottle_list(reading)
            if bottles:
                selected = self._select_bottle(bottles)
                if selected is not None:
                    data, labels = self._normalize_bottle(selected)
            if data is None:
                data = np.array([reading])
                labels = ["data"]

        if self.container is None:
            self.container = Container(self.name, max_size=1, container_type="perception", labels=labels)
        self.container.push(data, labels, timestamps=self.get_clock().now().nanoseconds)

        self.get_logger().debug("Publishing normalized " + self.name + " = " + str(self.container))
        sensor_msg = self.container.to_msg()
        self.perception_publisher.publish(sensor_msg)

    def filter_callback(self, msg):
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
        self._last_bottle_id = val
