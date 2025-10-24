"""
Bartender Perceptions - Deterministic version for EMDB
Maintains synchronization across perception nodes.
"""

from math import isclose
from copy import deepcopy
from std_msgs.msg import Float32
from cognitive_nodes.perception import Perception
from cognitive_node_interfaces.msg import PerceptionStamped
from core.utils import perception_dict_to_msg


# ================================================================
# BartenderPerception
# ================================================================
class BartenderPerception(Perception):
    """Deterministic perception node (glass, client, last_bottle)."""

    def __init__(self, name='perception', class_name='cognitive_nodes.perception.Perception',
                 default_msg=None, default_topic=None, normalize_data=None, **params):
        super().__init__(name, class_name, default_msg, default_topic, normalize_data, **params)

        self._msg_cache = PerceptionStamped()
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
        value = []
        data = getattr(self.reading, "data", None)

        if "last_bottle" in self.name:
            raw = data
            if isinstance(raw, list):
                raw = raw[0].data if raw else 0.0
            value.append(dict(data=self._process_last_bottle(raw)))

        elif "glass" in self.name and isinstance(data, list):
            for p in data:
                value.append(dict(
                    distance=(p.distance - self._distance_min) / self._distance_range,
                    angle=(p.angle - self._angle_min) / self._angle_range,
                    state=self._normalize_and_clamp(p.state, self._state_divisor)
                ))

        elif "client" in self.name and isinstance(data, list):
            for p in data:
                value.append(dict(
                    id=self._normalize_and_clamp(p.id, self._id_divisor),
                    preference=self._normalize_and_clamp(p.preference, self._preference_divisor)
                ))
        else:
            value.append(dict(data=data))

        self._msg_cache.perception = perception_dict_to_msg({self.name: value})
        self._msg_cache.timestamp = self.get_clock().now().to_msg()
        self.perception_publisher.publish(self._msg_cache)


# ================================================================
# BartenderFilterPerception
# ================================================================
class BartenderFilterPerception(Perception):
    """Deterministic bottle filter perception node."""

    def __init__(self, name='filter_perception', class_name='cognitive_nodes.perception.Perception',
                 default_msg=None, default_topic=None, normalize_data=None, **params):
        super().__init__(name, class_name, default_msg, default_topic, normalize_data, **params)

        self.extra_subscription = self.create_subscription(
            Float32,
            'cognitive_node/world_model/last_bottle',
            self.filter_callback,
            10
        )

        self._last_bottle_id = None
        self._msg_cache = PerceptionStamped()

        if normalize_data:
            self._distance_min = normalize_data.get("distance_min", 0.0)
            self._distance_range = max(1e-6, normalize_data.get("distance_max", 1.0) - self._distance_min)
            self._angle_min = normalize_data.get("angle_min", -1.0)
            self._angle_range = max(1e-6, normalize_data.get("angle_max", 1.0) - self._angle_min)
            self._id_divisor = max(1, normalize_data.get("n_ids", 2) - 1)
            self._state_divisor = max(1, normalize_data.get("n_states", 2) - 1)

    def _normalize_and_clamp(self, raw_value, divisor):
        n = raw_value / divisor
        return 0.98 if n >= 1.0 else max(0.0, n)

    def _normalize_bottle(self, p):
        return dict(
            distance=(p.distance - self._distance_min) / self._distance_range,
            angle=(p.angle - self._angle_min) / self._angle_range,
            state=self._normalize_and_clamp(p.state, self._state_divisor),
            id=p.id / self._id_divisor
        )

    def process_and_send_reading(self):
        data = getattr(self.reading, "data", None)
        value = []

        if "bottles" in self.name and isinstance(data, list):
            selected = None
            if self._last_bottle_id is not None:
                for p in data:
                    if isclose(p.id, self._last_bottle_id, abs_tol=1e-3):
                        selected = p
                        break
            if selected is None and data:
                selected = data[0]
            if selected:
                value.append(self._normalize_bottle(selected))

        elif "last_bottle" in self.name:
            raw = data
            if isinstance(raw, list):
                raw = raw[0].data if raw else 0.0
            value.append(dict(data=self._normalize_and_clamp(raw, self._id_divisor)))
            self._last_bottle_id = raw
        else:
            value.append(dict(data=data))

        self._msg_cache.perception = perception_dict_to_msg({self.name: value})
        self._msg_cache.timestamp = self.get_clock().now().to_msg()
        self.perception_publisher.publish(self._msg_cache)

    def filter_callback(self, msg: Float32):
        self._last_bottle_id = msg.data
