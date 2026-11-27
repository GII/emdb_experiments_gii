"""
Bartender Perceptions - Deterministic version for EMDB
Maintains synchronization across perception nodes.
"""
import numpy as np

from math import isclose
from rclpy.logging import LoggingSeverity
from std_msgs.msg import Float32


from cognitive_nodes.perception import Perception
from core.container import Container
from core.utils import class_from_classname



# ================================================================
# BartenderPerception
# ================================================================
class BartenderPerception(Perception):
    """
    Perception node of the bartender experiment for the glass and client sensors of the simulator.

    The behavior depends on the name of the node. If it contains ``glass`` and the reading is a list, the node publishes
    ``distance``, ``angle``, ``state``, ``was_used``, ``drink_type`` and ``is_shaken``. If it contains ``client`` and the
    reading is a list, it publishes ``id`` and ``preference``. Otherwise, the reading is published as a single feature
    named ``data``. Distances and angles are scaled to [0, 1] with the limits of ``normalize_data``. The ``id`` and the
    ``drink_type`` are divided by the number of ids or drink types minus one, and values of 1.0 or more are set to 0.98.
    Empty lists are not published.
    """

    def __init__(self, name='perception', class_name='cognitive_nodes.perception.Perception',
                 default_msg=None, default_topic=None, normalize_data=None, **params):
        """
        Constructor for the BartenderPerception class.

        :param name: The name of the node.
        :type name: str
        :param class_name: The name of the Perception class.
        :type class_name: str
        :param default_msg: Class name of the message of the default subscription.
        :type default_msg: str
        :param default_topic: The topic of the default subscription.
        :type default_topic: str
        :param normalize_data: Normalization limits: ``distance_min``, ``distance_max``, ``angle_min``, ``angle_max``, ``n_ids`` and ``n_drink_types``. The limits that are not given default to 0.0, 1.0, -1.0, 1.0, 2 and 2, respectively.
        :type normalize_data: dict
        :param params: Additional parameters passed to :class:`cognitive_nodes.perception.Perception`.
        :type params: dict
        """
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
        """
        Normalize the last reading and publish it as a :class:`core.container.Container` message on ``perception/<name>/value``.
        """
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

        # The text is built only if it is going to be logged: building it for every reading is expensive
        if self.get_logger().is_enabled_for(LoggingSeverity.DEBUG):
            self.get_logger().debug("Publishing normalized " + self.name + " = " + str(self.container))
        sensor_msg = self.container.to_msg()
        self.perception_publisher.publish(sensor_msg)



# ================================================================
# BartenderFilterPerception
# ================================================================
class BartenderFilterPerception(Perception):
    """
    Perception node of the bartender experiment for the bottles.

    The simulator publishes all the bottles. This node selects one of them and publishes its normalized ``distance``,
    ``angle`` and ``drink_type``. The selected bottle is the one held by the gripper, if there is one; otherwise, the one
    whose drink type matches the last bottle published by the World Models (``last_bottle_topic``); and otherwise, the first
    bottle of the list. The node needs two extra subscriptions, to the last bottle and to the gripper, both required.
    """

    def __init__(self, name='filter_perception', class_name='cognitive_nodes.perception.Perception',
                 default_msg=None, default_topic=None, normalize_data=None, last_bottle_topic=None, last_bottle_msg=None, gripper_topic=None, gripper_msg=None, **params):
        """
        Constructor for the BartenderFilterPerception class.

        :param name: The name of the node.
        :type name: str
        :param class_name: The name of the Perception class.
        :type class_name: str
        :param default_msg: Class name of the message of the bottles subscription.
        :type default_msg: str
        :param default_topic: The topic of the bottles subscription.
        :type default_topic: str
        :param normalize_data: Normalization limits: ``distance_min``, ``distance_max``, ``angle_min``, ``angle_max`` and ``n_drink_types``. The limits that are not given default to 0.0, 1.0, -1.0, 1.0 and 2, respectively.
        :type normalize_data: dict
        :param last_bottle_topic: Topic with the (normalized) drink type of the preferred bottle, published by the World Models.
        :type last_bottle_topic: str
        :param last_bottle_msg: Class name of the message of ``last_bottle_topic``.
        :type last_bottle_msg: str
        :param gripper_topic: Topic with the state of the gripper, whose messages have the fields ``data``, ``contents`` and ``contents_id``.
        :type gripper_topic: str
        :param gripper_msg: Class name of the message of ``gripper_topic``.
        :type gripper_msg: str
        :param params: Additional parameters passed to :class:`cognitive_nodes.perception.Perception`.
        :type params: dict
        :raises ValueError: If the topic or the message of the last bottle or of the gripper is not provided.
        """
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
        self._last_bottle_timestamp = None
        self._gripper_timestamp = None
        self._reading_timestamp = None

        if normalize_data:
            self._distance_min = normalize_data.get("distance_min", 0.0)
            self._distance_range = max(1e-6, normalize_data.get("distance_max", 1.0) - self._distance_min)
            self._angle_min = normalize_data.get("angle_min", -1.0)
            self._angle_range = max(1e-6, normalize_data.get("angle_max", 1.0) - self._angle_min)
            self._drinks_divisor = max(1, normalize_data.get("n_drink_types", 2) - 1)

    def read_perception_callback(self, reading):
        """
        Store the time of the reading and process it as in :class:`cognitive_nodes.perception.Perception`.

        :param reading: The message with the bottles, of the class given by ``default_msg``.
        """
        self._reading_timestamp = self.get_clock().now().nanoseconds
        super().read_perception_callback(reading)

    def _normalize_and_clamp(self, raw_value, divisor):
        n = raw_value / divisor
        return 0.98 if n >= 1.0 else max(0.0, n)

    def _normalize_bottle(self, p):
        distance=(p.distance - self._distance_min) / self._distance_range
        angle=(p.angle - self._angle_min) / self._angle_range
        drink_type = self._normalize_and_clamp(
            p.drink_type,
            self._drinks_divisor,
        )
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
        """
        Select a bottle from the last reading, normalize it and publish it as a :class:`core.container.Container` message on
        ``perception/<name>/value``. The timestamp of the published sample is the oldest of the readings, last bottle and gripper messages received. Nothing is
        published if the reading has no bottles.
        """
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
        timestamps = [
            self._reading_timestamp,
            self._last_bottle_timestamp,
            self._gripper_timestamp,
        ]
        timestamps = [timestamp for timestamp in timestamps if timestamp is not None]

        if self.container is None:
            self.container = Container(self.name, max_size=1, container_type="perception", labels=labels)
        self.container.push(data, labels, timestamps=min(timestamps))

        # The text is built only if it is going to be logged: building it for every reading is expensive
        if self.get_logger().is_enabled_for(LoggingSeverity.DEBUG):
            self.get_logger().debug("Publishing normalized " + self.name + " = " + str(self.container))
        sensor_msg = self.container.to_msg()
        self.perception_publisher.publish(sensor_msg)

    def last_bottle_callback(self, msg):
        """
        Callback of the last bottle subscription. It converts the received normalized drink type to a drink type identifier and,
        if there is already a reading of the bottles, processes it again so that the selected bottle is updated.

        :param msg: Message with the last bottle in its ``data`` field. Messages without a numeric ``data`` are discarded.
        """
        self._last_bottle_timestamp = self.get_clock().now().nanoseconds
        # Accept whatever message type is configured for the perception topic
        # (e.g., std_msgs.msg.Int8 from the simulator or std_msgs.msg.Float32
        # from the world_model). Extract numeric value in a tolerant way.
        try:
            val = float(msg.data)
        except (AttributeError, TypeError, ValueError) as exc:
            self.get_logger().error(f"Invalid last bottle message: {msg!r}: {exc}")
            return
        self._last_bottle_id = round(val * self._drinks_divisor)
        if getattr(self, "reading", None) is not None:
            self.process_and_send_reading()

    def gripper_callback(self, msg):
        """
        Callback of the gripper subscription. If the gripper holds a bottle (``data`` is true and ``contents`` is ``bottle``), the
        identifier ``contents_id`` is stored as the grasped bottle. If the gripper is empty, the grasped bottle is cleared.

        :param msg: Message with the state of the gripper, with the fields ``data``, ``contents`` and ``contents_id``.
        """
        self._gripper_timestamp = self.get_clock().now().nanoseconds
        if msg.data:
            if msg.contents == "bottle":
                self._grasped_bottle_id = msg.contents_id
            else:
                self.get_logger().warn("Gripper callback received unexpected contents: " + str(msg.contents))
        else:
            self._grasped_bottle_id = None
                
