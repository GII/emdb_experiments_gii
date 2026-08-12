"""
Perception subclass for emdb_simulator's perception_mode:=mdb topics.

The generic cognitive_nodes.perception.Perception is abstract
(process_and_send_reading raises), so even the scalar Bool/Float32 sensors need
a concrete subclass. This one handles both shapes emitted on
/emdb/simulator/sensor/*:

  * emdb_interfaces/ObjectStateArray (e.g. .../obj): flattens the first object's
    world position into normalized [x, y, z].
  * std_msgs/Bool, std_msgs/Float32 (.../obj/grasped, .../progress): passed
    through as a single 'data' feature (already in [0, 1]).
"""

import numpy as np

from core.container import Container
from cognitive_nodes.perception import Perception


class EmdbSimulatorPerception(Perception):
    """Flatten emdb_simulator mdb sensor messages into e-MDB perception values."""

    # Pass everything to the base by KEYWORD. The Perception base has a `node_type`
    # parameter between `class_name` and `default_msg`, so a positional super().__init__
    # would misalign the args (default_msg landing in the wrong slot). Keyword args are
    # robust to that; node_type defaults to "Perception" in the base, which is exactly
    # the LTM category this node should register under.
    def __init__(self, name="perception",
                 class_name="cognitive_nodes.perception.Perception",
                 default_msg=None, default_topic=None, normalize_data=None,
                 **params):
        super().__init__(name, class_name=class_name,
                         default_msg=default_msg, default_topic=default_topic,
                         normalize_data=normalize_data, **params)

    def _norm(self, value, key_min, key_max):
        nv = self.normalize_values
        if nv and key_min in nv and key_max in nv:
            lo, hi = nv[key_min], nv[key_max]
            if hi != lo:
                return (value - lo) / (hi - lo)
        return value

    def process_and_send_reading(self):
        reading = self.reading

        if hasattr(reading, "objects"):          # emdb_interfaces/ObjectStateArray
            if len(reading.objects) == 0:
                self.get_logger().warning("Received empty ObjectStateArray.")
                return
            pos = reading.objects[0].pose.position
            x = self._norm(float(pos.x), "x_min", "x_max")
            y = self._norm(float(pos.y), "y_min", "y_max")
            z = self._norm(float(pos.z), "z_min", "z_max")
            labels = ["x", "y", "z"]
            data = np.array([x, y, z])
        else:                                    # std_msgs/Bool or /Float32
            labels = ["data"]
            data = np.array([float(reading.data)])

        if self.container is None:
            self.container = Container(self.name, max_size=1,
                                       container_type="perception", labels=labels)
        self.container.push(data, labels,
                            timestamps=self.get_clock().now().nanoseconds)
        self.get_logger().debug(
            f"Publishing normalized {self.name} = {self.container}")
        self.perception_publisher.publish(self.container.to_msg())
