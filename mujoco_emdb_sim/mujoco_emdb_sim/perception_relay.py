"""
perception_relay -- TEMPORARY WORKAROUND.  # noqa

############################################################################
#  TEMPORARY -- REMOVE once emdb_simulator's scene_loader streams          #
#  perceptions continuously in control_mode:=rl.                           #
#                                                                          #
#  To remove: delete this file, its entry_point in setup.py, and the       #
#  relay Node in experiments/launch/lift_launch.py, then rebuild.          #
############################################################################

Why this exists
---------------
In control_mode:=rl, scene_loader only publishes /emdb/simulator/sensor/* on a
reset or a /step_action (no continuous timer -- the render loop that would drive
one is teleop-only). The e-MDB main loop, however, expects perceptions to keep
flowing (sim_ur5 streams them at 20 Hz); reading a perception that isn't being
re-published times out, and since the loop won't step the sim until it has read
a perception, it deadlocks.

The RIGHT fix is sim-side: have scene_loader publish perceptions on a timer in
rl mode (like sim_ur5). Until the friend does that, this node bridges the gap:
it caches the latest message seen on each sensor topic and re-emits it on the
SAME topic at a fixed rate. The perception nodes still receive the sim's real
values directly on each step; the relay only fills the gaps between steps so the
main loop's reads never time out. Re-emitting is timer-driven (never on receive),
so subscribing to the same topic it publishes to cannot create a feedback loop.

Runs in the architecture environment, so it needs emdb_interfaces on the path
(the TFM ros_packages overlay) for ObjectStateArray.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32
from emdb_interfaces.msg import ObjectStateArray


class PerceptionRelay(Node):
    """Re-emit the latest value on each mdb sensor topic at a fixed rate."""

    # (topic, message type) for perception_mode:=mdb with a single object "obj".
    SPECS = [
        ("/emdb/simulator/sensor/obj", ObjectStateArray),
        ("/emdb/simulator/sensor/obj/grasped", Bool),
        ("/emdb/simulator/sensor/progress", Float32),
    ]

    def __init__(self):
        super().__init__("perception_relay")
        rate = (
            self.declare_parameter("rate", 20.0)
            .get_parameter_value().double_value
        )

        self._cache = {}
        self._pubs = {}
        for topic, mtype in self.SPECS:
            self._pubs[topic] = self.create_publisher(mtype, topic, 1)
            self.create_subscription(mtype, topic, self._make_cb(topic), 1)

        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().warning(
            f"[TEMPORARY] perception_relay re-emitting {len(self.SPECS)} sensor "
            f"topics at {rate} Hz -- remove once scene_loader streams "
            f"perceptions in rl mode")

    def _make_cb(self, topic):
        def _cb(msg):
            self._cache[topic] = msg
        return _cb

    def _tick(self):
        for topic, msg in list(self._cache.items()):
            self._pubs[topic].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard Interrupt Detected: Shutting down perception relay...")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
