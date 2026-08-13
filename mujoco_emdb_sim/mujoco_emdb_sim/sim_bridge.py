"""
sim_bridge -- adapter between the e-MDB cognitive architecture and
emdb_simulator's RoboCasa/robosuite MuJoCo scenes.

The cognitive architecture speaks its own control protocol:
  * executed_action_service  (cognitive_node_interfaces/Action)
  * world_reset_service      (cognitive_processes_interfaces/WorldReset)
  * control_topic            (cognitive_processes_interfaces/ControlMsg)

emdb_simulator's scene_loader speaks its own RL protocol:
  * /step_action   (emdb_interfaces/StepAction)   -- one EE-delta physics step
  * /reset_episode (emdb_interfaces/ResetEpisode) -- restart the episode

This node hosts the e-MDB services and forwards each request to scene_loader.
It is task-agnostic: the action layout/bounds come from the experiment yaml's
EmdbSimulator.Actuation block, so the same bridge serves any emdb_simulator
scene. It does NOT publish perceptions: scene_loader already publishes them in
perception_mode:=mdb on /emdb/simulator/sensor/*, and the experiment's
Perception nodes subscribe there directly.

Runtime note: this node imports emdb_interfaces (from the TFM workspace), so
the shell that launches it must source BOTH this workspace's install and
/root/TFM/ros_packages/install (see the launch instructions).
"""

import os

import yaml
import yamlloader

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor

from core.service_client import ServiceClient
from core_interfaces.srv import LoadConfig
from core.container import Container
from core.utils import class_from_classname

# Friend's sim interface. Imported at runtime; requires the TFM ros_packages
# install to be sourced on top of this workspace.
from emdb_interfaces.srv import StepAction, ResetEpisode


class SimBridge(Node):
    """Translate e-MDB action/reset calls into scene_loader step/reset calls."""

    def __init__(self):
        super().__init__("sim_bridge")

        self.random_seed = (
            self.declare_parameter("random_seed", value=0)
            .get_parameter_value().integer_value
        )
        self.config_file = (
            self.declare_parameter(
                "config_file",
                descriptor=ParameterDescriptor(dynamic_typing=True),
            ).get_parameter_value().string_value
        )
        self.standalone = (
            self.declare_parameter("standalone", value=False)
            .get_parameter_value().bool_value
        )
        # scene_loader's native RL services (override if you remap them).
        self.step_service = (
            self.declare_parameter("step_service", value="/step_action")
            .get_parameter_value().string_value
        )
        self.reset_service = (
            self.declare_parameter("reset_service", value="/reset_episode")
            .get_parameter_value().string_value
        )

        self.cbgroup_server = MutuallyExclusiveCallbackGroup()

        self.actuation_config = None
        # Actuator label used in the experiment yaml's actuation_config
        # (feature labels arrive as "arm:dx", "arm:dy", ...).
        self.arm_name = "arm"

        # Sync clients to scene_loader, created lazily on first use so the bridge
        # can start before the sim finishes building its (heavy) scene.
        self._step_client = None
        self._reset_client = None
        # True once the world_reset SERVICE is hosted -> control-topic "reset_world"
        # commands are then ignored (the service is authoritative). This also avoids
        # two callbacks driving the shared reset client concurrently.
        self.service_world_reset = False

    # ------------------------------------------------ scene_loader clients
    def _step(self, **fields):
        if self._step_client is None:
            self.get_logger().info(
                f"Connecting to sim step service {self.step_service}...")
            self._step_client = ServiceClient(StepAction, self.step_service)
        return self._step_client.send_request(**fields)

    def _reset(self, **fields):
        if self._reset_client is None:
            self.get_logger().info(
                f"Connecting to sim reset service {self.reset_service}...")
            self._reset_client = ServiceClient(ResetEpisode, self.reset_service)
        return self._reset_client.send_request(**fields)

    # ------------------------------------------------ action decoding
    def denormalize_actuation(self, action: Container, actuation_config):
        """Un-normalize [0,1] Container features to engineering units via the
        yaml `bounds`. Same convention as Sim2DSimple / sim_ur5."""
        action_dims = action.feature_labels
        data = action.read()
        for dim in action_dims:
            actuator, param = dim.split(":", 1)
            if actuation_config[actuator][param]["type"] == "float":
                bounds = actuation_config[actuator][param]["bounds"]
                value = data.sel(features=dim).values
                data.loc[{"features": dim}] = (
                    bounds[0] + (value * (bounds[1] - bounds[0])))
        return data

    def _feature(self, vec, features, name, default=0.0):
        key = f"{self.arm_name}:{name}"
        if key in features:
            return float(vec.sel(features=key).values)
        return default

    # ------------------------------------------------ e-MDB service callbacks
    def executed_action_callback(self, request, response):
        action = Container.from_msg(request.action)
        vec = self.denormalize_actuation(action, self.actuation_config)
        features = list(vec.coords["features"].values)

        # scene_loader's StepAction: 6 EE deltas + int grasp. base_* / next_* are
        # left at 0 (fixed single UR5 arm, no mobile-base motion in this task).
        # grasp: float in [-1, 1] rounded to scene_loader's int32 (per the yaml).
        result = self._step(
            dx=self._feature(vec, features, "dx"),
            dy=self._feature(vec, features, "dy"),
            dz=self._feature(vec, features, "dz"),
            droll=self._feature(vec, features, "droll"),
            dpitch=self._feature(vec, features, "dpitch"),
            dyaw=self._feature(vec, features, "dyaw"),
            base_dx=0.0, base_dy=0.0, base_dyaw=0.0,
            grasp=int(round(self._feature(vec, features, "grasp"))),
            next_arm=0, next_robot=0,
        )
        response.success = bool(result.success) if result is not None else False
        return response

    def world_reset_callback(self, request, response):
        # -1 / -1 keeps the current layout/style; the episode just restarts.
        result = self._reset(layout_id=-1, style_id=-1)
        response.success = bool(result.success) if result is not None else False
        return response

    def control_callback(self, data):
        command = getattr(data, "command", "")
        if command == "reset_world" and not self.service_world_reset:
            self._reset(layout_id=-1, style_id=-1)
        elif command == "end":
            self.get_logger().info("Ending bridge as requested by LTM...")
            rclpy.shutdown()

    # ------------------------------------------------ yaml config
    def load_configuration(self):
        if not self.config_file or not os.path.isfile(self.config_file):
            self.get_logger().error(
                f"Config file '{self.config_file}' not found!")
            rclpy.shutdown()
            return
        config = yaml.load(
            open(self.config_file, "r", encoding="utf-8"),
            Loader=yamlloader.ordereddict.CLoader,
        )
        self.actuation_config = config["EmdbSimulator"]["Actuation"]
        self.setup_control_channel(config["Control"])

        if not self.standalone:
            self.load_experiment_file_in_commander()
        else:
            self.get_logger().info(
                "STANDALONE mode: not contacting the commander")

    def setup_control_channel(self, simulation):
        self.ident = simulation["id"]
        message = class_from_classname(simulation["control_msg"])
        self.create_subscription(
            message, simulation["control_topic"], self.control_callback, 0)

        service_action = simulation.get("executed_action_service")
        service_world_reset = simulation.get("world_reset_service")
        if simulation.get("executed_policy_topic"):
            raise RuntimeError(
                "Topic-triggered policies not supported; use "
                "executed_action_service")
        if service_action:
            msg_srv = class_from_classname(simulation["executed_action_msg"])
            self.create_service(
                msg_srv, service_action, self.executed_action_callback,
                callback_group=self.cbgroup_server)
        if service_world_reset:
            self.service_world_reset = True
            msg_reset = class_from_classname(simulation["world_reset_msg"])
            self.create_service(
                msg_reset, service_world_reset, self.world_reset_callback,
                callback_group=self.cbgroup_server)

    def load_experiment_file_in_commander(self):
        self.load_client = ServiceClient(LoadConfig, "commander/load_experiment")
        return self.load_client.send_request(file=self.config_file)


def main(args=None):
    rclpy.init(args=args)
    bridge = SimBridge()
    bridge.load_configuration()
    # Multi-threaded so world_reset / control can be serviced while an
    # executed_action call is blocked waiting on the sim's step.
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        rclpy.spin(bridge, executor=executor)
    except KeyboardInterrupt:
        print("Keyboard Interrupt Detected: Shutting down sim bridge...")
    finally:
        bridge.destroy_node()


if __name__ == "__main__":
    main()
