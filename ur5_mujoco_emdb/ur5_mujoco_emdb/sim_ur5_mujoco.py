"""e-MDB node of the MuJoCo UR5e simulator (executable sim_ur5).

It runs the scenario of the ur5_mujoco package (physics_simulators folder of emdb_discrete_event_simulator_gii) for
the gripper chosen in the experiment yaml, and connects it with the architecture as Sim2DSimple does with the 2D
simulator: it publishes the perceptions of the yaml, serves the action and world reset services of its Control
section and loads the experiment in the commander.
"""
import os.path
import yaml
import yamlloader
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor

from core.service_client import ServiceClient
from core_interfaces.srv import LoadConfig
from core.container import Container
from core.utils import class_from_classname, resolve_seed

from ur5_mujoco.scenarios import UR5Scenario, UR5GripperScenario, UR5TwoFG7Scenario


# ----------------------------------------------------------------------------- ROS2 node
class SimUR5Mujoco(Node):
    """MuJoCo UR5e simulator node for e-MDB. Mirrors Sim2DSimple's interface."""

    def __init__(self):
        super().__init__("sim_ur5_mujoco")

        self.random_seed = (
            self.declare_parameter("random_seed", value=0)
            .get_parameter_value().integer_value
        )
        self.config_file = (
            self.declare_parameter("config_file",
                                   descriptor=ParameterDescriptor(dynamic_typing=True))
            .get_parameter_value().string_value
        )
        visualize = (
            self.declare_parameter("visualize", value=True)
            .get_parameter_value().bool_value
        )
        self.standalone = (
            self.declare_parameter("standalone", value=False)
            .get_parameter_value().bool_value
        )
        realtime = (
            self.declare_parameter("realtime", value=False)
            .get_parameter_value().bool_value
        )
        sim_speed = (
            self.declare_parameter("sim_speed", value=2.0)
            .get_parameter_value().double_value
        )
        spawn_box = (
            self.declare_parameter("spawn_box", value=True)
            .get_parameter_value().bool_value
        )
        # 20 Hz is plenty: perception consumers do per-message Python/xarray
        # work, and 200 Hz across 10 sensor topics saturated their processes.
        self.perception_rate = (
            self.declare_parameter("perception_rate", value=20.0)
            .get_parameter_value().double_value
        )
        # Empty: the scene of the gripper chosen in the experiment yaml (see create_scenario)
        self.model_path = (
            self.declare_parameter("model_path", value="")
            .get_parameter_value().string_value
        )
        # Video of the trials (2FG7 scenario only): folder ("" = off) and every how many trials
        self.video_dir = (
            self.declare_parameter("video_dir", value="")
            .get_parameter_value().string_value
        )
        self.video_every = (
            self.declare_parameter("video_every", value=1)
            .get_parameter_value().integer_value
        )
        self.scenario_options = dict(visualize=visualize, realtime=realtime, speed=sim_speed, spawn_box=spawn_box)

        self.cbgroup_server = MutuallyExclusiveCallbackGroup()
        self.cbgroup_client = MutuallyExclusiveCallbackGroup()
        self.cbgroup_pub = MutuallyExclusiveCallbackGroup()  # perceptions: own group

        self.sim_publishers = {}
        self.perceptions = {}
        self.base_messages = {}
        self.service_world_reset = False

        self.sim = None  # created by load_configuration, once the yaml says which gripper
        self.arm_name = "left_arm"  # actuator name used in the yaml actuation_config

    SCENES = {
        "none": "ur5e_ball_box_scene.xml",          # magnet autograsp (the original experiment)
        "robotiq_2f85": "ur5e_2f85_ball_box_scene.xml",  # physical Robotiq 2F-85, grasp macro
        "onrobot_2fg7": "ur5e_2fg7_ball_box_scene.xml",  # the lab's OnRobot 2FG7, smooth motion, no macro
    }
    # Robot keys of the 2FG7 scenario (see UR5TwoFG7Scenario)
    TWOFG7_KEYS = ("grip_force", "grip_speed", "lift_height", "tool_speed", "tool_accel", "blend_time")

    def create_scenario(self, robot_cfg):
        """Create the MuJoCo scenario. ``robot_cfg`` is the ``Robot`` block of the simulator section of the
        experiment yaml: ``gripper`` (none | robotiq_2f85 | onrobot_2fg7, default none), and optionally
        ``hover_z``; with the Robotiq ``verify_lift`` (m); with the 2FG7 the TWOFG7_KEYS."""
        gripper = str(robot_cfg.get("gripper", "none"))
        if gripper not in self.SCENES:
            raise ValueError(f"Unknown gripper '{gripper}' (one of {list(self.SCENES)})")
        model_path = self.model_path or os.path.join(
            get_package_share_directory("ur5_mujoco"), "models", "universal_robots_ur5e", self.SCENES[gripper])
        options = dict(self.scenario_options, logger=self.get_logger())
        if "hover_z" in robot_cfg:
            options["hover_z"] = float(robot_cfg["hover_z"])
        if gripper == "none":
            self.sim = UR5Scenario(model_path, **options)
        elif gripper == "robotiq_2f85":
            self.sim = UR5GripperScenario(model_path, verify_lift=float(robot_cfg.get("verify_lift", 0.03)), **options)
        else:
            extra = {k: float(robot_cfg[k]) for k in self.TWOFG7_KEYS if k in robot_cfg}
            self.sim = UR5TwoFG7Scenario(model_path, video_dir=self.video_dir, video_every=self.video_every,
                                         **extra, **options)
        self.gripper = gripper
        self.get_logger().info(f"Scenario: {os.path.basename(model_path)} (gripper: {gripper})")

    # ------------------------------------------------------------- perceptions
    def get_perceptions(self):
        state = self.sim.get_state()  # thread-safe snapshot
        ee = state["ee"]
        heading = state["heading"]
        ball = state["ball"]
        box = state["box"]
        grasped = state["grasped"]
        in_box = state["ball_in_box"]

        def set_if(name, fn):
            if name in self.perceptions:
                fn(self.perceptions[name])

        def arm_msg(p):
            p.data[0].x = float(ee[0]); p.data[0].y = float(ee[1])
            p.data[0].angle = float(heading)
        set_if("left_arm", arm_msg)

        set_if("ball_in_left_hand", lambda p: setattr(p, "data", bool(grasped)))
        set_if("grasped_ball", lambda p: setattr(p, "data", 1.0 if grasped else 0.0))
        set_if("ball_in_box", lambda p: setattr(p, "data", 1.0 if in_box else 0.0))
        # End-effector height: makes the vertical gap observable now that z is
        # a free action (the 2D sim only had planar x/y perceptions).
        set_if("arm_height", lambda p: setattr(p, "data", float(ee[2])))

        def ball_msg(p):
            p.data[0].x = float(ball[0]); p.data[0].y = float(ball[1])
        set_if("ball", ball_msg)

        def box_msg(p):
            p.data[0].x = float(box[0]); p.data[0].y = float(box[1])
        set_if("box", box_msg)

        def dist_arm_ball(p):
            # 3D distance: z is a free action dimension and the autograsp
            # condition is 3D, so a planar distance would hide the height gap.
            p.data[0].distance = float(np.linalg.norm(np.array(ee) - np.array(ball)))
            p.data[0].angle = self.sim.wrap_angle(
                heading - self.sim.get_relative_angle(ee, ball))
        set_if("dist_left_arm_ball", dist_arm_ball)

        def dist_ball_box(p):
            p.data[0].distance = self.sim.get_distance(ball, box)
            p.data[0].angle = self.sim.get_relative_angle(ball, box)
        set_if("dist_ball_box", dist_ball_box)

        set_if("ball_angle", lambda p: setattr(p, "data", float(ball[0])))  # (it is the ball's x, old name)
        set_if("ball_x", lambda p: setattr(p, "data", float(ball[0])))
        set_if("box_angle", lambda p: setattr(p, "data", float(box[0])))

        # Gripper perceptions (with the magnet scene they refer to the flange)
        def dist_gripper_ball(p):
            # Planar distance and bearing relative to the heading; the height gap is gripper_ball_dz
            p.data[0].distance = self.sim.get_distance(ee, ball)
            p.data[0].angle = self.sim.wrap_angle(heading - self.sim.get_relative_angle(ee, ball))
        set_if("dist_gripper_ball", dist_gripper_ball)
        set_if("gripper_ball_dz", lambda p: setattr(p, "data", float(ee[2] - ball[2])))
        set_if("gripper_closed", lambda p: setattr(p, "data", 1.0 if state.get("closed", grasped) else 0.0))
        # Robot-side readings, as on the real UR5e + 2FG7: pinch height from the arm's kinematics, and the
        # gripper's aperture (m) and finger force (N) as OnRobot's driver reports them
        set_if("gripper_height", lambda p: setattr(p, "data", float(ee[2])))
        set_if("gripper_width", lambda p: setattr(p, "data", float(state.get("aperture", 0.0))))
        set_if("grip_force", lambda p: setattr(p, "data", float(state.get("grip_force", 0.0))))
        set_if("ball_lifted", lambda p: setattr(p, "data", 1.0 if state.get("lifted", False) else 0.0))

    def reset_world(self):
        self.sim.reset(self.rng)

    # ------------------------------------------------------------- actions
    def denormalize_actuation(self, action: Container, actuation_config):
        """Identical to Sim2DSimple.denormalize_actuation."""
        action_dims = action.feature_labels
        data = action.read()
        for dim in action_dims:
            actuator, param = dim.split(":", 1)
            if actuation_config[actuator][param]["type"] == "float":
                bounds = actuation_config[actuator][param]["bounds"]
                value = data.sel(features=dim).values
                data.loc[{"features": dim}] = bounds[0] + (value * (bounds[1] - bounds[0]))
        return data

    def execute_action(self, action):
        vec = self.denormalize_actuation(action, self.actuation_config)
        features = list(vec.coords["features"].values)

        if f"{self.arm_name}:dist" in features:
            dist = float(vec.sel(features=f"{self.arm_name}:dist").values)
            angle = float(vec.sel(features=f"{self.arm_name}:angle").values)
            dz = float(vec.sel(features=f"{self.arm_name}:z").values) \
                if f"{self.arm_name}:z" in features else None
            if f"{self.arm_name}:grip" in features:
                if self.gripper == "none":
                    self.get_logger().error("grip action without a gripper (set Robot: gripper in the yaml)")
                grip = float(vec.sel(features=f"{self.arm_name}:grip").values)
                self.sim.apply_action_polar(angle, dist, dz, grip)
            else:
                self.sim.apply_action_polar(angle, dist, dz)
        elif f"{self.arm_name}:x" in features:
            x = float(vec.sel(features=f"{self.arm_name}:x").values)
            y = float(vec.sel(features=f"{self.arm_name}:y").values)
            z = float(vec.sel(features=f"{self.arm_name}:z").values) \
                if f"{self.arm_name}:z" in features else self.sim.hover_z
            yaw = float(vec.sel(features=f"{self.arm_name}:yaw").values) \
                if f"{self.arm_name}:yaw" in features else self.sim.heading
            self.sim.apply_action_absolute(x, y, z, yaw)
        else:
            self.get_logger().error(f"Unknown action features: {features}")

    # ------------------------------------------------------------- callbacks (as in Sim2DSimple)
    def world_reset_service_callback(self, request, response):
        self.reset_world()
        response.success = True
        return response

    def new_command_callback(self, data):
        self.get_logger().debug(f"Command received... ITERATION: {data.iteration}")
        if data.command == "reset_world" and not self.service_world_reset:
            self.reset_world()
        elif data.command == "end":
            self.get_logger().info("Ending simulator as requested by LTM...")
            rclpy.shutdown()

    def new_action_service_callback(self, request, response):
        action = Container.from_msg(request.action)
        self.get_logger().info("Executing action " + str(action))
        self.execute_action(action)
        response.success = True
        return response

    def publish_perceptions(self):
        self.get_perceptions()
        for ident, publisher in self.sim_publishers.items():
            publisher.publish(self.perceptions[ident])

    # ------------------------------------------------------------- yaml config (as in Sim2DSimple)
    def load_experiment_file_in_commander(self):
        self.load_client = ServiceClient(LoadConfig, 'commander/load_experiment')
        return self.load_client.send_request(file=self.config_file)

    def load_configuration(self):
        if self.config_file is None or not os.path.isfile(self.config_file):
            self.get_logger().error(f"Config file '{self.config_file}' not found!")
            rclpy.shutdown()
            return
        config = yaml.load(open(self.config_file, "r", encoding="utf-8"),
                           Loader=yamlloader.ordereddict.CLoader)
        sim_cfg = config.get("SimulatedUR5", config.get("SimulatedBaxter"))
        self.create_scenario(sim_cfg.get("Robot", {}) or {})
        self.setup_perceptions(sim_cfg["Perceptions"])
        self.actuation_config = sim_cfg["Actuation"]
        self.arm_name = next(iter(self.actuation_config))  # the (only) actuator of the yaml, e.g. left_arm or arm
        self.setup_control_channel(config["Control"])

        self.random_seed = resolve_seed(self.random_seed)
        self.rng = np.random.default_rng(self.random_seed)
        self.get_logger().info(f"RNG seed {self.random_seed}")
        self.reset_world()
        if not self.standalone:
            self.load_experiment_file_in_commander()
        else:
            self.get_logger().info("STANDALONE mode: not contacting the commander")

    def setup_control_channel(self, simulation):
        self.ident = simulation["id"]
        message = class_from_classname(simulation["control_msg"])
        self.create_subscription(message, simulation["control_topic"],
                                 self.new_command_callback, 0)
        service_action = simulation.get("executed_action_service")
        service_world_reset = simulation.get("world_reset_service")
        if simulation.get("executed_policy_topic"):
            raise RuntimeError("Topic-triggered policies not supported; use executed_action_service")
        if service_action:
            msg_srv = class_from_classname(simulation["executed_action_msg"])
            self.create_service(msg_srv, service_action,
                                self.new_action_service_callback,
                                callback_group=self.cbgroup_server)
            # Perception timer in its OWN callback group so it keeps firing
            # while an action is being executed in the server group.
            self.perceptions_timer = self.create_timer(
                1.0 / max(self.perception_rate, 1.0), self.publish_perceptions,
                callback_group=self.cbgroup_pub)
        if service_world_reset:
            self.service_world_reset = True
            msg_reset = class_from_classname(simulation["world_reset_msg"])
            self.create_service(msg_reset, service_world_reset,
                                self.world_reset_service_callback,
                                callback_group=self.cbgroup_server)

    def setup_perceptions(self, perceptions):
        for perception in perceptions:
            sid = perception["name"]
            topic = perception["perception_topic"]
            classname = perception["perception_msg"]
            message = class_from_classname(classname)
            self.perceptions[sid] = message()
            if "List" in classname:
                self.perceptions[sid].data = []
                self.base_messages[sid] = class_from_classname(classname.replace("List", ""))
                self.perceptions[sid].data.append(self.base_messages[sid]())
            elif "Float" in classname:
                self.perceptions[sid].data = 0.0
            else:
                self.perceptions[sid].data = False
            self.get_logger().info("I will publish to... " + str(topic))
            self.sim_publishers[sid] = self.create_publisher(message, topic, 0)


def main(args=None):
    rclpy.init(args=args)
    sim = SimUR5Mujoco()
    sim.load_configuration()
    # Multi-threaded executor: perception timer runs concurrently with the
    # action service, so publishing keeps its rate during arm motions.
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        rclpy.spin(sim, executor=executor)
    except KeyboardInterrupt:
        print('Keyboard Interrupt Detected: Shutting down simulator...')
    finally:
        if sim.sim is not None and hasattr(sim.sim, "close"):
            sim.sim.close()  # stops the physics thread, finishes the video being recorded
        sim.destroy_node()


if __name__ == '__main__':
    main()
