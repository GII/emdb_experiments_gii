import os.path
import yaml
import yamlloader
import numpy as np
import rclpy
from copy import deepcopy
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor
from core.service_client import ServiceClient

from core_interfaces.srv import LoadConfig
from core.utils import class_from_classname, actuation_msg_to_dict, resolve_seed


class Slider1DScenario:
    """
    Minimal 1-D reach task.

    The agent is a scalar position on [0, 1000].  At each step the agent
    applies a velocity that is clamped so the agent stays in bounds.  The
    target position is randomised on every reset.  Success is declared when
    the agent comes within ``success_radius`` units of the target.
    """

    BOUNDS = (0.0, 1000.0)
    SUCCESS_RADIUS = 50.0

    def __init__(self):
        self.agent_pos = 500.0
        self.target_pos = 500.0

    # ------------------------------------------------------------------
    def reset(self, rng: np.random.Generator):
        """Randomise agent and target positions."""
        lo, hi = self.BOUNDS
        self.agent_pos  = float(rng.uniform(lo, hi))
        self.target_pos = float(rng.uniform(lo, hi))

    # ------------------------------------------------------------------
    def apply_action(self, velocity: float):
        """Move the agent by *velocity* units, clamped to bounds."""
        lo, hi = self.BOUNDS
        self.agent_pos = float(np.clip(self.agent_pos + velocity, lo, hi))

    # ------------------------------------------------------------------
    @property
    def signed_distance(self) -> float:
        """target_pos - agent_pos  ∈ [-1000, 1000]."""
        return self.target_pos - self.agent_pos

    @property
    def reached(self) -> bool:
        return abs(self.signed_distance) < self.SUCCESS_RADIUS


# ======================================================================

class Sim1DSlider(Node):
    """
    Minimal standalone ROS2 simulator node for the 1-D slider task.

    Reads its configuration from the ``Simulator1D`` section of the
    experiment YAML (same file passed to the commander).  Publishes two
    Float32 sensors:

    * ``/mdb/slider/sensor/dist_to_target``  — signed distance (raw)
    * ``/mdb/slider/sensor/reached_target``  — 1.0 if reached, else 0.0

    Exposes two ROS2 services mirroring the 2-D simulator API:

    * ``/mdb/slider/executed_action``   (cognitive_node_interfaces/srv/Action)
    * ``/mdb/slider/world_reset``       (cognitive_processes_interfaces/srv/WorldReset)
    """

    def __init__(self):
        super().__init__("sim_1d_slider")

        self.random_seed = (
            self.declare_parameter("random_seed", value=0)
            .get_parameter_value()
            .integer_value
        )
        self.config_file = (
            self.declare_parameter(
                "config_file", descriptor=ParameterDescriptor(dynamic_typing=True)
            )
            .get_parameter_value()
            .string_value
        )
        self.visualize = (
            self.declare_parameter("visualize", value=True)
            .get_parameter_value()
            .bool_value
        )

        self.cbgroup_server = MutuallyExclusiveCallbackGroup()
        self.cbgroup_client = MutuallyExclusiveCallbackGroup()

        self.sim_publishers  = {}
        self.perceptions     = {}
        self.base_messages   = {}
        self.service_world_reset = False

        self.load_client = ServiceClient(LoadConfig, "commander/load_experiment")

        self.scenario = Slider1DScenario()

    # ------------------------------------------------------------------
    # Simulation access
    # ------------------------------------------------------------------

    def get_perceptions(self):
        """Push current scenario state into the ROS perception messages."""
        self.perceptions["dist_to_target"].data  = self.scenario.signed_distance
        self.perceptions["reached_target"].data  = 1.0 if self.scenario.reached else 0.0

    def reset_world(self):
        """Randomise the scenario."""
        self.scenario.reset(self.rng)
        if self.visualize:
            self._update_visualization()

    def denormalize_actuation(self, actuation, actuation_config):
        """Rescale policy [0, 1] outputs back to physical units."""
        act = deepcopy(actuation)
        for actuator in act:
            for param in act[actuator][0]:
                if actuation_config[actuator][param]["type"] == "float":
                    bounds = actuation_config[actuator][param]["bounds"]
                    value  = act[actuator][0][param]
                    act[actuator][0][param] = bounds[0] + value * (bounds[1] - bounds[0])
        return act

    def execute_action(self, action: dict):
        """Denormalise and apply the action to the scenario."""
        action = self.denormalize_actuation(action, self.actuation_config)
        velocity = action["agent"][0]["velocity"]
        self.scenario.apply_action(velocity)
        if self.visualize:
            self._update_visualization()

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------

    def _setup_visualization(self):
        """Create the matplotlib window for the 1-D slider."""
        plt.rcParams['toolbar'] = 'None'
        plt.ioff()
        self._fig, self._ax = plt.subplots(figsize=(9, 3))
        self._fig.canvas.manager.set_window_title('1-D Slider Simulator')

        lo, hi = self.scenario.BOUNDS
        r = self.scenario.SUCCESS_RADIUS

        # Static elements
        self._ax.set_xlim(lo - 50, hi + 50)
        self._ax.set_ylim(-1, 1)
        self._ax.set_yticks([])
        self._ax.set_xlabel('Position')
        self._ax.set_title('1-D Slider')
        self._ax.axhline(0, color='black', linewidth=1.5)
        self._ax.axvline(lo, color='grey', linestyle='--', linewidth=1)
        self._ax.axvline(hi, color='grey', linestyle='--', linewidth=1)

        # Dynamic elements (will be updated each step)
        self._success_band = mpatches.Rectangle(
            (self.scenario.target_pos - r, -1), 2 * r, 2,
            alpha=0.25, color='green', label='success zone (\u00b150)'
        )
        self._ax.add_patch(self._success_band)
        self._target_line = self._ax.axvline(
            self.scenario.target_pos, color='red', linewidth=2, label='target'
        )
        self._agent_marker, = self._ax.plot(
            [self.scenario.agent_pos], [0], 'o',
            color='royalblue', markersize=14, label='agent'
        )
        self._info_text = self._ax.text(
            lo + 10, 0.75, '', fontsize=9, color='black'
        )
        self._ax.legend(loc='upper right', fontsize=8)
        self._fig.tight_layout()
        self._fig.canvas.draw()
        plt.pause(0.001)

    def _update_visualization(self):
        """Redraw the visualization with current scenario state."""
        r   = self.scenario.SUCCESS_RADIUS
        tgt = self.scenario.target_pos
        agt = self.scenario.agent_pos
        dist = self.scenario.signed_distance
        reached = self.scenario.reached

        # Update target line + success band
        self._target_line.set_xdata([tgt, tgt])
        self._success_band.set_x(tgt - r)

        # Update agent
        self._agent_marker.set_xdata([agt])
        self._agent_marker.set_color('green' if reached else 'royalblue')

        # Update info text
        status = 'REACHED' if reached else f'dist={dist:+.1f}'
        self._info_text.set_text(f'agent={agt:.1f}  target={tgt:.1f}  {status}')

        self._fig.canvas.draw()
        plt.pause(0.001)

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def world_reset_service_callback(self, request, response):
        self.reset_world()
        response.success = True
        return response

    def new_command_callback(self, data):
        self.get_logger().debug(f"Command received... ITERATION: {data.iteration}")
        if data.command == "reset_world":
            if not self.service_world_reset:
                self.reset_world()
        elif data.command == "end":
            self.get_logger().info("Ending simulator as requested by LTM...")
            rclpy.shutdown()

    def new_action_service_callback(self, request, response):
        action = actuation_msg_to_dict(request.action)
        self.get_logger().info("Executing action " + str(action))
        self.execute_action(action)
        return response

    def publish_perceptions(self):
        self.get_perceptions()
        for ident, publisher in self.sim_publishers.items():
            self.get_logger().debug(
                "Publishing " + ident + " = " + str(self.perceptions[ident].data)
            )
            publisher.publish(self.perceptions[ident])

    # ------------------------------------------------------------------
    # Configuration loading
    # ------------------------------------------------------------------

    def load_experiment_file_in_commander(self):
        loaded = self.load_client.send_request(file=self.config_file)
        return loaded

    def load_configuration(self):
        if not self.config_file:
            self.get_logger().error("No configuration file specified!")
            rclpy.shutdown()
            return
        if not os.path.isfile(self.config_file):
            self.get_logger().error(self.config_file + " does not exist!")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Loading configuration from {self.config_file}...")
        config = yaml.load(
            open(self.config_file, "r", encoding="utf-8"),
            Loader=yamlloader.ordereddict.CLoader,
        )

        self.setup_perceptions(config["Simulator1D"]["Perceptions"])
        self.actuation_config = config["Simulator1D"]["Actuation"]
        self.setup_control_channel(config["Control"])

        self.random_seed = resolve_seed(self.random_seed)
        self.rng = np.random.default_rng(self.random_seed)
        self.get_logger().info(
            f"Setting random number generator with seed {self.random_seed}"
        )

        # Randomise the scenario before the first episode
        self.scenario.reset(self.rng)

        if self.visualize:
            self._setup_visualization()

        self.load_experiment_file_in_commander()

    def setup_control_channel(self, simulation: dict):
        self.ident = simulation["id"]
        topic     = simulation["control_topic"]
        classname = simulation["control_msg"]
        message   = class_from_classname(classname)
        self.get_logger().info("Subscribing to... " + str(topic))
        self.create_subscription(message, topic, self.new_command_callback, 0)

        service_action      = simulation.get("executed_action_service")
        service_world_reset = simulation.get("world_reset_service")

        if service_action:
            self.get_logger().info("Creating action server... " + str(service_action))
            classname_action = simulation["executed_action_msg"]
            message_action   = class_from_classname(classname_action)
            self.create_service(
                message_action,
                service_action,
                self.new_action_service_callback,
                callback_group=self.cbgroup_server,
            )
            self.get_logger().info("Creating perception publisher timer...")
            self.perceptions_timer = self.create_timer(
                0.01, self.publish_perceptions, callback_group=self.cbgroup_server
            )
        if service_world_reset:
            self.service_world_reset = True
            classname_reset          = simulation["world_reset_msg"]
            self.message_world_reset = class_from_classname(classname_reset)
            self.create_service(
                self.message_world_reset,
                service_world_reset,
                self.world_reset_service_callback,
                callback_group=self.cbgroup_server,
            )

    def setup_perceptions(self, perceptions: list):
        for perception in perceptions:
            sid       = perception["name"]
            topic     = perception["perception_topic"]
            classname = perception["perception_msg"]
            message   = class_from_classname(classname)
            self.perceptions[sid] = message()
            if "List" in classname:
                self.perceptions[sid].data = []
                self.base_messages[sid]    = class_from_classname(
                    classname.replace("List", "")
                )
                self.perceptions[sid].data.append(self.base_messages[sid]())
            elif "Float" in classname:
                self.perceptions[sid].data = 0.0
            else:
                self.perceptions[sid].data = False
            self.get_logger().info("Publishing to... " + str(topic))
            self.sim_publishers[sid] = self.create_publisher(message, topic, 0)


def main(args=None):
    rclpy.init(args=args)
    sim = Sim1DSlider()
    sim.load_configuration()

    try:
        rclpy.spin(sim)
    except KeyboardInterrupt:
        print("Keyboard Interrupt Detected: Shutting down simulator...")
    finally:
        sim.destroy_node()


if __name__ == "__main__":
    main()
