import os
import os.path
import yaml
import yamlloader
import numpy as np
import rclpy
import matplotlib
# As in scenarios_2D: it needs no display unless a backend is asked for with MPLBACKEND
if not os.environ.get('MPLBACKEND'):
    matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
try:
    # Event-driven executor implemented in C++ (ROS 2 Jazzy and later).
    from rclpy.experimental import EventsExecutor
except ImportError:
    EventsExecutor = None
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor
from core.service_client import ServiceClient
from core.blocking_wait_check import register_executor_thread

from core_interfaces.srv import LoadConfig
from core.container import Container
from core.utils import class_from_classname, resolve_seed


class Slider1DScenario:
    """
    Minimal 1-D reach task.

    The agent is a scalar position in the range given by ``BOUNDS`` (0 to 1000). At each step the agent
    applies a velocity, and the resulting position is clamped to stay in bounds. The target position is
    randomized on every reset. The target is reached when the agent is closer to it than ``SUCCESS_RADIUS``
    (50 units).
    """

    BOUNDS = (0.0, 1000.0)
    SUCCESS_RADIUS = 50.0

    def __init__(self):
        """
        Constructor for the Slider1DScenario class. The agent and the target start at the middle of the range (500.0)
        until :meth:`reset` is called.
        """
        self.agent_pos = 500.0
        self.target_pos = 500.0

    # ------------------------------------------------------------------
    def reset(self, rng: np.random.Generator):
        """
        Randomize the positions of the agent and of the target, with a uniform distribution over ``BOUNDS``.

        :param rng: The random number generator used to draw both positions.
        :type rng: numpy.random.Generator
        """
        lo, hi = self.BOUNDS
        self.agent_pos  = float(rng.uniform(lo, hi))
        self.target_pos = float(rng.uniform(lo, hi))

    # ------------------------------------------------------------------
    def apply_action(self, velocity: float):
        """
        Move the agent by the given velocity, keeping its position within ``BOUNDS``.

        :param velocity: Displacement of the agent, in position units. It can be negative.
        :type velocity: float
        """
        lo, hi = self.BOUNDS
        self.agent_pos = float(np.clip(self.agent_pos + velocity, lo, hi))

    # ------------------------------------------------------------------
    @property
    def signed_distance(self) -> float:
        """
        Signed distance from the agent to the target (``target_pos - agent_pos``), in the range [-1000, 1000].

        :return: The signed distance.
        :rtype: float
        """
        return self.target_pos - self.agent_pos

    @property
    def reached(self) -> bool:
        """
        Whether the target has been reached: the absolute value of the signed distance is lower than ``SUCCESS_RADIUS``.

        :return: True if the agent is within the success radius of the target.
        :rtype: bool
        """
        return abs(self.signed_distance) < self.SUCCESS_RADIUS


# ======================================================================

class Sim1DSlider(Node):
    """
    Standalone ROS 2 simulator node for the 1-D slider task (see :class:`Slider1DScenario`).

    It reads its configuration from the ``Simulator1D`` section of the experiment YAML (the same file that is sent to the
    commander) and from the ``Control`` section. The ROS parameters of the node are ``random_seed``, ``config_file``
    (path of the experiment file) and ``visualize`` (whether to show the matplotlib window).

    It publishes the sensors listed in ``Simulator1D.Perceptions`` as ``std_msgs/msg/Float32``. In the provided experiment
    they are:

    - ``/mdb/slider/sensor/dist_to_target``: signed distance from the agent to the target (not normalized).
    - ``/mdb/slider/sensor/reached_target``: 1.0 if the target is reached, 0.0 otherwise.

    It offers two services, whose names and types are taken from the ``Control`` section:

    - ``executed_action_service`` (``/mdb/slider/executed_action``, :class:`cognitive_node_interfaces.srv.Action`): executes an action.
    - ``world_reset_service`` (``/mdb/slider/world_reset``, :class:`cognitive_processes_interfaces.srv.WorldReset`): resets the world.
    """

    def __init__(self):
        """
        Constructor for the Sim1DSlider class. It declares the ``random_seed``, ``config_file`` and ``visualize`` parameters,
        creates the scenario and the client of the commander service used to load the experiment.
        """
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

        self.load_client = ServiceClient(LoadConfig, "commander/load_experiment")

        self.scenario = Slider1DScenario()

    # ------------------------------------------------------------------
    # Simulation access
    # ------------------------------------------------------------------

    def get_perceptions(self):
        """
        Copy the state of the scenario into the perception messages: ``dist_to_target`` takes the signed distance and
        ``reached_target`` takes 1.0 or 0.0.
        """
        self.perceptions["dist_to_target"].data  = self.scenario.signed_distance
        self.perceptions["reached_target"].data  = 1.0 if self.scenario.reached else 0.0

    def reset_world(self):
        """
        Randomize the scenario (see :meth:`Slider1DScenario.reset`) and, if the visualization is enabled, redraw it.
        """
        self.scenario.reset(self.rng)
        if self.visualize:
            self._update_visualization()

    def denormalize_actuation(self, action: Container, actuation_config):
        """
        Rescale the values of an action from [0, 1] to the physical bounds of the actuation configuration.

        Every feature of the action is labeled ``<actuator>:<parameter>``. The features whose parameter is of type ``float`` in
        ``actuation_config`` are rescaled as ``min + value * (max - min)``, with the ``bounds`` of the parameter.

        :param action: The action to rescale.
        :type action: core.container.Container
        :param actuation_config: The ``Simulator1D.Actuation`` section of the experiment file: for each actuator, its parameters with ``type`` and ``bounds``.
        :type actuation_config: dict
        :return: The data of the action, with the float parameters in physical units.
        :rtype: xarray.DataArray
        """
        data = action.read()
        for dim in action.feature_labels:
            actuator, param = dim.split(":", 1)
            if actuation_config[actuator][param]["type"] == "float":
                bounds = actuation_config[actuator][param]["bounds"]
                value  = data.sel(features=dim).values
                data.loc[{"features": dim}] = bounds[0] + value * (bounds[1] - bounds[0])
        return data

    def execute_action(self, action: Container):
        """
        Rescale the action and apply the ``agent:velocity`` of its last row to the scenario. If the visualization is enabled, it is redrawn.

        :param action: The action to execute, with values in [0, 1].
        :type action: core.container.Container
        """
        data = self.denormalize_actuation(action, self.actuation_config)
        velocity = float(data.sel(features="agent:velocity").values[-1])
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
        """
        Callback of the world reset service. It resets the world with :meth:`reset_world`.

        :param request: The request to reset the world.
        :type request: cognitive_processes_interfaces.srv.WorldReset.Request
        :param response: The response of the service.
        :type response: cognitive_processes_interfaces.srv.WorldReset.Response
        :return: The response, with ``success`` set to True.
        :rtype: cognitive_processes_interfaces.srv.WorldReset.Response
        """
        self.reset_world()
        response.success = True
        return response

    def new_command_callback(self, data):
        """
        Callback of the control topic. When the command is ``end``, the simulator shuts down.

        :param data: Control message, with the iteration and the command.
        :type data: cognitive_processes_interfaces.msg.ControlMsg
        """
        self.get_logger().debug(f"Command received... ITERATION: {data.iteration}")
        # World resets arrive through the world reset service, not through this topic
        if data.command == "end":
            self.get_logger().info("Ending simulator as requested by LTM...")
            rclpy.shutdown()

    def new_action_service_callback(self, request, response):
        """
        Callback of the executed action service. It builds the action from the request and executes it with :meth:`execute_action`.

        :param request: The request, with the action to execute.
        :type request: cognitive_node_interfaces.srv.Action.Request
        :param response: The response of the service.
        :type response: cognitive_node_interfaces.srv.Action.Response
        :return: The response, with ``success`` set to True.
        :rtype: cognitive_node_interfaces.srv.Action.Response
        """
        action = Container.from_msg(request.action)
        self.get_logger().info("Executing action " + str(action))
        self.execute_action(action)
        response.success = True
        return response

    def publish_perceptions(self):
        """
        Update the perceptions from the scenario (see :meth:`get_perceptions`) and publish all of them. It is called by a timer every 0.01 s.
        """
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
        """
        Ask the commander to load the experiment file.

        :return: The result of the request.
        """
        loaded = self.load_client.send_request(file=self.config_file)
        return loaded

    def load_configuration(self):
        """
        Read the experiment file given in ``config_file`` and configure the simulator.

        It creates the perceptions (``Simulator1D.Perceptions``), reads the actuation configuration (``Simulator1D.Actuation``),
        sets up the control channel (``Control``), creates the random number generator from ``random_seed`` and randomizes the scenario.
        If the visualization is enabled, it creates the window. Finally, it asks the commander to load the same file. If no
        file is given or it does not exist, the node is shut down.
        """
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
        """
        Create the subscription to the control topic and the services of the simulator.

        The action service and the timer that publishes the perceptions are created if ``executed_action_service`` is in the
        configuration, and the world reset service if ``world_reset_service`` is.

        :param simulation: The ``Control`` section of the experiment file.
        :type simulation: dict
        """
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
            classname_reset          = simulation["world_reset_msg"]
            self.message_world_reset = class_from_classname(classname_reset)
            self.create_service(
                self.message_world_reset,
                service_world_reset,
                self.world_reset_service_callback,
                callback_group=self.cbgroup_server,
            )

    def setup_perceptions(self, perceptions: list):
        """
        Create the messages and the publishers of the perceptions.

        The initial value of each perception depends on its message type: 0.0 for Float types, an empty list for List types and False otherwise.

        :param perceptions: The ``Simulator1D.Perceptions`` section of the experiment file: a list with the ``name``, ``perception_topic`` and ``perception_msg`` of each perception.
        :type perceptions: list
        """
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
    """
    Entry point of the ``simple_1d_sim`` executable. It creates the simulator, loads its configuration and spins it until it is shut down.

    :param args: Command line arguments passed to ``rclpy.init``.
    :type args: list
    """
    rclpy.init(args=args)
    sim = Sim1DSlider()
    sim.load_configuration()
    # Much cheaper than the Python executors. The simulator runs in a single thread either way, and
    # none of its callbacks waits for anything, so callback groups are not needed.
    executor = EventsExecutor() if EventsExecutor is not None else SingleThreadedExecutor()
    executor.add_node(sim)
    register_executor_thread() # The callbacks of the EventsExecutor are not visible in the Python stack

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Keyboard Interrupt Detected: Shutting down simulator...")
    finally:
        sim.destroy_node()


if __name__ == "__main__":
    main()
