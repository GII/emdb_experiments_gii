from enum import Enum
import importlib
import os.path
import math
import yaml
import yamlloader
import numpy
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor
from core.service_client import ServiceClient

from core_interfaces.srv import LoadConfig
from core.utils import class_from_classname, actuation_msg_to_dict

from simulators.scenarios_2D import SimpleScenario



class Sim2DSimple(Node):
    def __init__(self):
        super().__init__("sim_2d")

        # Setup parameters
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
        # Setup parameters
        visualize = (
            self.declare_parameter("visualize", value=True)
            .get_parameter_value()
            .bool_value
        )
        
        #Callback groups
        self.cbgroup_server=MutuallyExclusiveCallbackGroup()
        self.cbgroup_client=MutuallyExclusiveCallbackGroup()

        # Publishers and perception messages
        self.sim_publishers = {}
        self.perceptions = {}
        self.base_messages = {}

        self.load_client=ServiceClient(LoadConfig, 'commander/load_experiment')

        #Simulator Instance
        self.sim=SimpleScenario(visualize=visualize)

    ##Methods that access the simulation

    def get_perceptions(self):
        left_arm=self.sim.baxter_left.get_pos()
        left_angle=self.sim.baxter_left.get_angle()
        right_arm=self.sim.baxter_right.get_pos()
        right_angle=self.sim.baxter_right.get_angle()
        ball=self.sim.objects[0].get_pos()
        box=self.sim.box1.get_pos()
        left_gripper= bool(self.sim.baxter_left.catched_object)
        right_gripper= bool(self.sim.baxter_right.catched_object)

        self.perceptions["left_arm"].data[0].x = float(left_arm[0])
        self.perceptions["left_arm"].data[0].y = float(left_arm[1])
        self.perceptions["left_arm"].data[0].angle = float(left_angle)
        self.perceptions["ball_in_left_hand"].data = left_gripper

        self.perceptions["right_arm"].data[0].x = float(right_arm[0])
        self.perceptions["right_arm"].data[0].y = float(right_arm[1])
        self.perceptions["right_arm"].data[0].angle = float(right_angle)
        self.perceptions["ball_in_right_hand"].data = right_gripper

        self.perceptions["box"].data[0].x = float(box[0])
        self.perceptions["box"].data[0].y = float(box[1])

        self.perceptions["ball"].data[0].x = float(ball[0])
        self.perceptions["ball"].data[0].y = float(ball[1])

        #Drive input
        if self.sim.box1.contents:
            if self.sim.box1.contents[0].name=="ball_1":
                self.perceptions["ball_in_box"].data = 1.0
            else:
                self.perceptions["ball_in_box"].data = 0.0

    def reset_world(self):
        self.sim.restart_scenario(self.rng)

    def execute_action(self, action):
        vel_l=action["left_arm"][0]["dist"]
        angle_l=action["left_arm"][0]["angle"]
        gripper_l=action["left_arm"][0]["gripper"]

        vel_r=action["right_arm"][0]["dist"]
        angle_r=action["right_arm"][0]["angle"]
        gripper_r=action["right_arm"][0]["gripper"]

        self.sim.apply_action(angle_l, angle_r, vel_l, vel_r, gripper_l, gripper_r)
        

    ##Callbacks for the world reset, action and perception services/topics

    def world_reset_service_callback(self, request, response):
        self.reset_world()
        response.success=True
        return response

    def new_command_callback(self, data):
        """
        Process a command received

        :param data: The message that contais the command received
        :type data: ROS msg defined in setup_control_channel
        """
        self.get_logger().debug(f"Command received... ITERATION: {data.iteration}")
        if data.command == "reset_world":
            self.reset_world(data)
        elif data.command == "end":
            self.get_logger().info("Ending simulator as requested by LTM...")
            rclpy.shutdown()


    def new_action_service_callback(self, request, response):
        """
        Execute a policy and publish new perceptions.

        :param request: The message that contains the policy to execute
        :type request: ROS srv defined in setup_control_channel
        :param response: Response of the success of the execution of the action
        :type response: ROS srv defined in setup_control_channel
        """
        action=actuation_msg_to_dict(request.action)
        self.get_logger().info("Executing action " + str(action))
        self.execute_action(action)
        return response

    def publish_perceptions(self):
        self.get_perceptions()
        for ident, publisher in self.sim_publishers.items():
            self.get_logger().debug("Publishing " + ident + " = " + str(self.perceptions[ident].data))
            publisher.publish(self.perceptions[ident])


    ##SIMULATION CONFIGURATION FROM YAML: THESE METHODS SHOULD BE GENERIC FOR EVERY SIMULATION
    ##TODO: REFACTOR THESE METHODS BELOW INTO A SINGLE CLASS 

    def load_experiment_file_in_commander(self):
        loaded = self.load_client.send_request(file = self.config_file)
        return loaded

    def load_configuration(self):
        """
        Load configuration from a file.

        :param random_seed: The seed for the random numbers generation
        :type random_seed: int
        :param config_file: The file with the params to configurate the simulation
        :type config_file: yaml file
        """
        if self.config_file is None:
            self.get_logger().error("No configuration file for the LTM simulator specified!")
            rclpy.shutdown()
        else:
            if not os.path.isfile(self.config_file):
                self.get_logger().error(self.config_file + " does not exist!")
                rclpy.shutdown()
            else:
                self.get_logger().info(f"Loading configuration from {self.config_file}...")
                config = yaml.load(
                    open(self.config_file, "r", encoding="utf-8"),
                    Loader=yamlloader.ordereddict.CLoader,
                )
                self.setup_perceptions(config["SimulatedBaxter"]["Perceptions"])
                # Be ware, we can not subscribe to control channel before creating all sensor publishers.
                self.setup_control_channel(config["Control"])
        if self.random_seed:
            self.rng = numpy.random.default_rng(self.random_seed)
            self.get_logger().info(f"Setting random number generator with seed {self.random_seed}")
        else:
            self.rng = numpy.random.default_rng()
        
        self.load_experiment_file_in_commander()

    def setup_control_channel(self, simulation):
        """
        Configure the ROS topic/service where listen for commands to be executed.

        :param simulation: The params from the config file to setup the control channel
        :type simulation: dict
        """
        self.ident = simulation["id"]
        topic = simulation["control_topic"]
        classname = simulation["control_msg"]
        message = class_from_classname(classname)
        self.get_logger().info("Subscribing to... " + str(topic))
        self.create_subscription(message, topic, self.new_command_callback, 0)
        topic = simulation.get("executed_policy_topic")
        service_action = simulation.get("executed_action_service")
        service_world_reset = simulation.get("world_reset_service")

        if topic:
            raise RuntimeError("The 2D Simulator is not compatible with topic-triggered policies. Please define a executed_policy_server parameter")
        if service_action:
            self.get_logger().info("Creating server... " + str(service_action))
            classname = simulation["executed_action_msg"]
            message_action_srv = class_from_classname(classname)
            self.create_service(message_action_srv, service_action, self.new_action_service_callback, callback_group=self.cbgroup_server)
            self.get_logger().info("Creating perception publisher timer... ")
            self.perceptions_timer = self.create_timer(0.01, self.publish_perceptions, callback_group=self.cbgroup_server)
        if service_world_reset:
            classname= simulation["world_reset_msg"]
            self.message_world_reset = class_from_classname(classname)
            self.create_service(self.message_world_reset, service_world_reset, self.world_reset_service_callback, callback_group=self.cbgroup_server)

    def setup_perceptions(self, perceptions):
        """
        Configure the ROS publishers where publish perception values.

        :param perceptions: The params from the config file to setup the perceptions
        :type perceptions: dict
        """
        for perception in perceptions:
            sid = perception["name"]
            topic = perception["perception_topic"]
            classname = perception["perception_msg"]
            message = class_from_classname(classname)
            self.perceptions[sid] = message()
            if "List" in classname:
                self.perceptions[sid].data = []
                self.base_messages[sid] = class_from_classname(classname.replace("List", "")) #TODO: Change this so that is easier to exchange message types
                self.perceptions[sid].data.append(self.base_messages[sid]())
            elif "Float" in classname:
                self.perceptions[sid].data = 0.0
            else:
                self.perceptions[sid].data = False
            self.get_logger().info("I will publish to... " + str(topic))
            self.sim_publishers[sid] = self.create_publisher(message, topic, 0) #TODO: ¿latch in ROS2?


def main(args=None):
    rclpy.init(args=args)
    sim = Sim2DSimple()
    sim.load_configuration()

    try:
        rclpy.spin(sim)
    except KeyboardInterrupt:
        print('Keyboard Interrupt Detected: Shutting down simulator...')
    finally:
        sim.destroy_node()


if __name__ == '__main__':
    main()
