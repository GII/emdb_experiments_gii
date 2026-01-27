import numpy as np
from math import pi
import yaml
import yamlloader
import os

# ros libraries
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future


# Interfaces
from rcl_interfaces.msg import ParameterDescriptor
from trajectory_msgs.msg import JointTrajectoryPoint
from oscar_interfaces.srv import ArmControl, GripperControl
from oscar_emdb_interfaces.srv import PerceptionMultiObj as PerceptionSrv
from oscar_emdb_interfaces.msg import PerceptionMultiObj
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import Pose, Quaternion, Point
from core_interfaces.srv import LoadConfig
from std_msgs.msg import Float32

# Utils
from core.service_client import ServiceClient, ServiceClientAsync
from core.utils import class_from_classname


class OscarMDB(Node):
    """
    This class provides a node that interfaces the OSCAR robot with the eMDB Cognitive Architecture
    """

    def __init__(self):
        """
        Constructor of the OscarMDB class
        """
        super().__init__("oscar_emdb_server")

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

        # Publishers and perception messages
        self.sim_publishers = {}
        self.perceptions = {}

        # Local variables init
        self.perception = PerceptionSrv.Response()
        self.old_perception = PerceptionSrv.Response()
        self.aprox_object = False
        self.base_messages = {}

        # Callback groups
        self.gazebo_cbg = MutuallyExclusiveCallbackGroup()
        self.oscar_cbg = MutuallyExclusiveCallbackGroup()
        self.perception_cbg = MutuallyExclusiveCallbackGroup()
        self.mdb_commands_cbg = MutuallyExclusiveCallbackGroup()

        # Service client for Oscar Perceptions
        self.cli_oscar_perception = ServiceClientAsync(
            self, PerceptionSrv, "oscar/request_perceptions", self.perception_cbg
        )

        # Service clients for Oscar Commander
        self.cli_right_arm = ServiceClientAsync(
            self, ArmControl, "oscar/right_arm_command", self.oscar_cbg
        )
        self.cli_left_arm = ServiceClientAsync(
            self, ArmControl, "oscar/left_arm_command", self.oscar_cbg
        )
        self.cli_left_gripper = ServiceClientAsync(
            self, GripperControl, "oscar/left_gripper_command", self.oscar_cbg
        )
        self.cli_right_gripper = ServiceClientAsync(
            self, GripperControl, "oscar/right_gripper_command", self.oscar_cbg
        )

        # Service client for Gazebo
        self.cli_set_state = self.create_client(
            SetEntityState, "set_entity_state", callback_group=self.gazebo_cbg
        )
        while not self.cli_set_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")

        # Service client for eMBD Commander
        self.cli_mdb_commander = ServiceClient(LoadConfig, "commander/load_experiment")

        # Reward Publisher
        self.reward_pub = self.create_publisher(
            Float32, "mdb/reward", 1
        )  # TODO: Implement dedicated interface

        self.x_object_close_limits = [0.2575, 0.3625]
        self.y_object_close_limits = [-0.458, 0.458]
        self.x_object_limits = [0.1125, 0.7375]
        self.y_object_limits = [-0.7415, 0.7415]
        self.x_basket_limits = [0.25, 0.35]
        self.y_basket_limits = [-0.55, 0.55]

    def load_configuration(self):
        """
        Load configuration from a file.
        """

        if self.config_file is None:
            self.get_logger().error(
                "No configuration file for the experiment specified!"
            )
            rclpy.shutdown()
        else:
            if not os.path.isfile(self.config_file):
                self.get_logger().error(self.config_file + " does not exist!")
                rclpy.shutdown()
            else:
                self.get_logger().info(
                    f"Loading configuration from {self.config_file}..."
                )
                config = yaml.load(
                    open(self.config_file, "r", encoding="utf-8"),
                    Loader=yamlloader.ordereddict.CLoader,
                )
                self.setup_perceptions(config["OSCAR"]["Perceptions"])
                # Be ware, we can not subscribe to control channel before creating all sensor publishers.
                self.setup_control_channel(config["Control"])
        if self.random_seed:
            self.rng = np.random.default_rng(self.random_seed)
            self.get_logger().info(
                f"Setting random number generator with seed {self.random_seed}"
            )
        else:
            self.rng = np.random.default_rng()

        self.load_experiment_file_in_commander()

    def setup_perceptions(self, perceptions):
        """
        This method creates publishers for each element of the perception list
        passed.

        :param perceptions: List of perceptions.
        :type perceptions: list
        """
        for perception in perceptions:
            sid = perception["name"]
            topic = perception["perception_topic"]
            classname = perception["perception_msg"]
            message = class_from_classname(classname)
            self.perceptions[sid] = message()
            if "List" in classname:
                self.perceptions[sid].data = []
                self.base_messages[sid] = class_from_classname(
                    classname.replace("List", "")
                )
                self.perceptions[sid].data.append(self.base_messages[sid]())
            elif "Float" in classname:
                self.perceptions[sid].data = 0.0
            else:
                self.perceptions[sid].data = False
            self.get_logger().info("I will publish to... " + str(topic))
            self.sim_publishers[sid] = self.create_publisher(
                message, topic, 0
            )  # TODO: ¿latch in ROS2?

    def setup_control_channel(self, simulation):
        """
        Configure the ROS topic/service where listen for commands to be executed.

        :param simulation: The params from the config file to setup the control channel.
        :type simulation: dict
        """
        self.ident = simulation["id"]
        topic = simulation["control_topic"]
        classname = simulation["control_msg"]
        message = class_from_classname(classname)
        self.get_logger().info("Subscribing to... " + str(topic))
        self.create_subscription(message, topic, self.new_command_callback, 0)
        topic = simulation.get("executed_policy_topic")
        service_policy = simulation.get("executed_policy_service")
        service_world_reset = simulation.get("world_reset_service")
        if topic:
            self.get_logger().info("Subscribing to... " + str(topic))
            self.create_subscription(message, topic, self.policy_callback, 0)
        if service_policy:
            self.get_logger().info("Creating server... " + str(service_policy))
            classname = simulation["executed_policy_msg"]
            message_policy_srv = class_from_classname(classname)
            self.create_service(
                message_policy_srv,
                service_policy,
                self.policy_service,
                callback_group=self.mdb_commands_cbg,
            )
            self.get_logger().info("Creating perception publisher timer... ")
            # Subscriber for the redescribed perceptions
            self.sensors_subs = self.create_subscription(
            PerceptionMultiObj,
            "/oscar/redescribed_sensors",
            self.publish_perceptions_callback,
            1)

        if service_world_reset:
            classname= simulation["executed_policy_msg"]
            self.message_world_reset = class_from_classname(simulation["world_reset_msg"])
            self.create_service(self.message_world_reset, service_world_reset, self.world_reset_service_callback, callback_group=self.mdb_commands_cbg)   
        

    def load_experiment_file_in_commander(self):
        """
        Load the configuration file in the commander node.

        :return: Response from the commander node indicating the success of the loading.
        :rtype: core_interfaces.srv.LoadConfig.Response
        """
        loaded = self.cli_mdb_commander.send_request(file=self.config_file)
        return loaded

    async def new_command_callback(self, data):
        """
        Process a command received

        :param data: The message that contais the command received.
        :type data: ROS msg defined in the config file. Typically cognitive_processes_interfaces.msg.ControlMsg
        """
        self.get_logger().debug(f"Command received... ITERATION: {data.iteration}")
        if data.command == "reset_world":
            await self.reset_world()
        elif data.command == "end":
            self.get_logger().info("Ending experiment as requested by LTM...")
            rclpy.shutdown()

    async def policy_callback(self, data):
        """
        Generic method that executes a policy according to the data
        published in a topic.

        :param data: Message with the name of the policy to be executed.
        :type data: ROS msg defined in the config file. 
        """
        self.get_logger().info(f"Executing {data.data} policy...")
        await self.update_perceptions()
        await getattr(self, data.data + "_policy")()
        await self.update_perceptions()
        await self.update_reward_sensor()
        self.publish_perception_reward()

    async def policy_service(self, request, response):
        """
        Generic method that executes a policy according to a service request.

        :param request: Message with the name of the policy to be executed.
        :type request: cognitive_node_interfaces.srv.Policy.Request
        :param response: Message with execution success information.
        :type response: cognitive_node_interfaces.srv.Policy.Response
        :return: Message with execution success information.
        :rtype: cognitive_node_interfaces.srv.Policy.Response
        """
        self.get_logger().info(f"Executing {request.policy} policy...")
        await self.update_perceptions()
        await getattr(self, request.policy + "_policy")()
        await self.update_perceptions()
        self.update_reward_sensor()
        self.publish_perceptions(self.perceptions)
        # self.publish_reward()
        response.success = True
        return response
    
    async def world_reset_service_callback(self, request, response):
        """
        Callback for the world reset service.

        :param request: The message that contains the request to reset the world.
        :type request: ROS msg defined in the config file Typically cognitive_processes_interfaces.srv.WorldReset.Request
        :param response: Response of the world reset service.
        :type response: ROS msg defined in the config file. Typically cognitive_processes_interfaces.srv.WorldReset.Response
        :return: Response indicating the success of the world reset.
        :rtype: ROS msg defined in the config file. Typically cognitive_processes_interfaces.srv.WorldReset.Response
        """
        await self.reset_world()
        response.success=True
        return response


    async def reset_world(self):
        """
        This method initializes the world. Moves the robot to home position
        and randomizes the position of the objects.
        """
        self.get_logger().info("Reseting World...")

        await self.random_positions()
        # Go Home
        await self.init_oscar()
        await self.update_perceptions()
        self.update_reward_sensor()

    async def init_oscar(self):
        """
        Moves OSCAR to home position with the grippers open.
        """
        self.get_logger().info("Initializing OSCAR Robot")
        await self.cli_left_arm.send_request_async(
            x=0.0, y=0.0, z=0.0, vel=0.15, named_pose="home"
        )
        await self.cli_right_arm.send_request_async(
            x=0.0, y=0.0, z=0.0, vel=0.15, named_pose="home"
        )
        await self.cli_right_gripper.send_request_async(close=False)
        await self.cli_left_gripper.send_request_async(close=False)

    async def grasp_right_policy(self):
        """
        This policy grasps the object with the right arm if it is within reach.
        """
        if (
            not self.perception.obj_in_left_hand
            and not self.perception.obj_in_left_hand
        ):
            pick_point = self.perception.red_object

            # Check if object is reachable
            plan = await self.cli_right_arm.send_request_async(
                x=pick_point.x, y=pick_point.y, z=pick_point.z, vel=0.0, named_pose=""
            )

            if plan.success:

                # Go Home and open the gripper
                await self.cli_right_arm.send_request_async(
                    x=0.0, y=0.0, z=0.0, vel=0.15, named_pose="home"
                )
                await self.cli_right_gripper.send_request_async(close=False)

                # Go above object
                await self.cli_right_arm.send_request_async(
                    x=pick_point.x, y=pick_point.y, z=0.9, vel=0.15, named_pose=""
                )

                # Grasp object
                await self.cli_right_arm.send_request_async(
                    x=pick_point.x, y=pick_point.y, z=0.8, vel=0.15, named_pose=""
                )
                await self.cli_right_gripper.send_request_async(close=True)

                # Lift Object
                await self.cli_right_arm.send_request_async(
                    x=pick_point.x, y=pick_point.y, z=0.9, vel=0.15, named_pose=""
                )

                # Go Home
                await self.cli_right_arm.send_request_async(
                    x=0.0, y=0.0, z=0.0, vel=0.15, named_pose="home"
                )

    async def grasp_left_policy(self):
        """
        This policy grasps the object with the left arm if it is within reach.
        """
        if (
            not self.perception.obj_in_left_hand
            and not self.perception.obj_in_left_hand
        ):
            pick_point = self.perception.red_object

            # Check if object is reachable
            plan = await self.cli_left_arm.send_request_async(
                x=pick_point.x, y=pick_point.y, z=pick_point.z, vel=0.0, named_pose=""
            )

            if plan.success:
                # Go Home and open the gripper
                await self.cli_left_arm.send_request_async(
                    x=0.0, y=0.0, z=0.0, vel=0.15, named_pose="home"
                )
                await self.cli_left_gripper.send_request_async(close=False)

                # Go above object
                await self.cli_left_arm.send_request_async(
                    x=pick_point.x, y=pick_point.y, z=0.85, vel=0.15, named_pose=""
                )

                # Grasp object
                await self.cli_left_arm.send_request_async(
                    x=pick_point.x, y=pick_point.y, z=0.8, vel=0.15, named_pose=""
                )
                await self.cli_left_gripper.send_request_async(close=True)

                # Lift Object
                await self.cli_left_arm.send_request_async(
                    x=pick_point.x, y=pick_point.y, z=0.85, vel=0.15, named_pose=""
                )

                # Go Home
                await self.cli_left_arm.send_request_async(
                    x=0.0, y=0.0, z=0.0, vel=0.15, named_pose="home"
                )

    async def press_button_policy(self):
        """
        This policy makes OSCAR press the button on the left. If the object is out of reach
        it is moved closer to the robot.
        """
        if (
            not self.perception.obj_in_left_hand
            and not self.perception.obj_in_right_hand
        ):
            # Go Home and close the gripper
            await self.cli_left_arm.send_request_async(
                x=0.0, y=0.0, z=0.0, vel=0.15, named_pose="home"
            )
            await self.cli_left_gripper.send_request_async(close=True)

            # Go above the button
            await self.cli_left_arm.send_request_async(
                x=0.12, y=0.5, z=0.85, vel=0.15, named_pose=""
            )

            # Press the button
            await self.cli_left_arm.send_request_async(
                x=0.12, y=0.5, z=0.8, vel=0.15, named_pose=""
            )

            # Check if object is reachable
            pick_point = self.perception.red_object
            left_plan = await self.cli_left_arm.send_request_async(
                x=pick_point.x, y=pick_point.y, z=pick_point.z, vel=0.0, named_pose=""
            )
            right_plan = await self.cli_right_arm.send_request_async(
                x=pick_point.x, y=pick_point.y, z=pick_point.z, vel=0.0, named_pose=""
            )

            if not left_plan.success and not right_plan.success:
                await self.bring_object_near()

            # Go above the button
            await self.cli_left_arm.send_request_async(
                x=0.12, y=0.5, z=0.85, vel=0.15, named_pose=""
            )

            # Go Home and open the gripper
            await self.cli_left_arm.send_request_async(
                x=0.0, y=0.0, z=0.0, vel=0.15, named_pose="home"
            )
            await self.cli_left_gripper.send_request_async(close=False)

    async def place_object_right_policy(self):
        """
        This policy makes OSCAR place the object in the box, given that it has the object in the right arm
        and the basket is within reach.
        """

        # Check if basket is reachable
        place_point = self.perception.basket
        plan = await self.cli_right_arm.send_request_async(
            x=place_point.x, y=place_point.y, z=0.9, vel=0.0, named_pose=""
        )

        if self.perception.obj_in_right_hand and plan.success:
            place_point = self.perception.basket

            # Go above basket and release object
            await self.cli_right_arm.send_request_async(
                x=place_point.x, y=place_point.y, z=0.9, vel=0.15, named_pose=""
            )
            await self.cli_right_gripper.send_request_async(close=False)

            # Go Home
            await self.cli_right_arm.send_request_async(
                x=0.0, y=0.0, z=0.0, vel=0.15, named_pose="home"
            )
        if (
            not plan.success and place_point.y < 0
        ):  # Basket not reachable when it should
            self.get_logger().fatal(
                f"ERROR: Basket not reachable x={place_point.x} y={place_point.y}"
            )

    async def place_object_left_policy(self):
        """
        This policy makes OSCAR place the object in the box, given that it has the object in the left arm
        and the basket is within reach.
        """

        # Check if basket is reachable
        place_point = self.perception.basket
        plan = await self.cli_left_arm.send_request_async(
            x=place_point.x, y=place_point.y, z=0.9, vel=0.0, named_pose=""
        )

        if self.perception.obj_in_left_hand and plan.success:
            place_point = self.perception.basket

            # Go above basket and release object
            await self.cli_left_arm.send_request_async(
                x=place_point.x, y=place_point.y, z=0.9, vel=0.15, named_pose=""
            )
            await self.cli_left_gripper.send_request_async(close=False)

            # Go Home
            await self.cli_left_arm.send_request_async(
                x=0.0, y=0.0, z=0.0, vel=0.15, named_pose="home"
            )
        if (
            not plan.success and place_point.y > 0
        ):  # Basket not reachable when it should
            self.get_logger().fatal(
                f"ERROR: Basket not reachable x={place_point.x} y={place_point.y}"
            )

    async def change_hands_policy(self):
        """
        This policy makes OSCAR exchange the object from one arm to the other.
        """
        if self.perception.obj_in_left_hand:
            # Move left hand to give position
            await self.cli_left_arm.send_request_async(
                x=0.0, y=0.0, z=0.0, vel=0.1, named_pose="switch_give"
            )

            # Move right hand to pre-receive position and open gripper
            await self.cli_right_arm.send_request_async(
                x=0.0, y=0.0, z=0.0, vel=0.1, named_pose="pre_switch_collect"
            )
            await self.cli_right_gripper.send_request_async(close=False)

            # Move right hand to receive position and close gripper
            await self.cli_right_arm.send_request_async(
                x=0.0, y=0.0, z=0.0, vel=0.1, named_pose="switch_collect"
            )
            await self.cli_right_gripper.send_request_async(close=True)

            # Open left gripper and retract arm
            await self.cli_left_gripper.send_request_async(close=False)
            await self.cli_left_arm.send_request_async(
                x=0.0, y=0.0, z=0.0, vel=0.1, named_pose="post_switch_give"
            )
            # Go home
            await self.cli_left_arm.send_request_async(
                x=0.0, y=0.0, z=0.0, vel=0.1, named_pose="home"
            )
            await self.cli_right_arm.send_request_async(
                x=0.0, y=0.0, z=0.0, vel=0.1, named_pose="home"
            )

        if self.perception.obj_in_right_hand:
            # Move right hand to give position
            await self.cli_right_arm.send_request_async(
                x=0.0, y=0.0, z=0.0, vel=0.1, named_pose="switch_give"
            )

            # Move left hand to pre-receive position and open gripper
            await self.cli_left_arm.send_request_async(
                x=0.0, y=0.0, z=0.0, vel=0.1, named_pose="pre_switch_collect"
            )
            await self.cli_left_gripper.send_request_async(close=False)

            # Move left hand to receive position and close gripper
            await self.cli_left_arm.send_request_async(
                x=0.0, y=0.0, z=0.0, vel=0.1, named_pose="switch_collect"
            )
            await self.cli_left_gripper.send_request_async(close=True)

            # Open right gripper and retract arm
            await self.cli_right_gripper.send_request_async(close=False)
            await self.cli_right_arm.send_request_async(
                x=0.0, y=0.0, z=0.0, vel=0.1, named_pose="post_switch_give"
            )

            # Go home
            await self.cli_left_arm.send_request_async(
                x=0.0, y=0.0, z=0.0, vel=0.1, named_pose="home"
            )
            await self.cli_right_arm.send_request_async(
                x=0.0, y=0.0, z=0.0, vel=0.1, named_pose="home"
            )

    async def update_perceptions(self):
        """
        This method requests the latest perceptions from the sensory system.

        :return: None if the maximum number of trials is reached. 
        :rtype: None
        """
        self.get_logger().info("Updating Perceptions...")
        self.old_perception = self.perception
        valid_perception = False
        trials = 0
        while not valid_perception:
            self.perception = await self.cli_oscar_perception.send_request_async()
            valid_perception = self.perception.success
            trials += 1
            if trials == 20:
                return None
        self.get_logger().info(
            f"Perception updated: {self.perception.object1.label} at ({self.perception.object1.x_position}, {self.perception.object1.y_position})"
        )

    def update_reward_sensor(self):
        """
        Update goal sensors' values.
        """
        for sensor in self.perceptions:
            reward_method = getattr(self, "reward_" + sensor, None)
            if callable(reward_method):
                reward_method()

    def reward_ball_in_box_goal(self):
        """
        Updates the ball_in_box_goal sensor with the reward value.
        If the object is in the basket, the reward is 1.0, otherwise 0.0
        """
        self.perceptions["ball_in_box_goal"].data = self.ball_in_box_reward()

    def ball_in_box_reward(self):
        """
        Checks if the object is in the basket. Returns a reward with value of 1.0, otherwise returns 0.0

        :return: Reward value based on the position of the object relative to the basket.
        :rtype: float
        """
        basket_x = self.perception.basket.x
        basket_y = self.perception.basket.y

        object_x = self.perception.red_object.x
        object_y = self.perception.red_object.y

        delta_x = abs(basket_x - object_x)
        delta_y = abs(basket_y - object_y)

        if delta_x < 0.043 and delta_y < 0.043:
            reward = 1.0
        else:
            reward = 0.0
        return reward

    def approximated_object_reward(self):
        """
        Checks if the object was brought closer to the robot with the button. Returns a reward with value of 0.25, otherwise returns 0.

        :return: Reward value based on whether the object was approximated.
        :rtype: float
        """
        if self.aprox_object:  # Read flag set by bring_object_near
            reward = 0.25
        else:
            reward = 0

        self.aprox_object = False  # Reset flag
        return reward

    def grasped_object_reward(self):
        """
        Checks if the object was grasped. Returns a reward with value of 0.5, otherwise returns 0.

        :return: Reward value based on whether the object was grasped.
        :rtype: float
        """
        # Check if object_in_left_hand or object_in_right_hand went from False to True
        if (
            self.perception.obj_in_left_hand
            and not self.old_perception.obj_in_left_hand
            and not self.old_perception.obj_in_right_hand
        ):
            reward = 0.5
        elif (
            self.perception.obj_in_right_hand
            and not self.old_perception.obj_in_right_hand
            and not self.old_perception.obj_in_left_hand
        ):
            reward = 0.5
        else:
            reward = 0
        return reward

    async def switched_hands_reward(self):
        """
        Checks if the object was switched to the correct arm when the original arm can't reach the basket. Returns a reward with value of 0.75, otherwise returns 0.

        :return: Reward value based on whether the object was switched to the correct arm.
        :rtype: float
        """
        place_point = self.perception.basket
        plan_right = await self.cli_right_arm.send_request_async(
            x=place_point.x, y=place_point.y, z=0.9, vel=0.0, named_pose=""
        )
        plan_left = await self.cli_left_arm.send_request_async(
            x=place_point.x, y=place_point.y, z=0.9, vel=0.0, named_pose=""
        )
        # Check if a hand switch was made
        if (
            self.perception.obj_in_left_hand and self.old_perception.obj_in_right_hand
        ):  # Change from right to left
            if (
                plan_left.success and not plan_right.success
            ):  # Basket reachable only by left hand
                reward = 0.75
            else:
                reward = 0
        elif (
            self.perception.obj_in_right_hand and self.old_perception.obj_in_left_hand
        ):  # Change from left to right
            if (
                plan_right.success and not plan_left.success
            ):  # Basket reachable only by right hand
                reward = 0.75
            else:
                reward = 0
        else:
            reward = 0
        return reward

    def publish_perception_reward(self):
        """
        Method that publishes the current perceptions and the reward values.
        """
        self.get_logger().info("Publishing Perceptions....")
        self.publish_perceptions(self.perceptions)
        self.publish_reward()

    def publish_perceptions_callback(self, msg: PerceptionMultiObj):
        """
        Method that publishes the current perceptions and the reward value in the appropriate topics.

        :param msg: Perception message with the latest perceptions.
        :type msg: cognitive_processes_interfaces.msg.Perception
        """
        self.get_logger().debug("DEBUG - Publishing perceptions")
        # Convert perceptions to distance, angle
        obj = OscarMDB.cartesian_to_polar(self.perception.red_object)
        bskt = OscarMDB.cartesian_to_polar(self.perception.basket)

        # Assign perceptions to MDB messages
        self.perceptions["cylinders"].data[0].distance = obj[0]
        self.perceptions["cylinders"].data[0].angle = obj[1]
        self.perceptions["cylinders"].data[0].diameter = 0.025

        self.perceptions["boxes"].data[0].distance = bskt[0]
        self.perceptions["boxes"].data[0].angle = bskt[1]
        self.perceptions["boxes"].data[0].diameter = 0.1

        self.perceptions["object_in_left_hand"].data = self.perception.obj_in_left_hand
        self.perceptions["object_in_right_hand"].data = (
            self.perception.obj_in_right_hand
        )

        self.publish_perceptions(self.perceptions)

    def publish_perceptions(self, perceptions):
        """
        This method iterates the list of publishers and publishes the corresponding value
        from the perception input.

        :param perceptions: Dictionary with perceptions.
        :type perceptions: dict
        """
        for ident, publisher in self.sim_publishers.items():
            self.get_logger().debug(
                "Publishing " + ident + " = " + str(perceptions[ident].data)
            )
            publisher.publish(perceptions[ident])

    def publish_reward(self):
        """
        Publishes the reward value.
        """
        self.reward_pub.publish(Float32(data=float(self.reward)))

    async def bring_object_near(self):
        """
        This method randomly moves the object so that it is
        within the reach of the robot and not inside the basket.
        """
        basket_x = self.perception.basket.x
        basket_y = self.perception.basket.y

        object_x = np.random.uniform(low=self.x_object_close_limits[0], high=self.x_object_close_limits[1])
        object_y = np.random.uniform(low=self.y_object_close_limits[0], high=self.y_object_close_limits[1])

        delta_x = basket_x - object_x
        delta_y = basket_y - object_y
        distance = np.sqrt(delta_x * delta_x + delta_y * delta_y)
        while distance < 0.15:
            object_x = np.random.uniform(low=self.x_object_close_limits[0], high=self.x_object_close_limits[1])
            object_y = np.random.uniform(low=self.y_object_close_limits[0], high=self.y_object_close_limits[1])

            delta_x = basket_x - object_x
            delta_y = basket_y - object_y
            distance = np.sqrt(delta_x * delta_x + delta_y * delta_y)

        object_pose = Pose(
            position=Point(x=object_x, y=object_y, z=0.8),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        obj_msg = SetEntityState.Request()

        obj_msg.state.name = "red_cylinder"
        obj_msg.state.pose = object_pose
        obj_msg.state.reference_frame = "world"

        move_resp = await self.cli_set_state.call_async(obj_msg)
        self.get_logger().debug(f"Moving object. Response: {move_resp.success}")
        self.aprox_object = True  # Aproximated Object Flag

    async def random_positions(self):
        """
        This method randomly places the basket and the object so that
        they are not coliding.
        """
        basket_x = np.random.uniform(low=self.x_basket_limits[0], high=self.x_basket_limits[1])
        basket_y = np.random.uniform(low=self.y_basket_limits[0], high=self.y_basket_limits[1])

        object_x = np.random.uniform(low=self.x_object_limits[0], high=self.x_object_limits[1])
        object_y = np.random.uniform(low=self.y_object_limits[0], high=self.y_object_limits[1])

        delta_x = basket_x - object_x
        delta_y = basket_y - object_y
        distance = np.sqrt(delta_x * delta_x + delta_y * delta_y)
        while distance < 0.15:
            object_x = np.random.uniform(low=self.x_object_limits[0], high=self.x_object_limits[1])
            object_y = np.random.uniform(low=self.y_object_limits[0], high=self.y_object_limits[1])

            delta_x = basket_x - object_x
            delta_y = basket_y - object_y
            distance = np.sqrt(delta_x * delta_x + delta_y * delta_y)

        basket_pose = Pose(
            position=Point(x=basket_x, y=basket_y, z=0.8),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        object_pose = Pose(
            position=Point(x=object_x, y=object_y, z=0.8),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )

        bskt_msg = SetEntityState.Request()
        obj_msg = SetEntityState.Request()

        bskt_msg.state.name = "basket"
        bskt_msg.state.pose = basket_pose
        bskt_msg.state.reference_frame = "world"

        obj_msg.state.name = "red_cylinder"
        obj_msg.state.pose = object_pose
        obj_msg.state.reference_frame = "world"

        move_resp = await self.cli_set_state.call_async(bskt_msg)
        self.get_logger().debug(move_resp.success)
        move_resp = await self.cli_set_state.call_async(obj_msg)
        self.get_logger().debug(move_resp.success)

    @staticmethod
    def cartesian_to_polar(point: Point):
        """
        Helper method that transforms a point (referenced to the base of the robot) to
        a polar representation.

        :param point: Point to be transformed
        :type point: geometry_msgs.msg.Point
        :return: Tuple with the distance and angle values.
        :rtype: tuple
        """
        distance = np.sqrt(point.x * point.x + point.y * point.y)
        angle = np.arctan2(point.y, point.x)
        return distance, angle
    
class OscarMDB_LLM(OscarMDB):
    """
    Child class of OscarMDB that implements the LLM alignment experiment.
    """
    def __init__(self):
        super().__init__()
        self.x_object_close_limits = [0.2575, 0.3625]
        self.y_object_close_limits = [-0.458, 0.0]
        self.x_object_limits = [0.1125, 0.7375]
        self.y_object_limits = [-0.7415, 0.0]
        self.x_basket_limits = [0.25, 0.35]
        self.y_basket_limits = [-0.55, 0.0]


    def publish_perceptions_callback(self, msg: PerceptionMultiObj):
        """
        Method that publishes the current perceptions and the reward value in the appropriate topics.

        :param msg: Perception message with the latest perceptions.
        :type msg: cognitive_processes_interfaces.msg.Perception
        """
        self.get_logger().debug("DEBUG - Publishing perceptions")
        # Assign perceptions to MDB messages
        self.perceptions["object1"].data[0].label = self.perception.object1.label
        self.perceptions["object1"].data[0].x_position = self.perception.object1.x_position
        self.perceptions["object1"].data[0].y_position = self.perception.object1.y_position
        self.perceptions["object1"].data[0].diameter = 0.025
        self.perceptions["object1"].data[0].color = self.perception.object1.color
        self.perceptions["object1"].data[0].state = self.perception.object1.state

        self.perceptions["object2"].data[0].label = self.perception.object2.label
        self.perceptions["object2"].data[0].x_position = self.perception.object2.x_position
        self.perceptions["object2"].data[0].y_position = self.perception.object2.y_position
        self.perceptions["object2"].data[0].diameter = 0.1
        self.perceptions["object2"].data[0].color = self.perception.object2.color
        self.perceptions["object2"].data[0].state = self.perception.object2.state

        self.perceptions["robot_hand"].data[0].state = self.perception.robot_hand.state
        self.perceptions["robot_hand"].data[0].x_position = self.perception.robot_hand.x_position
        self.perceptions["robot_hand"].data[0].y_position = self.perception.robot_hand.y_position

        self.publish_perceptions(self.perceptions)
        


def oscar_experiment():
    rclpy.init()

    oscar_server = OscarMDB()
    oscar_server.load_configuration()
    try:
        rclpy.spin(oscar_server)
    except KeyboardInterrupt:
        oscar_server.destroy_node()

def oscar_alignment_experiment():
    rclpy.init()

    oscar_server = OscarMDB_LLM()
    oscar_server.load_configuration()
    try:
        rclpy.spin(oscar_server)
    except KeyboardInterrupt:
        oscar_server.destroy_node()
