import numpy as np
from math import pi
import yaml
import yamlloader
import os

# ros libraries
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tf2_ros import Buffer, TransformListener


# Interfaces
from rcl_interfaces.msg import ParameterDescriptor
from trajectory_msgs.msg import JointTrajectoryPoint
from tiago_moveit_py_interfaces.srv import ArmControl, GripperControl, PickPlaceAction, TorsoControl, Trigger
from tiago_moveit_py_interfaces.msg import GripperState
from stereo_location_interfaces.msg import ObjDet, ObjDetArray
from simulators_interfaces.msg import FruitListMsg, FruitMsg, ScaleListMsg, ScaleMsg
from geometry_msgs.msg import Point, PoseStamped
from core_interfaces.srv import LoadConfig
from tts_msgs.action import TTS
import tf2_geometry_msgs

# Utils
from core.service_client import ServiceClient, ServiceClientAsync
from core.utils import class_from_classname




class TiagoMDB(Node):
    """
    This class provides a node that interfaces the OSCAR robot with the eMDB Cognitive Architecture
    """

    def __init__(self):
        """
        Constructor of the OscarMDB class
        """
        super().__init__("tiago_fruit_shop_server")

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
        self.perception_flags = {}

        self.qos_grippers = QoSProfile(
            depth=0,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.qos_detections = QoSProfile(
            depth=0,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE
        )


        # Local variables init
        self.base_messages = {}
        self.fruits = []
        self.closest_fruit = ObjDet()
        self.picked_fruit = ObjDet()
        self.tested_fruit = ObjDet()
        self.basket = ObjDet()
        self.pocket = ObjDet()
        self.scale = ObjDet()
        self.scale_active = False
        self.scale_state = 0 
        self.button_light = False
        self.grasping_enabled = False


        self.fruit_correctly_accepted = False
        self.fruit_correctly_rejected = False

        self.iteration = 0
        self.change_reward_iterations = {}

        # Local constants
        self.place_scale_z = 0.9
        self.place_z = 0.87
        self.pick_scale_z = 0.86

        # Callback groups
        self.client_cbg = MutuallyExclusiveCallbackGroup()
        self.server_cbg = MutuallyExclusiveCallbackGroup()

        # TF listeners
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)


        # Service clients for Tiago Commands

        self.cli_enable_grasping = ServiceClientAsync(
            self, Trigger, "tiago_dual/enable_grasping", self.client_cbg
        )

        self.cli_disable_grasping = ServiceClientAsync(
            self, Trigger, "tiago_dual/disable_grasping", self.client_cbg
        )

        self.cli_reset_grasping = ServiceClientAsync(
            self, Trigger, "tiago_dual/reset_grasping", self.client_cbg
        )

        self.cli_pick = ServiceClientAsync(
            self, PickPlaceAction, "tiago_dual/pick", self.client_cbg
        )

        self.cli_place = ServiceClientAsync(
            self, PickPlaceAction, "tiago_dual/place", self.client_cbg
        )

        self.cli_change_hands = ServiceClientAsync(
            self, Trigger, "tiago_dual/change_hands", self.client_cbg
        )

        self.cli_press_object = ServiceClientAsync(
            self, Trigger, "tiago_dual/press_object", self.client_cbg
        )

        self.right_arm_client = ServiceClientAsync(self, ArmControl, "tiago_dual/right_arm_command", self.client_cbg)
        self.left_arm_client = ServiceClientAsync(self, ArmControl, "tiago_dual/left_arm_command", self.client_cbg)
        self.arm_clients = {
            'right': self.right_arm_client,
            'left': self.left_arm_client
        }

        self.right_gripper_client = ServiceClientAsync(self, GripperControl, "tiago_dual/right_gripper_command", self.client_cbg)
        self.left_gripper_client = ServiceClientAsync(self, GripperControl, "tiago_dual/left_gripper_command", self.client_cbg)
        self.gripper_clients = {
            'right': self.right_gripper_client,
            'left': self.left_gripper_client
        }

        # Voice client
        self.tts_client = ActionClient(self, TTS, "/tts_engine/tts", callback_group=self.client_cbg)

        # Service client for eMBD Commander
        # self.cli_mdb_commander = ServiceClient(LoadConfig, "commander/load_experiment")

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
                self.setup_experiment_stages(config["DiscreteEventSimulator"]["Stages"])
                self.setup_perceptions(config["DiscreteEventSimulator"]["Perceptions"])
                # Be ware, we can not subscribe to control channel before creating all sensor publishers.
                self.setup_control_channel(config["Control"])
        if self.random_seed:
            self.rng = np.random.default_rng(self.random_seed)
            self.get_logger().info(
                f"Setting random number generator with seed {self.random_seed}"
            )
        else:
            self.rng = np.random.default_rng()

        # self.load_experiment_file_in_commander()
        self.get_logger().fatal("DEBUG : Load experiment file in commander is disabled, please enable it before running the experiment.")

    def setup_experiment_stages(self, stages):
        """
        Setup the stages of the experiment with their corresponding iterations.

        :param stages: A dictionary where keys are stage names and values are the iterations at which the stage starts.
        :type stages: dict
        """
        for stage in stages:
            self.change_reward_iterations[stage] = stages[stage]

    def setup_perceptions(self, perceptions):
        """
        This method creates publishers for each element of the perception list
        passed.

        :param perceptions: List of perceptions.
        :type perceptions: list
        """
        self.get_logger().info("Setting up perceptions...")
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
            )
            self.perception_flags[sid] = False


        # Subscribers for the robot sensors
        self.get_logger().info("Setting up subscribers for robot sensors...")
        self.gripper_subs = self.create_subscription(
            GripperState,
            'tiago_dual/gripper_state',
            self.process_gripper_state_callback,
            qos_profile=self.qos_grippers,
            callback_group=self.client_cbg
        )
        self.objects_subs = self.create_subscription(
            ObjDetArray,
            "cam/object_tracker/detections",
            self.process_objects_callback,
            qos_profile=self.qos_detections,
            callback_group=self.server_cbg
        )

        # Publishers for ad-hoc perceptions
        self.ad_hoc_perceptions_timer = self.create_timer(
            0.1, self.publish_ad_hoc_perceptions, callback_group=self.server_cbg
        )

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
                callback_group=self.server_cbg,
            )
        if service_world_reset:
            classname= simulation["executed_policy_msg"]
            self.message_world_reset = class_from_classname(simulation["world_reset_msg"])
            self.create_service(self.message_world_reset, service_world_reset, self.world_reset_service_callback, callback_group=self.server_cbg)   
        

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
        self.iteration = data.iteration
        if data.command == "reset_world":
            await self.reset_world()
        elif data.command == "end":
            self.get_logger().info("Ending experiment as requested by LTM...")
            rclpy.shutdown()

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
        await getattr(self, request.policy + "_policy")()
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
        # Go Home
        await self.init_tiago()
        self.fruit_correctly_accepted = False
        self.fruit_correctly_rejected = False
        self.scale_active = False
        self.scale_state = 0
        input("Set up the world and press Enter to continue...")


    def process_gripper_state_callback(self, msg: GripperState):
        ## TODO: TAKE MULTIPLE READINGS HERE SO THAT WE CAN AVOID BOUNCING
        self.perceptions["fruit_in_left_hand"].data = msg.left.opening > 0.001 and msg.left.current > 0.1
        self.perceptions["fruit_in_right_hand"].data = msg.right.opening > 0.001 and msg.right.current > 0.1
        self.perception_flags["fruit_in_left_hand"] = True
        self.perception_flags["fruit_in_right_hand"] = True
        self.publish_perceptions()

    def transform_detections_positions(self, detections, target_frame):
        """
        Transforms the position of each ObjDet in the ObjDetArray to the target_frame.
        Modifies the ObjDetArray in place.
        """
        for obj in detections.objects:
            pose_in = PoseStamped()
            pose_in.header = detections.header
            pose_in.header.stamp = rclpy.time.Time().to_msg()
            pose_in.pose.position = obj.position
            pose_in.pose.orientation.w = 1.0  # Identity quaternion

            try:
                pose_out = self.tf_buffer.transform(
                    pose_in, target_frame, timeout=rclpy.duration.Duration(seconds=0.05)
                )
                obj.position = pose_out.pose.position
            except Exception as e:
                self.get_logger().warn(
                    f"TF transform failed for {obj.class_name} from {obj.header.frame_id} to {target_frame}: {e}"
                )
        return detections


    def process_objects_callback(self, msg: ObjDetArray):
        # Obtain all fruits and select the closest. Except if one is already in the hand.
        self.fruits = []
        self.transform_detections_positions(msg, "base_footprint")
        for obj in msg.objects:
            if obj.class_name=="basket":
                self.basket = obj
            elif obj.class_name=="scale":
                self.scale = obj
            elif obj.class_name == "pocket":
                self.pocket = obj
            else:
                if obj.position.z > 0.75 and obj.position.z < 0.9:
                    self.fruits.append(obj)
        if not(self.perceptions["fruit_in_left_hand"].data or self.perceptions["fruit_in_right_hand"].data):
            if len(self.fruits) > 0:
                # Sort the fruits by distance to the robot base
                self.fruits.sort(key=lambda x: self.calculate_distance_to_robot_base(x))
                self.closest_fruit = self.fruits[0]
                self.get_logger().info(f"DEBUG: Closest fruit: {self.closest_fruit.class_name} at distance {self.calculate_distance_to_robot_base(self.closest_fruit)}")
            else:
                self.closest_fruit = ObjDet()

        fruit_distance, fruit_angle = self.cartesian_to_polar(self.closest_fruit.position)
        fruit_dim = self.closest_fruit.dimensions.major_dim
        self.perceptions["fruits"].data[0].distance = fruit_distance
        self.perceptions["fruits"].data[0].angle = fruit_angle
        self.perceptions["fruits"].data[0].dim_max = fruit_dim

        scale_distance, scale_angle = self.cartesian_to_polar(self.scale.position)

        self.perceptions["scales"].data[0].distance = scale_distance
        self.perceptions["scales"].data[0].angle = scale_angle
        self.perceptions["scales"].data[0].state = self.scale_state
        self.perceptions["scales"].data[0].active = self.scale_active
        self.perception_flags["fruits"] = True
        self.perception_flags["scales"] = True
        # Publish the perceptions
        self.publish_perceptions()
                    
            
    def publish_ad_hoc_perceptions(self):
        # Publish the perception of the button light and the drive sensors
        self.perceptions["button_light"].data = self.button_light
        self.perception_flags["button_light"] = True
        self.update_reward_sensor()
        self.publish_perceptions()


    async def init_tiago(self):
        """
        Enables Tiago grasping etc..
        """
        if not self.grasping_enabled:
            enable_grasping = await self.cli_enable_grasping.send_request_async()
            if enable_grasping.success:
                self.get_logger().info("Grasping enabled")
                input("Drive Tiago to the initial position and press Enter to continue...")
                self.grasping_enabled = True
                return
            else:
                self.get_logger().error(f"Could not enable grasping. Error: {enable_grasping.status}")

        # If grasping is already enabled, we can reset it
        reset_grasping = await self.cli_reset_grasping.send_request_async()

        if reset_grasping.success:
            self.get_logger().info("Grasping reset successfully")
            self.grasping_enabled = True
        else:
            self.get_logger().error(f"Could not reset grasping. Error: {reset_grasping.status}")
            self.grasping_enabled = False
                  
    async def pick_fruit_policy(self):
        if not self.scale_active:
            frame_id = self.closest_fruit.header.frame_id
            z = self.determine_pick_height(self.closest_fruit)
            response = await self.cli_pick.send_request_async(frame_id=frame_id, z=z)
            if response.success:
                self.get_logger().info(f"Picked fruit {self.closest_fruit.class_name} successfully.")
                self.picked_fruit = self.closest_fruit
                return True
            else:
                self.get_logger().error(f"Could not pick fruit {self.closest_fruit.class_name}. Error: {response.status}")
                return False
        else:
            return await self.pick_tested_fruit()
    
    async def place_fruit_policy(self):
        frame_id = None
        if self.perceptions["fruit_in_left_hand"].data:
            frame_id = "place_right"
        elif self.perceptions["fruit_in_right_hand"].data:
            frame_id = "place_left"

        if frame_id:
            response = await self.cli_place.send_request_async(frame_id=frame_id, z=self.place_z)
            if response.success:
                self.get_logger().info(f"Placed fruit {self.closest_fruit.class_name} successfully.")
                self.picked_fruit = ObjDet()  # Reset the picked fruit
            else:
                self.get_logger().error(f"Could not pick fruit {self.closest_fruit.class_name}. Error: {response.status}")
        else:
            self.get_logger().warn("No fruit in hand to place.")
            
    
    async def change_hands_policy(self):
        response = await self.cli_change_hands.send_request_async()
        if response.success:
            self.get_logger().info("Changed hands successfully.")
        else:
            self.get_logger().warn(f"Could not change hands. Error: {response.status}")
    
    async def test_fruit_policy(self):
        self.get_logger().info("Executing test fruit policy...")
        frame_id = self.scale.header.frame_id
        response = await self.cli_place.send_request_async(frame_id=frame_id, z=self.place_scale_z)
        if response.success:
            self.get_logger().info(f"Tested fruit {self.picked_fruit.class_name} successfully.")
            self.tested_fruit = self.picked_fruit
            self.picked_fruit = ObjDet()  # Reset the picked fruit
            self.scale_active = True
            if self.scale_state == 0:
                self.scale_state = 1 if self.rng.uniform() > 0.5 else 2
        else:
            self.get_logger().error(f"Could not test fruit {self.closest_fruit.class_name}. Error: {response.status}")

    async def pick_tested_fruit(self):
        frame_id = self.tested_fruit.header.frame_id
        fruit_transform, _ = self.obtain_transform("base_footprint", frame_id)
        if fruit_transform is None:
            frame_id = self.scale.header.frame_id
            self.get_logger().warn(f"Could not obtain transform for {self.tested_fruit.class_name}. Using scale frame instead.")
        z = self.pick_scale_z
        response = await self.cli_pick.send_request_async(frame_id=frame_id, z=z)
        if response.success:
            self.get_logger().info(f"Picked tested fruit {self.tested_fruit.class_name} successfully.")
            self.picked_fruit = self.tested_fruit
            self.tested_fruit = ObjDet()  # Reset the tested fruit
            self.scale_active = False
            return True
        else:
            self.get_logger().error(f"Could not pick tested fruit {self.tested_fruit.class_name}. Error: {response.status}")
            return False
    
    async def accept_fruit_policy(self):
        success = await self.classifying_sequence("accept")
        if success:
            self.get_logger().info(f"Accepted fruit {self.closest_fruit.class_name} successfully.")
            self.scale_active = False
            self.picked_fruit = ObjDet()
            if self.scale_state == 1:
                self.fruit_correctly_accepted = True
                self.scale_state = 0
        else:
            self.get_logger().error(f"Could not place fruit {self.closest_fruit.class_name} in pocket.")
    
    async def discard_fruit_policy(self):
        success = await self.classifying_sequence("discard")
        if success:
            self.get_logger().info(f"Discarded fruit {self.closest_fruit.class_name} successfully.")
            if self.scale_state == 2:
                self.fruit_correctly_rejected = True
                self.scale_active = False
                self.scale_state = 0
                self.picked_fruit = ObjDet()
        else:
            self.get_logger().error(f"Could not place fruit {self.closest_fruit.class_name} in basket.")

    async def classifying_sequence(self, action):
        """
        Executes a sequence of actions for classifying a fruit.
        :param action: The action to be executed (accept or discard).
        """
        success = False
        if action == "accept":
            frame_id = self.pocket.header.frame_id
        elif action == "discard":
            frame_id = self.basket.header.frame_id
        else:
            self.get_logger().error(f"Unknown action: {action}")
            return False 
        pick = await self.pick_fruit_policy()
        if pick:
            self.get_logger().info(f"Picked fruit {self.closest_fruit.class_name} successfully.")
            response = await self.cli_place.send_request_async(frame_id=frame_id, z=self.place_z)
            if response.success:
                self.get_logger().info(f"Placed fruit {self.closest_fruit.class_name} successfully")
                success = True
            else:
                self.get_logger().error(f"Could not place fruit {self.closest_fruit.class_name}. Changing hands...")
                await self.place_fruit_policy()
                second_pick = await self.pick_fruit_policy()
                response = await self.cli_place.send_request_async(frame_id=frame_id, z=self.place_z)
                if response.success and second_pick:
                    self.get_logger().info(f"Placed fruit {self.closest_fruit.class_name} successfully on second attempt")
                    success = True
                else:
                    success = False
        return success
                
    async def press_button_policy(self):
        response = await self.cli_press_object.send_request_async()
        if response.success:
            self.get_logger().info("Pressed button successfully.")
            self.button_light = not self.button_light  # Toggle the button light state
        else:
            self.get_logger().error(f"Could not press button. Error: {response.status}")

    async def ask_nicely_policy(self):
        goal = TTS.Goal()
        goal.input = "Pardon the imposition, but might I most humbly beseech your esteemed assistance in rectifying a most vexing anomaly within my subroutines?" 
        future = self.tts_client.send_goal_async(goal)
        await future
        input("Confirm help. Press Enter to continue...")
    
    def update_reward_sensor(self):
        """
        Update goal sensors' values.
        """
        for sensor in self.perceptions:
            reward_method = getattr(self, "reward_" + sensor, None) 
            if callable(reward_method):
                reward_method()
                self.perception_flags[sensor] = True

    def reward_ball_in_box_goal(self):
        """
        Updates the ball_in_box_goal sensor with the reward value.
        If the object is in the basket, the reward is 1.0, otherwise 0.0
        """
        self.perceptions["ball_in_box_goal"].data = self.ball_in_box_reward()

    def reward_classify_fruit_goal(self):
        """
        Gives a reward of 1.0 if the fruit is correctly classified.
        """
        reward = 0.0
        if self.iteration > self.change_reward_iterations['stage2']:
            self.get_logger().debug("STAGE 2 REWARD: CLASSIFY FRUIT")
            if self.fruit_correctly_accepted or self.fruit_correctly_rejected:
                reward = 1.0
        self.perceptions["classify_fruit_goal"].data = reward
    
    def reward_place_fruit_goal(self):
        """
        Gives a reward of 1.0 if the fruit is placed in the center of the table.
        """
        reward = 0.0 
        if (self.iteration > self.change_reward_iterations['stage0']) and (self.iteration <= self.change_reward_iterations['stage1']):
            self.get_logger().debug("STAGE 1 REWARD: PLACE FRUIT")
            if self.fruit_in_placed_pos(self.closest_fruit):
                reward = 1.0
                
        else:
            reward = 1.0
            if (self.iteration > self.change_reward_iterations['stage0']) and (self.iteration <= self.change_reward_iterations['stage2']):
                self.get_logger().debug("STAGE 2 REWARD: NONE")
            else:
                self.get_logger().debug("STAGE 0 REWARD: NONE")
            
        self.perceptions["place_fruit_goal"].data = reward
    
    def fruit_in_placed_pos(self, fruit: ObjDet, threshold=0.1):
        transform_left, _ = self.obtain_transform("base_footprint", "place_left")
        transform_right, _ = self.obtain_transform("base_footprint", "place_right")
        if transform_left is None or transform_right is None:
            return False
        # Check if the fruit is placed in the left or right pocket
        place_left = transform_left.transform.translation
        place_right = transform_right.transform.translation
        dist_left = Point()
        dist_right = Point()
        dist_left.x = place_left.x - fruit.position.x
        dist_left.y = place_left.y - fruit.position.y
        dist_right.x = place_right.x - fruit.position.x
        dist_right.y = place_right.y - fruit.position.y
        distance_left = np.sqrt(dist_left.x**2 + dist_left.y**2)
        distance_right = np.sqrt(dist_right.x**2 + dist_right.y**2)

        if distance_left < threshold or distance_right < threshold:
            return True
        else:
            return False
    
    def calculate_distance_to_robot_base(self, obj: ObjDet):
        distance, _ = self.__class__.cartesian_to_polar(obj.position)
        return distance
    
    def determine_pick_height(self, pick_obj: ObjDet, z_low=0.795, z_high=0.86):
        scale_bbox = self.scale.bounding_box
        pick_obj_bbox = pick_obj.bounding_box
        # Check if bounding boxes overlap (only x and y min/max)
        b1 = scale_bbox
        b2 = pick_obj_bbox
        overlap = (
            b1.x_min < b2.x_max and b1.x_max > b2.x_min and
            b1.y_min < b2.y_max and b1.y_max > b2.y_min
        )

        if overlap:
            return z_high
        else:
            return z_low

    def publish_perceptions(self):
        """
        This method iterates the list of publishers and publishes the corresponding value
        from the perception input.

        :param perceptions: Dictionary with perceptions.
        :type perceptions: dict
        """
        for ident, publisher in self.sim_publishers.items():
            if self.perception_flags[ident]:
                # Publish the perception if the flag is set
                self.get_logger().debug(
                    "Publishing " + ident + " = " + str(self.perceptions[ident].data)
                )
                publisher.publish(self.perceptions[ident])
                self.perception_flags[ident] = False

    def obtain_transform(self, target_frame, source_frame):
        """
        Obtain the transform between two frames.

        :param target_frame: The target frame to transform to.
        :type target_frame: str
        :param source_frame: The source frame to transform from.
        :type source_frame: str
        :return: The transform between the two frames.
        :rtype: TransformStamped
        """
        try: 
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform, None
        except Exception as e:
            self.get_logger().error(f"Error obtaining transform: {e}")
            return None, e


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


def main():
    rclpy.init()

    tiago_server = TiagoMDB()
    tiago_server.load_configuration()
    try:
        rclpy.spin(tiago_server)
    except KeyboardInterrupt:
        tiago_server.destroy_node()
