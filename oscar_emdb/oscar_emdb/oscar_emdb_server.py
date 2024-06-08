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
from oscar_interfaces.srv import ArmControl, GripperControl, Perception
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import Pose, Quaternion, Point
from core_interfaces.srv import LoadConfig
from std_msgs.msg import Float32

#Utils
from core.service_client import ServiceClient, ServiceClientAsync
from core.utils import class_from_classname




class OscarMDB(Node):
    def __init__(self):
        super().__init__('oscar_emdb_server')

        #Setup parameters
        self.random_seed = self.declare_parameter('random_seed', value = 0).get_parameter_value().integer_value
        self.config_file = self.declare_parameter('config_file', descriptor=ParameterDescriptor(dynamic_typing=True)).get_parameter_value().string_value

        #Publishers and perception messages
        self.sim_publishers={}
        self.perceptions={}

        #Local variables init
        self.perception=Perception.Response()
        self.old_perception=Perception.Response()
        self.aprox_object=False
        self.base_messages={}

        #Callback groups
        self.gazebo_cbg=MutuallyExclusiveCallbackGroup()
        self.oscar_cbg=MutuallyExclusiveCallbackGroup()
        self.perception_cbg=MutuallyExclusiveCallbackGroup()
        self.mdb_commands_cbg=MutuallyExclusiveCallbackGroup()
        
        #Service client for Oscar Perceptions
        self.cli_oscar_perception = ServiceClientAsync(self, Perception, 'oscar/request_perceptions', self.perception_cbg)

        #Service clients for Oscar Commander
        self.cli_right_arm = ServiceClientAsync(self, ArmControl, "oscar/right_arm_command", self.oscar_cbg)
        self.cli_left_arm = ServiceClientAsync(self, ArmControl, "oscar/left_arm_command", self.oscar_cbg)
        self.cli_left_gripper = ServiceClientAsync(
            self, GripperControl, "oscar/left_gripper_command", self.oscar_cbg
        )
        self.cli_right_gripper = ServiceClientAsync(
            self, GripperControl, "oscar/right_gripper_command", self.oscar_cbg
        )
        
        #Service client for Gazebo
        self.cli_set_state = self.create_client(SetEntityState, "set_entity_state", callback_group=self.gazebo_cbg)
        while not self.cli_set_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")

        #Service client for eMBD Commander
        self.cli_mdb_commander = ServiceClient(LoadConfig, "commander/load_experiment")

        #Reward Publisher
        self.reward_pub = self.create_publisher(Float32, 'mdb/reward', 1) #TODO: Implement dedicated interface
        


        
        

    def load_configuration(self):
        #Load file

        if self.config_file is None:
            self.get_logger().error("No configuration file for the experiment specified!")
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
                self.setup_perceptions(config["OSCAR"]["Perceptions"])
                # Be ware, we can not subscribe to control channel before creating all sensor publishers.
                self.setup_control_channel(config["Control"])
        if self.random_seed:
            self.rng = np.random.default_rng(self.random_seed)
            self.get_logger().info(f"Setting random number generator with seed {self.random_seed}")
        else:
            self.rng = np.random.default_rng()
        
        self.load_experiment_file_in_commander()


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
            else:
                self.perceptions[sid].data = False
            self.get_logger().info("I will publish to... " + str(topic))
            self.sim_publishers[sid] = self.create_publisher(message, topic, 0) #TODO: ¿latch in ROS2?

    def setup_control_channel(self, simulation):
        """
        Configure the ROS topic where listen for commands to be executed.

        :param simulation: The params from the config file to setup the control channel
        :type simulation: dict
        """
        self.ident = simulation["id"]
        topic = simulation["control_topic"]
        classname = simulation["control_msg"]
        message = class_from_classname(classname)
        self.get_logger().info("Subscribing to... " + str(topic))
        self.create_subscription(message, topic, self.new_command_callback, 0)
        topic = simulation["executed_policy_topic"]
        classname = simulation["executed_policy_msg"]
        message = class_from_classname(classname)
        self.get_logger().info("Subscribing to... " + str(topic))
        self.create_subscription(message, topic, self.policy_callback, 0)


    def load_experiment_file_in_commander(self):
        loaded = self.cli_mdb_commander.send_request(file = self.config_file)
        return loaded
    
    async def new_command_callback(self, data):
        """
        Process a command received

        :param data: The message that contais the command received
        :type data: ROS msg defined in setup_control_channel
        """
        self.get_logger().debug(f"Command received... ITERATION: {data.iteration}")
        if data.command == "reset_world":
            await self.reset_world()
        elif data.command == "end":
            self.get_logger().info("Ending experiment as requested by LTM...")
            rclpy.shutdown()    

    async def policy_callback(self, data):
        self.get_logger().info(f"Executing {data.data} policy...")
        await self.update_perceptions()
        await getattr(self, data.data + "_policy")()
        await self.update_perceptions()
        await self.update_reward()
        self.publish_perception_reward()

    async def reset_world(self):
        self.get_logger().info("Reseting World...")

        await self.random_positions()
        #Go Home
        await self.init_oscar()
        await self.update_perceptions()
        await self.update_reward()
        self.publish_perception_reward()

    async def init_oscar(self):
        self.get_logger().info('Initializing OSCAR Robot')
        await self.cli_left_arm.send_request_async(x=0.0, y=0.0, z=0.0,vel=0.15,named_pose="home")        
        await self.cli_right_arm.send_request_async(x=0.0, y=0.0, z=0.0,vel=0.15,named_pose="home") 
        await self.cli_right_gripper.send_request_async(close=False)
        await self.cli_left_gripper.send_request_async(close=False)


    async def grasp_right_policy(self):
        if not self.perception.obj_in_left_hand and not self.perception.obj_in_left_hand:
            pick_point=self.perception.red_object

            #Check if object is reachable
            plan= await self.cli_right_arm.send_request_async(x=pick_point.x,y=pick_point.y,z=pick_point.z,vel=0.0,named_pose="")

            if plan.success:

                #Go Home and open the gripper
                await self.cli_right_arm.send_request_async(x=0.0,y=0.0,z=0.0,vel=0.15,named_pose="home")
                await self.cli_right_gripper.send_request_async(close=False)

                #Go above object
                await self.cli_right_arm.send_request_async(x=pick_point.x,y=pick_point.y,z=0.9,vel=0.15,named_pose="")

                #Grasp object
                await self.cli_right_arm.send_request_async(x=pick_point.x,y=pick_point.y,z=0.8,vel=0.15,named_pose="")
                await self.cli_right_gripper.send_request_async(close=True)

                #Lift Object
                await self.cli_right_arm.send_request_async(x=pick_point.x,y=pick_point.y,z=0.9,vel=0.15,named_pose="")

                #Go Home
                await self.cli_right_arm.send_request_async(x=0.0,y=0.0,z=0.0,vel=0.15,named_pose="home")

    async def grasp_left_policy(self):
        if not self.perception.obj_in_left_hand and not self.perception.obj_in_left_hand:
            pick_point=self.perception.red_object

            #Check if object is reachable
            plan=await self.cli_left_arm.send_request_async(x=pick_point.x,y=pick_point.y,z=pick_point.z,vel=0.0,named_pose="")

            if plan.success:
                #Go Home and open the gripper
                await self.cli_left_arm.send_request_async(x=0.0,y=0.0,z=0.0,vel=0.15,named_pose="home")
                await self.cli_left_gripper.send_request_async(close=False)

                #Go above object
                await self.cli_left_arm.send_request_async(x=pick_point.x,y=pick_point.y,z=0.85,vel=0.15,named_pose="")

                #Grasp object
                await self.cli_left_arm.send_request_async(x=pick_point.x,y=pick_point.y,z=0.8,vel=0.15,named_pose="")
                await self.cli_left_gripper.send_request_async(close=True)

                #Lift Object
                await self.cli_left_arm.send_request_async(x=pick_point.x,y=pick_point.y,z=0.85,vel=0.15,named_pose="")

                #Go Home
                await self.cli_left_arm.send_request_async(x=0.0,y=0.0,z=0.0,vel=0.15,named_pose="home")

    async def press_button_policy(self):
        if not self.perception.obj_in_left_hand and not self.perception.obj_in_right_hand:
            #Go Home and close the gripper
            await self.cli_left_arm.send_request_async(x=0.0,y=0.0,z=0.0,vel=0.15,named_pose="home")
            await self.cli_left_gripper.send_request_async(close=True)

            #Go above the button
            await self.cli_left_arm.send_request_async(x=0.12,y=0.5,z=0.85,vel=0.15,named_pose="")

            #Press the button
            await self.cli_left_arm.send_request_async(x=0.12,y=0.5,z=0.8,vel=0.15,named_pose="")

            #Check if object is reachable
            pick_point=self.perception.red_object
            left_plan=await self.cli_left_arm.send_request_async(x=pick_point.x,y=pick_point.y,z=pick_point.z,vel=0.0,named_pose="")
            right_plan=await self.cli_right_arm.send_request_async(x=pick_point.x,y=pick_point.y,z=pick_point.z,vel=0.0,named_pose="")

            if not left_plan.success and not right_plan.success:
                await self.bring_object_near()

            #Go above the button
            await self.cli_left_arm.send_request_async(x=0.12,y=0.5,z=0.85,vel=0.15,named_pose="")

            #Go Home and open the gripper
            await self.cli_left_arm.send_request_async(x=0.0,y=0.0,z=0.0,vel=0.15,named_pose="home")
            await self.cli_left_gripper.send_request_async(close=False)
    
    async def place_object_right_policy(self):
        #Check if basket is reachable
        place_point=self.perception.basket
        plan=await self.cli_right_arm.send_request_async(x=place_point.x,y=place_point.y,z=place_point.z,vel=0.0,named_pose="")

        if self.perception.obj_in_right_hand and plan.success:
            place_point=self.perception.basket

            #Go above basket and release object
            await self.cli_right_arm.send_request_async(x=place_point.x,y=place_point.y,z=0.9,vel=0.15,named_pose="")
            await self.cli_right_gripper.send_request_async(close=False)

            #Go Home
            await self.cli_right_arm.send_request_async(x=0.0,y=0.0,z=0.0,vel=0.15,named_pose="home")
        if not plan.success and place_point.y<0: #Basket not reachable when it should
            self.get_logger().fatal(f"ERROR: Basket not reachable x={place_point.x} y={place_point.y}")

    async def place_object_left_policy(self):
        #Check if basket is reachable
        place_point=self.perception.basket
        plan=await self.cli_left_arm.send_request_async(x=place_point.x,y=place_point.y,z=place_point.z,vel=0.0,named_pose="")

        if self.perception.obj_in_left_hand and plan.success:
            place_point=self.perception.basket

            #Go above basket and release object
            await self.cli_left_arm.send_request_async(x=place_point.x,y=place_point.y,z=0.9,vel=0.15,named_pose="")
            await self.cli_left_gripper.send_request_async(close=False)

            #Go Home
            await self.cli_left_arm.send_request_async(x=0.0,y=0.0,z=0.0,vel=0.15,named_pose="home")
        if not plan.success and place_point.y>0: #Basket not reachable when it should
            self.get_logger().fatal(f"ERROR: Basket not reachable x={place_point.x} y={place_point.y}")

    async def change_hands_policy(self):
        if self.perception.obj_in_left_hand:
            #Move left hand to give position
            await self.cli_left_arm.send_request_async(x=0.0,y=0.0,z=0.0,vel=0.1, named_pose="switch_give")

            #Move right hand to pre-receive position and open gripper
            await self.cli_right_arm.send_request_async(x=0.0,y=0.0,z=0.0,vel=0.1,named_pose="pre_switch_collect")
            await self.cli_right_gripper.send_request_async(close=False)

            #Move right hand to receive position and close gripper
            await self.cli_right_arm.send_request_async(x=0.0,y=0.0,z=0.0,vel=0.1,named_pose="switch_collect")
            await self.cli_right_gripper.send_request_async(close=True)

            #Open left gripper and retract arm 
            await self.cli_left_gripper.send_request_async(close=False)
            await self.cli_left_arm.send_request_async(x=0.0,y=0.0,z=0.0,vel=0.1,named_pose="post_switch_give")
            #Go home
            await self.cli_left_arm.send_request_async(x=0.0,y=0.0,z=0.0,vel=0.1,named_pose="home")
            await self.cli_right_arm.send_request_async(x=0.0,y=0.0,z=0.0,vel=0.1,named_pose="home")

        if self.perception.obj_in_right_hand:
            #Move right hand to give position
            await self.cli_right_arm.send_request_async(x=0.0,y=0.0,z=0.0,vel=0.1,named_pose="switch_give")

            #Move left hand to pre-receive position and open gripper
            await self.cli_left_arm.send_request_async(x=0.0,y=0.0,z=0.0,vel=0.1,named_pose="pre_switch_collect")
            await self.cli_left_gripper.send_request_async(close=False)

            #Move left hand to receive position and close gripper
            await self.cli_left_arm.send_request_async(x=0.0,y=0.0,z=0.0,vel=0.1,named_pose="switch_collect")
            await self.cli_left_gripper.send_request_async(close=True)

            #Open right gripper and retract arm 
            await self.cli_right_gripper.send_request_async(close=False)
            await self.cli_right_arm.send_request_async(x=0.0,y=0.0,z=0.0,vel=0.1,named_pose="post_switch_give")

            #Go home
            await self.cli_left_arm.send_request_async(x=0.0,y=0.0,z=0.0,vel=0.1,named_pose="home")
            await self.cli_right_arm.send_request_async(x=0.0,y=0.0,z=0.0,vel=0.1,named_pose="home")

    async def update_perceptions(self):
        self.get_logger().info('Updating Perceptions...')
        self.old_perception=self.perception
        self.perception= await self.cli_oscar_perception.send_request_async()

        #Convert perceptions to distance, angle
        obj=OscarMDB.cartesian_to_polar(self.perception.red_object)
        bskt=OscarMDB.cartesian_to_polar(self.perception.basket)
        
        #Assign perceptions to MDB messages
        self.perceptions['cylinders'].data[0].distance=obj[0]
        self.perceptions['cylinders'].data[0].angle=obj[1]
        self.perceptions['cylinders'].data[0].diameter=0.025

        self.perceptions['boxes'].data[0].distance=bskt[0]
        self.perceptions['boxes'].data[0].angle=bskt[1]
        self.perceptions['boxes'].data[0].diameter=0.1

        self.perceptions['object_in_left_hand'].data=self.perception.obj_in_left_hand
        self.perceptions['object_in_right_hand'].data=self.perception.obj_in_right_hand

    async def update_reward(self):
        self.get_logger().info("Reading Reward....")

        #HACK Check rewards from highest value to lowest 
        self.reward=self.ball_in_box_reward() #This is the only one that should be checked
        if self.reward==0:
            self.reward= await self.switched_hands_reward()
        if self.reward==0:
            self.reward=self.grasped_object_reward()
        if self.reward==0:
            self.reward=self.approximated_object_reward()
        
        self.get_logger().info(f"Reward obtained: {self.reward}")

    def ball_in_box_reward(self):
        basket_x=self.perception.basket.x
        basket_y=self.perception.basket.y

        object_x=self.perception.red_object.x
        object_y=self.perception.red_object.y

        delta_x=abs(basket_x-object_x)
        delta_y=abs(basket_y-object_y)

        if delta_x<0.043 and delta_y<0.043:
            reward=1
        else:
            reward=0
        return reward
    
    def approximated_object_reward(self):
        if self.aprox_object: #Read flag set by bring_object_near
            reward=0.25
        else:
            reward=0
        
        self.aprox_object=False #Reset flag
        return reward

    def grasped_object_reward(self):
        #Check if object_in_left_hand or object_in_right_hand went from False to True
        if self.perception.obj_in_left_hand and not self.old_perception.obj_in_left_hand and not self.old_perception.obj_in_right_hand:
            reward=0.5
        elif self.perception.obj_in_right_hand and not self.old_perception.obj_in_right_hand and not self.old_perception.obj_in_left_hand:
            reward=0.5
        else:
            reward=0
        return reward

    async def switched_hands_reward(self):
        place_point=self.perception.basket
        plan_right= await self.cli_right_arm.send_request_async(x=place_point.x,y=place_point.y,z=place_point.z,vel=0.0,named_pose="")
        plan_left= await self.cli_left_arm.send_request_async(x=place_point.x,y=place_point.y,z=place_point.z,vel=0.0,named_pose="")
        #Check if a hand switch was made
        if self.perception.obj_in_left_hand and self.old_perception.obj_in_right_hand: #Change from right to left
            if plan_left.success and not plan_right.success: #Basket reachable only by left hand
                reward=0.75
            else:
                reward=0
        elif self.perception.obj_in_right_hand and self.old_perception.obj_in_left_hand: #Change from left to right
            if plan_right.success and not plan_left.success: #Basket reachable only by right hand
                reward=0.75
            else:
                reward=0    
        else:
            reward=0
        return reward  

    def publish_perception_reward(self):
        self.get_logger().info("Publishing Perceptions....")

        for ident, publisher in self.sim_publishers.items():
            self.get_logger().debug("Publishing " + ident + " = " + str(self.perceptions[ident].data))
            publisher.publish(self.perceptions[ident])

        self.reward_pub.publish(Float32(data=float(self.reward)))

    async def bring_object_near(self):
        basket_x=self.perception.basket.x
        basket_y=self.perception.basket.y

        object_x=np.random.uniform(low=0.2575,high=0.3625)
        object_y=np.random.uniform(low=-0.458,high=0.458)

        delta_x=basket_x-object_x
        delta_y=basket_y-object_y
        distance=np.sqrt(delta_x*delta_x+delta_y*delta_y)
        while distance<0.15:
            object_x=np.random.uniform(low=0.2575,high=0.3625)
            object_y=np.random.uniform(low=-0.458,high=0.458)

            delta_x=basket_x-object_x
            delta_y=basket_y-object_y
            distance=np.sqrt(delta_x*delta_x+delta_y*delta_y)

        object_pose=Pose(position=Point(x=object_x, y=object_y, z=0.8),orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=1.0))
        obj_msg=SetEntityState.Request()

        obj_msg.state.name="object"
        obj_msg.state.pose=object_pose
        obj_msg.state.reference_frame="world"
        
        move_resp= await self.cli_set_state.call_async(obj_msg)
        self.get_logger().debug(f'Moving object. Response: {move_resp.success}')
        self.aprox_object=True #Aproximated Object Flag

    async def random_positions(self):
        basket_x=np.random.uniform(low=0.25,high=0.35)
        basket_y=np.random.uniform(low=-0.55,high=0.55)

        object_x=np.random.uniform(low=0.1125,high=0.7375)
        object_y=np.random.uniform(low=-0.7415,high=0.7415)

        delta_x=basket_x-object_x
        delta_y=basket_y-object_y
        distance=np.sqrt(delta_x*delta_x+delta_y*delta_y)
        while distance<0.15:
            object_x=np.random.uniform(low=0.1125,high=0.7375)
            object_y=np.random.uniform(low=-0.7415,high=0.7415)

            delta_x=basket_x-object_x
            delta_y=basket_y-object_y
            distance=np.sqrt(delta_x*delta_x+delta_y*delta_y)
        
        basket_pose=Pose(position=Point(x=basket_x,y=basket_y,z=0.8), orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=1.0))
        object_pose=Pose(position=Point(x=object_x,y=object_y,z=0.8), orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=1.0))

        bskt_msg=SetEntityState.Request()
        obj_msg=SetEntityState.Request()

        bskt_msg.state.name="basket"
        bskt_msg.state.pose=basket_pose
        bskt_msg.state.reference_frame="world"

        obj_msg.state.name="object"
        obj_msg.state.pose=object_pose
        obj_msg.state.reference_frame="world"

        move_resp=await self.cli_set_state.call_async(bskt_msg)
        self.get_logger().debug(move_resp.success)
        move_resp=await self.cli_set_state.call_async(obj_msg)
        self.get_logger().debug(move_resp.success)

    @staticmethod
    def cartesian_to_polar(point: Point):
        distance=np.sqrt(point.x*point.x+point.y*point.y)
        angle=np.arctan2(point.y, point.x)
        return distance, angle

def main():
    rclpy.init()

    oscar_server = OscarMDB()
    oscar_server.load_configuration()
    try:
        rclpy.spin(oscar_server)
    except KeyboardInterrupt:
        oscar_server.destroy_node()


    







        

