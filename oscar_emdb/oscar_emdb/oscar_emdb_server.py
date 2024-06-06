import numpy as np
from math import pi

# ros libraries
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Interfaces
from oscar_interfaces.srv import Perception
from trajectory_msgs.msg import JointTrajectoryPoint
from oscar_interfaces.srv import ArmControl, GripperControl
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import Pose, Quaternion, Point
from core_interfaces.srv import LoadConfig

#Utils
from core.service_client import ServiceClient, ServiceClientAsync
from core.utils import class_from_classname




class OscarMDB(Node):
    def __init__(self):
        
        #Service client for Oscar Perceptions
        
        
        #Service clients for Oscar Commander
        
        
        #Service clients for Gazebo

        #Service client for eMBD Commander
        
        
        #Initialize robot
        
        pass

        
        

    def load_configuration(self):
        #Load file
        
        #Setup perceptions
        self.setup_perceptions('Perceptions')

        #Setup subscriptions and publishers for the control channels
        self.setup_control_channel('Control')

        pass

    def setup_perceptions(self):
        pass

    def setup_control_channel(self):
        pass

    def load_experiment_file_in_commander(self):
        pass

    def policy_callback(self):
        pass

    def reset_world_callback(self):
        pass

    def grasp_right_policy(self):
        pass

    def grasp_left_policy(self):
        pass

    def press_button_policy(self):
        pass
    
    def place_object_right_policy(self):
        pass

    def place_object_left_policy(self):
        pass

    def change_hands_policy(self):
        pass

    def update_perceptions(self):
        pass

    def update_reward(self):
        pass

    def ball_in_box_reward(self):
        pass

    def approximated_object_reward(self):
        pass

    def grasped_object_reward(self):
        pass

    def switched_hands_reward(self):
        pass

    def publish_perception(self):
        pass

    def bring_object_near(self):
        pass

    def random_positions(self):
        pass

    def cartesian_to_polar(self):
        pass

def main():
    rclpy.init()

    oscar_server = OscarMDB()
    try:
        rclpy.spin(oscar_server)
    except KeyboardInterrupt:
        oscar_server.destroy_node()


    







        

