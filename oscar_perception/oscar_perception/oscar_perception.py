import threading
import numpy as np

# ros libraries
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ros2_numpy import numpify

# Interfaces
from geometry_msgs.msg import PoseStamped, Point

from oscar_emdb_interfaces.srv import PerceptionMultiObj as PerceptionSrv
from oscar_emdb_interfaces.msg import PerceptionMultiObj as PerceptionMsg
from oscar_emdb_interfaces.msg import ObjectPerception
from sensor_msgs.msg import Image
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint

# Image packages
import skimage as ski
import cv2
from cv_bridge import CvBridge
from skimage.measure import regionprops, label
import cameratransform as ct

# Used for the robots's hand position
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped

from gazebo_msgs.msg import ModelStates


class OscarPerception(Node):
    """
    This class provides a node that reads the low-level sensors of the OSCAR robot and
    redescribes to high-level sensors.
    """

    def __init__(self):
        """
        Initializes the OSCAR perception node.
        """
        super().__init__("oscar_perception")

        # Callback groups
        self.perception_cbg = MutuallyExclusiveCallbackGroup()
        self.timer_cbg = MutuallyExclusiveCallbackGroup()
        self.image_cbg = MutuallyExclusiveCallbackGroup()
        self.gripper_cbg = MutuallyExclusiveCallbackGroup()

        # Service server and topic subscription
        self.srv_perception = self.create_service(
            PerceptionSrv,
            "oscar/request_perceptions",
            self.perception_service_callback,
            callback_group=self.perception_cbg,
        )
        self.sub_image = self.create_subscription(
            Image,
            "/oscar/camera/rgb/image_raw",
            self.image_process_callback,
            0,
            callback_group=self.image_cbg,
        )
        self.sub_right_gripper = self.create_subscription(
            JointTrajectoryControllerState,
            "/right_gripper_controller/controller_state",
            lambda msg: self.gripper_process_callback(msg, "right"),
            1,
            callback_group=self.gripper_cbg,
        )
        self.sub_left_gripper = self.create_subscription(
            JointTrajectoryControllerState,
            "/left_gripper_controller/controller_state",
            lambda msg: self.gripper_process_callback(msg, "left"),
            1,
            callback_group=self.gripper_cbg,
        )

        self.sensor_publisher = self.create_publisher(
            PerceptionMsg, "/oscar/redescribed_sensors", 0
        )
        self.pub_timer = self.create_timer(
            0.01, self.perception_publication, callback_group=self.timer_cbg
        )
        self.pub_trials = 0

        # Threading
        self.image_flag = threading.Event()
        self.gripper_flags = dict(left=threading.Event(), right=threading.Event())
        self.image_semaphore = threading.Semaphore()
        self.gripper_semaphore = threading.Semaphore()

        self.image = None
        self.gripper_error = dict(left=0.0, right=0.0)

        self.configure_camera()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.model_states = None
        self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
            )
        
        self.object_last_diameter = 0.0
        self.bskt_last_diameter = 0.0

        self.get_logger().info("OSCAR perception services ready")

    def image_process_callback(self, msg):
        """
        Callback that reads raw images and transforms them to a numpy array.

        :param msg: Raw image message.
        :type msg: sensor_msgs.msg.Image
        """
        self.image_semaphore.acquire()
        msg_np = numpify(msg)
        self.image = msg_np
        self.image_flag.set()
        self.image_semaphore.release()

    def gripper_process_callback(self, msg: JointTrajectoryControllerState, arm):
        """
        Callback that reads the data from gripper controllers and calculates
        position error.

        :param msg: State of the gripper controllers.
        :type msg: control_msgs.msg.JointTrajectoryControllerState
        :param arm: Selects the arm that corresponds to the data.
        :type arm: str
        """
        self.gripper_semaphore.acquire()
        pos_error = msg.error.positions
        self.gripper_error[arm] = pos_error
        self.gripper_flags[arm].set()
        self.gripper_semaphore.release()

    def perception_service_callback(self, _, response: PerceptionSrv.Response):
        """
        Service callback that provides the latest processed perceptions.

        :param response: Message with the position of the objects and state of the grippers.
        :type response: oscar_emdb_interfaces.srv.Perception.Response
        :return: Message with the position of the objects and state of the grippers.
        :rtype: oscar_emdb_interfaces.srv.Perception.Response
        """
        self.get_logger().info("Processing perception...")
        found, obj, bskt, left, right = self.process_perception()
        robot_hand_x_position, robot_hand_y_position = self.update_hand_position_perceptions()
        sim_obj_x_position, sim_obj_y_position = self.get_sim_object_pose("red_cylinder")
        sim_basket_x_position, sim_basket_y_position = self.get_sim_object_pose("basket")
        self.get_logger().info("Perception processing completed")
        response.success = found
        response.red_object = obj
        response.basket = bskt
        response.obj_in_left_hand = left
        response.obj_in_right_hand = right

        object1 = ObjectPerception()
        object1.label = 1#"cylinder"
        object1.x_position = obj.x
        object1.y_position = obj.y
        object1.color = 1 #"red"
        object1.diameter = self.object_last_diameter
        object1.state = 0
        response.object1 = object1

        object2 = ObjectPerception()
        object2.label = 0 #"basket"
        object2.x_position = bskt.x
        object2.y_position = bskt.y
        object2.color = 0 #"blue"
        object2.diameter = self.bskt_last_diameter
        object2.state = 0
        response.object2 = object2

        robot_hand = ObjectPerception()
        robot_hand.x_position = robot_hand_x_position
        robot_hand.y_position = robot_hand_y_position
        robot_hand.state = right
        response.robot_hand = robot_hand


        return response

    def perception_publication(self):
        """
        Periodic publication of the processed perceptions.
        """

        self.get_logger().debug("DEBUG - Publishing redescribed sensors")
        msg = PerceptionMsg()
        found, obj, bskt, left, right = self.process_perception()
        robot_hand_x_position, robot_hand_y_position = self.update_hand_position_perceptions()
        sim_obj_x_position, sim_obj_y_position = self.get_sim_object_pose("red_cylinder")
        sim_basket_x_position, sim_basket_y_position = self.get_sim_object_pose("basket")

        if found:
            self.pub_trials = 0

        else:
            self.pub_trials += 1

        if found or self.pub_trials >= 20:
            msg.red_object = obj
            msg.basket = bskt
            msg.obj_in_left_hand = left
            msg.obj_in_right_hand = right

            object1 = ObjectPerception()
            object1.label = 1 #"red_cylinder"
            object1.x_position = obj.x
            object1.y_position = obj.y
            object1.color = 1 #"red"
            object1.diameter = self.object_last_diameter
            object1.state = 0
            msg.object1 = object1
            
            object2 = ObjectPerception()
            object2.label = 0 #"basket"
            object2.x_position = bskt.x
            object2.y_position = bskt.y
            object2.color = 0 #"blue"
            object2.diameter = self.bskt_last_diameter
            object2.state = 0
            msg.object2 = object2

            robot_hand = ObjectPerception()
            robot_hand.x_position = robot_hand_x_position
            robot_hand.y_position = robot_hand_y_position
            robot_hand.state = 0
            msg.robot_hand = robot_hand

            self.sensor_publisher.publish(msg)

    def process_perception(self):
        """
        Processes the latest perception data from the camera and grippers.

        :return: Tuple with the position of the objects and state of the grippers. If objects were properly segmented, the flag 'found' is set to True.
        :rtype: tuple
        """

        # Wait for data flags and acquire semaphores to avoid overwriting of data during processing
        self.image_flag.clear()
        self.image_flag.wait()
        for arm in self.gripper_flags.values():
            arm.wait()
        self.image_semaphore.acquire()
        self.gripper_semaphore.acquire()

        found, obj, bskt = self.visual_processing(self.image)
        left, right = self.tactile_processing(
            self.gripper_error["left"], self.gripper_error["right"]
        )

        # Release semaphores and clear flags
        self.image_semaphore.release()
        self.gripper_semaphore.release()
        self.image_flag.clear()
        for arm in self.gripper_flags.values():
            arm.clear()

        return found, obj, bskt, left, right

    def configure_camera(self):
        """
        Sets up the camera settings for segmentation and transformations.
        """
        # intrinsic camera parameters
        self.f = 3.04  # in mm
        self.sensor_size = (3.68, 2.76)  # in mm
        self.image_size = (3280, 2464)  # in px

        # Extrinsic Parameters
        self.camera_elevation = 1.250  # Camera height measured from the table

        # Color Limits
        self.r = (0, 5)
        self.g = (0, 5)
        self.b = (0, 5)
        self.obj_r = (200, 255)
        self.bskt_b = (200, 255)

        # Color Limits HSV
        self.v = np.array([30, 100])
        self.s = np.array([80, 100])
        self.obj_h = np.array([340, 20])
        self.bskt_h = np.array([235, 245])

        # Object height measured from the table (z=0)
        self.object_z = 0.025  # Works for both object and basket

        # Camera
        self.cam = ct.Camera(
            ct.RectilinearProjection(
                focallength_mm=self.f, sensor=self.sensor_size, image=self.image_size
            ),
            ct.SpatialOrientation(elevation_m=self.camera_elevation, tilt_deg=0),
        )

    def visual_processing(self, data: np.ndarray):
        """
        Obtains object poses from an Image.

        :param data: Raw Image in R8G8B8 format.
        :type data: np.ndarray
        :return: Tuple with the positions of the objects. If objects were properly segmented, the flag 'found' is set to True.
        :rtype: tuple
        """

        im = np.frombuffer(data.data, dtype=np.uint8).reshape(
            data.shape[0], data.shape[1], -1
        )
        found = True
        obj = Point()
        bskt = Point()

        im_hsv = cv2.cvtColor(im, cv2.COLOR_RGB2HSV)
        # Segment object and basket
        obj_bin = self.color_segment_hsv(im_hsv, self.obj_h, self.s, self.v)
        bskt_bin = self.color_segment_hsv(im_hsv, self.bskt_h, self.s, self.v)

        # Find object in image
        obj_find, obj_area = self.find_centroid(obj_bin, "red_box")
        bskt_find, bskt_area = self.find_centroid(bskt_bin, "basket")

        # Get object poses in the image frame (Centered in the table, y pointing forward, x pointing right)

        # Object
        if obj_find[0]:
            obj_pose_img = self.cam.spaceFromImage(
                [obj_find[1][0], obj_find[1][1]], Z=self.object_z
            )
            obj_diameter_px = 2 * np.sqrt(obj_area / np.pi)
            self.object_last_diameter = obj_diameter_px / self.image_size[0] 
        else:
            found = False
            obj_pose_img = [
                1.0,
                0.325,
                self.object_z,
            ]  # If not found map to the  right corner of the table

        # Basket
        if bskt_find[0]:
            bskt_pose_img = self.cam.spaceFromImage(
                [bskt_find[1][0], bskt_find[1][1]], Z=self.object_z
            )
            bskt_diameter_px = 2 * np.sqrt(bskt_area / np.pi)
            self.bskt_last_diameter = bskt_diameter_px / self.image_size[0] 
        else:
            found = False
            bskt_pose_img = [
                -1.0,
                0.325,
                self.object_z,
            ]  # If not found map to the left corner of the table

        # MANUALLY Transforming from image frame to World frame

        # Object
        obj.x = obj_pose_img[1] + 0.425  # Translate 425mm
        obj.y = obj_pose_img[0] * -1  # Invert direction
        obj.z = 0.8  # Pick Height
        # Basket
        bskt.x = bskt_pose_img[1] + 0.425  # Translate 425mm
        bskt.y = bskt_pose_img[0] * -1  # Invert direction
        bskt.z = 0.8  # Pick Height

        return found, obj, bskt

    def tactile_processing(self, left_hand, right_hand):
        """
        Detects if the grippers are holding an object.

        :param left_hand: Raw error value for the fingers of the left gripper.
        :type left_hand: list
        :param right_hand: Raw error value for the fingers of the right gripper.
        :type right_hand: list
        :return: Tuple with a boolean for each hand. True if an object was detected.
        :rtype: tuple
        """

        # Error over 5mm in the gripper's PID means an object is gripped
        left_error = [left_hand[0] * -1, left_hand[1] * -1]  # Error is negative
        right_error = [right_hand[0] * -1, right_hand[1] * -1]

        # Average over the two teeth of the gripper
        left_avg = np.mean(left_error)
        right_avg = np.mean(right_error)

        if left_avg > 0.005:
            left = True
        else:
            left = False

        if right_avg > 0.005:
            right = True
        else:
            right = False

        return left, right

    def color_segment_rgb(
        self, rgb_img, r_limits=(0, 255), g_limits=(0, 255), b_limits=(0, 255)
    ):
        """
        Color segmentation according to RGB limits.

        :param rgb_img: Raw image in R8G8B8 format.
        :type rgb_img: np.ndarray
        :param r_limits: Tuple with low, high limits for red channel.
        :type r_limits: tuple
        :param g_limits: Tuple with low, high limits for green channel.
        :type g_limits: tuple
        :param b_limits: Tuple with low, high limits for blue channel.
        :type b_limits: tuple
        :return: Binary image.
        :rtype: np.ndarray
        """

        r_img = rgb_img[:, :, 0]
        g_img = rgb_img[:, :, 1]
        b_img = rgb_img[:, :, 2]

        r_img_bin = (r_img >= r_limits[0]) & (r_img <= r_limits[1])

        g_img_bin = (g_img >= g_limits[0]) & (g_img <= g_limits[1])

        b_img_bin = (b_img >= b_limits[0]) & (b_img <= b_limits[1])

        img_bin = r_img_bin & g_img_bin & b_img_bin

        return img_bin

    def color_segment_hsv(self, hsv_img, h_limits, s_limits, v_limits):
        """
        Color segmentation in the HSV space.

        :param hsv_img: Image in OpenCV HSV format.
        :type hsv_img: np.ndarray
        :param h_limits: Tuple with low, high values for the hue channel.
        :type h_limits: tuple
        :param s_limits: Tuple with low, high values for the saturation channel.
        :type s_limits: tuple
        :param v_limits: Tuple with low, high values for the value channel.
        :type v_limits: tuple
        :return: Binary image.
        :rtype: np.ndarray
        """
        h_limits = ((h_limits / 2).astype(int)).tolist()
        s_limits = ((255 * s_limits / 100).astype(int)).tolist()
        v_limits = ((255 * v_limits / 100).astype(int)).tolist()

        if h_limits[0] <= h_limits[1]:
            mask_low = (h_limits[0], s_limits[0], v_limits[0])
            mask_high = (h_limits[1], s_limits[1], v_limits[1])
            img_bin = cv2.inRange(hsv_img, mask_low, mask_high)

        else:
            mask1_low = (h_limits[0], s_limits[0], v_limits[0])
            mask1_high = (255, s_limits[1], v_limits[1])

            mask2_low = (0, s_limits[0], v_limits[0])
            mask2_high = (h_limits[1], s_limits[1], v_limits[1])

            mask1 = cv2.inRange(hsv_img, mask1_low, mask1_high)
            mask2 = cv2.inRange(hsv_img, mask2_low, mask2_high)
            img_bin = mask1 + mask2

        return img_bin

    def find_centroid(self, img_bin, info_string):
        """
        Finds the centroid of an object in a binary image.

        :param img_bin: Binary image.
        :type img_bin: np.ndarray
        :param info_string: Name of the object (for logging purposes only).
        :type info_string: str
        :return: Tuple with: Success Flag, (pixel height, pixel width).
        :rtype: tuple
        """

        label_image = label(img_bin)
        region = regionprops(label_image)

        num_region = len(region)
        small_region = False

        if len(region) == 1:
            if region[0].area > 1500:
                position = region[0].centroid
                area = region[0].area
                return [True, (
                    position[1],
                    position[0],
                )], area  # Returns flag and image centroid
            else:
                small_region = True         

        # Show warnings if object not found
        if num_region == 0:
            self.get_logger().warn(f"Object {info_string} not found.")

        elif small_region:
            self.get_logger().error(f"Object {info_string}: Small region found.")

        else:
            self.get_logger().error(f"Object {info_string}: Multiple regions found.")

        return [False, (0, 0)], 0.0
    
    def get_hand_position_world(self, hand_frame="right_arm_gripper_link", reference_frame="world"):
        """
        Get the position of the robot hand in the reference frame (default world).

        :param hand_frame: The TF frame of the robot hand.
        :type hand_frame: str
        :param reference_frame: The TF reference frame.
        :type reference_frame: str
        :return: x and y position of the robot hand in the reference frame.
        :rtype: tuple
        """
        try:
            trans = self.tf_buffer.lookup_transform(reference_frame, hand_frame, rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return x, y
        except Exception as e:
            self.get_logger().error(f"TF transform failed: {e}")
            return 0, 0
        
    def update_hand_position_perceptions(self):
        """
        Updates the robot hand position in the perceptions.

        :return: x and y position of the robot hand in the world frame.
        :rtype: tuple
        """
        robot_hand_x_position, robot_hand_y_position = self.get_hand_position_world()
        return robot_hand_x_position, robot_hand_y_position

    def model_states_callback(self, msg):
        """
        Callback to get the model states from Gazebo.

        :param msg: ModelStates message from Gazebo.
        :type msg: gazebo_msgs.msg.ModelStates  
        """
        self.model_states = msg

    def get_sim_object_pose(self, object_name):
        """
        Get the position of a simulated object in Gazebo.

        :param object_name: Name of the object in Gazebo.
        :type object_name: str
        :return: x and y position of the object in the world frame.
        :rtype: tuple
        """
        if self.model_states is None:
            return 0.0, 0.0
        try:
            idx = self.model_states.name.index(object_name)
            pose = self.model_states.pose[idx]
            return pose.position.x, pose.position.y
        except ValueError:
            self.get_logger().warn(f"Modelo {object_name} no encontrado en /gazebo/model_states")
            return 0.0, 0.0


def main():
    rclpy.init()

    oscar_perception = OscarPerception()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(oscar_perception)

    try:
        executor.spin()
    except KeyboardInterrupt:
        executor.shutdown()
        oscar_perception.destroy_node()
