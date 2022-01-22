import copy
import os

import cv2
import numpy as np
import yaml

import rospy
from duckietown_msgs.msg import EpisodeStart, WheelEncoderStamped, WheelsCmdStamped
from sensor_msgs.msg import CameraInfo, CompressedImage, Imu

from aido_schemas import logger
#import timestamp
class ROSAgent:

    def __init__(self):
        # Get the vehicle name, which comes in as HOSTNAME
        self.vehicle = os.getenv("VEHICLE_NAME")
        topic = f"/{self.vehicle}/wheels_driver_node/wheels_cmd"
        self.ik_action_sub = rospy.Subscriber(topic, WheelsCmdStamped, self._ik_action_cb)
        # Place holder for the action, which will be read by the agent in solution.py
        self.action = np.array([0.0, 0.0])
        self.updated = True
        self.initialized = False

        # Publishes onto the corrected image topic
        # since image out of simulator is currently rectified
        # topic = "/{}/image_topic".format(self.vehicle)
        topic = f"/{self.vehicle}/camera_node/image/compressed"
        self.cam_pub = rospy.Publisher(topic, CompressedImage, queue_size=10)

        # Publisher for camera info - needed for the ground_projection
        # topic = "/{}/camera_info_topic".format(self.vehicle)
        topic = f"/{self.vehicle}/camera_node/camera_info"
        self.cam_info_pub = rospy.Publisher(topic, CameraInfo, queue_size=1)

        episode_start_topic = "/{}/episode_start".format(self.vehicle)
        self.episode_start_pub = rospy.Publisher(episode_start_topic, EpisodeStart, queue_size=1, latch=True)

        topic = f"/{self.vehicle}/imu"
        self.imu_pub = rospy.Publisher(topic, Imu, queue_size=10)

        # copied from camera driver:

        left_encoder_topic = "/{}/left_wheel_encoder_node/tick".format(self.vehicle)
        self.left_encoder_pub = rospy.Publisher(left_encoder_topic, WheelEncoderStamped, queue_size=1)
        right_encoder_topic = "/{}/right_wheel_encoder_node/tick".format(self.vehicle)
        self.right_encoder_pub = rospy.Publisher(right_encoder_topic, WheelEncoderStamped, queue_size=1)

        # For intrinsic calibration
        self.cali_file_folder = '/data/config/calibrations/camera_intrinsic/'
        self.frame_id = rospy.get_namespace().strip('/') + '/camera_optical_frame'
        self.cali_file = self.cali_file_folder + f"{self.vehicle}.yaml"

        # Locate calibration yaml file or use the default otherwise
        if not os.path.isfile(self.cali_file):
            rospy.logwarn("Calibration not found: %s.\n Using default instead." % self.cali_file)
            self.cali_file = (self.cali_file_folder + "default.yaml")

        # Shutdown if no calibration file not found
        if not os.path.isfile(self.cali_file):
            rospy.signal_shutdown("Found no calibration file. Aborting")

        # Load the calibration file
        self.original_camera_info = self.load_camera_info(self.cali_file)
        self.original_camera_info.header.frame_id = self.frame_id
        self.current_camera_info = copy.deepcopy(self.original_camera_info)
        rospy.loginfo(f"Using calibration file: {self.cali_file}")

        # Initializes the node
        rospy.loginfo('Just before init_node [start]')
        logger.info('my message 1')
        rospy.loginfo('Just before init_node [end]')
        rospy.init_node("ROSTemplate", log_level=rospy.DEBUG)
        rospy.loginfo('Just after init_node [start]')
        logger.info('my message 2')
        rospy.loginfo('Just after init_node [end]')

        self.frame_id = 0
        
    def _ik_action_cb(self, msg):
        """
        Callback to listen to last outputted action from inverse_kinematics node
        Stores it and sustains same action until new message published on topic
        """
        self.initialized = True
        vl = msg.vel_left
        vr = msg.vel_right
        self.action = np.array([vl, vr])
        self.updated = True

    def _publish_info(self):
        """
        Publishes a default CameraInfo - TODO: Fix after distortion applied in simulator
        """
        # Publish the CameraInfo message
        stamp = rospy.Time.now()
        self.current_camera_info.header.stamp = stamp
        self.cam_info_pub.publish(self.current_camera_info)

    def _publish_imu(self, timestamp, angular_velocity, accel):
        imu = Imu()
        
        imu.header.frame_id = f"{self.frame_id}"
        imu.header.stamp = rospy.Time.from_sec(timestamp)
        #time = timestamp #rospy.get_rostime()
        #imu.header.stamp.secs = time.secs
        #imu.header.stamp.nsecs = time.nsecs
        '''
        not required by slam
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        '''
        imu.linear_acceleration.x = accel[0]
        imu.linear_acceleration.y = accel[1]
        imu.linear_acceleration.z = accel[2]
        imu.angular_velocity.x = angular_velocity[0]
        imu.angular_velocity.y = angular_velocity[1]
        imu.angular_velocity.z = angular_velocity[2]
        self.imu_pub.publish(imu)

    def _publish_episode_start(self, episode_name: str, payload_yaml: str):
        episode_start_message = EpisodeStart()
        stamp = rospy.Time.now()
        episode_start_message.header.stamp = stamp
        episode_start_message.episode_name = episode_name
        episode_start_message.other_payload_yaml = payload_yaml
        self.episode_start_pub.publish(episode_start_message)

    def _publish_img(self, obs, time):
        """
        Publishes the image to the compressed_image topic, which triggers the lane following loop
        """

        # XXX: make this into a function (there were a few of these conversions around...)
        img_msg = CompressedImage()

        print('my publish')

        #time = rospy.get_rostime()
        img_msg.header.stamp = rospy.Time.from_sec(time)
        #img_msg.header.stamp.secs = time.secs
        #img_msg.header.stamp.nsecs = time.nsecs

        img_msg.format = "jpeg"
        img_msg.data = obs

        self.cam_pub.publish(img_msg)

    def _publish_odometry(self, resolution_rad: float, left_rad: float, right_rad: float):
        """
        :param resolution_rad:
        :param left_rad:
        :param right_rad:
        :return: none
        """
        if resolution_rad == 0:
            rospy.logerr("Can't interpret encoder data with resolution 0")

        self.left_encoder_pub.publish(
            WheelEncoderStamped(
                data=int(np.round(left_rad / resolution_rad)),
                resolution=int(np.round(np.pi * 2 / resolution_rad)),
                type=WheelEncoderStamped.ENCODER_TYPE_INCREMENTAL
            )
        )
        self.right_encoder_pub.publish(
            WheelEncoderStamped(
                data=int(np.round(right_rad / resolution_rad)),
                resolution=int(np.round(np.pi * 2 / resolution_rad)),
                type=WheelEncoderStamped.ENCODER_TYPE_INCREMENTAL
            )
        )

    @staticmethod
    def load_camera_info(filename):
        """Loads the camera calibration files.

        Loads the intrinsic and extrinsic camera matrices.

        Args:
            filename (:obj:`str`): filename of calibration files.

        Returns:
            :obj:`CameraInfo`: a CameraInfo message object

        """
        with open(filename, 'r') as stream:
            calib_data = yaml.load(stream, Loader=yaml.Loader)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info
