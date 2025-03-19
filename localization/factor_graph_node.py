"""
    Jason Hughes
    December 2024
    
    ROS2 Node to build factor graph on drone
"""

import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.time import Time
from px4_msgs.msg import VehicleGlobalPosition, SensorGps
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import SensorCombined
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber

from localization.factor_manager import FactorManager 
from typing import Dict, Tuple

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1  # Only keep the latest message
)

class FactorGraphNode(Node):

    def __init__(self, config : Dict = { }) -> None:
        super().__init__('factor_localizer_node')

        self.pose_ = np.zeros(3)
        self.orientation_ = np.zeros(4)

        self.declare_parameter("sim", False)
        sim = self.get_parameter("sim").get_parameter_value().bool_value
        self.factor_manager_ = FactorManager(config=config)

        # use two callback groups so GPS doesn't block 
        imu_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        gps_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        if sim:
            #imu_sub = self.create_subscription(SensorCombined, "/imu", lambda msg: None, qos_profile, callback_group=imu_group)
            #att_sub = self.create_subscription(VehicleAttitude, "/attitude", lambda msg: None, qos_profile, callback_group=imu_group)
            
            self.gps_sub_ = self.create_subscription(SensorGps, "/gps", self.gpsPixhawkCallback, qos_profile, callback_group=gps_group) #TODO incompatible QoS profile
            self.imu_sub_ = Subscriber(self, SensorCombined, "/imu", callback_group=imu_group, qos_profile=qos_profile)
            self.attitude_sub_ = Subscriber(self, VehicleAttitude, "/attitude", callback_group=imu_group, qos_profile=qos_profile)

            self.ts_cb_ = ApproximateTimeSynchronizer([self.imu_sub_, self.attitude_sub_], queue_size=2, slop=0.1, allow_headerless=True)
            self.ts_cb_.registerCallback(self.imu_sim_callback)
        else:
            #TODO imu will come from vectornav, use that to calculate quaternion
            self.imu_sub_ = self.create_subscription(Imu, "/imu", self.imuCallback, 1, callback_group=imu_group)
            self.gps_sub_ = self.create_subscription(NavSatFix, "/gps", self.gpsCallback, qos_profile, callback_group=gps_group) #TODO incompatible QoS profile
        self.odom_pub_ = self.create_publisher(Odometry, "/odom", 1)

        # For sim only
        self.imu_imu_ = 0.0
        self.gyro_factor_ = np.zeros(3)
        self.accel_factor_ = np.zeros(3)
        self.get_logger().debug("DEBUG TEST")

        self.get_logger().info("Initialized with sim set to %s" %sim)

    @classmethod
    def get_time(self, stamp) -> int:
        return Time.from_msg(stamp).nanoseconds


    @classmethod
    def quaternion_multiply(self, q1 : np.ndarray, q2 : np.ndarray) -> np.ndarray:
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        
        return np.array([x, y, z, w])

    def gpsCallback(self, msg : NavSatFix) -> None:
        """ GPS output from VectorNAV """
        #self.get_logger().debug("Adding GPS factor")
        gps_factor = np.zeros(3)

        gps_factor[0] = msg.latitude
        gps_factor[1] = msg.longitude
        gps_factor[2] = msg.altitude

        timestamp = self.get_time(msg.header.stamp)
    
        self.factor_manager_.add_gps_factor(timestamp, gps_factor)

        translation, quaternion = self.factor_manager_.runner()
        # TODO change this, we get utm frame from runner.
        if translation is not None:
            self.pose_ = translation
            self.orientation_ = quaternion #self.quaternion_multiply(quaternion, self.orientation_)
            
            # publish  in local frame
            odom_msg = Odometry()
            odom_msg.pose.pose.position.x = self.pose_[0]
            odom_msg.pose.pose.position.y = self.pose_[1]
            odom_msg.pose.pose.position.z = self.pose_[2]

            odom_msg.pose.pose.orientation.x = self.orientation_[0]
            odom_msg.pose.pose.orientation.y = self.orientation_[1]
            odom_msg.pose.pose.orientation.z = self.orientation_[2]
            odom_msg.pose.pose.orientation.w = self.orientation_[3]

            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "utm"

            self.odom_pub_.publish(odom_msg)
    
    def gpsPixhawkCallback(self, msg : SensorGps) -> None:
        """ GPS output from pixhawk """
        #self.get_logger().debug("Adding GPS factor")
        gps_factor = np.zeros(3)

        gps_factor[0] = msg.latitude_deg
        gps_factor[1] = msg.longitude_deg
        gps_factor[2] = msg.altitude_msl_m

        timestamp = int(msg.timestamp * 1000)
    
        self.factor_manager_.add_gps_factor(timestamp, gps_factor)

        translation, quaternion = self.factor_manager_.runner()
        # TODO change this, we get utm frame from runner.
        if translation is not None:
            self.pose_ = translation
            self.orientation_ = quaternion #self.quaternion_multiply(quaternion, self.orientation_)
            
            # publish  in local frame
            odom_msg = Odometry()
            odom_msg.pose.pose.position.x = self.pose_[0]
            odom_msg.pose.pose.position.y = self.pose_[1]
            odom_msg.pose.pose.position.z = self.pose_[2]

            odom_msg.pose.pose.orientation.x = self.orientation_[0]
            odom_msg.pose.pose.orientation.y = self.orientation_[1]
            odom_msg.pose.pose.orientation.z = self.orientation_[2]
            odom_msg.pose.pose.orientation.w = self.orientation_[3]

            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "utm"

            self.odom_pub_.publish(odom_msg)


    def imuCallback(self, msg : Imu) -> None:
        """ callback for vector nav imu """
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        
        quat = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])

        timestamp = self.get_time(msg.header.stamp)

        self.factor_manager_.add_imu_factor(timestamp, accel, gyro, quat)


    ## SIM CALLBACKS -- this is garbage
    def imu_sim_callback(self, imu_msg : SensorCombined, att_msg : VehicleAttitude) -> None:
        """ 
            callback for gyro and accelerometer direct from pixhawk. There is no orientation, thats a seperate message.
            FOR SIM ONLY
        """
        #self.get_logger().info("got imu")
        gyro = np.array(imu_msg.gyro_rad)
        accel = np.array(imu_msg.accelerometer_m_s2)

        quat = np.array([att_msg.q[3], att_msg.q[0], att_msg.q[1], att_msg.q[2]])
        timestamp = int(imu_msg.timestamp * 1000)
        print(timestamp)
        self.factor_manager_.add_imu_factor(imu_msg.timestamp, accel, gyro, quat)


