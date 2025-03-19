"""
    Jason Hughes
    December 2024
    
    ROS Node to build factor graph on drone
"""
import rospy

from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry

from localization.factor_manager import FactorManager, nanosecInt2Float 


class FactorGraphNode(FactorManager)

    def __init__(self, config : dict = {}) -> None:
        super().__init__(config)

        self.pose_ = np.zeros(3)
        self.orientation_ = np.zeros(4)
        
        self.odom_pub_ = rospy.Publisher("/global/odom", PoseStamped, queue_size=1)

        rospy.Subscriber("/gps", NavSatFix, self.gps_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
    
    def get_time(self, stamp) -> int:
        return stamp.to_nsec()

    def spin() -> None:
        rospy.spin()

    def imu_callback(self, msg : Imu) -> None:
        """ callback for vector nav imu """
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        
        quat = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])

        timestamp = self.get_time(msg.header.stamp)

        self.factor_manager_.add_imu_factor(timestamp, accel, gyro, quat)


    def odom_callback(self, msg : Odometry) -> None:
        """ callback for odometry """
        pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.y], dtype=np.float32)
        quat = np.array([msg.pose.pose.orientation.x, 
                         msg.pose.pose.orientation.y, 
                         msg.pose.pose.orientation.x, 
                         msg.pose.pose.orientation.w], dtype=np.float32)
        timestamp = self.get_time(self, msg.header.stamp)

        self.add_odometry_factor(timestamp, pose, quat)


    def gps_callback(self, msg : NavSatFix) -> None:
        """ callback for gps message """
        gps_factor = np.zeros(3)

        gps_factor[0] = msg.latitude
        gps_factor[1] = msg.longitude
        gps_factor[2] = msg.altitude

        timestamp = self.get_time(msg.header.stamp)
    
        self.add_gps_factor(timestamp, gps_factor)

        translation, quaternion = self.factor_manager_.runner()
        # TODO change this, we get utm frame from runner.
        if translation is not None:
            self.pose_ = translation
            self.orientation_ = quaternion 
            
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
