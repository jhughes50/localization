"""
    Jason Hughes
    December 2024
    
    ROS Node to build factor graph on drone
"""
import rospy

from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry

from localization.factor_manager import FactorManager, nanosecInt2Float 
from converter import LLtoUTM, UTMtoLL

class FactorGraphNode(FactorManager)

    def __init__(self, config : dict = {}) -> None:
        super().__init__(config)

        self.pose_ = np.zeros(3)
        self.orientation_ = np.zeros(4)
        
        self.starting_utm_ = (0, 0)
        self.initialized_ = False
        self.zone_ = '18r'

        self.local_pub_ = rospy.Publisher("/local/odom", Odometry, queue_size=1)
        self.global_pub_ = rospy.Publisher("/global/odom", NavSatFix, queue_size=1)

        rospy.Subscriber("/gps", NavSatFix, self.gps_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        
        
    @classmethod
    def get_time(self, stamp : Time) -> int:
        return int(stamp.to_nsec())


    @classmethod
    def quaternion_multiply(self, q1 : np.ndarry, q2 : np.ndarray) -> np.ndarray:
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        
        return np.array([x, y, z, w])


    def imu_callback(self, msg : Imu) -> None:
        """ callback for vector nav imu """
        gyro = np.array([msg.angular_velocity.x, msg.angular_velcoity.y, msg.angular_velcoity.z])
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        
        quat = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])

        timestamp = self.get_time(msg.header.stamp)

        self.add_imu_factor(timestamp, accel, gyro, quat)


    def odom_callback(self, msg : Odometry) -> None:
        """ callback for odometry """
        pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.y], dtype=np.float32)
        quat = np.array([msg.pose.pose.orientation.x, 
                         msg.pose.pose.orientation.y, 
                         msg.pose.pose.orientation.x, 
                         msg.pose.pose.orientation.w], dtype=np.float32)
        timestamp = self.get_time(self, msg.header.stamp)

        self.add_odometry_factor(timestamp, pose, quat)

        translation, quat = self.runner()

        self.pose_ += translation
        self.orientation_ = self.quaternion_multiply(quat, self.orientation_)
        
        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = self.pose_[0]
        odom_msg.pose.pose.position.y = self.pose_[1]
        odom_msg.pose.pose.position.z = self.pose_[2]

        odom_msg.pose.pose.orientation.x = self.orientation_[0]
        odom_msg.pose.pose.orientation.y = self.orientation_[1]
        odom_msg.pose.pose.orientation.z = self.orientation_[2]
        odom_msg.pose.pose.orientation.w = self.orientation_[3]

        self.local_pub_.publish(odom_msg)

        if self.initialized_:
            current_easting = self.starting_utm_[0] + self.pose_[0]
            current_northing = self.starting_utm_[1] + self.pose_[1]
            lat, lon = UTMtoLL(23, current_northing, current_easting, self.zone_)

            nsf_msg = NavSatFix()
            nsf_msg.lat = lat
            nsf_msg.lon = lon
            nsf_msg.altitude = self.pose_[2]

            self.global_pub_.publish(nsf_msg)
        

    def gpsCallback(self, msg : NavSatFix) -> None:
        """ callback for gps message """
        if self.initialized_:
            # probably not the way to do this 
            self.zone_, easting, northing = LLtoUTM(23, msg.lat, msg.lon)
            self.start_utm_ = (easting, northing)
            self.initialized_ = True

        gps_factor = np.zeros(3)

        gps_factor[0] = msg.lat
        gps_factor[1] = msg.lon
        gps_factor[2] = msg.alt
        timestamp = self.get_time(msg.header.stamp)
    
        self.add_gps_factor(timestamp, gps_factor)
