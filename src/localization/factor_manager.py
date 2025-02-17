import gtsam
import numpy as np
import utm
from gtsam.symbol_shorthand import B, V, X


def vector3(x, y, z) -> np.ndarray:
    """Create 3d double numpy array."""
    return np.array([x, y, z], dtype=float)


NED2ENU = np.array([[0., 1., 0.],
                    [1., 0., 0.],
                    [0., 0., -1.]])


def nanosecInt2Float(timestamp: int) -> float:
    return timestamp * 1e-9


class FactorManager():
    def __init__(self, config: dict) -> None:
       
        self.config = {}
        self.config["acceleration_covariance"] = 0.1
        self.config["gyroscope_covariance"] = 0.1
        self.config["integration_covariance"] = 0.1
        self.config["use_second_order"] = False
        self.config["bias_covariance"] = 0.1
        self.config["origin"] = np.array([0., 0.])
        self.config["gravity"] = 9.81
        self.config["bias_num_measurements"] = 100
        self.config["T_imu2body"] = np.array([[1., 0., 0.],
                                              [0., 1., 0.],
                                              [0., 0., 1.]])
        self.config["gps_noise"] = 1.0  # meters
        self.config["odom_noise"] = 1.0
        
        # Allow defaults by not overwriting
        for k in config.keys():
            self.config[k] = config[k]

        self.gravity_vec = np.array([0., 0., self.config["gravity"]])
        self.bias_estimate_vec = np.zeros((self.config["bias_num_measurements"], 6))
        self.init_counter = 0.
        self.imu2body = self.config["T_imu2body"]
        
        # All _ variables are private
        self._initialized = False
        self._key_index = 0
        self._lastOptimizeTime = 0.
        self._lastImuTime = 0.
        self.params = self.defaultParams(self.config["gravity"])

        self._prior_noise = \
            gtsam.noiseModel.Isotropic.Sigma(6, self.config["gps_noise"])
        self._odom_noise = \
            gtsam.noiseModel.Isotropic.Sigma(6, self.config["odom_noise"])
        self._gps_noise = \
            gtsam.noiseModel.Isotropic.Sigma(3, self.config["gps_noise"])

        # Optimization related variables
        self._graph = gtsam.NonlinearFactorGraph()
        self._params = gtsam.GaussNewtonParams()
        self._initials = gtsam.Values()
        self._parameters = gtsam.ISAM2Params()
        self._parameters.setRelinearizeThreshold(0.1)
        self._parameters.relinearizeSkip = 1
        self._isam = gtsam.ISAM2(self._parameters)

        print("Factor Manager Initialized")

    @staticmethod
    def defaultParams(g: float):
        """Create default parameters with Z *up* """
        # TODO some problem here
        params = gtsam.PreintegrationParams.MakeSharedU(g)
        kGyroSigma = np.radians(0.5) / 60  # 0.5 degree ARW
        kAccelSigma = 0.1 / 60  # 10 cm VRW
        I = np.eye(3)
        params.setGyroscopeCovariance(kGyroSigma**2 * I) # PROBLEM IS HERE TODO fix this
        params.setAccelerometerCovariance(kAccelSigma**2 * I)
        params.setIntegrationCovariance(0.0000001**2 * I)

        return params

    def imu_initialize(self, accel_meas: np.ndarray,
                       gyro_meas: np.ndarray, orient: np.ndarray) -> None:
        if self.init_counter < self.config["bias_num_measurements"]:
            self.bias_estimate_vec[self.init_counter, :3] = \
                self.imu2body @ accel_meas - self.gravity_vec
            self.bias_estimate_vec[self.init_counter, 3:] = \
                self.imu2body @ gyro_meas
            self.init_counter += 1

        if self.init_counter == self.config["bias_num_measurements"]:
            self.bias = np.mean(self.bias_estimate_vec, axis=1)
            self.bias = \
                gtsam.imuBias.ConstantBias(self.bias[:3], self.bias[3:])
            self.pim = \
                gtsam.PreintegratedCombinedMeasurements(self.params, self.bias)

            # GTSAM expects quaternion as w,x,y,z
            self.initial_orientation = self.imu2body @ gtsam.Rot3(orient[0],
                                                                  orient[1],
                                                                  orient[2],
                                                                  orient[3]).matrix()
            self.initial_orientation = gtsam.Rot3(self.initial_orientation)
            self._initialized = True

    # Called on every GPS measurement
    def add_gps_factor(self, timestamp: int, gps: np.array) -> None:
        if not self._initialized:
            return
        meas = np.zeros(3)
        meas = np.array(utm.from_latlon(gps[0], gps[1])[0])
        if self._key_index == 0:
            self.navstate_pose = \
                    gtsam.Pose3(self.initial_orientation,
                                np.array([meas[0],
                                          meas[1],
                                          meas[2]]))
            self.init_navstate = \
                gtsam.NavState(self.navstate_pose, gtsam.Point3(), self.bias)
            self.lastNavState = self.init_navstate
            self._initials.insert(X(self._key_index), self.navstate_pose)
            self._initials.insert(V(self._key_index), gtsam.Point3())
            self._initials.insert(B(self._key_index), self.bias)

        self._graph.add(gtsam.GPSFactor(self._key_index,
                                        meas, self._gps_noise))
        if self._key_index > 0:
            self._graph.add(gtsam.CombinedImuFactor(X(self._key_index),
                                                    V(self._key_index),
                                                    X(self._key_index-1),
                                                    V(self._key_index-1),
                                                    B(self._key_index),
                                                    B(self._key_index-1),
                                                    self.pim))

        self._key_index += 1

    def add_odometry_factor(self, timestamp: int, pose : np.array, quat : np.array) -> None:
        if not self._initialized:
            return
        rotation = gtsam.Rot3.Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        meas = gtsam.Pose3(rotation, pose)
        key = X(timestamp)
        self._graph.add(gtsam.BetweenFactorPose3(X,
                                                 self._key_index,
                                                 meas,
                                                 self._odom_noise))
        self._key_index = key

    # Called on every IMU measurement
    def add_imu_factor(self,
                       timestamp: int,
                       accel: np.ndarray,
                       gyro: np.ndarray,
                       orient: np.ndarray) -> None:
        if not self._initialized:
            self._lastOptimizeTime = nanosecInt2Float(timestamp)
            self._lastImuTime = nanosecInt2Float(timestamp)
            self.imu_initialize(accel, gyro, orient)
            return
        accel_meas = self.imu2body @ accel
        gyro_meas = self.imu2body @ gyro
        dT = nanosecInt2Float(timestamp) - self._lastImuTime
        self.pim.integrateMeasurement(accel_meas, gyro_meas, dT)
        self._lastImuTime

    def initialize_graph(self) -> None:
        if not self._initialized:
            return
        self._initials = gtsam.InitializePose3.initialize(self._graph)

    def optimize(self) -> gtsam.Values:
        if not self._initialized:
            return None
        self._optimizer = gtsam.GaussNewtonOptimizer(self._graph,
                                                     self._initials,
                                                     self._params)
        result = self._optimizer.optimize()
        print("\nFinal:{}".format(result))
        return result

    # Main call loop
    def runner(self) -> Tuple[np.ndarray, np.ndarray]:
        if not self._initialized:
            return None
        self.initialize_graph()
        result = self.optimize()
        self.pim.resetIntegration()

        optimized_pose = result.atPose3(self._key_index) 
        rotation = optimized_pose.rotation()
        translation = optimized_pose.translation().vector()
        quaternion = rotation.quaternion()
        quaternion = np.array([quat.x(), quat.y(), quat.z(), quat.w()])

        return translation, quaternion


# Unit test things
if __name__ == "__main__":
    print("Adding factors")
    fm = FactorManager(config={})
    fm.add_gps_factor(0, np.array([20.2, 7.2]))
    fm.add_odometry_factor(0, gtsam.Pose3())
    fm.initialize_graph()
    fm.optimize()
