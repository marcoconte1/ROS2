import rclpy
from rclpy.node import Node
import tf_transformations
import numpy as np
import angles

from nav_msgs.msg import Odometry
from landmark_msgs.msg import LandmarkArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

from lab04_pkg import utils

# IMPORTANTISSIMO: usa la tua Gaussian_Filters del package (non quella in ~/.local)
from .Gaussian_Filters import probabilistic_models as pm
from .Gaussian_Filters.ekf import RobotEKF


file_path = "/home/sirointerlandi/ros_new/src/turtlebot3_perception/turtlebot3_perception/config/landmarks.yaml"


class ExtKalmanNode(Node):
    def __init__(self):
        super().__init__('ext_kalman_node')

        # SUBS
        self.cmdsub = self.create_subscription(Odometry, '/odom', self.velocity_callback, 10)
        self.landsub = self.create_subscription(LandmarkArray, '/landmarks', self.landmark_callback, 10)
        self.imusub = self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.wheelsub = self.create_subscription(Odometry, "/odom", self.whell_encoder_callback, 10)

        # TIMER prediction
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.prediction_callback)

        # PUB
        self.publisher_ = self.create_publisher(Odometry, '/ekf', 10)

        # NOISE
        std_lin_vel = 0.1
        std_ang_vel = np.deg2rad(1.0)
        self.sigma_u = np.array([std_lin_vel, std_ang_vel], dtype=float)

        std_range = 0.1
        std_bearing = np.deg2rad(1.0)
        self.Qt_land = np.diag([std_range**2, std_bearing**2]).astype(float)

        # EKF
        # usa il modello 5D del tuo pm (quello “due/due state” che hai nel tuo workspace)
        eval_gux = pm.sample_velocity_motion_model_due
        _, eval_Gt, eval_Vt = pm.velocity_mm_simpy_due()

        self.kalman_filter = RobotEKF(dim_x=5, dim_u=2, eval_gux=eval_gux, eval_Gt=eval_Gt, eval_Vt=eval_Vt)
        self.kalman_filter.mu = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)
        self.kalman_filter.Sigma = np.diag([0.1, 0.1, 0.1, 0.01, 0.01]).astype(float)
        self.kalman_filter.Mt = np.diag([std_lin_vel**2, std_ang_vel**2]).astype(float)

        # LANDMARK MODEL (deterministico, NO rumore in hx!)
        self.eval_hx_landm, self.eval_Ht_landm = pm.landmark_sm_simpy_due()

        # MAP landmarks
        self.landmarks_coordinate = utils.load_landmarks(file_path)

        # cmd
        self.v = 0.0
        self.w = 0.0
        self.ekf_ready = False

    # ---------- measurement models COSTANTI per IMU e Wheel ----------
    def hx_imu(self, x, y, theta, v, w):
        return np.array([w], dtype=float)

    def Ht_imu(self, x, y, theta, v, w):
        return np.array([[0.0, 0.0, 0.0, 0.0, 1.0]], dtype=float)

    def hx_wheel(self, x, y, theta, v, w):
        return np.array([v, w], dtype=float)

    def Ht_wheel(self, x, y, theta, v, w):
        return np.array([
            [0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0],
        ], dtype=float)

    # --------------------------------------------------------------

    def velocity_callback(self, msg: Twist):
        self.v = float(msg.twist.twist.linear.x)
        self.w = float(msg.twist.twist.angular.z)
        self.ekf_ready = True

    def prediction_callback(self):
        if not self.ekf_ready:
            return
        
        w = self.w
        if abs(w) < 1e-6:
            w = 1e-6


        self.kalman_filter.predict(
            u=np.array([self.v, w], dtype=float),
            sigma_u=self.sigma_u,
            g_extra_args=(self.timer_period,)
        )
        self.kalman_filter.mu[2] = angles.normalize_angle(self.kalman_filter.mu[2])
        self.publish_ekf()

    def landmark_callback(self, msg: LandmarkArray):
        if not self.ekf_ready:
            return

        for meas in msg.landmarks:
            lm_id = meas.id
            if lm_id not in self.landmarks_coordinate:
                continue

            z = np.array([meas.range, meas.bearing], dtype=float)
            lm = self.landmarks_coordinate[lm_id]  # tipicamente [mx, my]

            self.kalman_filter.update(
                z,
                eval_hx=self.eval_hx_landm,
                eval_Ht=self.eval_Ht_landm,
                Qt=self.Qt_land,
                Ht_args=(*self.kalman_filter.mu[0:3], *lm),
                hx_args=(*self.kalman_filter.mu[0:3], *lm),
                residual=utils.custom_residuals,
                angle_idx=-1
            )

            self.kalman_filter.mu[2] = angles.normalize_angle(self.kalman_filter.mu[2])

        self.publish_ekf()

    def imu_callback(self, msg: Imu):
        if not self.ekf_ready:
            return

        z_imu = np.array([msg.angular_velocity.z], dtype=float)
        std_imu = 0.1
        Qt_imu = np.diag([std_imu**2]).astype(float)

        self.kalman_filter.update(
            z_imu,
            eval_hx=self.hx_imu,
            eval_Ht=self.Ht_imu,
            Qt=Qt_imu,
            Ht_args=(*self.kalman_filter.mu,),
            hx_args=(*self.kalman_filter.mu,),
            residual=np.subtract
        )

        self.kalman_filter.mu[2] = angles.normalize_angle(self.kalman_filter.mu[2])
        self.publish_ekf()

    def whell_encoder_callback(self, msg: Odometry):
        if not self.ekf_ready:
            return

        z_wheel = np.array([msg.twist.twist.linear.x, msg.twist.twist.angular.z], dtype=float)
        std_v = 0.1
        std_w = 0.1
        Qt_wheel = np.diag([std_v**2, std_w**2]).astype(float)

        self.kalman_filter.update(
            z_wheel,
            eval_hx=self.hx_wheel,
            eval_Ht=self.Ht_wheel,
            Qt=Qt_wheel,
            Ht_args=(*self.kalman_filter.mu,),
            hx_args=(*self.kalman_filter.mu,),
            residual=np.subtract
        )

        self.kalman_filter.mu[2] = angles.normalize_angle(self.kalman_filter.mu[2])
        self.publish_ekf()

    def publish_ekf(self):
        ekf_odom_msg = Odometry()
        ekf_odom_msg.header.stamp = self.get_clock().now().to_msg()
        ekf_odom_msg.header.frame_id = 'odom'

        ekf_odom_msg.pose.pose.position.x = float(self.kalman_filter.mu[0])
        ekf_odom_msg.pose.pose.position.y = float(self.kalman_filter.mu[1])

        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, float(self.kalman_filter.mu[2]))
        ekf_odom_msg.pose.pose.orientation.x = quat[0]
        ekf_odom_msg.pose.pose.orientation.y = quat[1]
        ekf_odom_msg.pose.pose.orientation.z = quat[2]
        ekf_odom_msg.pose.pose.orientation.w = quat[3]

        ekf_odom_msg.twist.twist.linear.x = float(self.kalman_filter.mu[3])
        ekf_odom_msg.twist.twist.angular.z = float(self.kalman_filter.mu[4])

        # cov (solo x,y,theta “mappati” su pose.covariance)
        cov = np.zeros(36, dtype=float)
        S = self.kalman_filter.Sigma
        cov[0] = S[0, 0]
        cov[1] = S[0, 1]
        cov[5] = S[0, 2]
        cov[6] = S[1, 0]
        cov[7] = S[1, 1]
        cov[11] = S[1, 2]
        cov[30] = S[2, 0]
        cov[31] = S[2, 1]
        cov[35] = S[2, 2]
        ekf_odom_msg.pose.covariance = cov.tolist()

        self.publisher_.publish(ekf_odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ExtKalmanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
