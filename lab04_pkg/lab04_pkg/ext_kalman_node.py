import rclpy
from rclpy.node import Node
import tf_transformations
import numpy as np

from lab04_pkg import utils

import angles
from Gaussian_Filters import probabilistic_models as pm 
from Gaussian_Filters.ekf import RobotEKF


from nav_msgs.msg import Odometry
from landmark_msgs.msg import LandmarkArray

file_path = "/home/sirointerlandi/ros_new/src/turtlebot3_perception/turtlebot3_perception/config/landmarks.yaml"


class ExtKalmanNode(Node):
    def __init__(self):
        super().__init__('ext_kalman_node')
        
        ####ROS NODE PART####

        #SUBSCRIBERS

        #Odometry subscriber
        self.subscription = self.create_subscription(Odometry,'/odom',self.velocity_callback,10)
        self.subscription  #prevent unused variable warning
        #Landmark subscriber (reads the landmarks seen from the robot)
        self.subscription = self.create_subscription(LandmarkArray,'/landmarks',self.update_callback,10)

        #Timer to call the prediction 
        self.timer_period=0.05
        self.timer = self.create_timer(self.timer_period, self.prediction_callback)

        #PUBLISHER
        
        #Odometry publisher 
        self.publisher_ = self.create_publisher(Odometry,'/ekf',10)


        #### EKF INSTANTIATION ####
        
        #NOISE PARAMETERS
        #Motion model
        std_lin_vel = 0.1               #[m/s]
        std_ang_vel = np.deg2rad(1.0)   #[rad/s]
        self.sigma_u = np.array([std_lin_vel, std_ang_vel])
        self.sigma_u_odom = 0

        #Sensor model
        std_range = 0.1  # [m]
        std_bearing = np.deg2rad(1.0)  # [rad]
        self.sigma_z = np.array([std_range, std_bearing])
        self.Qt = np.diag([std_range**2, std_bearing**2])

        
        #Istanciate an Ekf object 
        eval_gux = pm.sample_velocity_motion_model
        _, eval_Gt, eval_Vt = pm.velocity_mm_simpy()
        self.kalman_filter = RobotEKF(dim_x=3, dim_u=2, eval_gux=eval_gux, eval_Gt=eval_Gt, eval_Vt=eval_Vt)
        self.kalman_filter.mu = np.array([0.0, 0.0, 0.0])                    #State initialization
        self.kalman_filter.Sigma = np.diag([0.1, 0.1, 0.1])                  #Initial uncertainty
        self.kalman_filter.Mt = np.diag([std_lin_vel**2, std_ang_vel**2])    #Noise parameters

        # Define H Jacobian function
        _, self.eval_Ht = pm.landmark_sm_simpy()
        self.eval_hx_landm = pm.landmark_range_bearing_model
        

        #Map data of the landmarks (taken from a yaml file) and put into a dictionary
        self.landmarks_coordinate=utils.load_landmarks(file_path)

        
        #Command initialization
        self.v = 0.0
        self.w = 0.0
        #Initialization of odometry message read from robot
        self.odom_msg = None
        #State to check if the topics have received some data
        self.ekf_ready = False

        



    def velocity_callback(self, msg:Odometry):
        
        self.odom_msg = msg

        self.v = msg.twist.twist.linear.x   #Linear velocity
        self.w = msg.twist.twist.angular.z  #Angular velocity

        self.ekf_ready = True               #Set the state to True once it starts receiving command informations
        

    def prediction_callback(self):
        #Check to see if it has some commands stored
        if not self.ekf_ready:
            return  
        self.kalman_filter.predict(u=np.array([self.v,self.w]), sigma_u=self.sigma_u, g_extra_args=(0.05,)) #Prediction method    
    
    
    def update_callback(self, msg:LandmarkArray):
        #Check to see if it has some commands stored
        if not self.ekf_ready:
            return

        landmarks_measured = msg                                                           #Store the seen landmarks 
        self.get_logger().info(f'Landmarks received:{len(landmarks_measured.landmarks)}')  #Writes how many landmarks are received
        
        #Every landmark measurement has to be processed by itself so a for cycle is used
        for lmark in landmarks_measured.landmarks: 
            
            z = np.array([lmark.range, lmark.bearing])            #z is the measurement vector
            self.get_logger().info(f'Landmark seen= {lmark.id}')  #Writes the id name of the landmark seen
            id_meas_lmark = lmark.id
            #Calling the update method 
            self.kalman_filter.update(
                        z,
                        eval_hx = self.eval_hx_landm,
                        eval_Ht = self.eval_Ht,
                        Qt = self.Qt,
                        Ht_args = (*self.kalman_filter.mu, *self.landmarks_coordinate[id_meas_lmark]),  # the Ht function requires the parameters unpacked 
                        hx_args = (self.kalman_filter.mu, self.landmarks_coordinate[id_meas_lmark], self.sigma_z),
                        residual = utils.custom_residuals, #A custom function is called from the utils file to compute the residuals
                        angle_idx = -1,)
            
            self.kalman_filter.mu[2] = angles.normalize_angle(self.kalman_filter.mu[2]) #normalization
        
        #CREATING AND PUBLISHING THE ODOMETRY MESSAGE   

        ekf_odom_msg = Odometry()
        #Time
        ekf_odom_msg.header.stamp = self.get_clock().now().to_msg()
        ekf_odom_msg.header.frame_id = 'odom'
        #X,Y Pose
        ekf_odom_msg.pose.pose.position.x = self.kalman_filter.mu[0]
        ekf_odom_msg.pose.pose.position.y = self.kalman_filter.mu[1]
        #Orientation (Yaw)
        quat = tf_transformations.quaternion_from_euler(0, 0, self.kalman_filter.mu[2])
        ekf_odom_msg.pose.pose.orientation.x = quat[0]
        ekf_odom_msg.pose.pose.orientation.y = quat[1]
        ekf_odom_msg.pose.pose.orientation.z = quat[2]
        ekf_odom_msg.pose.pose.orientation.w = quat[3]

        covariance_array = list(np.zeros(36)) #The covariance is a matrix of 36 elements, only the ones of interest are filled
        Sigma = self.kalman_filter.Sigma      #Write the values in a variable
        covariance_array[0] = Sigma[0, 0]
        covariance_array[1] = Sigma[0, 1]
        covariance_array[5] = Sigma[0, 2]
        covariance_array[6] = Sigma[1, 0]
        covariance_array[7] = Sigma[1, 1]
        covariance_array[11] = Sigma[1, 2]
        covariance_array[30] = Sigma[2, 0]
        covariance_array[31] = Sigma[2, 1]
        covariance_array[35] = Sigma[2, 2]
        #Write the matrix into the message
        ekf_odom_msg.pose.covariance = covariance_array

        #Publish
        self.publisher_.publish(ekf_odom_msg)
        self.get_logger().info(f'EKF computed pose: x={self.kalman_filter.mu[0]}, y={self.kalman_filter.mu[1]}, theta={self.kalman_filter.mu[2]}')

    

        
        
        


def main(args=None):
    rclpy.init(args=args)

    node = ExtKalmanNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()