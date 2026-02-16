import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
#Import the necessary message types
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
#IMport the necessary libraries for computations
from tf_transformations import euler_from_quaternion
import math



class BumpAndGo(Node):
    def __init__(self):
        super().__init__('controller')

        #Publisher: Velocity commands
        self.cmd_velocity = self.create_publisher(Twist, '/cmd_vel', 10)

        #Subscibers:  Scan and Odometry topics
        #self.scan_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)                      #Simulation subscriber
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data)  #Real Robot subscriber
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Timer: controls frequency of velocity updates
        timer_period=0.2  # seconds
        self.timer = self.create_timer(timer_period, self.velocity_callback)

        #Initializing the laser data and odometry data
        self.scan_info = None
        self.yaw_odom_info = 0.0

        #Parameters 
        self.linear_vel = 0.2                           # linear velocity at which the robot goes [m/s] 
        self.angular_vel = 0.4                          # angular velocity at which the robot goes [rad/s]
        self.front_dist_threshold = 0.2                 # lowest distance value for which it can go forward[m]
        self.yaw_tolerance = math.radians(3)            # tolerance for the rotation angle [rad]
        self.max_turn_angle = math.radians(45)          # the maximum angle that the robot uses to rotate [rad]

        # State variables
        #these variables are used to manage the rotation during time
        self.rotating = False
        self.target_yaw = None

    
    #Subscriber callbacks
    def laser_callback(self, msg: LaserScan):
        self.scan_info = msg

    def odom_callback(self, msg: Odometry):
        quat = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.yaw_odom_info = yaw #we are interested only in the yaw information from the odometry

    
    #Functions that are helpful for the algorithm

    def search_direction(self, ranges):
        #This function searches the best direction in which the robot has to rotate to avoid collision
        total_points = len(ranges)
        angle_increment = self.scan_info.angle_increment   #in our case it is 1 degree (0.01750 rad) and it's the resolution of the lidar 
        angle_min = self.scan_info.angle_min
        max_angle = self.max_turn_angle                    #checks around itself in a 45° sector 

        sector_points = int(max_angle / angle_increment)   #setting the number of points in each sector

        # Left sector from 0° to 45° (based on the convention of LaserScan)
        left_sector = ranges[:sector_points]
        # Right sector from -45° to 0°
        right_sector = ranges[-sector_points:]

        #looking for the maximum distance in the two sectors
        left_max = max(left_sector)
        right_max = max(right_sector)

        #choosing the best direction based on the comparison between the two maximums
        if left_max > right_max:
            best_index = left_sector.index(left_max)   #taking the index of the maximum point 
            angle = best_index * angle_increment       #positive angle
            side = "left"                              #used in the getlogger message
        else:
            best_index = right_sector.index(right_max)
            angle = -((sector_points - best_index) * angle_increment) #if the robot chooses to rotate right it has to 
                                                                      #do a negative rotation -((45°-index)*angle_increment)
            side = "right"

        #The angle chosen must be limited from turning too much so it can rotate +-45° 
        # or less with a value calculated above
        angle = max(-self.max_turn_angle, min(self.max_turn_angle, angle))

        # Compute absolute target yaw
        new_target_yaw = self.yaw_odom_info + angle #It's the actual yaw plus the angle calculated above
        new_target_yaw = (new_target_yaw + math.pi) % (2 * math.pi) - math.pi  #Normalization of the angle

        self.get_logger().info(
        f"Chosen direction: {side}, relative angle: {math.degrees(angle):.1f}°, "
        f"target yaw: {math.degrees(new_target_yaw):.1f}°")

        return new_target_yaw


    def scan_front(self, ranges):
        #This function checks 20° of the laser scan with the center at the front of the robot
        #and returns the minimum distance that detects in that sector
        front_sector = ranges[-10:] + ranges[:10]
        return min(front_sector)

    #Publisher control callback

    def velocity_callback(self):
        if self.scan_info is None:             #check if there is data to process
            return

        twist = Twist()                        #initializing the Twist message
        #calling the scan front to see how much we are close to an obstacle
        front_min = self.scan_front(self.scan_info.ranges)

        #From here we enter into the control of the robot guided by the states it can assume
        if not self.rotating: #If the robot is not rotating...

            if front_min > self.front_dist_threshold: #it can go forward
                twist.linear.x = self.linear_vel
                twist.angular.z = 0.0
                self.get_logger().info(f'Moving Forward, Linear: {twist.linear.x:.2f}') #publishes that is going forward
            
            else:                         #if it cannot go forward it has to rotate
                self.rotating = True      #updating the state to rotating
                self.target_yaw = self.search_direction(self.scan_info.ranges)
                twist.linear.x = 0.0
                twist.angular.z = self.angular_vel
                self.get_logger().info('Obstacle detected, starting rotation...')
        else:
            #Once entered in the rotaing state can accomplish the rotation
            yaw_error = self.target_yaw - self.yaw_odom_info
            yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi  # normalize

            if abs(yaw_error) > self.yaw_tolerance: #It rotates until the error is less than the tolerance
                twist.linear.x = 0.0
                twist.angular.z = self.angular_vel if yaw_error > 0 else -self.angular_vel #here chooses if it has to rotate left or right
                self.get_logger().info(f'Rotating... yaw error: {math.degrees(yaw_error):.1f}°')
            else:
                #When the error is under the tolerance it can stop and start again the forward motion
                self.rotating = False  #updates the state
                self.target_yaw = None #resets the target yaw
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info('Rotation complete. Checking the path.')

        self.cmd_velocity.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    controller = BumpAndGo()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()