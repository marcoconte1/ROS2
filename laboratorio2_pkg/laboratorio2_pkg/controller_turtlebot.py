import rclpy
from rclpy.node import Node
#Import here message types
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class Controller_turtlebot(Node):

    def __init__(self):
        super().__init__('controller_turtlebot') #Inheritance from Node class
        
        #SUBSCRIBER PART LASERSCAN INFO
        self.subscription= self.create_subscription(LaserScan,'/scan',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        
        #SUBSCRIBER PART ODOMETRY INFO
        self.subscription1 = self.create_subscription(Odometry,'/odom',self.listener_callback1,10)
        self.subscription1  # prevent unused variable warning

        #PUBLISHER PART
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.scan_info= LaserScan()
        self.odom_info= Odometry()

    #SUBSCRIBER CALLBACK FUNCTION
    def listener_callback(self, msg):
        self.scan_info= msg

     #SUBSCRIBER CALLBACK FUNCTION
    def listener_callback1(self, message):
        self.odom_info= message
    
    #PUBLISHER CALLBACK FUNCTION
    def timer_callback(self):
        if not hasattr(self.scan_info, 'ranges'):
            return
        self.Twist_info= Twist()

        if self.scan_info.ranges[0]>0.3:
            self.Twist_info.linear.x=0.22
            self.Twist_info.linear.y=0.0
            self.Twist_info.angular.z=0.0
            if min(self.scan_info.ranges)<0.3 and self.scan_info.ranges[0]>self.scan_info.ranges[-1]:
                self.Twist_info.linear.y=0.22
            if min(self.scan_info.ranges)<0.3 and self.scan_info.ranges[0]<=self.scan_info.ranges[-1]:
                self.Twist_info.linear.y=-0.22
        

        if self.scan_info.ranges[0]<=0.3:
            self.Twist_info.linear.x=0.0
            self.Twist_info.linear.y=0.0
            if self.scan_info.ranges[0]>self.scan_info.ranges[-1]:
                self.Twist_info.angular.z=1.5
            if self.scan_info.ranges[0]<=self.scan_info.ranges[-1]:
                self.Twist_info.angular.z=-1.5




        
        
        self.publisher_.publish(self.Twist_info)
        self.get_logger().info('Odometry, Angular:"%f", linear:"%f"' %(self.Twist_info.angular.z,self.Twist_info.linear.x) )
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    NODO= Controller_turtlebot()

    rclpy.spin(NODO)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    NODO.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()