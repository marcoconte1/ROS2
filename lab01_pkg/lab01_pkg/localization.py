import rclpy
from rclpy.node import Node
#Import here message types
from geometry_msgs.msg import Twist,Pose


class Localization(Node):

    def __init__(self):
        super().__init__('localization') #Inheritance from Node class
        
        # Initialize stored twist message in an attribute to have it for later
        self.twist = Twist()
        
        #SUBSCRIBER PART
        self.subscription = self.create_subscription(Twist,'/cmd_vel',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        
        #PUBLISHER PART
        self.publisher_ = self.create_publisher(Pose,'/pose', 10)
        self.timer_period=1.0 #seconds
        self.timer=self.create_timer(self.timer_period, self.timer_callback)
        self.i=0
    
    #SUBSCRIBER CALLBACK FUNCTION
    def listener_callback(self,msg):
        # Since from the listener callback function we receive the Twist message in the msg argument,
        #we store it in the attribute previously created 
        self.twist = msg
        #If you want to print what you listened to, uncomment the following line
        #self.get_logger().info('The robot is moving at linear speed: x= "%f" y= "%f" z="%f" and Angular speed: x="%f" y="%f" z="%f"' % (msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z))
    
    #PUBLISHER CALLBACK FUNCTION
    def timer_callback(self):
        pub_msg = Pose()
        # Use the stored twist message to compute the linear position
        pub_msg.position.x = self.twist.linear.x * self.i
        self.publisher_.publish(pub_msg)
        self.get_logger().info('Publishing pose:position x= "%f" y= "%f" z="%f" orientation x= "%f" y= "%f" z="%f" w="%f" time:"%f" seconds' % (pub_msg.position.x,pub_msg.position.y,pub_msg.position.z,pub_msg.orientation.x,pub_msg.orientation.y,pub_msg.orientation.z,pub_msg.orientation.w,self.i))
        # Update counter i that in this case represents the time
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    NODO=Localization()

    rclpy.spin(NODO)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    NODO.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()