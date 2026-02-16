import rclpy
from rclpy.node import Node
#Import here message types
from geometry_msgs.msg import Pose 
from std_msgs.msg import Bool


class Reset_node(Node):

    def __init__(self):
        super().__init__('reset_node') #Inheritance from Node class
        
        #SUBSCRIBER PART
        self.subscription = self.create_subscription(Pose,'/pose',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        
        #PUBLISHER PART
        self.publisher_ = self.create_publisher(Bool, '/reset', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.pose= Pose()#Initialize variable to store received messages
    #SUBSCRIBER CALLBACK FUNCTION
    def listener_callback(self, msg):
        self.pose=msg
        #self.get_logger().info('I heard: "%s"' % msg.data)
    
    #PUBLISHER CALLBACK FUNCTION
    def timer_callback(self):
        msg = Bool()
        if self.pose.position.x>6.0:
            msg.data = True
        else:
            msg.data = False
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    NODO= Reset_node()

    rclpy.spin(NODO)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    NODO.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()