import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter=0
        self.N=1
        self.phase=0

    def timer_callback(self):
        msg = Twist()
        if self.phase==0:
            msg.linear.x=1.0
            msg.linear.y=0.0
        if self.phase==1:
            msg.linear.x=0.0
            msg.linear.y=1.0
        if self.phase==2:
            msg.linear.x=-1.0
            msg.linear.y=0.0
        if self.phase==3:
            msg.linear.x=0.0
            msg.linear.y=-1.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing velocity:x= "%f"y= "%f"N= "%d"phase= "%d"' % (msg.linear.x,msg.linear.y,self.N,self.phase))
        #counters
        self.counter+=1
        if self.counter ==self.N:
            self.phase+=1
            self.counter=0
            if self.phase==4:
                self.phase=0
                self.N+=1


def main(args=None):
    rclpy.init(args=args)

    node = Controller()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()