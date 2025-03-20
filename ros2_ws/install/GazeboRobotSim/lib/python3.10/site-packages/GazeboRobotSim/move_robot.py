import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_forward)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
	
    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.2  # Move forward
        msg.angular.z = 0.0  # No rotation
        self.publisher.publish(msg)
    
    def odom_callback(self, msg):
    	position = msg.pose.pose.position
    	orientation = msg.pose.pose.orientation
    	self.get_logger().info(f'Position -> x: {position.x}, y: {position.y}, z: {position.z}')

def main():
    rclpy.init()
    node = MoveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
