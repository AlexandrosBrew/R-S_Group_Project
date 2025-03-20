import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import math

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        #self.timer = self.create_timer(0.1, self.move_forward)
        #self.pose_subscription = self.create_subscription(Pose, '/pose', self.pose_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.move_forward, 10)
        #self.timer = self.create_timer(0.1, self.move_forward)
        
    def move_forward(self, msg):       
        goalx = 1
        goaly = 1
        movement = Twist()
        
        distanceToGoal = math.sqrt( (goalx - msg.pose.pose.position.x)**2 + (goaly - msg.pose.pose.position.y)**2)
        
        angleToGoal = math.atan2( goaly - msg.pose.pose.position.y , goalx - msg.pose.pose.position.x)
        
        distanceTolerance = 0.1
        angleTolerance = 0.01
        
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (orientation.w*orientation.z + orientation.x*orientation.y), 1.0 - 2.0 * (orientation.y*orientation.y + orientation.z*orientation.z))
        angleDiff = angleToGoal - yaw
        
        if angleDiff > math.pi:
        	angleDiff -= 2*math.pi
        elif angleDiff < -math.pi:
        	angleDiff += 2*math.pi
        

        #angleError = angleToGoal - yaw Might be wrong
        #self.get_logger().info(f'Angle Error: {angleError}, Yaw: {yaw}, AngleToGoal: {angleToGoal}')
        

        if abs(angleDiff) > angleTolerance:
        	movement.angular.z = float(0.3 * angleDiff)
        else:
        	movement.angular.z = movement.angular.z
        
        if distanceToGoal >= distanceTolerance:
       		movement.linear.x = 1.1
        else:
       		movement.linear.x = 0.0
       		quit()
        
        
        self.publisher.publish(movement)
    

def main():
    rclpy.init()
    node = MoveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
