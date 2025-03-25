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
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.move_forward, 10)
        
    def move_forward(self, msg):       
        goalx = -1
        goaly = -1
        movement = Twist()
        
        #Get distance to goal
        distanceToGoal = math.sqrt( (goalx - msg.pose.pose.position.x)**2 + (goaly - msg.pose.pose.position.y)**2)
        
        #Get angle to goal
        angleToGoal = math.atan2( goaly - msg.pose.pose.position.y , goalx - msg.pose.pose.position.x)        

        orientation = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (orientation.w*orientation.z + orientation.x*orientation.y), 1.0 - 2.0 * (orientation.y*orientation.y + orientation.z*orientation.z)) #Get yaw angle from quaternion
        
        #Adjust yaw and angleToGoal angles to be between 0 and 2pi
        yaw += math.pi
        angleToGoal += math.pi

        #Movement Parameters
        turningSpeed = 0.2
        drivingSpeed = 0.5

        angleAccuracy = 0.1
        distanceAccuracy = 0.1

        angleDifference  = abs(yaw - angleToGoal)

        currentAngular = 0.0

        if yaw < (angleToGoal - angleAccuracy):
            movement.angular.z = float(turningSpeed * angleDifference *-1)
            currentAngular = float(turningSpeed * angleDifference *-1)
            message = "Turning"
        elif yaw > (angleToGoal + angleAccuracy):
            movement.angular.z = float(turningSpeed * angleDifference)
            currentAngular = float(turningSpeed * angleDifference)
            message = "Turning"

        else:
            movement.angular.z = 0.0
            currentAngular = 0.0
            message = "Aligned"
            if distanceToGoal > distanceAccuracy:
                movement.linear.x = float(drivingSpeed*distanceToGoal*-1) #Robot moving in reverse as it is facing the wrong way
                message = "Aligned and Moving"
            else:
                movement.linear.x = 0.0
                message = "At Goal"

        self.get_logger().info(f'Current yaw: {yaw} Goal Angle: {angleToGoal} message: {message} currentAngular: {currentAngular}') 
        self.publisher.publish(movement)
    


def main():
    rclpy.init()
    node = MoveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
