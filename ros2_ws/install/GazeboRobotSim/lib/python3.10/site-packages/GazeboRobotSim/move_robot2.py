import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import array

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.laser_msg = LaserScan()
        self.odom_msg = Odometry()
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser_subscription = self.create_subscription(LaserScan, '/laser/out', self.laser_callback, 10)

        self.state = "Deciding"
        self.chosenDirection = []
        self.movementDistance = 0.3

        self.movementGoalY = 0
        self.movementGoalX = 0

        self.move_forward_timer = self.create_timer(0.1, self.move_forward)

    def laser_callback(self, msg):
        self.laser_msg = msg

    def odom_callback(self, msg):
        self.odom_msg = msg

    def move_forward(self):       
        goalx = 0
        goaly = 2
        movement = Twist()
        

        if self.state == "Deciding":
            #####Laser Handling
            laserArray = self.laser_msg.ranges
            laserArray.reverse()  #To keep laser readings as left to right

            #Split Laser into three lists
            segments = np.array_split(laserArray, 3)   # returns 3 sub-arrays
            leftSensorMin  = min(segments[0])
            frontSensorMin = min(segments[1])
            rightSensorMin = min(segments[2])


            self.get_logger().info(f"Left Sensor Array: {segments[0]} Front Sensor Array: {segments[1]} Right Sensor Array: {segments[2]}")


            #Get Distances to goal from direction robot is facing
            distanceToGoalLeft = math.sqrt( (goalx - (self.odom_msg.pose.pose.position.x - self.movementDistance))**2 + (goaly - self.odom_msg.pose.pose.position.y)**2) 
            distanceToGoalRight = math.sqrt( (goalx - (self.odom_msg.pose.pose.position.x + self.movementDistance))**2 + (goaly - self.odom_msg.pose.pose.position.y)**2)
            distanceToGoalFront = math.sqrt( (goalx - self.odom_msg.pose.pose.position.x)**2 + (goaly - self.odom_msg.pose.pose.position.y + self.movementDistance)**2)
            
            directions = [
                ["Left", distanceToGoalLeft, leftSensorMin],
                ["Front", distanceToGoalFront, frontSensorMin],
                ["Right", distanceToGoalRight, rightSensorMin]
            ]

            smallestDistance = 0
            self.chosenDirection = directions[0] #Default to left
            for direction in directions:
                if direction[2] > self.movementDistance and direction[1] > smallestDistance:
                    smallestDistance = direction[1]
                    self.chosenDirection = direction


            if self.chosenDirection[0] == "Left":
                self.movementGoalX = self.odom_msg.pose.pose.position.x - self.movementDistance
                self.movementGoalY = self.odom_msg.pose.pose.position.y
            
            elif self.chosenDirection[0] == "Front":
                self.movementGoalX = self.odom_msg.pose.pose.position.x
                self.movementGoalY = self.odom_msg.pose.pose.position.y + self.movementDistance
            
            elif self.chosenDirection[0] == "Right":
                self.movementGoalX = self.odom_msg.pose.pose.position.x + self.movementDistance
                self.movementGoalY = self.odom_msg.pose.pose.position.y

            self.state = "Moving"
        
        if self.state == "Moving":

            #Get angle to goal
            if self.chosenDirection[0] == "Left":
                angleToMovement = math.atan2( self.movementGoalY - self.odom_msg.pose.pose.position.y , self.movementGoalX - self.odom_msg.pose.pose.position.x)
                distanceToMovement = math.sqrt( (self.movementGoalX - self.odom_msg.pose.pose.position.x)**2 + (self.movementGoalY - self.odom_msg.pose.pose.position.y)**2)

            elif self.chosenDirection[0] == "Front":
                angleToMovement = math.atan2( self.movementGoalY - self.odom_msg.pose.pose.position.y , self.movementGoalX - self.odom_msg.pose.pose.position.x)
                distanceToMovement = math.sqrt( (self.movementGoalX - self.odom_msg.pose.pose.position.x)**2 + (self.movementGoalY - self.odom_msg.pose.pose.position.y)**2)

            elif self.chosenDirection[0] == "Right":
                angleToMovement = math.atan2( self.movementGoalY - self.odom_msg.pose.pose.position.y , self.movementGoalX - self.odom_msg.pose.pose.position.x)
                distanceToMovement = math.sqrt( (self.movementGoalX - self.odom_msg.pose.pose.position.x)**2 + (self.movementGoalY - self.odom_msg.pose.pose.position.y)**2)


            orientation = self.odom_msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (orientation.w*orientation.z + orientation.x*orientation.y), 1.0 - 2.0 * (orientation.y*orientation.y + orientation.z*orientation.z)) #Get yaw angle from quaternion
            
            #Adjust yaw and angleToGoal angles to be between 0 and 2pi
            yaw += math.pi
            angleToMovement += math.pi


            angleDifference  = abs(yaw - angleToMovement)

            #Movement Parameters
            turningSpeed = 1
            drivingSpeed = 1
            angleAccuracy = 0.1
            distanceAccuracy = 0.1

            #Turn to face goal
            facingGoal = False
            if yaw < (angleToMovement-angleAccuracy) and not facingGoal:
                movement.angular.z = float(turningSpeed * angleDifference *-1)

            elif yaw > (angleToMovement+angleAccuracy) and not facingGoal:
                movement.angular.z = float(turningSpeed * angleDifference)
            
            else:
                movement.angular.z = 0.0
                facingGoal = True
            
            #Move towards goal
            if facingGoal:
                if distanceToMovement > distanceAccuracy:
                    movement.linear.x = float(drivingSpeed*distanceToMovement*-1)
                else:
                    movement.linear.x = 0.0
                    self.state = "Deciding"

            #self.get_logger().info(f"Chosen Direction: {self.chosenDirection[0]} Distance to Goal: {self.chosenDirection[1]} Distance to Obstacle: {self.chosenDirection[2]} Angle to Movement: {angleToMovement} Yaw: {yaw} Angle Difference: {angleDifference}") 
            
            self.get_logger().info(f"State {self.state} Distance to Movement {distanceToMovement} Angle to Movement {angleToMovement} Yaw {yaw} Angle Difference {angleDifference}")


            self.publisher.publish(movement)

























            
        self.publisher.publish(movement)
    


def main():
    rclpy.init()
    node = MoveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
