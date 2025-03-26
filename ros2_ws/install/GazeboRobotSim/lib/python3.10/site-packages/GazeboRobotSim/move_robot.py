import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math
import array

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.laser_msg = LaserScan()
        self.odom_msg = Odometry()
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser_subscription = self.create_subscription(LaserScan, '/laser/out', self.laser_callback, 10)

        self.move_forward_timer = self.create_timer(0.1, self.move_forward)

    def laser_callback(self, msg):
        self.laser_msg = msg

    def odom_callback(self, msg):
        self.odom_msg = msg

    def move_forward(self):       
        goalx = 0
        goaly = 2
        movement = Twist()
        
        #Get distance to goal
        distanceToGoal = math.sqrt( (goalx - self.odom_msg.pose.pose.position.x)**2 + (goaly - self.odom_msg.pose.pose.position.y)**2)
        
        #Get angle to goal
        angleToGoal = math.atan2( goaly - self.odom_msg.pose.pose.position.y , goalx - self.odom_msg.pose.pose.position.x)        

        orientation = self.odom_msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (orientation.w*orientation.z + orientation.x*orientation.y), 1.0 - 2.0 * (orientation.y*orientation.y + orientation.z*orientation.z)) #Get yaw angle from quaternion
        #Adjust yaw and angleToGoal angles to be between 0 and 2pi
        yaw += math.pi
        angleToGoal += math.pi

        #Movement Parameters
        turningSpeed = 0.3
        drivingSpeed = 0.5
        angleAccuracy = self.laser_msg.angle_increment
        distanceAccuracy = 0.1

        currentAngular = 0.0


        #####Laser Handling
        laserArray = self.laser_msg.ranges
        laserArray.reverse()                            #To keep laser readings as left to right
        #self.get_logger().info(f"Laser: {laserArray}")

        ##Get Middle Measurement (front of robot)
        middleIndex = len(laserArray)//2
        middleMeasurement = laserArray[middleIndex]

        rayAngleIncrement = self.laser_msg.angle_increment

        obstacleDetected = False
        detectionDistance = 0.3
        detectionRange = 50
        closestFreeAngle = "N/A"

        for i in range(0,detectionRange):
            if laserArray[middleIndex - i] < detectionDistance or laserArray[middleIndex + i] < detectionDistance:
                obstacleDetected = True
                break
        
        if obstacleDetected:
            angleToGoal = yaw

            for i in range(1,(len(laserArray)//2)-detectionRange):

                #Move to the left
                if laserArray[middleIndex - (i+detectionRange)] > detectionDistance:
                    closestFreeAngle = str(middleIndex - (i+detectionRange))
                    angleToGoal += rayAngleIncrement*(i+detectionRange)
                    while angleToGoal > 2*math.pi:
                        angleToGoal -= 2*math.pi
                    break
                #Move to the right
                if laserArray[middleIndex + (i+detectionRange)] > detectionDistance:
                    closestFreeAngle = str(middleIndex + (i+detectionRange))
                    angleToGoal -= rayAngleIncrement*(i+detectionRange)
                    while angleToGoal < 0:
                        angleToGoal += 2*math.pi
                    break
        
        
        self.get_logger().info(f"Obstacle Detected: {obstacleDetected} ClosestFreeAngle {closestFreeAngle} Angle to Goal: {angleToGoal} Yaw: {yaw} rayAngleIncrement: {rayAngleIncrement}")
        

        angleDifference  = abs(yaw - angleToGoal)

        if yaw < angleToGoal:
            movement.angular.z = float(turningSpeed * angleDifference *-1)
            currentAngular = float(turningSpeed * angleDifference *-1)

            message = "Turning"
        elif yaw > angleToGoal:
            movement.angular.z = float(turningSpeed * angleDifference)
            currentAngular = float(turningSpeed * angleDifference)
            message = "Turning"

        else:
            movement.angular.z = 0.0
            currentAngular = 0.0
            message = "Aligned"

        if distanceToGoal > distanceAccuracy and not obstacleDetected:
            movement.linear.x = float(drivingSpeed*distanceToGoal*-1) #Robot moving in reverse as it is facing the wrong way
        elif not obstacleDetected:
            movement.linear.x = 0.0
            message = "At Goal"

        
            
        self.publisher.publish(movement)
    


def main():
    rclpy.init()
    node = MoveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
