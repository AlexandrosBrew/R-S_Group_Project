import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math
import array
import time
class MoveRobot(Node):
    '''Manages the robots object detection and avoidence'''
    def __init__(self):
        super().__init__('move_robot')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10) #Twist publisher to control robots movements
        
        self.laser_msg = LaserScan() 
        self.odom_msg = Odometry() 
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10) # Subscription for robots position 
        self.laser_subscription = self.create_subscription(LaserScan, '/laser/out', self.laser_callback, 10) # Subscrition for the data from hokuyo laser scanner

        self.move_forward_timer = self.create_timer(0.1, self.move_forward) # Calls the move robot function every 0.1 seconds

        self.turnAround = False
        self.turnAroundDegree = 0.0

        self.stuckTimer = time.time() 
        self.stuckBool = False

    def laser_callback(self, msg):
        self.laser_msg = msg

    def odom_callback(self, msg):
        self.odom_msg = msg

    def move_forward(self):
        '''Main function controlling robots movemnts and object detection. Publishes robots movements using the publisher variable.'''       
        movement = Twist()
        goalx = 1.5
        goaly = 1.5
        
        #Get distance to goal
        distanceToGoal = math.sqrt( (goalx - self.odom_msg.pose.pose.position.x)**2 + 
                                   (goaly - self.odom_msg.pose.pose.position.y)**2)
        
        #Get angle to goal
        angleToGoal = math.atan2( goaly - self.odom_msg.pose.pose.position.y , 
                                 goalx - self.odom_msg.pose.pose.position.x)        

        orientation = self.odom_msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (orientation.w*orientation.z + orientation.x*orientation.y), 
                         1.0 - 2.0 * (orientation.y*orientation.y + orientation.z*orientation.z)) #Get yaw angle from quaternion
        #Adjust yaw and angleToGoal angles to be between 0 and 2pi
        yaw += math.pi
        angleToGoal += math.pi

        #Movement Parameters
        turningSpeed = 0.5
        drivingSpeed = 0.3
        distanceAccuracy = 0.1

        #####Laser Handling
        laserArray = self.laser_msg.ranges
        laserArray.reverse() #To keep laser readings as left to right

        ##Get Middle Measurement (front of robot)
        middleIndex = len(laserArray)//2
        middleMeasurement = laserArray[middleIndex]

        rayAngleIncrement = self.laser_msg.angle_increment

        #Obstacle detection variables
        obstacleDetected = False
        detectionDistance = 0.3 # Tolerance for object distance
        criticalDetectionDistance = 0.2 # Tolerance for critical decision
        detectionRange = 50 
        criticalDetectionRange = 5

        blockedRays=0
        criticalDetection = False
        ### Object Detection for distance data received from laser.
        for i in range(0,180):
            if laserArray[i] < detectionDistance:
                blockedRays += 1

            if i<detectionRange:
                if laserArray[middleIndex - i] < detectionDistance or laserArray[middleIndex + i] < detectionDistance:
                    obstacleDetected = True
            if i<criticalDetectionRange:
                if laserArray[middleIndex - i] < criticalDetectionDistance or laserArray[middleIndex + i] < criticalDetectionDistance:
                    criticalDetection = True
        
        ### Collision detection and critical obstacle avoidance
        xVelocity = self.odom_msg.twist.twist.linear.x
        yVelocity = self.odom_msg.twist.twist.linear.y

        current_speed = math.sqrt(xVelocity**2 + yVelocity**2)
        
        if criticalDetection:
            turningSpeed+=0.5 # Increase turning speed incase robot is in risk of collision to avoid.
        
        # Stuck robot management
        if current_speed < 0.03 : # Robots stationary
            if self.stuckBool == False:
                self.stuckBool = True
                self.stuckTimer = time.time()
            
            if (time.time() - self.stuckTimer) > 10 :
                self.turnAround = True
                self.stuckTimer = time.time()
        else:
            self.stuckTimer = time.time()
            self.stuckBool = False

        
        if (blockedRays >= 135 and current_speed < 0.05) and not self.turnAround: # Turn robot around when stuck
            self.get_logger().info(f"Robot is stuck with {blockedRays} blocked rays")
            self.turnAroundDegree = (yaw + math.pi) % (2 * math.pi)
            self.turnAround = True

        if self.turnAround: # Adjust robots angular speed so robot matches desired angle
            
            if blockedRays < 135:
                self.get_logger().info(f"Robot is no longer stuck with {blockedRays} blocked rays")
                self.turnAround = False
                self.stuckTimer = time.time()
                movement.angular.z = 0.0
                movement.linear.x = 0.0
                self.get_logger().info("Turned around")



            self.get_logger().info(f"Turning around to {self.turnAroundDegree} degrees from {yaw} degrees")

            angle_diff = abs(yaw - self.turnAroundDegree)

            # Normalize angle_diff in case it wraps around 2π
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff

            turningSpeed = 0.2  # Adjust as needed

            if angle_diff > 0.05:  # Not facing target yet
                movement = Twist()
                movement.linear.x = 0.0
                movement.angular.z = turningSpeed
            else:
                self.get_logger().info("Turned around")
                self.turnAround = False

        
        else:
            # Object avoidance
            if obstacleDetected:
                angleToGoal = yaw
                for i in range(1,(len(laserArray)//2-detectionRange)):
                    try:
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
                    
                    except IndexError:
                        break
                

                    
            ### Normal movement for robot and adjustments to direct towards goal.
            angleDifference  = abs(yaw - angleToGoal)

            if yaw < angleToGoal: # Turn towards goal
                movement.angular.z = float(turningSpeed * angleDifference *-1)

            elif yaw > angleToGoal: # Turn towrds goal
                movement.angular.z = float(turningSpeed * angleDifference)
            else: #Stop turning
                movement.angular.z = 0.0

            if distanceToGoal > distanceAccuracy and not criticalDetection:
                movement.linear.x = float(drivingSpeed*-1) #Robot moving in reverse as it is facing the wrong way

            elif ((0.5-distanceToGoal) > distanceAccuracy) and not criticalDetection:
                movement.linear.x = float(drivingSpeed*distanceToGoal*-1) #Robot moving in reverse as it is facing the wrong way

            else:#elif not obstacleDetected:
                movement.linear.x = 0.0

            
        #self.get_logger().info(f"Distance to goal: {distanceToGoal} Angle to goal: {angleToGoal} Yaw: {yaw} Laser: {laserArray[middleIndex]} Obstacle Detected: {obstacleDetected} Critical Detection: {criticalDetection} Current Speed: {current_speed} TurnAround: {self.turnAround} Stuck Timer: {time.time() - self.stuckTimer}")
        self.publisher.publish(movement) # Publish the current twist movements to the robot.
    


def main():
    rclpy.init()
    node = MoveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
