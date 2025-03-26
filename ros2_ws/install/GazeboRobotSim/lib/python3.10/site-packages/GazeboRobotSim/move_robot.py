import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import atan2, sqrt, pi

class PathfindingRobot(Node):
    def __init__(self):
<<<<<<< HEAD
        super().__init__('pathfinding_robot')

        # Create a subscriber to the Hokuyo laser scan data
        self.create_subscription(LaserScan, '/laser/out', self.scan_callback, 10)

        # Publisher to send velocity commands to the robot
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize movement
        self.cmd_vel = Twist()

        # Define the goal position (goal_x, goal_y) in robot's local frame (in meters)
        self.goal_x = 5.0  # Example goal x position
        self.goal_y = 5.0  # Example goal y position

        # Assume the robot's initial position and orientation (for simulation purposes)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0  # Robot's orientation in radians (0 means facing the positive x-axis)

    def scan_callback(self, msg):
        # Extract laser scan data
        laser_data = msg.ranges

        # Define important indices for the laser data (left, center, right)
        left_range = laser_data[20]  # Left side of the robot
        center_range = laser_data[len(laser_data) // 2]  # Center of the robot
        right_range = laser_data[-20]  # Right side of the robot

        # Define the minimum safe distance to avoid collisions
        min_safe_distance = 1.0

        # Proportional control constants
        k_linear = 0.5  # Speed scaling factor
        k_angular = 2.0  # Turning scaling factor

        # Proportional collision avoidance logic
        if center_range < min_safe_distance:
            # There is an obstacle in front, calculate the turn direction based on side readings
            if left_range > right_range:
                # More space on the left, turn left
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = k_angular * (1.0 - left_range / min_safe_distance)
            else:
                # More space on the right, turn right
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = -k_angular * (1.0 - right_range / min_safe_distance)
        else:
            # No obstacle in front, move forward with proportional control
            self.cmd_vel.linear.x = k_linear * (center_range / min_safe_distance)  # Go forward faster if no obstacles
            self.cmd_vel.angular.z = 0.0  # Move straight

        # Move towards the goal if there are no obstacles
        self.move_towards_goal()

        # Publish the velocity command
        self.velocity_publisher.publish(self.cmd_vel)

    def move_towards_goal(self):
        # Here we use a simple proportional control approach to reach the goal.

        # Calculate the distance and angle to the goal
        delta_x = self.goal_x - self.robot_x
        delta_y = self.goal_y - self.robot_y
        distance_to_goal = sqrt(delta_x**2 + delta_y**2)
        angle_to_goal = atan2(delta_y, delta_x)

        # Normalize angle_to_goal to be within [-pi, pi]
        angle_to_goal = (angle_to_goal - self.robot_theta + pi) % (2 * pi) - pi

        # Proportional control for linear speed (move faster if closer to the goal)
        k_linear = 0.5  # Speed scaling factor
        self.cmd_vel.linear.x = k_linear * distance_to_goal

        # Proportional control for angular speed (adjust orientation to face the goal)
        k_angular = 2.0  # Turning scaling factor
        self.cmd_vel.angular.z = k_angular * angle_to_goal

        # If the robot is close to the goal, stop moving
        if distance_to_goal < 0.1:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0

        # Update robot's pose based on the movement (for simulation purposes)
        self.robot_x += self.cmd_vel.linear.x * 0.1  # Assuming a time step of 0.1s
        self.robot_y += 0.0  # We assume no lateral movement
        self.robot_theta += self.cmd_vel.angular.z * 0.1  # Update robot's orientation

        # Ensure the robot's theta is within [-pi, pi]
        self.robot_theta = (self.robot_theta + pi) % (2 * pi) - pi
=======
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
        goalx = 1.5
        goaly = 1.5
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
    
>>>>>>> f373a3563734f81900eebc75c97cee28594d936b

def main(args=None):
    rclpy.init(args=args)
    node = PathfindingRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
