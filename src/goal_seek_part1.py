#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import time

# Global variables for current position and velocity
current_position = None
current_orientation = None

# Goal positions loaded from the goals.txt file
goals = []

# Parameters
MAX_LINEAR_VELOCITY = 0.2  # Max linear speed
MAX_ANGULAR_VELOCITY = 1.0  # Max angular speed
DIST_THRESHOLD = 0.1  # Distance threshold to consider goal reached
ANGLE_THRESHOLD = 0.1  # Angle threshold to consider goal reached
WAIT_TIME = 5  # Wait time after reaching each goal (seconds)

# Callback function to update current position and orientation from /odom
def odom_callback(msg):
    global current_position, current_orientation
    current_position = msg.pose.pose.position
    current_orientation = msg.pose.pose.orientation

# Function to read the goal positions from goals.txt
def load_goals(file_path):
    global goals
    with open(file_path, 'r') as file:
        for line in file:
            x, y = map(float, line.split())
            goals.append((x, y))

# Function to compute the distance between current position and goal position
def distance_to_goal(goal):
    global current_position
    if current_position:
        dx = goal[0] - current_position.x
        dy = goal[1] - current_position.y
        return math.sqrt(dx ** 2 + dy ** 2)
    return float('inf')  # Return a large value if current position is not available

# Function to compute the angle between current orientation and goal
def angle_to_goal(goal):
    global current_position, current_orientation
    if current_position and current_orientation:
        dx = goal[0] - current_position.x
        dy = goal[1] - current_position.y
        target_angle = math.atan2(dy, dx)

        # Convert current orientation (quaternion) to yaw angle
        _, _, current_yaw = euler_from_quaternion(current_orientation)
        
        # Compute the angular difference
        angle_diff = target_angle - current_yaw
        return angle_diff
    return 0.0

# Helper function to convert quaternion to euler angles
def euler_from_quaternion(quaternion):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    """
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    roll_x = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_y = math.asin(2.0 * (w * y - z * x))
    yaw_z = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll_x, pitch_y, yaw_z

# Function to stop the robot
def stop_robot(pub):
    twist = Twist()
    pub.publish(twist)

# Main function
def main():
    # Initialize the ROS node
    rospy.init_node('goal_seek_part1', anonymous=True)

    # Set up publisher for /cmd_vel
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Subscribe to /odom to get current position and orientation
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Load goals from file
    load_goals('/home/cse4568/catkin_ws/src/goal_seek/goals.txt')

    # Set loop rate
    rate = rospy.Rate(10)

    for goal in goals:
        # Wait for /odom to provide current position data
        rospy.loginfo("Navigating to goal: %s", str(goal))

        while not rospy.is_shutdown():
            # Compute distance and angle to goal
            distance = distance_to_goal(goal)
            angle = angle_to_goal(goal)

            # If the robot is within the threshold, consider goal reached
            if distance < DIST_THRESHOLD and abs(angle) < ANGLE_THRESHOLD:
                rospy.loginfo("Goal reached: %s", str(goal))
                stop_robot(pub)
                rospy.sleep(WAIT_TIME)  # Wait for the set amount of time
                break

            # Create Twist message for movement
            twist = Twist()

            # Set linear velocity
            if distance > DIST_THRESHOLD:
                twist.linear.x = min(MAX_LINEAR_VELOCITY, distance)

            # Set angular velocity
            if abs(angle) > ANGLE_THRESHOLD:
                twist.angular.z = min(MAX_ANGULAR_VELOCITY, max(-MAX_ANGULAR_VELOCITY, angle))

            # Publish the Twist message to move the robot
            pub.publish(twist)

            # Sleep for the set loop rate
            rate.sleep()

    rospy.loginfo("All goals reached!")

if __name__ == '__main__':
    main()

