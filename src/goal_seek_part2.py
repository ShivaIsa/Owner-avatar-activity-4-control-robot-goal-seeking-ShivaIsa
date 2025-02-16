#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import os

# Current position and orientation of the robot
robot_position = None
robot_orientation = None

# Goal positions loaded from a file
goal_positions = []

# PID coefficients for linear and angular motion
Kp_linear, Ki_linear, Kd_linear = 1.0, 0.0, 0.1
Kp_angular, Ki_angular, Kd_angular = 2.0, 0.0, 0.5

# PID controller states
linear_integral, angular_integral = 0.0, 0.0
prev_linear_error, prev_angular_error = 0.0, 0.0

# Constants for motion
MAX_LINEAR_VELOCITY = 0.2
MAX_ANGULAR_VELOCITY = 1.0
DISTANCE_THRESHOLD = 0.1
ANGLE_THRESHOLD = 0.1
WAIT_TIME_AFTER_GOAL = 5  # Time to wait after reaching each goal

# Callback to update robot's position and orientation
def update_odometry(msg):
    global robot_position, robot_orientation
    robot_position = msg.pose.pose.position
    robot_orientation = msg.pose.pose.orientation

# Load goals from a file
def load_goals_from_file(file_path):
    global goal_positions
    if not os.path.exists(file_path):
        rospy.logerr(f"Goal file '{file_path}' does not exist!")
        return False
    
    with open(file_path, 'r') as file:
        goal_positions = [tuple(map(float, line.split())) for line in file]
    
    return True

# Calculate Euclidean distance to the goal
def calculate_distance_to_goal(goal):
    if robot_position:
        dx = goal[0] - robot_position.x
        dy = goal[1] - robot_position.y
        return math.hypot(dx, dy)
    return float('inf')

# Calculate the angle to the goal in radians
def calculate_angle_to_goal(goal):
    if robot_position and robot_orientation:
        dx = goal[0] - robot_position.x
        dy = goal[1] - robot_position.y
        target_angle = math.atan2(dy, dx)

        _, _, robot_yaw = quaternion_to_euler_angles(robot_orientation)
        return target_angle - robot_yaw
    return 0.0

# Convert quaternion to Euler angles (roll, pitch, yaw)
def quaternion_to_euler_angles(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(2.0 * (w * y - z * x))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw

# PID Controller for linear or angular velocity
def pid_controller(error, prev_error, integral, Kp, Ki, Kd, dt):
    P = Kp * error
    integral += error * dt
    D = Kd * (error - prev_error) / dt if dt > 0 else 0
    return P + Ki * integral + D, integral

# Stop the robot
def stop_robot(pub):
    pub.publish(Twist())

# Main robot navigation function
def navigate_to_goals(pub):
    global prev_linear_error, prev_angular_error, linear_integral, angular_integral

    rospy.init_node('goal_navigator', anonymous=True)
    rospy.Subscriber('/odom', Odometry, update_odometry)
    rate = rospy.Rate(10)

    # Load goals from file
    goal_file_path = '/home/cse4568/catkin_ws/src/goal_seek/goals.txt'
    if not load_goals_from_file(goal_file_path):
        return

    for goal in goal_positions:
        rospy.loginfo(f"Navigating to goal: {goal}")

        while not rospy.is_shutdown():
            distance_to_goal = calculate_distance_to_goal(goal)
            angle_to_goal = calculate_angle_to_goal(goal)

            # If within thresholds, consider goal reached
            if distance_to_goal < DISTANCE_THRESHOLD and abs(angle_to_goal) < ANGLE_THRESHOLD:
                rospy.loginfo(f"Goal reached: {goal}")
                stop_robot(pub)
                rospy.sleep(WAIT_TIME_AFTER_GOAL)  # Wait for a moment before moving to next goal
                break

            twist = Twist()

            # Compute linear velocity using PID control
            if distance_to_goal > DISTANCE_THRESHOLD:
                linear_error = distance_to_goal
                linear_velocity, linear_integral = pid_controller(
                    linear_error, prev_linear_error, linear_integral, Kp_linear, Ki_linear, Kd_linear, 1.0 / 10
                )
                twist.linear.x = min(MAX_LINEAR_VELOCITY, linear_velocity)

            # Compute angular velocity using PID control
            if abs(angle_to_goal) > ANGLE_THRESHOLD:
                angular_error = angle_to_goal
                angular_velocity, angular_integral = pid_controller(
                    angular_error, prev_angular_error, angular_integral, Kp_angular, Ki_angular, Kd_angular, 1.0 / 10
                )
                twist.angular.z = min(MAX_ANGULAR_VELOCITY, angular_velocity)

            pub.publish(twist)
            rate.sleep()

    rospy.loginfo("All goals have been reached!")

if __name__ == '__main__':
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    navigate_to_goals(pub)

