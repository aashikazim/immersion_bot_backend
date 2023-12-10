#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, sin, cos, atan, asin
# import matplotlib.pyplot as plt

MAX_VELOCITY = 0.4
MAX_ANGULAR = 0.5

x_offset = None
y_offset = None
theta_offset = None
x, y, theta = 0, 0, 0

# CONTROL PARAMETERS
integral = 0
output = 0
previous_error = 0
Kp = 10.0
Ki = 0.01
Kh = 2.0

DELTA_TIME = 0.1

robot_pos_x_history = []
robot_pos_y_history = []
vrobot_pos_x_history = []
vrobot_pos_y_history = []

# PURSUIT CIRCLE PARAMETERS
TARGET_VELOCITY = 0.2
PURSUIT_DISTANCE = 0.0
RADIUS = 0.5
CENTER_X = 0.5
CENTER_Y = 0.5
ANGULAR_VELOCITY = TARGET_VELOCITY / RADIUS
xt, yt = 0, 0
theta_t = 0

is_init = False

def odomCallback(data):
    global x, y, theta, is_init, CENTER_X, CENTER_Y, xt, yt, theta_t
    pos_data = data.pose.pose.position
    rot_data = data.pose.pose.orientation
    x, y = pos_data.x, pos_data.y
    qx, qy, qz, qw = rot_data.x, rot_data.y, rot_data.z, rot_data.w
    theta = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    
    if not is_init:
        is_init = True
        CENTER_X += x
        CENTER_Y += y
        theta_t = 0
        xt = CENTER_X + RADIUS * cos(theta_t)
        yt = CENTER_Y + RADIUS * sin(theta_t)

def main():
    global xt, yt, theta_t, integral, output
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.init_node('rosbot_turn', anonymous=True)
    rospy.Subscriber("/odom", Odometry, odomCallback)
    rate = rospy.Rate(10)
    vel = Twist()
    vel.linear.x = 0.0
    vel.angular.z = 0.0
    
    # plt.figure(figsize=(5, 5))

    count = 0
    while not rospy.is_shutdown():
        if not is_init:
            continue
        vel_pub.publish(vel)
        
        # Revolving Pursuit Point
        theta_t += ANGULAR_VELOCITY * DELTA_TIME
        xt = CENTER_X + RADIUS * cos(theta_t)
        yt = CENTER_Y + RADIUS * sin(theta_t)
        print(xt, yt)

        # Calculating Error
        distance = sqrt((xt - x)**2 + (yt - y)**2)
        error = distance - PURSUIT_DISTANCE
        error = error - vel.linear.x
        theta_d = atan2(yt - y, xt - x)
        angle_error = atan2(sin(theta_d - theta),  cos(theta_d - theta))
        print("Distance Error: {}\t{}\tAngle Error: {}\t{}\t{}".format(distance, error, angle_error, vel.linear.x, vel.angular.z))
        
        # Applying Control
        integral += error * DELTA_TIME
        output = (Kp * error + Ki * integral)
        vel.linear.x = max(-MAX_VELOCITY, min(vel.linear.x + output * DELTA_TIME, MAX_VELOCITY))
        vel.angular.z = Kh * angle_error
        
        robot_pos_x_history.append(x)
        robot_pos_y_history.append(y)
        vrobot_pos_x_history.append(xt)
        vrobot_pos_y_history.append(yt)
        
        # plt.clf()
        # plt.plot(robot_pos_x_history[:], robot_pos_y_history[:], 'b-', linewidth=1)
        # plt.plot(robot_pos_x_history[-1], robot_pos_y_history[-1], 'bo', linewidth=1)
        # plt.plot(vrobot_pos_x_history[:], vrobot_pos_y_history[:], 'r-', linewidth=1)
        # plt.plot(vrobot_pos_x_history[-1], vrobot_pos_y_history[-1], 'ro', linewidth=1)
        # plt.pause(0.0001)
        
        # Termination Condition
        count += 1
        if count == 100000:
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            break

        rate.sleep()
if __name__ == "__main__":
    main()

