#!/usr/bin/env python
import json
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Range, Imu
from math import atan2, pi
import tf

from udp_sender import send
from udp_receiver import UDPReceiver
from command_type import CommandType
from util import TASKS
from config import Config

class ImmersionBot:
    def __init__(self):
        # Configuration Constants
        self.UDP_IP_1 = "192.168.0.103" # if VR app is running from comp
        self.UDP_IP_2 = "192.168.0.203" # if VR app is running at Quest 3
        self.UDP_PORT = 8080

        self.MAX_VELOCITY = 0.5
        self.MAX_ANGULAR = 2

        self.ir_range = [0 for _ in range(4)]

        self.is_status_sent = False
        
        # State Variables
        self.velocity_x = 0
        self.angular_z = 0
        self.x, self.y, self.theta = 0, 0, 0
        self.xt, self.yt = None, None
        self.ranges = [0 for _ in range(720)]
        self.is_init, self.INIT_X, self.INIT_Y, self.INIT_THETA = False, 0, 0, 0

        # Command Variables
        self.command = None
        self.arguments ={}
        self.is_new_command = False

        # ir_data counter
        self.ir_data_count = 0

        # Initialize ROS Node
        rospy.init_node('rosbot_turn', anonymous=True)
        self.listener = tf.TransformListener()

        # Publisher and Subscriber
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/mcl_pose", PoseStamped, self.odom_callback_slam)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        rospy.Subscriber("/range/fl", Range, self.rangeCallback)
        rospy.Subscriber("/range/fr", Range, self.rangeCallback)
        rospy.Subscriber("/range/rl", Range, self.rangeCallback)
        rospy.Subscriber("/range/rr", Range, self.rangeCallback)
        rospy.Subscriber("/imu", Imu, self.imuCallback)
        rospy.Subscriber("/velocity", Twist, self.velocityCallback)



        

    '''
    |---------------------------
    | SENSOR CALLBACKS
    |---------------------------
    '''

    def imuCallback(self, data):
        la = data.linear_acceleration
        av = data.angular_velocity

        imu_data = {"type": "imu", "data": {"linear_acceleration": [la.x, la.y, la.z], "angular_velocity": [av.x, av.y, av.z]}}
        self.send_data(imu_data)

    def velocityCallback(self, data):
        lv = self.vel.linear
        av = self.vel.angular
        velocity_data = {"type": "velocity", "data": {"linear": [lv.x, lv.y, lv.z], "angular": [av.x, av.y, av.z]}}
        self.send_data(velocity_data)

    def rangeCallback(self, data):
        if (data.header.frame_id == 'range_fl'):
            self.ir_range[0] = data.range
            self.ir_data_count += 1
        elif (data.header.frame_id == 'range_fr'):
            self.ir_range[1] = data.range
            self.ir_data_count += 1
        elif (data.header.frame_id == 'range_rl'):
            self.ir_range[2] = data.range
            self.ir_data_count += 1
        elif (data.header.frame_id == 'range_rr'):
            self.ir_range[3] = data.range
            self.ir_data_count += 1
        
        if (self.ir_data_count == 4):
            self.ir_data_count = 0
            ir_data = {"type": "ir", "data": {"ranges": self.ir_range}}
            self.send_data(ir_data)
        
    def odom_callback_slam(self, data):
        pos = data.pose.position
        rot = data.pose.orientation

        if Config.MODE_ODOM_SLAM_ACML == Config.SLAM:
            self.x, self.y = pos.x, pos.y
            qx, qy, qz, qw = rot.x, rot.y, rot.z, rot.w
            self.theta = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            
            if not self.is_init:
                self.is_init = True
                self.INIT_X = self.x
                self.INIT_Y = self.y
                self.INIT_THETA = self.theta
            pose_data = {"type": "pose", "data": {"y": self.x, "x": -self.y, "theta": -self.theta * (180 / pi)}}
            self.send_data(pose_data)

    def odom_callback(self, data):
        pos = data.pose.pose.position
        rot = data.pose.pose.orientation

        if Config.MODE_ODOM_SLAM_ACML == Config.ODOM:
            self.x, self.y = pos.x, pos.y
            qx, qy, qz, qw = rot.x, rot.y, rot.z, rot.w
            self.theta = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

            if not self.is_init:
                self.is_init = True
                self.INIT_X = self.x
                self.INIT_Y = self.y
                self.INIT_THETA = self.theta
            pose_data = {"type": "pose", "data": {"y": self.x, "x": -self.y, "theta": -self.theta * (180 / pi)}}
            self.send_data(pose_data)

    def transform_callback(self):
        try:
            (pos, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            self.x, self.y = pos[0], pos[1]
            qx, qy, qz, qw = rot[0], rot[1], rot[2], rot[3]
            self.theta = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

            if not self.is_init:
                self.is_init = True
                self.INIT_X = self.x
                self.INIT_Y = self.y
                self.INIT_THETA = self.theta

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("transform Exception!")

    def lidar_callback(self, data):
        self.ranges = data.ranges
        lidar_data = {"type": "lidar", "data": {"ranges": self.ranges}}
        self.send_data(lidar_data)


    '''
    |---------------------------
    | TASKS
    |---------------------------
    '''
    def go_to_goal(self, x, y):
        print("In Go TO GOAL")
        TASKS.go_2_goal(self, x, y)

    def manual_control(self, linear_x, angular_z):
        print("In MANUAL CONTROL")
        TASKS.manual_control(self, linear_x, angular_z)

    def stop(self):
        print("In STOP")
        TASKS.stop(self)

    def pursuit(self, x, y, r, speed):
        print("In PURSUIT")
        TASKS.pursuit(self, x, y, r, speed)
        
    def config(self, max_velocity, max_angular, mode_slam_odom):
        print("In CONFIG")
        TASKS.config(self, max_velocity, max_angular, mode_slam_odom)




    def send_data(self, data):
        send(json.dumps(data), self.UDP_IP_1, self.UDP_PORT)
        send(json.dumps(data),  self.UDP_IP_2, self.UDP_PORT)
    

    def udp_callback(self, data, addr):
        data = json.loads(data)
        self.command = data['command']
        self.arguments = data['arguments']
        self.is_new_command = True


    def run(self):
        rate = rospy.Rate(10)
        self.vel = Twist()
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        count = 0

        while not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            
            print("command: ", self.command, count)
            if self.command:
                if self.command == CommandType.GO_TO_GOAL:
                    self.go_to_goal(self.arguments['x'], self.arguments['y'])
                elif self.command == CommandType.STOP:
                    self.stop()
                elif self.command == CommandType.MANUAL_CONTROL:
                    self.manual_control(self.arguments['linear_x'], self.arguments['angular_z'])
                elif self.command == CommandType.PURSUIT:
                    self.pursuit(self.arguments['x'], self.arguments['y'], self.arguments['r'], self.arguments['speed'])
                elif self.command == CommandType.CONFIG:
                    self.config(self.arguments['max_velocity'], self.arguments['max_angular'], self.arguments['mode_slam_odom'])
            else:
                if not self.is_status_sent:
                    TASKS.acknowledge_status(self)
                    self.stop()
                    self.is_status_sent = True

            self.is_new_command = False
            count += 1
            rate.sleep()

if __name__ == "__main__":
    
    controller = ImmersionBot()

    udp_receiver = UDPReceiver("0.0.0.0", 8080, controller.udp_callback)
    udp_receiver.start()

    controller.run()
    