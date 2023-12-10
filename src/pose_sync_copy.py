#!/usr/bin/env python
import json
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Range, Imu
from math import atan2, pi
import tf

from udp_sender import send #,ThreadedUDPSender
from udp_receiver_copy import UdpReceiver
from command_type import CommandType
from util import TASKS
from config import Config

class ImmersionBot:
    def __init__(self):
        # Configuration Constants
        self.UDP_IP_1 = "192.168.0.103" #103
        self.UDP_IP_2 = "192.168.0.201"
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


        # udp data receiver
        self.udp_receiver = UdpReceiver("0.0.0.0", 8080)
        self.udp_receiver.start()
        self.command = None


        # ir_data counter
        self.ir_data_count = 0

        # Initialize ROS Node
        rospy.init_node('rosbot_turn', anonymous=True)
        self.listener = tf.TransformListener()

        # Publisher and Subscriber
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.odom_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/mcl_pose", PoseStamped, self.odom_callback_slam)
        # rospy.Subscriber("/acml_pose", PoseWithCovarianceStamped, self.odom_callback_acml)
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

    # def odom_callback_acml(self, data):
    #     pos = data.pose.pose.position
    #     rot = data.pose.pose.orientation

    #     if Config.MODE_ODOM_SLAM_ACML == Config.ACML:
    #         self.x, self.y = pos.x, pos.y
    #         qx, qy, qz, qw = rot.x, rot.y, rot.z, rot.w
    #         self.theta = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    #         if not self.is_init:
    #             self.is_init = True
    #             self.INIT_X = self.x
    #             self.INIT_Y = self.y
    #             self.INIT_THETA = self.theta
    #         pose_data = {"type": "pose", "data": {"y": self.x, "x": -self.y, "theta": -self.theta * (180 / pi)}}
    #         self.send_data(pose_data)

    def imuCallback(self, data):
        la = data.linear_acceleration
        av = data.angular_velocity

        imu_data = {"type": "imu", "data": {"linear_acceleration": [la.x, la.y, la.z], "angular_velocity": [av.x, av.y, av.z]}}
        self.send_data(imu_data)

    def velocityCallback(self, data):
        # lv = data.linear
        # av = data.angular
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
        # pos = data.pose.pose.position
        # rot = data.pose.pose.orientation
        if Config.MODE_ODOM_SLAM_ACML == Config.SLAM:
            self.x, self.y = pos.x, pos.y
            qx, qy, qz, qw = rot.x, rot.y, rot.z, rot.w
            self.theta = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            # print(self.x, self.y, self.theta)

            if not self.is_init:
                self.is_init = True
                self.INIT_X = self.x
                self.INIT_Y = self.y
                self.INIT_THETA = self.theta
            pose_data = {"type": "pose", "data": {"y": self.x, "x": -self.y, "theta": -self.theta * (180 / pi)}}
            # print("Sent Pose")
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
            # self.listener.waitForTransform("/map", "/base_link", rospy.Time(1), rospy.Duration(0.1))
            (pos, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            # print(pos, rot)
            # trans is the translation (x, y, z)
            # rot is the rotation in quaternion (x, y, z, w)
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
        # except Exception as e:
            # print("transform Exception!", e)

    def lidar_callback(self, data):
        self.ranges = data.ranges
        lidar_data = {"type": "lidar", "data": {"ranges": self.ranges}}
        self.send_data(lidar_data)


    '''
    |---------------------------
    | TASKS
    |---------------------------
    '''
    def go_to_goal(self):
        print("In Go TO GOAL")
        TASKS.go_2_goal(self)

    def manual_control(self):
        print("In MANUAL CONTROL")
        TASKS.manual_control(self)

    def stop(self):
        print("In STOP")
        TASKS.stop(self)

    def pursuit(self):
        print("In PURSUIT")
        TASKS.pursuit(self)
        

    def config(self):
        print("In CONFIG")
        TASKS.config(self)




    def send_data(self, data):
        send(json.dumps(data), self.UDP_IP_1, self.UDP_PORT)
        send(json.dumps(data),  self.UDP_IP_2, self.UDP_PORT)


    def run(self):
        rate = rospy.Rate(10)
        self.vel = Twist()
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        count = 0

        while not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)
            
            # print("running....")
            # command received
            if self.udp_receiver.is_data_received:
                self.udp_receiver.is_data_received = False
                self.command = self.udp_receiver.parsed_data["command"]

            # print("command: ", self.command, count)
            if self.command:
                if self.command == CommandType.GO_TO_GOAL:
                    self.go_to_goal()
                elif self.command == CommandType.STOP:
                    self.stop()
                elif self.command == CommandType.MANUAL_CONTROL:
                    self.manual_control()
                    if self.udp_receiver.prev_manual_time:
                        if (time.time() - self.udp_receiver.prev_manual_time) > 0.5: 
                            self.stop()
                            print("Manually stopped.")
                            self.udp_receiver.prev_manual_time = None
                    
                elif self.command == CommandType.PURSUIT:
                    self.pursuit()
                elif self.command == CommandType.CONFIG:
                    self.config()
                else:
                    print("Error in data format!")
                
            else:
                if not self.is_status_sent:
                    TASKS.acknowledge_status(self)
                    # self.stop()
                    self.is_status_sent = True

            count += 1
            # if(count == 10000):
            #     self.vel.linear.x = 0.0
            #     self.vel.angular.z = 0.0
            #     break
            rate.sleep()

if __name__ == "__main__":
    
    controller = ImmersionBot()
    controller.run()
    