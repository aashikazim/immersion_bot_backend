from config import Config
from math import sqrt, atan2, sin, cos, atan, asin, pi



class TASKS:

    @staticmethod
    def acknowledge_status(robot, value="Idle"):
        status = {"type": "status", "data": {"status": value}}
        # print(status)
        robot.send_data(status)
    
    #"Pursuing..."


    @staticmethod
    def go_2_goal(robot, x, y):
        if robot.is_new_command:
            robot.xt = x
            robot.yt = y
            TASKS.acknowledge_status(robot, value="Going to goal...")
        
        Kv = 1.0
        Kp = 2.0
        distance = sqrt((robot.xt - robot.x)**2 + (robot.yt - robot.y)**2)
        theta_d = atan2(robot.yt - robot.y, robot.xt - robot.x)
        angle_error = atan2(sin(theta_d - robot.theta),  cos(theta_d - robot.theta))
        print("Distance Error: {}\tAngle Error: {}".format(distance, angle_error))

        robot.vel.angular.z = max(-Config.MAX_ANGULAR, min(Kp * angle_error, Config.MAX_ANGULAR))
        if abs(angle_error) < 0.5:
            robot.vel.linear.x = min(Kv * distance, Config.MAX_VELOCITY)
        else: 
            robot.vel.linear.x = 0

        if distance < 0.1:
            robot.vel.angular.z = 0.0
            robot.vel.linear.x = 0.0
            robot.xt, robot.yt = None, None
            robot.command = None

            # reset all static variables
            if not TASKS.CENTER_X:
                TASKS.reset()

    @staticmethod
    def stop(robot):
        # print("Stopped")
        TASKS.acknowledge_status(robot)
        robot.vel.angular.z = 0.0
        robot.vel.linear.x = 0.0
        robot.command = None

        # reset all static variables
        if not TASKS.CENTER_X:
            TASKS.reset()
    
    @staticmethod
    def reset():
        TASKS.TARGET_VELOCITY = 1.0
        TASKS.PURSUIT_DISTANCE = 0.0
        TASKS.RADIUS = 0.3
        TASKS.CENTER_X = None
        TASKS.CENTER_Y = None
        TASKS.theta_t = 0
        TASKS.ANGULAR_VELOCITY = TASKS.TARGET_VELOCITY / TASKS.RADIUS
        TASKS.DELTA_TIME = 0.1

        TASKS.integral = 0
        TASKS.output = 0
        TASKS.previous_error = 0

    
    @staticmethod
    def manual_control(robot, linear_x, angular_z):
        robot.vel.linear.x = linear_x
        robot.vel.angular.z = angular_z
        robot.command = None
        TASKS.acknowledge_status(robot, value="Teleoperating...")
        # reset all static variables
        if not TASKS.CENTER_X:
            TASKS.reset()
            


    # static varibales for pursuit
    TARGET_VELOCITY = 0.5
    PURSUIT_DISTANCE = 0.05
    RADIUS = 0.3
    CENTER_X = None
    CENTER_Y = None
    theta_t = 0
    ANGULAR_VELOCITY = TARGET_VELOCITY / RADIUS
    DELTA_TIME = 0.1

    integral = 0
    output = 0
    previous_error = 0
    t_steering = 0
    

    @staticmethod
    def init_pursuit(robot, x, y, r, speed):
        TASKS.CENTER_X = x
        TASKS.CENTER_Y = y
        TASKS.RADIUS = r
        TASKS.TARGET_VELOCITY = speed
        TASKS.theta_t = 0
        TASKS.t_steering = 0
        TASKS.ANGULAR_VELOCITY = TASKS.TARGET_VELOCITY / TASKS.RADIUS
        robot.xt = TASKS.CENTER_X + TASKS.RADIUS * cos(TASKS.theta_t)
        robot.yt = TASKS.CENTER_Y + TASKS.RADIUS * sin(TASKS.theta_t)

    @staticmethod
    def pursuit(robot, x, y, r, speed):
        Kp = 3.0
        Ki = 0.01
        Kh = 0.5

        if robot.is_new_command:
            TASKS.init_pursuit(robot, x, y, r, speed)
            TASKS.acknowledge_status(robot, value="Pursuing...")

        # Revolving Pursuit Point
        TASKS.theta_t += TASKS.ANGULAR_VELOCITY * TASKS.DELTA_TIME
        robot.xt = TASKS.CENTER_X + TASKS.RADIUS * cos(TASKS.theta_t)
        robot.yt = TASKS.CENTER_Y + TASKS.RADIUS * sin(TASKS.theta_t)

        # Calculating Error
        distance = sqrt((robot.xt - robot.x)**2 + (robot.yt - robot.y)**2)
        error = distance - TASKS.PURSUIT_DISTANCE
        # error = error - robot.vel.linear.x
        theta_d = atan2(robot.yt - robot.y, robot.xt - robot.x)
        angle_error = atan2(sin(theta_d - robot.theta),  cos(theta_d - robot.theta))
        print("Distance Error: {}\t{}\tAngle Error: {}\t{}\t{}".format(distance, error, angle_error, robot.vel.linear.x, robot.vel.angular.z))
        
        # Applying Control
        TASKS.integral += error * TASKS.DELTA_TIME
        TASKS.output = (Kp * error + Ki * TASKS.integral)
        robot.vel.linear.x = max(-Config.MAX_VELOCITY, min(TASKS.output *  TASKS.DELTA_TIME, Config.MAX_VELOCITY))
        robot.vel.angular.z = Kh * angle_error #max(-0.5, min(Kh * angle_error, 0.5))

        # calculating vRobot pose
        TASKS.t_steering += (TASKS.TARGET_VELOCITY/TASKS.RADIUS) * TASKS.DELTA_TIME 

        # {"type": "pose", "data": {"y": self.x, "x": -self.y, "theta": -self.theta * (180 / pi)}}
        v_robot_pose = {"type": "v_robot", "data": {"y": robot.xt, "x": -robot.yt, "theta": -TASKS.t_steering * (180/pi)}}
        robot.send_data(v_robot_pose)

    
    @staticmethod
    def config(robot, max_velocity, max_angular, mode_slam_odom):
        Config.MAX_VELOCITY = max_velocity
        Config.MAX_ANGULAR = max_angular
        Config.MODE_ODOM_SLAM_ACML = mode_slam_odom
        robot.command = None
        TASKS.acknowledge_status(robot)

        # print("Max VALUES: ", Config.MAX_VELOCITY, Config.MAX_ANGULAR, Config.MODE_ODOM_SLAM_ACML)
        # reset all static variables
        if not TASKS.CENTER_X:
            TASKS.reset()




