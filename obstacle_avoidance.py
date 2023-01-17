#!/usr/bin/env python3

'''
Field based Obstacle Avoidance
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from math import sin, cos, atan2, sqrt, pi
from tf.transformations import euler_from_quaternion

class RobotState:
    def __init__(self, x=0.0, y=0.0, theta=0.0, vx=0.0, vy=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.vx = vx
        self.vy =vy

    def print_state(self):
        print("Robot State X: {} Y: {} Theta: {}", self.x, self.y, self.theta)

class ObstacleAvoidanceController:
    def __init__(self, goal_pose: PoseStamped):

        self.velocity_to_publish = Twist()
        self.current_pose = PoseStamped()
        self.goal_pose = goal_pose
        self.robot_state = RobotState()

        self.ranges = []

        self.RUNNING = False
        self.GOAL_TOLERANCE = 0.1
        self.MAX_VELOCITY = 0.5
        self.rate = rospy.Rate(10)
        self.LIDAR_ANG_INCREMENT = 0.017

        self.OBSTACLE_SAFETY_THRESHOLD = 0.1
        self.ANGULAR_CORRECTION_CONSTANT = 0.0045
        self.RADIAL_CORRECTION_CONSTANT = 0.00001

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        # self.map_publisher = rospy.Publisher('/map', OccupancyGrid, queue_size=2)

        self.goal_pose_subscriber = rospy.Subscriber('/goal_pose', PoseStamped, callback=self.goal_callback)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, callback=self.odom_callback)
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, callback=self.lidar_callback)

        # Update map only if robot is stationary
        self.update_map = True
        self.MAPPING_SPEED_TOLERANCE = 0.001

    def publish_velocity(self):

        if not self.RUNNING:
            pass
        else:
            (vx, vy) = self.get_target_velocity(self.current_pose, self.goal_pose)
            # print("Received: ", vx, vy)        
            (vx_corrected, vy_corrected) = self.get_corrected_velocity(vx, vy, self.ranges, self.OBSTACLE_SAFETY_THRESHOLD)

        self.velocity_to_publish.linear.x = vx_corrected
        self.velocity_to_publish.linear.y = vy_corrected

        self.velocity_publisher.publish(self.velocity_to_publish)

    def get_target_velocity(self, current_pose, goal_pose):
        ex = (goal_pose.pose.position.x - current_pose.pose.position.x)
        ey = (goal_pose.pose.position.y - current_pose.pose.position.y)
        # print("Error: ", ex, ey)
        reachedX = False
        reachedY = False
        if(abs(ex) < self.GOAL_TOLERANCE):
            reachedX = True
        if(abs(ey) < self.GOAL_TOLERANCE):
            reachedY = True

        theta = atan2(ey, ex + 0.000001)
        e_mag = sqrt(ex**2 + ey**2)
        e_mag = min(e_mag, self.MAX_VELOCITY)

        vx = e_mag * cos(theta)
        vy = e_mag * sin(theta)
        # print("Velocity: ", vx, vy)
        if(reachedX):
            vx = 0
        if(reachedY):
            vy = 0
        return (vx, vy)

    def get_corrected_velocity(self, vx, vy, ranges, d_threshold):
        theta = atan2(vx, vy+0.000001)

        vx_corrected = vx
        vy_corrected = vy

        for i in range(len(ranges)):
            if(ranges[i] != float('inf')):

                alpha = self.normalize_pi(self.LIDAR_ANG_INCREMENT * i)
                if(alpha > (theta + 0.7 * pi)) or (alpha < (theta - 0.7 * pi)):
                    continue
                # elif ranges[i] > 2.0:
                #     continue
                else:
                    r = ranges[i]
                    d_obs = abs(r*sin(alpha) - (vy/(vx + 0.000001))*r*cos(alpha)) / (sqrt(1 + (vy/(vx + 0.000001))**2))

                    # v_ang = self.ANGULAR_CORRECTION_CONSTANT / d_obs
                    # vx_corrected += v_ang * sin(theta)
                    # vy_corrected -= v_ang * cos(theta)
                    # print(vx_corrected, vy_corrected)

                    if d_obs > d_threshold:
                        v_ang = self.ANGULAR_CORRECTION_CONSTANT / d_obs
                        if(theta < pi/2):
                            if(alpha < theta):
                                SIGN_X = -1
                                SIGN_Y = +1
                            else:
                                SIGN_X = +1
                                SIGN_Y = -1
                            vx_corrected = vx_corrected  + SIGN_X * (v_ang * sin(theta))
                            vy_corrected = vy_corrected  + SIGN_Y * (v_ang * cos(theta))

                        elif(theta < pi):
                            if(alpha < theta):
                                SIGN_X = -1
                                SIGN_Y = -1
                            else:
                                SIGN_X = +1
                                SIGN_Y = +1

                            vx_corrected = vx_corrected  + SIGN_X * (v_ang * sin(pi - theta))
                            vy_corrected = vy_corrected  + SIGN_Y * (v_ang * cos(pi - theta))

                        elif(theta < 3 * pi/2):
                            if(alpha < theta):
                                SIGN_X = +1
                                SIGN_Y = -1
                            else:
                                SIGN_X = -1
                                SIGN_Y = +1
                                
                            vx_corrected = vx_corrected  + SIGN_X * (v_ang * sin(3 * pi/2 - theta))
                            vy_corrected = vy_corrected  + SIGN_Y * (v_ang * cos(3 * pi/2 - theta))

                        elif(theta < 2 * pi):
                            if(alpha < theta):
                                SIGN_X = +1
                                SIGN_Y = +1
                            else:
                                SIGN_X = -1
                                SIGN_Y = -1
                                
                            vx_corrected = vx_corrected  + SIGN_X * (v_ang * sin(2 * pi - theta))
                            vy_corrected = vy_corrected  + SIGN_Y * (v_ang * cos(2 * pi - theta))


                    elif d_obs <= d_threshold:
                        d_impact = abs(r * cos(alpha - theta))
                        v_rad = self.RADIAL_CORRECTION_CONSTANT / (d_obs + d_impact)
                        if(alpha < pi/2):
                            vx_corrected -= v_rad * cos(alpha)
                            vy_corrected -= v_rad * sin(alpha)
                        elif(alpha < pi):
                            vx_corrected += v_rad * cos(pi - alpha)
                            vy_corrected -= v_rad * sin(pi - alpha)
                        elif(alpha < 1.5 * pi):
                            vx_corrected += v_rad * sin(1.5*pi - alpha)
                            vy_corrected += v_rad * cos(1.5*pi - alpha)
                        elif(alpha < 2 * pi):
                            vx_corrected -= v_rad * cos(2* pi - alpha)
                            vy_corrected += v_rad * sin(2 * pi - alpha)                            

                        # vx_corrected -= v_rad * cos(alpha)
                        # vy_corrected -= v_rad * sin(alpha)

        return (vx_corrected, vy_corrected)

    def odom_callback(self, data: Odometry):
        self.current_pose = data.pose

    def lidar_callback(self, data: LaserScan):
        self.ranges = list(data.ranges)

    def goal_callback(self, data: PoseStamped):
        self.goal_pose = data

    def normalize_pi(self, alpha):
        if(alpha > pi):
            return -(2 * pi - alpha)
        else:
            return alpha

    def run(self):
        self.RUNNING = True
        while self.RUNNING:
            self.publish_velocity()
            self.rate.sleep()

    def stop(self):
        self.RUNNING = False
        self.velocity_publisher.publish(Twist())
        
if __name__ == '__main__':
    
    rospy.init_node('exploration_mapping_node')
    goal_pose = PoseStamped()
    goal_pose.pose.position.x = 5.0
    goal_pose.pose.position.y = 5.0
    controller = ObstacleAvoidanceController(goal_pose)
    controller.run()
    rospy.spin()
    controller.stop()
    rospy.sleep(2)