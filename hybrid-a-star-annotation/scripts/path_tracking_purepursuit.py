#!/usr/bin/env python
# from importlib.resources import path
import math
import copy
import numpy as np
import tf
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class PathTracking():
    def path_cb(self, path):
        #self.updated_path = path
        if len(path.poses) > 0:
            self.updated_path = path
            self.current_target_idx = 0
            self.target_x = path.poses[-1].pose.position.x
            self.target_y = path.poses[-1].pose.position.y
        else:
            self.target_x = float("inf")
            self.target_y = float("inf")
            #pass

    def vel_cb(self, vel):
        self.recommend_speed = vel.data

    def odom_cb(self, odom):
        self.current_v = odom.twist.twist.linear.x

    def pure_pursuit(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.path = copy.deepcopy(self.updated_path)
            try:
                (pos,
                 rot) = self.listener.lookupTransform("map", "base_link",
                                                      rospy.Time())
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                continue
            rospy.loginfo_once('TF is ready!')

            if len(self.path.poses) > 0:
                print('Path has {} points!'.format(len(self.path.poses)))
                current_x = pos[0]
                current_y = pos[1]
                #print('current_x: {}, current_y: {}'.format(current_x, current_y))
                current_theta = tf.transformations.euler_from_quaternion(
                    rot)[2]
                dist_goal = np.hypot(current_x - self.target_x, current_y - self.target_y)
                print('dist_goal is {}.'.format(dist_goal))

                twist = Twist()
                if (dist_goal < 0.1) or (dist_goal == float("inf")):
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                else:
                    dx = current_x - self.path.poses[
                        self.current_target_idx].pose.position.x
                    dy = current_y - self.path.poses[
                        self.current_target_idx].pose.position.y
                    dist_target = np.hypot(dx, dy)

                    n = 3
                    if (dist_target < n * self.DT) and (
                            self.current_target_idx <
                            len(self.path.poses) - n * self.step):
                        self.current_target_idx += n * self.step
                        #print("Current target idx: {}".format(self.current_target_idx))
                        dx = current_x - self.path.poses[
                            self.current_target_idx].pose.position.x
                        dy = current_y - self.path.poses[
                            self.current_target_idx].pose.position.y
                        dist_target = np.hypot(dx, dy)
                    else:
                        self.current_target_idx == len(self.path.poses) - 1
                        #print("Current target idx: {}".format(self.current_target_idx))
                        dx = current_x - self.path.poses[
                            self.current_target_idx].pose.position.x
                        dy = current_y - self.path.poses[
                            self.current_target_idx].pose.position.y
                        dist_target = np.hypot(dx, dy)

                    print('Update target to {}, distance to target is {}.'.
                          format(self.current_target_idx, dist_target))

                    alpha = math.atan2(-dy, -dx) - current_theta
                    alpha = np.mod(alpha + math.pi, 2 * math.pi) - math.pi

                    angular_z = 0.5 * alpha
                    if angular_z > self.MaxAngular:
                        angular_z = self.MaxAngular
                    elif angular_z < -self.MaxAngular:
                        angular_z = -self.MaxAngular

                    #twist.linear.x = self.MaxLinear / (1 + pow(10000, 0.5 - 2 * dist_target))
                    twist.linear.x = min(
                        self.current_v + self.acc, self.recommend_speed
                    ) if self.current_v < self.recommend_speed else max(
                        self.current_v - self.acc, self.recommend_speed)
                    twist.angular.z = angular_z

                self.cmd_vel_pub.publish(twist)

        rospy.spin()
        rate.sleep()

    def __init__(self):

        rospy.init_node('path_tracking', anonymous=False)
        # member var
        self.path = Path()
        self.updated_path = Path()
        self.DT = 0.2  # dist_target
        self.current_target_idx = 0
        self.step = 1
        self.target_x = float("inf")
        self.target_y = float("inf")
        self.recommend_speed = 0.0
        self.current_v = 0.0
        self.acc = 0.3
        # param
        # self.DL = rospy.get_param('~DL', 1.0)
        # self.MaxLinear = rospy.get_param('~MaxLinear', 1.2)
        self.MaxAngular = rospy.get_param('~MaxAngular', 1.0)

        # subs and pubs
        self.path_sub = rospy.Subscriber("path_in_globalmap", Path,
                                         self.path_cb)
        self.vel_sub = rospy.Subscriber("recommend_speed", Float32,
                                        self.vel_cb)
        self.odom_sub = rospy.Subscriber("vehicle/odom", Odometry,
                                         self.odom_cb)

        self.listener = tf.TransformListener()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)


if __name__ == '__main__':

    try:
        tracker = PathTracking()
        tracker.pure_pursuit()

    except rospy.ROSInterruptException:

        rospy.loginfo("Path Tracking finished.")
