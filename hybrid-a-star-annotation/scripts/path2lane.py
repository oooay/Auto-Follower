#!/usr/bin/env python
# coding:utf-8
import rospy
from nav_msgs.msg import Path
from autoware_msgs.msg import Waypoint, Lane, LaneArray


class Path2Lane():
    def path_cb(self, path_msg):
        # print(len(path_msg.poses))
        if self.path_size < len(path_msg.poses):
            ln_array = LaneArray()
            ln_array.id = 0
            ln_array.lanes = []

            ln = Lane()
            ln.header.seq = 0
            ln.header.stamp.secs = 0
            ln.header.stamp.nsecs = 0
            ln.header.frame_id = ''
            ln.increment = 0
            ln.lane_id = 0
            ln.waypoints = []

            for pose in path_msg.poses:

                wp = Waypoint()

                wp.gid = 1
                wp.lid = 0
                wp.pose.header.seq = 0
                wp.pose.header.stamp.secs = 0
                wp.pose.header.stamp.nsecs = 0
                wp.pose.header.frame_id = ''
                wp.pose.pose = pose.pose

                wp.twist.header.seq = 0
                wp.twist.header.stamp.secs = 0
                wp.twist.header.stamp.nsecs = 0
                wp.twist.header.frame_id = ''
                wp.twist.twist.linear.x = self.velocity
                wp.twist.twist.linear.y = 0
                wp.twist.twist.linear.z = 0
                wp.twist.twist.angular.x = 0
                wp.twist.twist.angular.y = 0
                wp.twist.twist.angular.z = 0
                wp.dtlane.dist = 0.0
                wp.dtlane.dir = 0.0
                wp.dtlane.apara = 0.0
                wp.dtlane.r = 0.0
                wp.dtlane.slope = 0.0
                wp.dtlane.cant = 0.0
                wp.dtlane.lw = 0.0
                wp.dtlane.rw = 0.0
                wp.change_flag = 0
                wp.wpstate.aid = 0
                wp.wpstate.lanechange_state = 0
                wp.wpstate.steering_state = 0
                wp.wpstate.accel_state = 0
                wp.wpstate.stop_state = 0
                wp.wpstate.event_state = 0
                wp.lane_id = 1
                wp.left_lane_id = 0
                wp.right_lane_id = 0
                wp.stop_line_id = 4294967295
                wp.cost = 0.0
                wp.time_cost = 0.0
                wp.direction = 0.0

                ln.waypoints.append(wp)

            ln.lane_index = 0
            ln.cost = 0.0
            ln.closest_object_distance = 0.0
            ln.closest_object_velocity = 0.0
            ln.is_blocked = False

            # ln_array.lanes.append(ln)
            ln_array.lanes += (ln, )

            self.laneArr_pub.publish(ln_array)
            self.path_size = len(path_msg.poses)
            rospy.loginfo("Lane has updated successfully!")
        else:
            pass

    def run(self):
        self.rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            rospy.spin()
            self.rate.sleep()

    def __init__(self):
        rospy.init_node('path2lane', anonymous=False)
        self.velocity = 0.5
        self.path_size = 0
        # subs and pubs
        self.path_sub = rospy.Subscriber("/path_in_globalmap",
                                         Path,
                                         self.path_cb,
                                         queue_size=1)
        self.laneArr_pub = rospy.Publisher("/lane_waypoints_array",
                                           LaneArray,
                                           queue_size=1)


if __name__ == "__main__":
    try:
        path2lane = Path2Lane()
        path2lane.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('Path2Lane is finished')
