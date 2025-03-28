#!/usr/bin/env python
import rospy
import message_filters
import csv
from std_msgs.msg import Int32
from geometry_msgs.msg import Point


def multi_callback(rotation, position):
    row_data = [rotation.data, position.x, position.y, rospy.Time.now().to_sec()]
    global writer
    writer.writerow(row_data)
    print(row_data)


if __name__ == "__main__":
    rospy.init_node("record_rotation_and_position_node", anonymous=True)
    columns = ["rotation", "position.x", "position.y", "timestamp"]
    # with open("record_rotation_and_position.csv", "a", newline="") as f:  # python3 
    with open("record_rotation_and_position.csv", "w") as f:
        writer = csv.writer(f)
        writer.writerow(columns)
        sub_rotation = message_filters.Subscriber("/motor_angle", Int32, queue_size=1)
        sub_position = message_filters.Subscriber(
            "/visual_follow_move/target_motor_position", Point, queue_size=1
        )
        sync = message_filters.ApproximateTimeSynchronizer(
            [sub_rotation, sub_position], 10, 0.1, allow_headerless=True
        )
        sync.registerCallback(multi_callback)
        rospy.spin()
