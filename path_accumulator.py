#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import nav_msgs.msg
import std_msgs.msg

class CombinedPathAccumulator:
    def __init__(self):
        # Initialize node
        rospy.init_node('combined_path_accumulator')

        # Create a buffer and listener for tf
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publishers for the accumulated paths
        self.path_pub_mocap_laser = rospy.Publisher('/mocap_laser_path_in_mocap', nav_msgs.msg.Path, queue_size=10)
        self.path_pub_base_scan = rospy.Publisher('/base_scan_path_in_map', nav_msgs.msg.Path, queue_size=10)

        # Path messages to store accumulated paths
        self.path_mocap_laser = nav_msgs.msg.Path()
        self.path_mocap_laser.header.frame_id = "mocap"
        self.path_base_scan = nav_msgs.msg.Path()
        self.path_base_scan.header.frame_id = "map"

    def accumulate_paths(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            try:
                # Dynamic transform from mocap to mocap_laser_link
                trans_mocap_to_laser = self.tf_buffer.lookup_transform('mocap', 'mocap_laser_link', rospy.Time(0))
                pose_mocap_laser = geometry_msgs.msg.PoseStamped()
                pose_mocap_laser.header.stamp = rospy.Time.now()
                pose_mocap_laser.header.frame_id = "mocap"
                pose_mocap_laser.pose.position = trans_mocap_to_laser.transform.translation
                pose_mocap_laser.pose.orientation = trans_mocap_to_laser.transform.rotation
                self.path_mocap_laser.poses.append(pose_mocap_laser)
                self.path_pub_mocap_laser.publish(self.path_mocap_laser)

                # Dynamic transform from map to base_link and static transform from base_link to base_scan
                trans_map_to_base_link = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
                trans_base_link_to_base_scan = self.tf_buffer.lookup_transform('base_link', 'base_scan', rospy.Time(0), rospy.Duration(1.0))
                pose_base_scan_in_map = tf2_geometry_msgs.do_transform_pose(geometry_msgs.msg.PoseStamped(
                    header=std_msgs.msg.Header(stamp=rospy.Time.now(), frame_id="base_link"),
                    pose=geometry_msgs.msg.Pose(position=trans_base_link_to_base_scan.transform.translation)
                ), trans_map_to_base_link)
                self.path_base_scan.poses.append(pose_base_scan_in_map)
                self.path_pub_base_scan.publish(self.path_base_scan)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

            rate.sleep()

if __name__ == '__main__':
    pa = CombinedPathAccumulator()
    pa.accumulate_paths()



