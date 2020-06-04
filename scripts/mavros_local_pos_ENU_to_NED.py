#!/usr/bin/env python
"""ENU to NED from MAVROS to ROS general
/mavros/local_position/pose TO
/mavros/local_position/pose_NED
"""



import numpy as np

from pyquaternion import Quaternion
import rospy

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


publisher_position_boat_NED = rospy.Publisher("/mavros/local_position/pose_NED", PoseStamped, queue_size=1)

rate = None
rviz = True
if rviz:
    publisher_marker = rospy.Publisher('/pose_boat_NED', Marker, queue_size=1)

qx_180 = Quaternion(axis=[1, 0, 0], angle=np.pi)
qz_90p = Quaternion(axis=[0, 0, 1], angle=np.pi / 2)


def callback(msg):
    """"""
    global rate
    # msg=PoseStamped()# SPAETER ENTFERNEN@@@@@@@@@@@@@@@@@@@@@@@@@@@

    # msg.pose.position.x
    # print(p.shape)
    pos_boat_x = msg.pose.position.y
    pos_boat_y = msg.pose.position.x
    pos_boat_z = -msg.pose.position.z
    # z 90 erst dann x 180
    tmpquat = Quaternion(w=msg.pose.orientation.w,
                         x=msg.pose.orientation.x,
                         y=msg.pose.orientation.y,
                         z=msg.pose.orientation.z)

    tmpquat = qz_90p * qx_180 * tmpquat * qx_180
    NED = PoseStamped()
    NED.header = msg.header
    NED.header.frame_id = 'global_tank'
    NED.pose.position.x = pos_boat_x
    NED.pose.position.y = pos_boat_y
    NED.pose.position.z = pos_boat_z
    NED.pose.orientation.w = tmpquat.w
    NED.pose.orientation.x = tmpquat.x
    NED.pose.orientation.y = tmpquat.y
    NED.pose.orientation.z = tmpquat.z
    publisher_position_boat_NED.publish(NED)

    if rviz:
        marker = Marker()
        marker.header.frame_id = "global_tank"
        marker.id = 0
        marker.type = marker.MESH_RESOURCE
        marker.action = marker.ADD
        marker.scale.x = 0.8
        marker.scale.y = 0.8
        marker.scale.z = 0.8
        marker.color.r = 1
        marker.color.a = 1  # transparency
        marker.pose.orientation.w = tmpquat.w
        marker.pose.orientation.x = tmpquat.x
        marker.pose.orientation.y = tmpquat.y
        marker.pose.orientation.z = tmpquat.z
        marker.pose.position.x = pos_boat_x  # x
        marker.pose.position.y = pos_boat_y  # y
        marker.pose.position.z = pos_boat_z  # z
        marker.mesh_resource = "package://hippocampus_tools_ros/models/uuv_hippocampus.stl"
        publisher_marker.publish(marker)


def main():
    rospy.init_node('enu_to_ned')
    global rate
    rate = rospy.Rate(30)

    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()
