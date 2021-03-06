"""
Collection of helper functions for coordinate transformation management for any
node *within one vehicle*

This node doesn't actually publish any tf data, the tfPublisher defines and
publishes the transforms between different frames within one vehicle.
"""

import os
import numpy as np
import rospy
import tf2_ros
import tf.transformations
from geometry_msgs.msg import TransformStamped, Quaternion
import math
from hippocampus_common.node import Node

Q_NED_ENU = tf.transformations.quaternion_from_euler(math.pi, 0.0,
                                                     math.pi / 2.0)
Q_FRD_FLU = tf.transformations.quaternion_from_euler(math.pi, 0.0, 0.0)


class TfHelper(object):
    def __init__(self, vehicle_name):
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.id_prefix = vehicle_name
        self._base_link_id = None
        self._base_link_frd_id = None
        self._base_link_ground_truth_id = None
        self._front_camera_frame_id = None
        self._vertical_camera_frame_id = None
        self._front_camera_link_id = None
        self._vertical_camera_link_id = None
        self._barometer_link_id = None

        self._base_link_flu_to_frd_static_tf = None
        self._base_link_frd_to_flu_static_tf = None
        self._vertical_camera_frame_to_base_link_static_tf = None
        self._front_camera_frame_to_base_link_static_tf = None

    def _get_base_link_id(self):
        default = os.path.join(self.id_prefix, "base_link").strip("/")
        return Node.get_param("~base_link", default)

    def _get_base_link_frd_id(self):
        default = os.path.join(self.id_prefix, "base_link_frd").strip("/")
        return Node.get_param("~base_link_frd", default)

    def _get_base_link_ground_truth_id(self):
        default = os.path.join(self.id_prefix,
                               "base_link_ground_truth").strip("/")
        return Node.get_param("~base_link_ground_truth", default)

    def _get_front_camera_link_id(self):
        default = os.path.join(self.id_prefix,
                               "front_camera/camera_link").strip("/")
        return Node.get_param("~front_camera_link", default)

    def _get_front_camera_frame_id(self):
        default = os.path.join(self.id_prefix,
                               "front_camera/camera_frame").strip("/")
        return Node.get_param("~front_camera_frame", default)

    def _get_vertical_camera_link_id(self):
        default = os.path.join(self.id_prefix,
                               "vertical_camera/camera_link").strip("/")
        return Node.get_param("~vertical_camera_link", default)

    def _get_vertical_camera_frame_id(self):
        default = os.path.join(self.id_prefix,
                               "vertical_camera/camera_frame").strip("/")
        return Node.get_param("~vertical_camera_frame", default)

    def _get_barometer_link_id(self):
        default = os.path.join(self.id_prefix, "barometer_link").strip("/")
        return Node.get_param("~barometer_link", default)

    def get_base_link_id(self):
        if not self._base_link_id:
            self._base_link_id = self._get_base_link_id()
        return self._base_link_id

    def get_base_link_frd_id(self):
        if not self._base_link_frd_id:
            self._base_link_frd_id = self._get_base_link_frd_id()
        return self._base_link_frd_id

    def get_base_link_ground_truth_id(self):
        if not self._base_link_ground_truth_id:
            self._base_link_ground_truth_id = \
                self._get_base_link_ground_truth_id()
        return self._base_link_ground_truth_id

    def get_front_camera_link_id(self):
        if not self._front_camera_link_id:
            self._front_camera_link_id = self._get_front_camera_link_id()
        return self._front_camera_link_id

    def get_front_camera_frame_id(self):
        if not self._front_camera_frame_id:
            self._front_camera_frame_id = self._get_front_camera_frame_id()
        return self._front_camera_frame_id

    def get_vertical_camera_link_id(self):
        if not self._vertical_camera_link_id:
            self._vertical_camera_link_id = self._get_vertical_camera_link_id()
        return self._vertical_camera_link_id

    def get_vertical_camera_frame_id(self):
        if not self._vertical_camera_frame_id:
            self._vertical_camera_frame_id = self._get_vertical_camera_frame_id(
            )
        return self._vertical_camera_frame_id

    def get_barometer_link_id(self):
        if not self._barometer_link_id:
            self._barometer_link_id = self._get_barometer_link_id()
        return self._barometer_link_id

    def _get_base_link_flu_to_frd_tf(self):
        transform = TransformStamped()
        q = tf.transformations.quaternion_inverse(Q_FRD_FLU)
        transform.transform.rotation = Quaternion(*q)
        transform.child_frame_id = self.get_base_link_frd_id()
        transform.header.frame_id = self.get_base_link_id()
        return transform

    def _get_base_link_frd_to_flu_tf(self):
        transform = TransformStamped()
        transform.transform.rotation = Quaternion(*Q_FRD_FLU)
        transform.child_frame_id = self.get_base_link_id()
        transform.header.frame_id = self.get_base_link_frd_id()
        return transform

    def _get_front_camera_frame_to_base_link_tf(self):
        if self.tf_buffer.can_transform(self.get_base_link_id(),
                                        self.get_front_camera_frame_id(),
                                        rospy.Time()):
            transform = self.tf_buffer.lookup_transform(
                target_frame=self.get_base_link_id(),
                source_frame=self.get_front_camera_frame_id(),
                time=rospy.Time(0),
                timeout=rospy.Duration(10))
        else:
            rospy.logfatal(
                "[{}] Transform from front camera to base link not found. \
                    Check if correct vehicle type and camera name.".format(
                    rospy.get_name()))
        return transform

    def _get_vertical_camera_frame_to_base_link_tf(self):
        if self.tf_buffer.can_transform(self.get_base_link_id(),
                                        self.get_vertical_camera_frame_id(),
                                        rospy.Time()):
            transform = self.tf_buffer.lookup_transform(
                target_frame=self.get_base_link_id(),
                source_frame=self.get_vertical_camera_frame_id(),
                time=rospy.Time(0),
                timeout=rospy.Duration(10))
        else:
            rospy.logfatal(
                "[{}] Transform from vertical camera to base link not found. \
                    Check if correct vehicle type and camera name.".format(
                    rospy.get_name()))
        return transform

    def get_base_link_flu_to_frd_tf(self):
        if not self._base_link_flu_to_frd_static_tf:
            self._base_link_flu_to_frd_static_tf = \
                self._get_base_link_flu_to_frd_tf()
        return self._base_link_flu_to_frd_static_tf

    def get_base_link_frd_to_flu_tf(self):
        if not self._base_link_frd_to_flu_static_tf:
            self._base_link_frd_to_flu_static_tf = \
                self._get_base_link_frd_to_flu_tf()
        return self._base_link_frd_to_flu_static_tf

    def get_front_camera_frame_to_base_link_tf(self):
        if not self._front_camera_frame_to_base_link_static_tf:
            self._front_camera_frame_to_base_link_static_tf = \
                self._get_front_camera_frame_to_base_link_tf()
        return self._front_camera_frame_to_base_link_static_tf

    def get_vertical_camera_frame_to_base_link_tf(self):
        if not self._vertical_camera_frame_to_base_link_static_tf:
            self._vertical_camera_frame_to_base_link_static_tf = \
                self._get_vertical_camera_frame_to_base_link_tf()
        return self._vertical_camera_frame_to_base_link_static_tf

    def get_camera_frame_to_base_link_tf(self, camera_name):
        # use this when a node potentially works with any camera.
        # for now, all camera names need to be hardcoded here
        if camera_name == "front_camera":
            return self.get_front_camera_frame_to_base_link_tf()
        elif camera_name == "vertical_camera":
            return self.get_vertical_camera_frame_to_base_link_tf()
        else:
            rospy.logfatal("[{}] Specified camera_name does not exist. \
                    No transform found!".format(rospy.get_name()))
            exit(-1)

    def get_map_to_base_link_ground_truth_tf(self):
        transform = self.tf_buffer.lookup_transform(
            target_frame=self.get_base_link_ground_truth_id(),
            source_frame="map",
            time=rospy.Time(),
            timeout=rospy.Duration(1))

        transform.child_frame_id = self.get_base_link_ground_truth_id()
        transform.header.frame_id = "map"
        return transform

    def pose_flu_to_frd(self, pose):
        transform = self.get_base_link_flu_to_frd_tf()
        orientation = pose.pose.orientation
        q_orig = (orientation.x, orientation.y, orientation.z, orientation.w)
        rotation = transform.transform.rotation
        q_rot = (rotation.x, rotation.y, rotation.z, rotation.w)
        translation = transform.transform.translation
        q_new = tf.transformations.quaternion_multiply(q_orig, q_rot)

        pose.pose.orientation.x = q_new[0]
        pose.pose.orientation.y = q_new[1]
        pose.pose.orientation.z = q_new[2]
        pose.pose.orientation.w = q_new[3]

        pose.pose.position.x += translation.x
        pose.pose.position.y += translation.y
        pose.pose.position.z += translation.z
        return pose

    def pose_frd_to_flu(self, pose):
        transform = self.get_base_link_frd_to_flu_tf()
        orientation = pose.pose.orientation
        q_orig = (orientation.x, orientation.y, orientation.z, orientation.w)
        rotation = transform.transform.rotation
        q_rot = (rotation.x, rotation.y, rotation.z, rotation.w)
        translation = transform.transform.translation
        q_new = tf.transformations.quaternion_multiply(q_orig, q_rot)

        pose.pose.orientation.x = q_new[0]
        pose.pose.orientation.y = q_new[1]
        pose.pose.orientation.z = q_new[2]
        pose.pose.orientation.w = q_new[3]

        pose.pose.position.x += translation.x
        pose.pose.position.y += translation.y
        pose.pose.position.z += translation.z
        return pose

    def twist_ground_truth_map_to_body_frame(self, twist):
        transform = self.get_map_to_base_link_ground_truth_tf()

        # only apply rotation to twist
        rotation = transform.transform.rotation
        q_rot = (rotation.x, rotation.y, rotation.z, rotation.w)
        rotation_matrix = tf.transformations.quaternion_matrix(q_rot)[:3, :3]
        linear_map = np.array(
            [twist.twist.linear.x, twist.twist.linear.y,
             twist.twist.linear.z]).reshape((-1, 1))
        angular_map = np.array([
            twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z
        ]).reshape((-1, 1))
        linear_body = np.matmul(rotation_matrix, linear_map)
        angular_body = np.matmul(rotation_matrix, angular_map)

        twist.twist.linear.x = linear_body[0]
        twist.twist.linear.y = linear_body[1]
        twist.twist.linear.z = linear_body[2]
        twist.twist.angular.x = angular_body[0]
        twist.twist.angular.y = angular_body[1]
        twist.twist.angular.z = angular_body[2]

        return twist
