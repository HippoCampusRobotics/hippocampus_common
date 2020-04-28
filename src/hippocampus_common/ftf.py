# -*- coding: utf-8 -*-
"""This module provides functions for transformations between px4/airframe and gazebo/baselink coordinate systems.

Transformations are python adapations of the c++ sourcecode of the mavros package found at https://github.com/mavlink/mavros

Example:
    Either::
        orientation_px4 = orientation_gazebo_to_px4(orientation_gazebo)
    or::
        orientation_gazebo = orientation_px4_to_gazebo(orientation_px4)
"""
import math
import tf.transformations
import pyquaternion
import numpy

__author__ = "Thies Lennart Alff"

NED_ENU_Q = tf.transformations.quaternion_from_euler(math.pi, 0.0,
                                                     math.pi / 2.0)
# numpy.roll is needed since tf's order is xyzw and pyquaternion's order is wxyz
NED_ENU_Q = pyquaternion.Quaternion(numpy.roll(NED_ENU_Q, 1))

AIRCRAFT_BASELINK_Q = tf.transformations.quaternion_from_euler(math.pi, 0, 0)
# numpy.roll is needed since tf's order is xyzw and pyquaternion's order is wxyz
AIRCRAFT_BASELINK_Q = pyquaternion.Quaternion(
    numpy.roll(AIRCRAFT_BASELINK_Q, 1))


def quaternion_from_rpy(roll, pitch, yaw):
    """Get the quaternion representation of a twist given by roll, pitch and yaw.

    Args:
        roll (float): roll angle in radians
        pitch (float): ptich angle in radians
        yaw (float): yaw angle in radians

    Returns:
        [pyquaternion.Quaternion]: Quaternion object 
    """
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return pyquaternion.Quaternion(numpy.roll(quaternion, 1))


def rpy_from_quaternion(quaternion):
    """Get the roll-pitch-yaw representation of a twist given by quaternions.

    Args:
        quaternion (pyquaternion.Quaternion): Twist

    Returns:
        [tuple]: Tuple of roll, pitch and yaw angles.
    """
    (yaw, pitch, roll) = quaternion.yaw_pitch_roll
    return (roll, pitch, yaw)


def _transform_orientation(transform, orientation_q):
    if transform == "body_frames":
        return orientation_q * AIRCRAFT_BASELINK_Q
    elif transform == "global_frames":
        return NED_ENU_Q * orientation_q


def orientation_baselink_to_aircraft(orientation_q):
    return _transform_orientation("body_frames", orientation_q)


def orientation_aircraft_to_baselink(orientation_q):
    return _transform_orientation("body_frames", orientation_q)


def orientation_ned_to_enu(orientation_q):
    return _transform_orientation("global_frames", orientation_q)


def orientation_enu_to_ned(orientation_q):
    return _transform_orientation("global_frames", orientation_q)


def orientation_gazebo_to_px4(orientation_q):
    """Twist of the baselink to the static ENU frame <-> Twist of the airframe to the static NED frame
    
    Args:
        orientation_q (pyquaternion.Quaternion): Orientation of the baselink in respect to the ENU frame.
    
    Returns:
        pyquaternion.Quaternion: Orientation of the airframe in respect to the NED frame.
    """
    return orientation_enu_to_ned(
        orientation_baselink_to_aircraft(orientation_q))


def orientation_px4_to_gazebo(orientation_q):
    """Twist of the airframe to the static NED frame <-> Twist of the baselink to the static ENU frame.
    
    Args:
        orientation_q (pyquaternion.Quaternion): Orientation of the airframe in respect to the NED frame.
    
    Returns:
        pyquaternion.Quaternion: Orientation of the baselink in respect to the ENU frame.
    """
    return orientation_aircraft_to_baselink(
        orientation_ned_to_enu(orientation_q))


if __name__ == "__main__":
    pass