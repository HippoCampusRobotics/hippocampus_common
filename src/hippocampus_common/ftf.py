# -*- coding: utf-8 -*-
"""This module provides functions for transformations between px4/airframe and
gazebo/baselink coordinate systems.

Transformations are python adapations of the c++ sourcecode of the mavros
package found at https://github.com/mavlink/mavros

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
AIRCRAFT_BASELINK_Q = pyquaternion.Quaternion(numpy.roll(
    AIRCRAFT_BASELINK_Q, 1))


def quaternion_from_rpy(roll, pitch, yaw):
    """Get quaternion from roll, pitch, yaw.

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
    else:
        return None


def tf_baselink_to_aircraft(orientation_q):
    """Transforms from baselink to aircraft frame.

    Args:
        orientation_q (pyquaternion.Quaternion): Orientation in baselink frame.

    Returns:
        pyquaternion.Quaternion: Orientation in aircraft frame.
    """
    return _transform_orientation("body_frames", orientation_q)


def tf_aircraft_to_baselink(orientation_q):
    """Transforms from aircraft to baselink frame.

    Args:
        orientation_q (pyquaternion.Quaternion): Orientation in aircraft frame.

    Returns:
        pyquaternion.Quaternion: Orientation in baselink frame.
    """
    return _transform_orientation("body_frames", orientation_q)


def tf_ned_to_enu(orientation_q):
    """Transforms from NED to ENU as reference frame for the orientation.

    Args:
        orientation_q (pyquaternion.Quaternion): Orientation with NED as
            reference frame.

    Returns:
        pyquaternion.Quaternion: Orientation with ENU as reference frame.
    """
    return _transform_orientation("global_frames", orientation_q)


def tf_enu_to_ned(orientation_q):
    """Transform from ENU to NED as reference frame for the orientation.

    Args:
        orientation_q (pyquaternion.Quaternion): Orientation with ENU as
            reference frame.

    Returns:
        pyquaternion.Quaternion: Orientation with NED as reference frame.
    """
    return _transform_orientation("global_frames", orientation_q)


def orientation_gazebo_to_px4(orientation_q):
    """ Transformation from gazebo frame to px4 frame.
    
    Twist of the baselink to the static ENU frame <-> Twist of the airframe to
    the static NED frame
    
    Args:
        orientation_q (pyquaternion.Quaternion): Orientation of
            the baselink in respect to the ENU frame.
    
    Returns:
        pyquaternion.Quaternion: Orientation of the airframe in respect to the
            NED frame.
    """
    return tf_enu_to_ned(tf_baselink_to_aircraft(orientation_q))


def orientation_px4_to_gazebo(orientation_q):
    """ Transformatio from px4 frame to gazebo frame.
    
    Twist of the airframe to the static NED frame <-> Twist of the baselink to
    the static ENU frame.
    
    Args:
        orientation_q (pyquaternion.Quaternion): Orientation of the
            airframe in respect to the NED frame.
    
    Returns:
        pyquaternion.Quaternion: Orientation of the baselink in respect to the
            ENU frame.
    """
    return tf_aircraft_to_baselink(tf_ned_to_enu(orientation_q))


def position_gazebo_to_px4(position_x, position_y, position_z):
    """Transforms position from gazebo's ENU frame to px4's NED frame.

    Args:
        position_x (float): x-coordinate in ENU frame.
        position_y (float): y-coordinate in ENU frame.
        position_z (float): z-coordinate in ENU frame.

    Returns:
        tuple: Tuple of the transformed position.
    """
    return (position_y, position_x, -position_z)


def position_px4_to_gazebo(position_x, position_y, position_z):
    """Transforms position from px4's NED frame to gazebo's ENU frame.

    Args:
        position_x (float): x-coordinate in NED frame.
        position_y (float): y-coordinate in NED frame.
        position_z (float): z-coordinate in NED frame.

    Returns:
        tuple: Tuple of the transformed position.
    """
    return (position_y, position_x, -position_z)


if __name__ == "__main__":
    pass
