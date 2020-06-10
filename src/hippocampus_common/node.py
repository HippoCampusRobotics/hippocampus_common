"""This module provides a very basic Node class template
"""
import rospy


class Node(object):
    """A basic node class to start off with, when implementing a ROS node.
    """
    def __init__(self, name):
        """Initializes a ROS node by calling `rospy.init_node`.

        Args:
            name (str): Name of the node.
        """
        rospy.init_node(name)
        rospy.loginfo("[{}] Initialized.".format(rospy.get_name()))

    def run(self):
        """Enters a loop until ROS is shut down to keep the program from exiting
        prematurely.
        """
        while not rospy.is_shutdown():
            rospy.spin()
        rospy.loginfo("[{}] Shutting down...".format(rospy.get_name()))

    def _get_param(self, name, default):
        """Get a parameter from the ROS parameter server.

        Logs a warning if parameter does not exist sets the default value
        afterwards.

        Args:
            name (str): Name of the parameter.
            default : Default value to be used if parameter does not exist.
        Returns:
            [XmlRpcLegalType]: Either the read parameter or the default value if
                it does not exist.
        """
        try:
            param = rospy.get_param(name)
        except KeyError:
            rospy.set_param(name, default)
            param = default
            rospy.logwarn("[{}] Parameter '{}' does not exist.".format(
                rospy.get_name(), name))
        finally:
            rospy.loginfo("[{}] {}={}".format(rospy.get_name(), name, param))
        return param
