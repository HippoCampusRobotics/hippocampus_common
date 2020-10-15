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

    @staticmethod
    def get_param(name, default=None, verbose=True, limit=80 * 5):
        """Get a parameter from the ROS parameter server.

        Logs a warning if parameter does not exist sets the default value
        afterwards.

        Args:
            name (str): Name of the parameter.
            default : Default value to be used if parameter does not exist.
            verbose: Log parameter values
            limit: Limit the length of the printed parameter value to specified
                   number of characters if > 0.
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
            if default is None:
                rospy.logwarn("[{}] No default value given for parameter '{}', unexpected behaviour possible.".format(
                    rospy.get_name(), name))
        finally:
            if verbose:
                param_string = "{}".format(param)
                if limit > 0 and len(param_string) > limit:
                    param_string = param_string[:limit] + "..."
                rospy.loginfo("[{}] {}={}".format(rospy.get_name(), name,
                                                  param_string))
        return param

    @staticmethod
    def set_param(name, value, verbose=True, limit=80 * 5):
        """Set parameter verbosly.

        Args:
            name (str): Name of the parameter.
            value : Value of the parameter that is set.
            verbose: Log parameter values.
            limit: Limit the length of the printed parameter value to specified
                   number of characters if > 0.
        """
        rospy.set_param(name, value)
        value_string = "{}".format(value)
        if limit > 0 and len(value_string) > limit:
            value_string = value_string[:limit] + "..."
        rospy.loginfo("[{}] {}={}".format(rospy.get_name(), name, value_string))
