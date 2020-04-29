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
