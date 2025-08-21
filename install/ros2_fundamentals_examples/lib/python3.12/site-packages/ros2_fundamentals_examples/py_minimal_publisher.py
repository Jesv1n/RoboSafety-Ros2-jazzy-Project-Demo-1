#! /usr/bin/env python3
"""
Description:
    This ROS 2 node periodically publishes "Hello World" message to a topic.

-------
Publishing Topics:
      The channel containing the "hello world" messages
      /py_example_topic - std_msgs/String

Subscription Topics:
    None
-------
Author: Jesvin Paul
Date: August 14, 2025
"""
import rclpy  # import the ROS 2 client library for Python
from rclpy.node import Node  # Import the Node class used for creating nodes

from std_msgs.msg import String  # import String message type for ROS 2

timer_period = 0.5  # seconds

class MinimalPyPublisher(Node):
    """
    Create a minimal publisher node
    """

    def __init__(self):
        """
        Create a custom node class for publishing messages
        """
        # initialize the node with a name
        super().__init__('minimal_py_publisher')

        # create a publisher on the topic with a queue size of 10 messages
        self.publisher_1 = self.create_publisher(String, '/py_example_topic', 10)

        # create a timer with a period of 0.5 seconds to trigger publishing of message
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # initialize a counter variable for message content
        self.i = 0

    def timer_callback(self):
        """
        Callback function executed periodically by the timer
        """
        # create a new string message object
        msg = String()

        # set the message data with a counter
        msg.data = 'Hello World: %d' % self.i

        # publish the message you created above to a topic
        self.publisher_1.publish(msg)

        # log a message indicating the message has been published
        self.get_logger().info('Publishing: "%s"' % msg.data)

        self.i = self.i + 1

def main(args=None):
    """
    Main function to start the ROS 2 node

    ARGS:
        args (List, optional): command-line arguments. Default to None
    """
    rclpy.init(args=args)

    # create an instance of the minimal publisher node
    minimal_py_publisher = MinimalPyPublisher()

    rclpy.spin(minimal_py_publisher)
    
    # destroy the node explicitly instead of Ctrl-C
    minimal_py_publisher.destroy_node()

    # shutdown ROS 2 communication
    rclpy.shutdown()

if __name__ == '__main__':
    # execute the main function if the script is run directly
    main()
