#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class PublisherNode(Node):
    """
    A ROS2 node that publishes messages to a 'greeting' topic at regular intervals.
    
    Attributes:
        publisher (rclpy.publisher.Publisher): Publisher for String messages
        timer (rclpy.timer.Timer): Timer for periodic publishing
    """
    
    def __init__(self):
        """
        Initialize the publisher node, create publisher and timer.
        """
        super().__init__('publisher_node')
        
        # Quality of Service profile for reliable communication
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )
        
        # Create publisher with String message type and custom QoS
        self.publisher = self.create_publisher(
            String,
            'greeting',
            qos_profile
        )
        
        # Create timer for periodic publishing (0.5 seconds)
        self.timer = self.create_timer(0.5, self.publish_message)
        
        self.get_logger().info("Publisher node initialized and started")
    
    def publish_message(self):
        """
        Callback function for timer. Creates and publishes a message.
        """
        msg = String()
        msg.data = "Hello ROS2 - from Python Publisher"
        self.publisher.publish(msg)
        self.get_logger().debug(f'Published: "{msg.data}"')

def main(args=None):
    """
    Main function to initialize and run the publisher node.
    """
    rclpy.init(args=args)
    
    try:
        publisher_node = PublisherNode()
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        publisher_node.get_logger().info("Keyboard interrupt received, shutting down")
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()