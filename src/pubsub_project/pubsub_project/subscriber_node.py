#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    """
    A ROS2 node that subscribes to messages from the 'greeting' topic.
    
    Attributes:
        subscription (rclpy.subscription.Subscription): Subscription to String messages
    """
    
    def __init__(self):
        """
        Initialize the subscriber node and create subscription.
        """
        super().__init__('subscriber_node')
        
        # Quality of Service profile matching the publisher
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )
        
        # Create subscription with callback
        self.subscription = self.create_subscription(
            String,
            'greeting',
            self.listener_callback,
            qos_profile
        )
        
        self.get_logger().info("Subscriber node initialized and listening")
    
    def listener_callback(self, msg):
        """
        Callback function for received messages.
        
        Args:
            msg (std_msgs.msg.String): The received message
        """
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    """
    Main function to initialize and run the subscriber node.
    """
    rclpy.init(args=args)
    
    try:
        subscriber_node = SubscriberNode()
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        subscriber_node.get_logger().info("Keyboard interrupt received, shutting down")
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()