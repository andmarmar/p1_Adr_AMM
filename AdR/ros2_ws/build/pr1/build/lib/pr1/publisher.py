import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

class NavSatFixPublisher(Node):
    def __init__(self):
        super().__init__('navsatfix_publisher')
        self.publisher = self.create_publisher(NavSatFix, 'gps_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        msg = NavSatFix()

        # Header info
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_frame'

        # NavSatFix message data
        msg.status.status = 0  
        msg.status.service = 1  
        msg.latitude = 45 
        msg.longitude = -130.5  
        msg.altitude = 88.0  

        self.publisher.publish(msg)
        self.get_logger().info(f'Publicando: {msg.latitude}, {msg.longitude}, {msg.altitude}')

def main(args=None):
    rclpy.init(args=args)
    node = NavSatFixPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
