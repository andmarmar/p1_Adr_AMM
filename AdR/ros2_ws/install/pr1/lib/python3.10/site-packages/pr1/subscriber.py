import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class NavSatFixSubscriber(Node):
    def __init__(self):
        super().__init__('navsatfix_subscriber')
        self.subscription = self.create_subscription(
            NavSatFix,
            'gps_topic',
            self.callback,
            10
        )
        self.subscription  # Evitar advertencia por variable no utilizada

    def callback(self, msg):
        # Imprimir la información del mensaje recibido
        self.get_logger().info(f'Recibido:')
        self.get_logger().info(f'Latitud: {msg.latitude}')
        self.get_logger().info(f'Longitud: {msg.longitude}')
        self.get_logger().info(f'Altitud: {msg.altitude}')

def main(args=None):
    rclpy.init(args=args)
    node = NavSatFixSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
