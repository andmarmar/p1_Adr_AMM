from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'square_number', self.numero_cuadrado)

    def numero_cuadrado(self, request, response):

        number = request.a
        response.sum = number * number
        self.get_logger().info(f'Número recibido: {number}, cuadrado: {response.sum}')
        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
