import sys
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'square_number')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio no disponible, esperando...')
        self.req = AddTwoInts.Request()

    def send_request(self, number):
        self.req.a = number 
        self.req.b = 0  
        return self.cli.call_async(self.req)


def main():
    rclpy.init()


    if len(sys.argv) < 2:
        print("Error de comando, introduce: ros2 run pr1 minimal_client_async <número>")
        sys.exit(1)

    number = int(sys.argv[1])

    minimal_client = MinimalClientAsync()

    # Enviamos el número ingresado al servicio
    future = minimal_client.send_request(number)
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()

    # Mostramos el cuadrado recibido
    minimal_client.get_logger().info(
        f'Resultado del cuadrado de {number}: {response.sum}'
    )

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
