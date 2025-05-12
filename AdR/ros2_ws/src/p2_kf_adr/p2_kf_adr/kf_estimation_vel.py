import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

import numpy as np

from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, generate_noisy_measurement_2
from .filters.kalman_filter import KalmanFilter_2
from .visualization import Visualizer

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
#from kalman_filter import KalmanFilter_2

class KalmanFilterPureNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_pure_node')

        initial_state = np.zeros(6)
        initial_covariance = np.eye(6) * 0.1

        self.kf = KalmanFilter_2(initial_state, initial_covariance)
        self.visualizer = Visualizer()
        self.initial_pose = None
        self.first_prediction_done = False
        self.prev_time = None  
        self.u = np.zeros(2) 


        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf2_estimate',
            10
        )

    def odom_callback(self, msg):
        
        # Extrae datos del mensaje Odometry
        pose = odom_to_pose2D(msg) 
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        vx = linear.x
        vy = linear.y if hasattr(linear, 'y') else 0.0
        omega = angular.z
        
        # Inicializa el estado y guarda el tiempo actual
        if self.initial_pose is None:
            self.initial_pose = pose
            self.kf.mu = np.array(list(self.initial_pose) + [0, 0, 0])
            self.prev_time = self.get_clock().now().nanoseconds  
            return
        
        # Posicion relativa 
        current_pose = pose
        self.normalized_pose = np.array(get_normalized_pose2D(self.initial_pose, current_pose))

        # Actualiza el control y tiempo
        self.u = np.array([vx, omega])

        curr_time = self.get_clock().now().nanoseconds
        if self.prev_time is not None:
            dt = (curr_time - self.prev_time) / 1e9
        else:
            dt = 0.0
        self.prev_time = curr_time  
        # Prediccion del filtro de Kalman
        mu, sigma = self.kf.predict(self.u, dt)
        self.first_prediction_done = True

        # Ruido y actualizacion
        vx, vy, omega = self.kf.mu      
        z = generate_noisy_measurement_2(current_pose, vx, vy, omega)

        self.get_logger().info(f"Drifted Pose: {z}")
        mu, sigma = self.kf.update(z)

        self.visualizer.update(self.normalized_pose, mu, sigma, step="update")
        self.publish_estimated_pose(mu, sigma)
        self.publish_real_pose(self.normalized_pose)
        self.get_logger().info(f"Updated state: {mu}")
        self.get_logger().info(f"Measured Pose: {self.normalized_pose}")

    def publish_estimated_pose(self, mu, sigma):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = mu[0]
        msg.pose.pose.position.y = mu[1]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = np.sin(mu[2] / 2.0)
        msg.pose.pose.orientation.w = np.cos(mu[2] / 2.0)
        
        for i in range(3):
            for j in range(3):
                msg.pose.covariance[i * 6 + j] = sigma[i, j]
        self.publisher.publish(msg)

    def publish_real_pose(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = np.sin(pose[2] / 2.0)
        msg.pose.pose.orientation.w = np.cos(pose[2] / 2.0)
        self.publisher.publish(msg)    

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterPureNode()
    rclpy.spin(node)
    rclpy.shutdown()

