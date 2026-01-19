import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__("sensor_fusion_node")
        self.odom_subscription = self.create_subscription(Odometry, "odom", self.odom_callback,10)
        self.imu_subscription = self.create_subscription(Imu,'imu', self.imu_callback,10)
        self.publisher = self.create_publisher(Odometry, "/odom_filtered",10)
        self.kalman_x = 0.0
        self.kalman_y = 0.0
        self.kalman_theta = 0.0
        self.last_odom_time = self.get_clock().now()
        self.get_logger().info("Sensor fusion node started")

    def odom_callback(self, msg):
        self.kalman_x = msg.pose.pose.position.x
        self.get_logger().info(f"kalman x: {self.kalman_x}")
    def imu_callback(self, msg):
        self.kalman_theta = msg.angular_velocity.z
        self.get_logger().info(f"kalman theta: {self.kalman_theta}")

def main():
    rclpy.init()
    sensor_fusion_node = SensorFusionNode()
    rclpy.spin(sensor_fusion_node)
    sensor_fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
