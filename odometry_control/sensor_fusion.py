import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
from geometry_msgs.msg import Quaternion


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__("sensor_fusion_node")
        self.odom_subscription = self.create_subscription(Odometry, "odom", self.odom_callback,10)
        self.imu_subscription = self.create_subscription(Imu,'imu', self.imu_callback,10)
        self.publisher = self.create_publisher(Odometry, "/odom_filtered",10)
        self.current_velocity = 0.0 #stores the lastest odom speed from wheel odometry
        self.current_yaw_rate = 0.0 #stores the latest imu yaw rate from imu
        self.x = 0.0 #stores the latest x position
        self.y = 0.0 #stores the latest y position
        self.theta = 0.0 #stores the latest orientation
        self.last_time = self.get_clock().now()
        self.create_timer(0.033,self.timer_callback)
        self.get_logger().info("Sensor fusion node started")


    def odom_callback(self,msg):
        self.current_velocity = msg.twist.twist.linear.x
    def imu_callback(self, msg):
        self.current_yaw_rate = msg.angular_velocity.z
    def timer_callback(self):
        dt = (self.get_clock().now() - self.last_time).nanoseconds / 1e9
        self.last_time = self.get_clock().now()
        self.x += (self.current_velocity * math.cos(self.theta) * dt) #position x update
        self.y += (self.current_velocity * math.sin(self.theta) * dt) #position y update
        self.theta += (self.current_yaw_rate * dt) #orientation update

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = self.euler_to_quaternion(self.theta)
        self.publisher.publish(msg)

    def euler_to_quaternion(self,yaw):
        q=Quaternion()
        q.w = math.cos(yaw/2)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw/2)
        return q


def main():
    rclpy.init()
    sensor_fusion_node = SensorFusionNode()
    rclpy.spin(sensor_fusion_node)
    sensor_fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
