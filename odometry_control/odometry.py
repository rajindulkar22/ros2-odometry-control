import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from odometry_control.diff_drive import DiffDriveRobot
from tf2_ros import TransformBroadcaster
import math
import time


class OdometryNode(Node):
    def __init__(self):
        super().__init__("odometry_node")

        self.robot = DiffDriveRobot(wheel_radius = 0.05, wheel_base = 0.2)
        self.tf_broadcaster = TransformBroadcaster(self)

        #current position
        self.x =0.0
        self.y =0.0
        self.theta =0.0

        #previous wheel position
        self.left_wheel_position_old =0.0
        self.right_wheel_position_old =0.0
        self.first_run = True


        self.publisher = self.create_publisher(Odometry,"odom",10)
        self.subscription = self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)
        self.get_logger().info("Odometry node started")

    def joint_state_callback(self, msg):
        #Get current wheel positions from the message
        current_left_position = msg.position[0]
        current_right_position = msg.position[1]

        #for the first run, we need to remember the previous wheel position
        if self.first_run:
            self.left_wheel_position_old = current_left_position
            self.right_wheel_position_old = current_right_position
            self.first_run = False
            return

        #calculate the change which is delta
        delta_left_position = current_left_position - self.left_wheel_position_old
        delta_right_position = current_right_position - self.right_wheel_position_old

        #Use self.robot.forward_kinematics to get dist and angle change
        # NEW (Swap them!)
        d_linear, delta_theta = self.robot.forward_kinematics(delta_right_position, delta_left_position)

        #update the position
        self.theta += delta_theta
        self.x += d_linear * math.cos(self.theta)
        self.y += d_linear * math.sin(self.theta)

        self.left_wheel_position_old = current_left_position
        self.right_wheel_position_old = current_right_position

        self.publish_odometry()

    def publish_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y =self.y
        msg.pose.pose.orientation = self.euler_to_quaternion(self.theta)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id ="odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = self.euler_to_quaternion(self.theta)
        self.tf_broadcaster.sendTransform(t)
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
    odometry_node = OdometryNode()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
       


