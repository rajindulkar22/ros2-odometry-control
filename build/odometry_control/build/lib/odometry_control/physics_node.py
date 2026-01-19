import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from odometry_control.diff_drive import DiffDriveRobot
import math
import time
import random



class PhysicsNode(Node):
    def __init__(self):
        super().__init__("physics_node")
        
        self.robot = DiffDriveRobot(wheel_radius = 0.05, wheel_base = 0.2) #we initialize the robot with wheel radius and wheel base
        #current position 
        ## We need to remember where we are. Start at (0,0) with 0 rotation.
        self.x = 0.0
        self.y =0.0
        self.theta =0.0

        #current speed
        # We need to remember the current speed
        self.current_velocity = 0.0
        self.current_angular_velocity = 0.0

        #track the wheel rotation in radians #encoder position
        self.left_wheel_position=0.0
        self.right_wheel_position=0.0

        #publish the wheel position
        self.publisher = self.create_publisher(JointState, "joint_states",10)
        self.subscription = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.new_publisher = self.create_publisher(Odometry, "ground_truth", 10)
        self.imu_publisher = self.create_publisher(Imu, "imu", 10)
        
        # We want to update the position 30 times a second (30 Hz).
        # Calculate the period (1/30).
        self.dt = 1.0/30.0 # period
        self.timer = self.create_timer(self.dt, self.update_physics)

        self.get_logger().info("Physics node started")


    def cmd_vel_callback(self,msg):
       self.current_velocity = msg.linear.x
       self.current_angular_velocity = msg.angular.z
       self.get_logger().info(f"Received velocity command: {self.current_velocity}, {self.current_angular_velocity}")


    def update_physics(self):
       

        #update the position
        delta_theta = (self.current_angular_velocity * self.dt)
        delta_x = self.current_velocity * math.cos(self.theta) * self.dt
        delta_y = self.current_velocity * math.sin(self.theta) * self.dt
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        self.get_logger().info(f"Updated position: x={self.x}, y={self.y}, theta={self.theta}")

#publish the truth odometry what we need to know
        truth_msg = Odometry()
        truth_msg.header.stamp = self.get_clock().now().to_msg()
        truth_msg.header.frame_id = "odom"
        truth_msg.child_frame_id = "base_link_truth"
        truth_msg.pose.pose.position.x = self.x
        truth_msg.pose.pose.position.y = self.y
        truth_msg.pose.pose.orientation = self.euler_to_quaternion(self.theta)
        self.new_publisher.publish(truth_msg)

        #to get speed of the wheels
        wl_speed, wr_speed = self.robot.inverse_kinematics(self.current_velocity, self.current_angular_velocity)
        #we add random noise
        noisy_wl_speed = wl_speed + random.gauss(0.0, 0.05)
        noisy_wr_speed = wr_speed + random.gauss(0.0, 0.05)
        self.left_wheel_position += noisy_wl_speed * self.dt
        self.right_wheel_position += noisy_wr_speed * self.dt

        #publish the joint state message
        msg= JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name=["left_wheel_joint","right_wheel_joint"]
        msg.position=[self.left_wheel_position,self.right_wheel_position]
        msg.velocity=[noisy_wl_speed,noisy_wr_speed]
        self.publisher.publish(msg)

#measures change in speed (angular velocity or acceleration)
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link"
        # We only target the Z-axis (Yaw rate).
        imu_msg.angular_velocity.z = self.current_angular_velocity + random.gauss(0.0, 0.01)
        self.imu_publisher.publish(imu_msg)

        

    def euler_to_quaternion(self,yaw):
        q=Quaternion()
        q.w = math.cos(yaw/2)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw/2)
        return q


def main():
    rclpy.init()
    physics_node = PhysicsNode()
    rclpy.spin(physics_node)
    physics_node.destroy_node()
    rclpy.shutdown()

    
if __name__ == "__main__":
    main()
