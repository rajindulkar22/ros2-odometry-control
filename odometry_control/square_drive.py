import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class SqaureDrive(Node):
    def __init__(self):
        super().__init__("sqaure_drive_node")
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

        self.linear_velocity = 0.2
        self.angular_velocity = 0.2
        self.side_length = 2.0

        self.drive_duration = self.side_length / self.linear_velocity #time to drive one side
        self.turn_duration = (math.pi /2) / self.angular_velocity #time to turn 90 degrees

# 0=Forward, 1=StopForTurn, 2=Turn, 3=StopForNext
        self.state = 0
        self.start_time = self.get_clock().now().nanoseconds #this is the time when the state started
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        msg = Twist()
        current_time = self.get_clock().now().nanoseconds
        elapsed_time = (current_time - self.start_time) / 1e9

        if self.state == 0: #drive forward
            if elapsed_time < self.drive_duration:
                self.get_logger().info("Driving forward")
                msg.linear.x = self.linear_velocity
            else:
                self.get_logger().info("stop for turn")
                msg.linear.x = 0.0
                self.switch_state(1)

        elif self.state == 1: #stop for turn
            if elapsed_time < 1.0:
                self.get_logger().info("Stopping for turn")
                pass
            else:
                self.switch_state(2)

        elif self.state == 2: #turn
            if elapsed_time < self.turn_duration:
                self.get_logger().info("turning")
                msg.angular.z = self.angular_velocity
            else:
                self.get_logger().info("stop for next")
                msg.angular.z = 0.0
                self.switch_state(3)

        elif self.state == 3: #stop for next
            if elapsed_time < 1.0:
                self.get_logger().info("Stopping for next")
                pass
            else:
                self.switch_state(0)
        self.publisher.publish(msg)


    def switch_state(self, new_state):
        self.state = new_state
        self.start_time = self.get_clock().now().nanoseconds
        self.get_logger().info(f"Switching to state {self.state}")

def main():
    rclpy.init()
    square_drive_node = SqaureDrive()
    rclpy.spin(square_drive_node)
    square_drive_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()





        

