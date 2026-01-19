import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion


class GoalController(Node):
    def __init__(self):
        super().__init__("goal_controller")
        self.publisher = self.create_publisher(Twist, "cmd_vel",10)
        self.subscription = self.create_subscription(Odometry, "odom", self.odom_callback, 10)

        #goal position
        self.goal_x = 2.0
        self.goal_y = 2.0

        #current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        #PID parameters
        self.kp_linear = 0.5
        self.kp_angular = 0.5
        

        #control loop 
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Goal controller started")


    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (roll, pitch, self.current_theta) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def control_loop(self):
        msg = Twist()
        self.distance_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
        self.goal_angle = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        angular_error = self.goal_angle - self.current_theta

        while angular_error > math.pi:
            angular_error -= 2 * math.pi
        while angular_error < -math.pi:
            angular_error += 2 * math.pi

        if self.distance_to_goal < 0.1:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            self.get_logger().info("Goal reached")
            return

        if abs(angular_error) > 0.5: # 0.5 radians is approx 30 degrees
            msg.linear.x = 0.0
            msg.angular.z = self.kp_angular * angular_error

        else:
            msg.linear.x = self.kp_linear * self.distance_to_goal
            msg.angular.z = self.kp_angular * angular_error
        
        self.publisher.publish(msg)

def main():
    rclpy.init()
    goal_controller = GoalController()
    rclpy.spin(goal_controller)
    goal_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

    



