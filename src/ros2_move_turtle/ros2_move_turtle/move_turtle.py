import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class MoveTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.get_logger().info("MoveTurtle node started")

    def pose_callback(self, msg):
        cmd = Twist()
        cmd.linear.x = 2.0  # always move forward
        cmd.angular.z = 0.0

        # If x or y position > 7 → turn in place
        if msg.x > 7.0 or msg.y > 7.0:
            cmd.linear.x = 0.0      # stop moving forward
            cmd.angular.z = 2.0     # rotate
            self.get_logger().info(f"Turtle reacting at limit: x={msg.x:.2f}, y={msg.y:.2f}")

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
