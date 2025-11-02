import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class MoveTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle')

        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.has_stopped = False  # ✅ Flag to avoid repeated logging

        self.get_logger().info("✅ /move_turtle node started")

    def pose_callback(self, pose: Pose):
        cmd = Twist()

        # Stop condition
        if pose.x > 7.0 or pose.y > 7.0:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            # ✅ Log only the first time we reach the stopping condition
            if not self.has_stopped:
                self.get_logger().info(f"🛑 Turtle stopped at x={pose.x:.2f}, y={pose.y:.2f}")
                self.has_stopped = True
        else:
            cmd.linear.x = 2.0
            cmd.angular.z = 0.0

        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
