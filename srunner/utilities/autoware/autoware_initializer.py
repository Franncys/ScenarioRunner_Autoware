import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from autoware_auto_vehicle_msgs.msg import Engage

class AutowareInitializer(Node):
    def __init__(self):
        super().__init__('autoware_initializer')
        self.gnss_received = False

        # Subscriber for GNSS pose
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/sensing/gnss/pose_with_covariance',
            self.gnss_callback,
            10
        )

        # Publisher for initial pose
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose3d',
            10
        )

        # Publisher for target goal
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/planning/mission_planning/goal',
            10
        )

        # Publisher for engage command
        self.engage_pub = self.create_publisher(
            Engage,
            '/autoware/engage',
            10
        )

        # Publisher for sensor logging control
        self.sensor_logging_pub = self.create_publisher(
            bool,
            '/sensor_logging_control',
            10
        )

        # Timer handles
        self._target_timer = None
        self._engage_timer = None
        self._shutdown_timer = None

    def gnss_callback(self, msg: PoseWithCovarianceStamped):
        if not self.gnss_received:
            self.gnss_received = True
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initialpose_pub.publish(msg)
            self.get_logger().info("Published GNSS pose to /initialpose3d")
            self._target_timer = self.create_timer(1.0, self.publish_target_goal)

    def publish_target_goal(self):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = -41.679256439208984
        goal_msg.pose.position.y = 39.53258514404297
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.7035624909446608
        goal_msg.pose.orientation.w = 0.7106333944698519

        self.goal_pub.publish(goal_msg)
        self.get_logger().info("Published target goal to /planning/mission_planning/goal")
        
        if self._target_timer is not None:
            self._target_timer.cancel()

        self._engage_timer = self.create_timer(1.0, self.publish_engage)
        

    def publish_engage(self):
        engage_msg = Engage()
        engage_msg.engage = True
        self.engage_pub.publish(engage_msg)
        self.get_logger().info("Published engage command to /autoware/engage")
        
        # Publish the sensor logging control message
        logging_msg = bool()
        logging_msg.data = True
        self.sensor_logging_pub.publish(logging_msg)
        self.get_logger().info("Published sensor logging control message to /sensor_logging_control")
        
        if self._engage_timer is not None:
            self._engage_timer.cancel()
        
        # Set a shutdown timer to finish the process after a brief delay.
        self._shutdown_timer = self.create_timer(0.5, self.shutdown_node)
        self.shutdown_node()

    def shutdown_node(self):
        self.get_logger().info("Sequence complete, shutting down node.")
        if self._shutdown_timer is not None:
            self._shutdown_timer.cancel()
        rclpy.shutdown()  # This stops the spinning, but the process may still linger.
        sys.exit(0)       # Explicitly exit the process.

def main(args=None):
    rclpy.init(args=args)
    node = AutowareInitializer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

if __name__ == '__main__':
    main()