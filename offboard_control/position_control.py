import rclpy
import rclpy.node
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.clock import Clock

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition

class OffboardControlNode(Node):
    def __init__(self):
        super().__init__('offboard_control')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.subscription = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_position_callback, qos_profile)

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)

        self.current_x = None
        self.current_y = None
        self.current_z = None

        self.current_index = 0
        self.goal_reached = False
        self.height = -1.5  # Altura fija

        self.trajectory = [(1.0, 0.0)]

    def vehicle_position_callback(self, msg):
        self.current_x = float(msg.x)
        self.current_y = float(msg.y)
        self.current_z = float(msg.z)

    def cmdloop_callback(self):
        if self.goal_reached or self.current_x is None:
            return

        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        self.publisher_offboard_mode.publish(offboard_msg)

        target_x, target_y = self.trajectory[self.current_index]
        target_z = float(self.height)

        # Verifica si se llegó al punto
        distance = ((self.current_x - target_x)**2 + (self.current_y - target_y)**2)**0.5
        if distance < 0.2:
            self.current_index += 1
            self.get_logger().info(f"Llegó al punto ({self.current_index}/{len(self.trajectory)})")
            if self.current_index >= len(self.trajectory):
                self.goal_reached = True
                self.get_logger().info("Ruta completada!")
                return

            target_x, target_y = self.trajectory[self.current_index]

        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        trajectory_msg.x = target_x
        trajectory_msg.y = target_y
        trajectory_msg.z = target_z
        self.publisher_trajectory.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
