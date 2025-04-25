import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.clock import Clock

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleCommand

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
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.subscription = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_position_callback, qos_profile)

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)

        self.current_x = None
        self.current_y = None
        self.current_z = None

        self.current_index = 0
        self.goal_reached = False
        self.height = -1.5  # Altura fija

        self.armed = False
        self.offboard_started = False

        self.takeoff_done = False
        self.takeoff_altitude = -1.5  # Altura inicial de despegue

        # Trayectoria a seguir después del despegue
        self.trajectory = self.generate_trajectory()

    def generate_trajectory(self):
        # Cuadrado simple de 1m
        side_length = 1.0
        return [
            (0.0, 0.0),
            (side_length, 0.0),
            (side_length, side_length),
            (0.0, side_length),
            (0.0, 0.0)
        ]

    def vehicle_position_callback(self, msg):
        self.current_x = float(msg.x)
        self.current_y = float(msg.y)
        self.current_z = float(msg.z)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.publisher_vehicle_command.publish(msg)

    def cmdloop_callback(self):
        if self.current_x is None:
            return

        # 1. Armar
        if not self.armed:
            self.get_logger().info('Armando vehículo...')
            self.send_vehicle_command(400, 1.0)  # VEHICLE_CMD_COMPONENT_ARM_DISARM = 400, param1=1.0 para armar
            self.armed = True
            return

        # 2. Activar OFFBOARD
        if not self.offboard_started:
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = True
            self.publisher_offboard_mode.publish(offboard_msg)

            # Enviar un TrajectorySetpoint para que PX4 acepte OFFBOARD
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            trajectory_msg.x = self.current_x
            trajectory_msg.y = self.current_y
            trajectory_msg.z = self.takeoff_altitude  # subir a altura segura
            self.publisher_trajectory.publish(trajectory_msg)

            self.offboard_started = True
            self.get_logger().info('Modo OFFBOARD activado.')
            return

        # 3. Subir a la altura de despegue
        if not self.takeoff_done:
            if abs(self.current_z - self.takeoff_altitude) < 0.2:
                self.takeoff_done = True
                self.get_logger().info('Despegue completado.')
            else:
                trajectory_msg = TrajectorySetpoint()
                trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
                trajectory_msg.x = self.current_x
                trajectory_msg.y = self.current_y
                trajectory_msg.z = self.takeoff_altitude
                self.publisher_trajectory.publish(trajectory_msg)
            return

        # 4. Ya despegó, seguir la trayectoria
        if self.goal_reached:
            return

        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        self.publisher_offboard_mode.publish(offboard_msg)

        target_x, target_y = self.trajectory[self.current_index]
        target_z = self.height

        distance = ((self.current_x - target_x) ** 2 + (self.current_y - target_y) ** 2) ** 0.5
        if distance < 0.2:
            self.current_index += 1
            self.get_logger().info(f"Llegó al punto ({self.current_index}/{len(self.trajectory)})")
            if self.current_index >= len(self.trajectory):
                self.goal_reached = True
                self.get_logger().info('Ruta completada!')
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
