import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus

class OffboardControlNode(Node):
    def _init_(self):

        super()._init_('offboard_control_node')

        # Configurar QoS
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        )

        # Suscripción y publicaciones
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos)

        self.pub_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)


        # Parámetros o posición objetivo fija (en NED: norte, este, abajo)
        self.target_x = 10.0   # metros hacia el norte
        self.target_y = 5.0    # metros hacia el este
        self.target_z = -5.0   # metros en NED (z negativo = altitud positiva, e.g. 5 m arriba)
        
        # Contador de ciclos para delay inicial (se envía OffboardControlMode antes de armar)
        self.offboard_counter = 0
        self.nav_state = None
        self.arming_state = None

        # Timer periódica (10 Hz) para enviar mensajes de control
        timer_period = 0.1  # segundos
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def status_callback(self, msg: VehicleStatus):
        # Almacena estado de navegación y armado
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def timer_callback(self):
        # Publicar OffboardControlMode en cada ciclo
        offb = OffboardControlMode()
        offb.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offb.position = True       # habilitar control por posición
        offb.velocity = False
        offb.acceleration = False
        offb.attitude = False
        offb.body_rate = False
        offb.thrust_and_torque = False
        offb.direct_actuator = False
        self.pub_offboard_mode.publish(offb)

        # Después de ~10 ciclos, solicitar modo offboard y armado
        if self.offboard_counter == 10:
            self.engage_offboard_mode()
            self.arm_vehicle()

        # Si estamos en OFFBOARD y armado, enviar setpoint de posición objetivo
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            traj = TrajectorySetpoint()
            traj.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            traj.position = [self.target_x, self.target_y, self.target_z]
            traj.yaw = 0.0   # ángulo yaw deseado (rad)
            self.pub_trajectory.publish(traj)

        # Incrementar contador hasta 11 para solo una vez el cambio de modo
        if self.offboard_counter < 11:
            self.offboard_counter += 1

    def engage_offboard_mode(self):
        # Comando para cambiar a modo OFFBOARD
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0   # 1 = modo personalizado
        cmd.param2 = 6.0   # 6 = modo offboard (px4)
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_cmd.publish(cmd)
        self.get_logger().info('Comando: cambiar a modo OFFBOARD')

    def arm_vehicle(self):
        # Comando para armar (VEHICLE_CMD_COMPONENT_ARM_DISARM)
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0   # 1 = armar, 0 = desarmar
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_cmd.publish(cmd)
        self.get_logger().info('Comando: ARME enviado')

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()