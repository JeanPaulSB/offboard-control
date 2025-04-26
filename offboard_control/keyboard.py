import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

class OffboardControl(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control')
        
        # Configurar perfil QoS para publicaciones
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Crear publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
       
        # Lista de puntos (x, y) especificados, z fijo en -5.0
        self.points = [
        (0.0, 0.0, -5), 
        (-1.0, -1.0, -5), 
        (-2.0, -2.0, -5), 
        (-3.0, -3.0, -5), 
        (-4.0, -4.0, -5), 
        (-5.0, -5.0, -5), 
        (-5.0, -6.0, -5), 
        (-5.0, -7.0, -5), 
        (-5.0, -8.0, -5), 
        (-5.0, -9.0, -5), 
        (-5.0, -10.0, -5), 
        (-5.0, -11.0, -5), 
        (-6.0, -12.0, -5), 
        (-7.0, -13.0, -5), 
        (-8.0, -14.0, -5), 
        (-8.0, -15.0, -5), 
        (-8.0, -16.0, -5), 
        (-8.0, -17.0, -5), 
        (-8.0, -18.0, -5), 
        (-8.0, -19.0, -5), 
        (-8.0, -20.0, -5), 
        (-9.0, -21.0, -5), 
        (-10.0, -21.0, -5), 
        (-11.0, -21.0, -5), 
        (-12.0, -21.0, -5), 
        (-13.0, -21.0, -5), 
        (-14.0, -21.0, -5), 
        (-15.0, -21.0, -5), 
        (-16.0, -21.0, -5), 
        (-17.0, -21.0, -5), 
        (-18.0, -21.0, -5), 
        (-19.0, -21.0, -5), 
        (-20.0, -21.0, -5), 
        (-21.0, -21.0, -5), 
        (-22.0, -21.0, -5), 
        (-23.0, -21.0, -5), 
        (-24.0, -21.0, -5), 
        (-25.0, -21.0, -5), 
        (-26.0, -20.0, -5), 
        (-26.0, -19.0, -5), 
        (-26.0, -18.0, -5), 
        (-27.0, -17.0, -5), 
        (-27.0, -16.0, -5), 
        (-28.0, -15.0, -5), 
        (-28.0, -14.0, -5), 
        (-29.0, -13.0, -5), 
        (-29.0, -12.0, -5), 
        (-30.0, -11.0, -5), 
        (-30.0, -10.0, -5), 
        (-30.0, -9.0, -5), 
        (-31.0, -8.0, -5), 
        (-31.0, -7.0, -5), 
        (-32.0, -6.0, -5), 
        (-32.0, -5.0, -5), 
        (-33.0, -4.0, -5), 
        (-33.0, -3.0, -5), 
        (-34.0, -2.0, -5), 
        (-34.0, -1.0, -5), 
        (-35.0, 0.0, -5), 
        (-35.0, 1.0, -5)]
        self.current_index = 0
        self.offboard_setpoint_counter = 0
        self.hold_cycles = 40  # mantener 2 segundos en cada punto
        self.current_hold_counter = 0


        
        # Crear timer para publicar comandos periódicamente
        self.timer = self.create_timer(0.1, self.timer_callback)

    def publish_offboard_control_heartbeat_signal(self) -> None:
        """Publica la señal de modo offboard (latido/heartbeat)."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float) -> None:
        """Publica un setpoint de posición (x, y, z)."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # orientación de 90 grados (opcional)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publicando setpoint de posición: [{x}, {y}, {z}]")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publica un comando de vehículo (arm, offboard, etc.)."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def arm(self) -> None:
        """Envía comando de armado del dron."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Enviando comando de armado")

    def engage_offboard_mode(self) -> None:
        """Cambia a modo OFFBOARD."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Cambiando a modo OFFBOARD")

    def timer_callback(self) -> None:
        """Función callback del temporizador: publica nuevos setpoints de forma continua."""
        # Publicar señal de modo offboard en cada ciclo
        self.publish_offboard_control_heartbeat_signal()
        
        # Primeros ciclos: enviar varios setpoints para que Offboard sea aceptado
        if self.offboard_setpoint_counter < 10:
            self.offboard_setpoint_counter += 1
            return
        
        # En el ciclo 10, activar OFFBOARD y armar
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.offboard_setpoint_counter += 1
            return
        
        # A partir del ciclo 11 en adelante: publicar los puntos de la lista secuencialmente
        if self.current_index < len(self.points):
            x, y = self.points[self.current_index]
            z = -5.0
            self.publish_position_setpoint(abs(x) -3, abs(y), z)
            
            self.current_hold_counter += 1
            if self.current_hold_counter >= self.hold_cycles:
                self.current_index += 1
                self.current_hold_counter = 0

  

def main(args=None) -> None:
    print('Iniciando nodo de control OFFBOARD...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '_main_':
    try:
        main()
    except KeyboardInterrupt:
        pass