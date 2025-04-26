import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

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
            (14.0, 8.0),
            (13.993421622064858, 8.49995672307085),
            (13.55125934365837, 8.733393051765384),
            (13.108821170725916, 8.96630604653423),
            (12.649041946059826, 8.769829420531416),
            (12.219660654427111, 9.026016278890351),
            (12.012716016775116, 9.481179894469502),
            (11.569390106332863, 9.712398702377827),
            (11.192291557957219, 10.040722756953459),
            (10.796077089287468, 10.345704879079898),
            (10.343714298625432, 10.558702310107728),
            (10.071711510236453, 10.978243110398498),
            (9.891157982243012, 11.444505293648568),
            (9.393876493053435, 11.39243687222639),
            (8.992620834437743, 11.690755320246097),
            (8.495985716145475, 11.632845396408943),
            (8.002510083803472, 11.552335767887497),
            (7.640444374264667, 11.897166772839432),
            (7.1414941850344755, 11.929550544502973),
            (6.955942825368972, 12.393846455454472),
            (6.478347127738247, 12.245852031219565),
            (5.981482927064096, 12.301762370947),
            (5.497234498916165, 12.426275261188362),
            (4.9978364550284216, 12.401747852823522),
            (4.634982883134046, 12.74574972107934),
            (4.625323392852279, 13.245656406619249),
            (4.525654813173892, 13.73562188930644),
            (4.312035185620949, 14.187691192316656),
            (4.122282663421324, 14.650286026181973),
            (4.507647750275157, 14.968866863386815),
            (4.19420327219047, 15.358421171742169),
            (4.122109146303414, 15.853196309570032),
            (4.110932640020986, 16.35307137966987),
            (3.8950324503666107, 16.80405597819537),
            (3.7414071243020257, 17.279870289867002),
            (3.560388015731832, 17.745951916121946),
            (3.8995306438493675, 18.11335125987957),
            (4.136511057247532, 18.55362422482931),
            (4.038308664519765, 19.04388567554978),
            (4.509258360595475, 19.21183323809892),
            (4.891581636567977, 19.53405817917216),
            (4.821871471331988, 20.029174824885846),
            (4.839906305282439, 20.528849463790383),
            (5.2444668625678, 20.822670422009768),
            (5.744305060463862, 20.809951305530344),
            (5.943419947804368, 21.26859415311371),
            (6.336715541359555, 21.57733121938544),
            (6.578288003632588, 22.015101416488147),
            (7.078284557397123, 22.01695781719599),
            (7.498196853924926, 22.28838674569247),
            (7.897497980179431, 22.58931632312195),
            (7.776272857523844, 23.074398243664383),
            (8.227882764015623, 23.288987368175587),
            (8.642878335411996, 23.567875653554157),
            (8.89354895667976, 24.000500479047602),
            (9.140704854130846, 24.435142817476738),
            (9.57761688698884, 24.678263930389406),
            (10.014528919846835, 24.921385043302074),
            (10.381411617112407, 25.261086509956456),
            (10.820629768396147, 25.500016239339164),
            (11.291334224952527, 25.668649909102836),
            (11.768246646945553, 25.51846821331821),
            (12.168710123370102, 25.817849248205132),
            (12.64007207397765, 25.98463627263883),
            (12.950680846351133, 25.592817193822295),
            (13.370084525843977, 25.865031362498108),
            (13.528797069542085, 26.339173038026505),
            (14.01812881627886, 26.441907850215642),
            (13.978771253734237, 26.94035642556775),
            (13.75, 27.0)
        ]
        self.current_index = 0
        self.offboard_setpoint_counter = 0
        
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
            self.publish_position_setpoint(x, y, z)
            self.current_index += 1
        else:
            # Una vez alcanzado el último punto, publicar continuamente ese mismo setpoint
            x, y = self.points[-1]
            z = -5.0
            self.publish_position_setpoint(x, y, z)


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