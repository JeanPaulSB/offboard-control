import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus

class OffboardControlNode(Node):
    def __init__(self):

        super().__init__('offboard_control_node')

        # configure QoS
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        )

        # sub and pub
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos)

        self.pub_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.waypoints = [
        [0.0, 0.0, -5.0], 
        [-1.0, -1.0, -5.0], 
        [-2.0, -2.0, -5.0], 
        [-3.0, -3.0, -5.0], 
        [-4.0, -4.0, -5.0], 
        [-5.0, -5.0, -5.0], 
        [-6.0, -6.0, -5.0], 
        [-7.0, -7.0, -5.0], 
        [-8.0, -8.0, -5.0], 
        [-8.0, -9.0, -5.0], 
        [-8.0, -10.0, -5.0], 
        [-8.0, -11.0, -5.0], 
        [-8.0, -12.0, -5.0], 
        [-8.0, -13.0, -5.0], 
        [-8.0, -14.0, -5.0], 
        [-8.0, -15.0, -5.0], 
        [-8.0, -16.0, -5.0], 
        [-8.0, -17.0, -5.0], 
        [-8.0, -18.0, -5.0], 
        [-8.0, -19.0, -5.0], 
        [-9.0, -20.0, -5.0], 
        [-10.0, -20.0, -5.0], 
        [-11.0, -20.0, -5.0], 
        [-12.0, -20.0, -5.0], 
        [-13.0, -20.0, -5.0], 
        [-14.0, -20.0, -5.0], 
        [-15.0, -20.0, -5.0], 
        [-16.0, -20.0, -5.0], 
        [-17.0, -20.0, -5.0], 
        [-18.0, -20.0, -5.0], 
        [-19.0, -20.0, -5.0], 
        [-20.0, -20.0, -5.0], 
        [-21.0, -20.0, -5.0], 
        [-22.0, -20.0, -5.0], 
        [-23.0, -20.0, -5.0], 
        [-24.0, -20.0, -5.0], 
        [-25.0, -20.0, -5.0], 
        [-26.0, -19.0, -5.0], 
        [-26.0, -18.0, -5.0], 
        [-27.0, -17.0, -5.0], 
        [-27.0, -16.0, -5.0], 
        [-28.0, -15.0, -5.0], 
        [-28.0, -14.0, -5.0], 
        [-29.0, -13.0, -5.0], 
        [-29.0, -12.0, -5.0], 
        [-30.0, -11.0, -5.0], 
        [-30.0, -10.0, -5.0], 
        [-30.0, -9.0, -5.0], 
        [-31.0, -8.0, -5.0], 
        [-31.0, -7.0, -5.0], 
        [-32.0, -6.0, -5.0], 
        [-32.0, -5.0, -5.0], 
        [-33.0, -4.0, -5.0], 
        [-33.0, -3.0, -5.0], 
        [-34.0, -2.0, -5.0], 
        [-34.0, -1.0, -5.0], 
        [-35.0, 0.0, -5.0], 
        [-35.0, 1.0, -5.0]
        ]


        # objective pos
        self.target_x = 0.5 # N
        self.target_y = 0.5 # E
        self.target_z = -5.0 # D
        
        self.current_wp_index = 0
        self.offboard_wp_counter = 0
       
        self.offboard_counter = 0
        self.nav_state = None
        self.arming_state = None

 
        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def status_callback(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def timer_callback(self):
        offb = OffboardControlMode()
        offb.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offb.position = True      
        offb.velocity = False
        offb.acceleration = False
        offb.attitude = False
        offb.body_rate = False
        offb.thrust_and_torque = False
        offb.direct_actuator = False
        self.pub_offboard_mode.publish(offb)

        
        if self.offboard_counter == 10:
            self.engage_offboard_mode()
            self.arm_vehicle()

        
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            traj = TrajectorySetpoint()
            traj.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            traj.position = self.waypoints[-1]
            traj.yaw = 0.0   
            self.pub_trajectory.publish(traj)

        if self.offboard_counter < 11:
            self.offboard_counter += 1

    def engage_offboard_mode(self):

        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0  
        cmd.param2 = 6.0   
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_cmd.publish(cmd)
        self.get_logger().info('Command Offboard')

    def arm_vehicle(self):
        
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0  
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_cmd.publish(cmd)
        self.get_logger().info('Command Arm')

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()