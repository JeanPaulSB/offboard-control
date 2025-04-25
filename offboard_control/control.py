import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus,VehicleCommand, OffboardControlMode



class OffboardControl(Node):
    def __init__(self):
        super().__init__('minimal_publisher')

        qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',qos_profile
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',qos_profile)
        
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint,
        "/fmu/in/trajectory_setpoint", qos_profile)

        self.waypoints = [
            (4,4,4)
        ]
        

    def arm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)   
        self.cmd_pub.publish(msg)
        print("Arming drone...")

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    

        
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

        if msg.arming_state == 2:
            pass
        else:
            self.arm()
            self.publish_offboard_control_mode()

            msg = TrajectorySetpoint()
            tol = 0.2

            if len(self.waypoints) > 0:
                msg.position = [4.0,4.0,4.0]
                msg.yaw = 0
               
                self.waypoints.pop(0)   

            msg.timestamp = int(Clock().now().nanoseconds / 1000)   # time in microseconds
            self.trajectory_setpoint_pub.publish(msg) 





def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()