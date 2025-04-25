import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition
class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth=1
        )
        # Publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.timer_period = 0.02 # 50 Hz update rate
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)
        # Target position parameters
        self.declare_parameter('target_x', 50.0)
        self.declare_parameter('target_y', 50.0)
        self.declare_parameter('target_z', -5.0)
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.target_z = self.get_parameter('target_z').value

        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.reached_goal = False # Flag to stop publishing

    def vehicle_position_callback(self, msg):
        """ Update current position from drone sensors """
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    def cmdloop_callback(self):
        """ Publish commands to control the drone """
        if self.reached_goal:
            return # Stop sending commands if the goal is reached
            # Publish offboard control mode
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)
        # Publish trajectory setpoint to move to the target position
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        trajectory_msg.position = [self.target_x, self.target_y, self.target_z]
        trajectory_msg.yaw = 0.0 # Optional: set yaw to face forward
        self.publisher_trajectory.publish(trajectory_msg)
