#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand, TrajectorySetpoint, VehicleLocalPosition, VehicleStatus, VehicleCommandAck

class my_offboardcontrol_node(Node):
    def __init__(self) -> None:
        super().__init__('my_offboard_control_node')
        # config quality of service
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        #create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,'/fmu/in/offboard_control_mode',qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,'/fmu/in/vehicle_command',qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,'/fmu/in/trajectory_setpoint',qos_profile)

        # create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,'/fmu/out/vehicle_local_position',self.vehicle_local_position_callback,qos_profile)
        self.vehicle_status_subcriber = self.create_subscription(
            VehicleStatus,'/fmu/out/vehicle_status',self.vehicle_status_callback,qos_profile)
        self.vehicle_command_ack_subcriber = self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack',self.vehicle_command_ack_callback,qos_profile)
        
        # Initialization
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.vehicle_command_ack = VehicleCommandAck()
        self.take_off_height = -2.0
        self.counter = 0

        # Create a timer to publish commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, position):
        self.vehicle_local_position = position
    
    def vehicle_status_callback(self, status):
        self.vehicle_status = status
    
    def vehicle_command_ack_callback(self, vehicle_command_ack):
        self.vehicle_command_ack = vehicle_command_ack
    
    def publish_vehicle_command(self, command, **kwargs) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = kwargs.get("param1", 0.0)
        msg.param2 = kwargs.get("param2", 0.0)
        msg.param3 = kwargs.get("param3", 0.0)
        msg.param4 = kwargs.get("param4", 0.0)
        msg.param5 = kwargs.get("param5", 0.0)
        msg.param6 = kwargs.get("param6", 0.0)
        msg.param7 = kwargs.get("param7", 0.0)
        msg.source_component = 1
        msg.target_component = 1
        msg.from_external = True
        msg.source_system = 1
        msg.target_system = 1
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def publish_2_hz_signal(self):
        #self.get_logger().info("publish the liveless signal (>2Hz)")
        msg = OffboardControlMode()
        msg.position = True
        msg.acceleration = False
        msg.velocity = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
    
    def arm(self):
        self.get_logger().info("arm signal's sent")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def dis_arm(self):
        self.get_logger().info("disarm signal's sent")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def engage_offboard_control(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=0.0)
        self.get_logger().info("Engage Offboard control mode!!!!")
        self.get_logger().info(f"{self.vehicle_status.nav_state}")
    
    def land(self):
        self.get_logger().info("Switching to land mode")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing trajectories (position) setpoints {[x, y, z]}")
    
    def Print_Everything(self):
        self.get_logger().info(f"Vehicle status: {self.vehicle_status.nav_state}")
        self.get_logger().info(f"Vehicle local position {self.vehicle_local_position.z}")
        self.get_logger().info(f"Vehicle Command Acknownledge {self.vehicle_command_ack.command} and {self.vehicle_command_ack.result}")
       

    def timer_callback(self) -> None:

        self.publish_2_hz_signal()
        self.publish_position_setpoint(0.0, 0.0, self.take_off_height)
        #if self.counter == 10:
            #self.engage_offboard_control()
        #    self.arm()

        if self.vehicle_local_position.z > self.take_off_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(0.0, 0.0, self.take_off_height)
        #elif self.vehicle_local_position.z <= self.take_off_height:
        #    self.land()
        #    exit(0)


        #if self.counter < 11:
        #    self.counter += 1
        self.Print_Everything()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = my_offboardcontrol_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
