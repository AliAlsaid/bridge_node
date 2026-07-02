#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from autoware_control_msgs.msg import Control
from vehiclecontrol_msgs.msg import VehicleControl
from std_msgs.msg import Float64MultiArray
from autoware_vehicle_msgs.msg  import VelocityReport
from autoware_vehicle_msgs.msg import SteeringReport
from autoware_vehicle_msgs.msg  import ControlModeReport
from rclpy.executors import MultiThreadedExecutor


class BridgeNode(Node):

    def __init__(self):
        # Forward path: Autoware → CarMaker ---

        super().__init__("bridge_node")
        self.timer = self.create_timer(0.001, self.timer_callback)  # ZOH at 1000Hz  
        self.acc = None
        self.steer = None
        self.Gas = 0.0
        self.Brake = 0.0
        self.steer_output = 0.0
        self.velocity = 0.0
        self.steerAngle = 0.0
        self.cmd_control = self.create_subscription(
            Control, "/control/command/control_cmd", self.control_callback, 10)
        self.publish_control = self.create_publisher(VehicleControl, "/carmaker/VehicleControl", 10)
        # Return path: CarMaker → Autoware 

        self.sub_feedback = self.create_subscription(Float64MultiArray, "/carmaker/vehicle_state", self.feedback_callback, 10)
        self.pub_feedback_vel = self.create_publisher(VelocityReport, "/vehicle/status/velocity_status", 10)
        self.pub_feedback_ang = self.create_publisher(SteeringReport, "/vehicle/status/steering_status", 10)
        self.pub_feedback_mode = self.create_publisher(ControlModeReport, "/vehicle/status/control_mode", 10)




    # ============ FORWARD PATH ============

    def control_callback(self, msg: Control): # this callback recive command from autoware at 10Hz 
        self.steer = msg.lateral.steering_tire_angle
        self.acc = msg.longitudinal.acceleration

    def timer_callback(self): #This callback receive data from control_callback and convert it to torque and steer output at 1000Hz
        msg = VehicleControl()
        if self.acc is not None and 0 <= self.acc <= 3.0:
            self.Gas = self.acc/3.0
            self.Brake = 0.0
        elif self.acc is not None and -3.0 <= self.acc <= 0:
            self.Gas = 0.0
            self.Brake = -self.acc/3.0  

        if self.steer is not None:
            self.steer_output = self.steer 

        msg.gas = self.Gas
        msg.brake = self.Brake
        msg.steer_ang = self.steer_output
        msg.steer_ang_vel = 0.0  # TODO: compute via differentiation in validation phase
        msg.steer_ang_acc = 0.0  # TODO: compute via differentiation in validation phase
        msg.selector_ctrl = 1
        msg.use_vc = True
        self.publish_control.publish(msg)

        self.get_logger().info(f"Gas: {self.Gas}, Brake: {self.Brake}, Steer: {self.steer_output}", throttle_duration_sec=1.0)

    # ============ RETURN PATH ============

    def feedback_callback(self, msg: Float64MultiArray):
        self.velocity = msg.data[0]
        self.steerAngle = msg.data[1]

        # Velocity
        vel_msg = VelocityReport()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = "base_link"
        vel_msg.longitudinal_velocity = self.velocity
        self.pub_feedback_vel.publish(vel_msg)

        # Steering
        steer_msg = SteeringReport()
        steer_msg.stamp = self.get_clock().now().to_msg()
        steer_msg.steering_tire_angle = self.steerAngle
        self.pub_feedback_ang.publish(steer_msg)

        # Control mode 
        control_mode_msg = ControlModeReport()
        control_mode_msg.stamp = self.get_clock().now().to_msg()
        control_mode_msg.mode = 1   # AUTONOMOUS
        self.pub_feedback_mode.publish(control_mode_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()