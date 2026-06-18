#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from autoware_control_msgs.msg import Control


class BridgeNode(Node):

    def __init__(self):
        super().__init__("bridge_node")
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.acc = None
        self.steer = None
        self.torque_output = 0.0
        self.steer_output = 0.0
        self.cmd_control = self.create_subscription(
            Control, "/control/command/control_cmd", self.control_callback, 10)
    
    def control_callback(self, msg: Control):
        self.steer = msg.lateral.steering_tire_angle
        self.acc = msg.longitudinal.acceleration

    def timer_callback(self):
        K_torque = 20
        K_steer = 100
        if self.acc is not None and self.steer is not None:
            self.torque_output = K_torque * self.acc
            self.steer_output = K_steer * self.steer
        else:
            pass
        self.get_logger().info(f"Torque: {self.torque_output}, Steer: {self.steer_output}", throttle_duration_sec=1.0)



def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()