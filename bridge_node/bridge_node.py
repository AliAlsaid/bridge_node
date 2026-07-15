#! /usr/bin/env python3
    # ============================================================
    # SECTION 1: Imports the toolbox, Message types
    # ============================================================

from autoware_control_msgs import msg
import rclpy
from rclpy.node import Node
from autoware_control_msgs.msg import Control
from vehiclecontrol_msgs.msg import VehicleControl
from std_msgs.msg import Float64MultiArray
from autoware_vehicle_msgs.msg  import VelocityReport
from autoware_vehicle_msgs.msg import SteeringReport
from autoware_vehicle_msgs.msg  import ControlModeReport
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import AccelWithCovarianceStamped, TransformStamped
    # ============================================================
    # SECTION 2: WIRING — shelves, tuning knobs, subs, pubs
    # ============================================================
class BridgeNode(Node):

    def __init__(self):

        super().__init__("bridge_node")
        self.timer = self.create_timer(0.02, self.run_control_loop)  # ZOH at 50HZ match the input from carmaker, which is 50Hz  
        self.cmd_accel = None 
        self.cmd_steer = None
        self.Gas = 0.0
        self.Brake = 0.0
        self.steer_output = 0.0
        self.velocity = 0.0
        self.measured_steer = 0.0
        self.yaw_rate = 0.0
        self.prev_velocity = 0.0
        self.prev_time = None
        self.acceleration = 0.0
        self.target_velocity = None
        self.error_integral = 0.0
        self.prev_loop_time = None
        self.STEER_RATIO = 6.7   # wheel/tire ratio: measured 0.70 rad cmd -> 0.105 rad tire, matches Rack2StWhl table
        self.Kp = 0.3    # gas/brake per (m/s) of speed error — tune
        self.Ki = 0.05   # removes steady-state error — tune
        self.tf_broadcaster = TransformBroadcaster(self)
        # Forward path: Autoware → CarMaker ---
        
        self.cmd_control = self.create_subscription(
            Control, "/control/command/control_cmd", self.on_autoware_command, 10)
        self.publish_control = self.create_publisher(VehicleControl, "/carmaker/VehicleControl", 10)
       
        # Return path: CarMaker → Autoware 

        self.sub_feedback = self.create_subscription(Float64MultiArray, "/carmaker/vehicle_state", self.on_vehicle_state, 10)
        self.pub_feedback_vel = self.create_publisher(VelocityReport, "/vehicle/status/velocity_status", 10)
        self.pub_feedback_ang = self.create_publisher(SteeringReport, "/vehicle/status/steering_status", 10)
        self.pub_feedback_mode = self.create_publisher(ControlModeReport, "/vehicle/status/control_mode", 10)

        self.pub_acc = self.create_publisher(AccelWithCovarianceStamped, "/localization/acceleration", 10)

        self.sub_pose = self.create_subscription(Float64MultiArray, "/carmaker/pose", self.on_vehicle_pose, 10)
        self.pub_pose = self.create_publisher(Odometry, "/localization/kinematic_state", 10)

        self.sub_imu = self.create_subscription(Imu, "/carmaker/imu", self.on_imu, 10)
        self.pub_imu = self.create_publisher(Imu, "/sensing/imu/imu_data", 10)




    # ============================================================
    # SECTION 3 — FORWARD PATH: Autoware commands -> gas/brake/steer
    # ============================================================

    def on_autoware_command(self, msg: Control): # this callback recive command from autoware at 10Hz 
         self.cmd_steer = msg.lateral.steering_tire_angle
         self.cmd_accel = msg.longitudinal.acceleration
         self.target_velocity = msg.longitudinal.velocity

    def run_control_loop(self):
        msg = VehicleControl()
        now = self.get_clock().now().nanoseconds / 1e9
        dt = 0.02 if self.prev_loop_time is None else (now - self.prev_loop_time)
        self.prev_loop_time = now


        if self.target_velocity is not None:
            error = self.target_velocity - self.velocity     # + = too slow, - = too fast

            # integral with anti-windup (so it can't grow forever)
            self.error_integral += error * dt          # timer dt = 20 ms
            self.error_integral = max(-2.0, min(self.error_integral, 2.0))

            u = self.Kp * error + self.Ki * self.error_integral

            if self.target_velocity < 0.1 and self.velocity < 0.3:
                # hold the car at standstill
                self.Gas = 0.0
                self.Brake = 0.3
            elif u >= 0.0:
                self.Gas = min(u, 1.0)
                self.Brake = 0.0
            else:
                self.Gas = 0.0
                self.Brake = min(-u, 1.0)

        if self.cmd_steer is not None:
            self.steer_output = self.cmd_steer

        msg.gas = self.Gas
        msg.brake = self.Brake
        msg.steer_ang = self.steer_output * self.STEER_RATIO   # Autoware tire rad → CarMaker wheel rad
        msg.steer_ang_vel = 0.0
        msg.steer_ang_acc = 0.0
        msg.selector_ctrl = 1
        msg.use_vc = True
        self.publish_control.publish(msg)

        self.get_logger().info(
            f"Gas: {self.Gas:.2f}, Brake: {self.Brake:.2f}, steer: {self.steer_output:.2f}, err: {self.target_velocity - self.velocity if self.target_velocity is not None else 0:.2f}, accel: {self.acceleration:.2f}",
            throttle_duration_sec=1.0)
    # ============================================================
    # SECTION 4 — RETURN PATH (STATE): velocity, accel, steer, mode
    # ============================================================

    def on_vehicle_state(self, msg: Float64MultiArray):
        self.velocity = msg.data[0]
        self.measured_steer = msg.data[1]

        # Velocity
        vel_msg = VelocityReport()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = "base_link"
        vel_msg.longitudinal_velocity = self.velocity
        vel_msg.heading_rate = self.yaw_rate        # fill the field, using stored yaw

        self.pub_feedback_vel.publish(vel_msg)

        # Acceleration
        time_now = self.get_clock().now().nanoseconds/1e9  # Convert to seconds

        if self.prev_time is None:
            self.prev_velocity = self.velocity
            self.prev_time = time_now
            self.acceleration = 0.0
        else:
            dt = time_now - self.prev_time
            if dt > 0:
                accel_raw = (self.velocity - self.prev_velocity) / dt
                self.acceleration = 0.7 * self.acceleration + 0.3 * accel_raw
            else:
                self.acceleration = 0.0

            # update memory AFTER using the old values

            self.prev_velocity = self.velocity
            self.prev_time = time_now

        acc_msg = AccelWithCovarianceStamped()
        acc_msg.header.stamp = self.get_clock().now().to_msg()
        acc_msg.header.frame_id = "base_link"
        acc_msg.accel.accel.linear.x = self.acceleration
        acc_msg.accel.accel.linear.y = 0.0
        acc_msg.accel.accel.linear.z = 0.0

        self.pub_acc.publish(acc_msg)

        # Steering
        steer_msg = SteeringReport()
        steer_msg.stamp = self.get_clock().now().to_msg()
        steer_msg.steering_tire_angle = self.measured_steer
        self.pub_feedback_ang.publish(steer_msg)

        # Control mode 
        control_mode_msg = ControlModeReport()
        control_mode_msg.stamp = self.get_clock().now().to_msg()
        control_mode_msg.mode = 1   # AUTONOMOUS
        self.pub_feedback_mode.publish(control_mode_msg)

    # ============================================================
    # SECTION 5 — RETURN PATH (POSE): odometry, twist, TF
    # ============================================================

    def on_imu(self, msg: Imu):
        
        self.yaw_rate = msg.angular_velocity.z  # Assuming yaw rate is the z-component of angular velocity
        
    def on_vehicle_pose(self, msg: Float64MultiArray):
        pose_msg = Odometry()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.child_frame_id = "base_link"
        pose_msg.pose.pose.position.x = msg.data[0] 
        pose_msg.pose.pose.position.y = msg.data[1] 
        pose_msg.pose.pose.position.z = msg.data[2]
        yaw = msg.data[3]

        qx = 0.0  # Assuming no roll or pitch
        qy = 0.0  # Assuming no roll or pitch
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        pose_msg.pose.pose.orientation.x = qx
        pose_msg.pose.pose.orientation.y = qy
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw


        pose_msg.twist.twist.linear.x = self.velocity
        pose_msg.twist.twist.angular.z = self.yaw_rate
        self.pub_pose.publish(pose_msg)

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.data[0] 
        t.transform.translation.y = msg.data[1] 
        t.transform.translation.z = msg.data[2]
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    # ============================================================
    # SECTION 6 — ENTRY POINT
    # ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()