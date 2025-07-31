#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class OffboardNode(Node):
    def __init__(self):
        super().__init__('offboard_node')
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_cb, 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)
        self.arm_cli = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, 'mavros/set_mode')
        self.current_state = State()
        self.timer = self.create_timer(0.05, self.loop)

        self.pose = PoseStamped()
        self.pose.pose.position.z = 2.0
        self.initialized = False

    def state_cb(self, msg):
        self.current_state = msg

    def loop(self):
        if not self.current_state.connected:
            self.get_logger().info('Waiting FCU connection...')
            return

        # publish initial setpoints
        if not self.initialized:
            for _ in range(100):
                self.setpoint_pub.publish(self.pose)
                rclpy.spin_once(self, timeout_sec=0.1)
            # set mode to OFFBOARD
            req = SetMode.Request(custom_mode='OFFBOARD')
            self.mode_cli.call_async(req)
            # arm
            arm_req = CommandBool.Request(value=True)
            self.arm_cli.call_async(arm_req)
            self.initialized = True
            self.get_logger().info('Sent OFFBOARD & arm requests')

        # after armed + offboard, move forward
        self.pose.pose.position.x += 0.1
        self.setpoint_pub.publish(self.pose)

def main(args=None):
    rclpy.init(args=args)
    node = OffboardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
