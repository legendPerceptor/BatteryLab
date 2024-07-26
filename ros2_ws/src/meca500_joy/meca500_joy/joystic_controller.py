#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from BatteryLab.robots.Meca500 import Meca500
from BatteryLab.helper.utils import get_proper_port_for_device, SupportedDevices

class JoystickController(Node):
    def __init__(self, meca500_ip: str='192.168.0.1'):
        super().__init__('joystick_controller')
        self.declare_parameter('robot_ip', meca500_ip)  # Replace with your robot's IP address
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        
        # Initialize Mecademic API
        self.robot = Meca500(robot_address=self.robot_ip, vacuum_port=get_proper_port_for_device(SupportedDevices.SuctionPump))
        
        # Set up subscriber for joystick messages
        self.joy_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
    def joy_callback(self, msg):
        # Example joystick control mapping
        x_axis = msg.axes[0]
        y_axis = msg.axes[1]
        
        # Convert joystick inputs to robot commands
        # You need to map joystick axes/buttons to robot movements here
        if x_axis > 0.5:
            self.robot.move_to_position(x_axis, y_axis, 0)  # Replace with appropriate API calls
        else:
            self.robot.stop()  # Example stop command

def main(args=None):
    rclpy.init(args=args)
    node = JoystickController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
