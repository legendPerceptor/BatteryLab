from .AssemblyRobot import AssemblyRobot, assembly_robot_command_loop
from .CrimperRobot import CrimperRobot, crimper_robot_command_loop
from .LiquidRobot import LiquidRobot, liquid_robot_command_loop

import rclpy
from rclpy.node import Node

class AutoBatteryLab(Node):
    def __init__(self):
        super().__init__("auto_battery_lab")
        logger = self.get_logger()
        # Initialize the Assembly Robot
        self.assembly_robot = AssemblyRobot(logger=logger, robot_address="192.168.0.100")
        self.assembly_robot.initialize_and_home_robots()
        # Initialize the Liquid Robot
        self.liquid_robot = LiquidRobot(ip="192.168.0.107", logger=logger)
        ok = self.liquid_robot.initialize_robot()
        if not ok:
            print("Failed to initialize the Liquid Robot, program aborted!")
            exit()
        # Initialize the Crimper Robot
        self.crimper_robot = CrimperRobot(logger=logger, robot_address="192.168.0.101")
        self.crimper_robot.initialize_and_home_robots()
        logger.info("Finish intializing the Auto Battery Lab")
    
    def assemble_a_battery(self):
        pass

def command_loop(batterylab: AutoBatteryLab):
    prompt= """Press [Enter] to quit, [Assembly] to go to assembly_robot's command list,
[Liquid] to go to liquid_robot's command list, [Crimper] to go to crimper_robot's command list.
[A] to finish assemble a battery from scratch to storage.
:> """
    while True:
        user_input = input(prompt)
        if user_input == '':
            break
        elif user_input == 'Assembly':
            assembly_robot_command_loop(batterylab.assembly_robot)
        elif user_input == 'Liquid':
            liquid_robot_command_loop(batterylab.liquid_robot)
        elif user_input == 'Crimper':
            crimper_robot_command_loop(batterylab.crimper_robot)
        else:
            print("The choice is not valid. Please follow the instructions to use the battery lab app.")
        

def main():
    rclpy.init()
    batterylab = AutoBatteryLab()
    batterylab.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()