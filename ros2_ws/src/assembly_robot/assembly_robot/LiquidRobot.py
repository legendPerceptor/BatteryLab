#!/home/yuanjian/Research/BatteryLab/lab_venv/bin/python3
from BatteryLab.robots.MG400 import MG400, main_loop
from BatteryLab.helper.Logger import Logger
from BatteryLab.robots.Constants import RobotTool
import rclpy
from rclpy.node import Node
from camera_service.camera_client import ImageClient
from sartorius.sartorius_client import SartoriusClient

class LiquidRobot(Node):
    def __init__(self, logger=None):
        super().__init__("liquid_robot")
        self.logger = self.get_logger() if logger is None else logger
        self.sartorius = SartoriusClient()
        self.MG400 = MG400(logger=self.logger, sartorius_rline=self.sartorius)

    def initialize(self):
        ok = self.MG400.intialize_robot()
        if not ok:
            print("Failed to initialize MG400, program aborted!")
            exit()
        self.MG400.move_home()

def main():
    rclpy.init()
    liquid_robot = LiquidRobot()
    liquid_robot.initialize()
    main_loop(liquid_robot.MG400)
    rclpy.shutdown()

if __name__ == '__main__':
    main()