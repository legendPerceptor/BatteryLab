import os
import time
import json
import threading
import logging

import serial
from Meca500 import Meca500, RobotTool
from ZaberRail import ZaberRail
from Logger import Logger


class AssemblyRobot():

    def __init__(self, logger = None):
        self.rail_meca500 = Meca500(logger=logger, log_path="./rail_meca500.log", robot_address="192.168.0.101")
        self.zaber_rail = ZaberRail()
        self.status = dict(Progress=dict(Initiate=0, LastStep=None), Meca500Ready=False, ZaberRailReady=False)
        self.logger = Logger(device_name="Assembly Robot", log_path="./assembly_robot.log") if logger is None else logger
        self.initialize_and_home_robots()

    def initialize_and_home_robots(self):
        ok = self.rail_meca500.initializeRobot()
        if not ok:
            print("The Meca500 cannot be connected")
            exit()
        ok = self.zaber_rail.connect()
        if not ok:
            print("Zaber Rail cannot be connected!")
            exit()

    def grab_component(self, rail_position, grab_position, is_grab=True):
        """Grab a component for battery.
        This requires a cooperation between the rail and the Meca500 robotic arm on top of it.
        """
        # Move to the rail position
        self.logger.info(f"Assembly Robot Moving to {rail_position}.")
        self.zaber_rail.move(rail_position)

        # Prepare proper tooling for grabbing components
        if abs(grab_position[0]) >= 160:
            self.rail_meca500.change_tool(RobotTool.GRIPPER)
        else:
            self.rail_meca500.change_tool(RobotTool.SUCTION)

        # Move home based on the tooling
        self.rail_meca500.move_home()

        # Let Meca500 pick up the component and move it home
        self.rail_meca500.pick_place(grab_position, is_grab=is_grab)


def main():
    robot = AssemblyRobot()
    steps = [{
        "rail_position": 10,
        "grab_position": [95.649, -91.317, 20.045, 180.0, 0.0, -90.0]
    }, {
        "rail_position": 20,
        "grab_position": [96.049, -114.317, 20.045, 180.0, 0.0, -90.0]
    }, {
        "rail_position": 30,
        "grab_position": [170.0, -90.0, 20.0, 180.0, 0.0, -90.0]
    }]
    for step in steps:
        robot.grab_component(step["rail_position"], step["grab_position"], is_grab=True)
        robot.grab_component(step["rail_position"], step["grab_position"], is_grab=False)

if __name__ == '__main__':
    main()