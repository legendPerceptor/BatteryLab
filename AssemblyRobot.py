import os
import time
import json
import threading
import logging

import serial
from Meca500 import Meca500, RobotTool
from ZaberRail import ZaberRail
from Logger import Logger

import cv2
import numpy as np
from pydantic import BaseModel

from enum import Enum
from typing import List
from pathlib import Path
from AutoCorrection import AutoCorrection

class Components(Enum):
    Anode_Case=1
    Anode_Spacer=2
    Anode=3
    Separator=4
    Cathode=5
    Cathode_Spacer=6
    Washer=7
    Cathode_Case=8

class AssemblySteps(Enum):
    Grab=1
    Drop=2
    Press=3
    Retrieve=4
    Store=5

class ComponentProperty(BaseModel):
    railPo: List[float]
    dropPo: List[float]
    grabPo: dict[int, List[float]]

class AssemblyRobotConstants(BaseModel):
    POST_C_SK_PO: List[float] = [91.516, 198.318, 57.244, 180.0, 0.0, 90.0]
    POST_RAIL_LOCATION: int = 100
    Anode_Case: ComponentProperty
    Anode_Spacer: ComponentProperty
    Anode: ComponentProperty
    Separator: ComponentProperty
    Cathode: ComponentProperty
    Cathode_Spacer: ComponentProperty
    Washer: ComponentProperty
    Cathode_Case: ComponentProperty

class AssemblyRobot():

    def __init__(self, logger = None):
        self.dir_name = "experiment_results"
        self.rail_meca500 = Meca500(logger=logger, log_path="./rail_meca500.log", robot_address="192.168.0.101")
        self.zaber_rail = ZaberRail()
        self.status = dict(Progress=dict(Initiate=0, LastStep=None), Meca500Ready=False, ZaberRailReady=False)
        self.logger = Logger(device_name="Assembly Robot", log_path="./assembly_robot.log") if logger is None else logger
        self.assemblyRobotConstants = AssemblyRobotConstants()
        self.auto_correction = AutoCorrection()
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
        """Grab a component for battery or return it to the tray.
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

    def drop_component(self, drop_po, component:Components, nr:int, auto_calib:bool=True, grab_check:bool=True, save_img:bool=True, show_image:bool=False):
        """Drop the component to the assembly pod"""
        drop_pos = np.array(drop_po, dtype=np.float32)
        if component != Components.Washer:
            drop_po[:2] = self.assemblyRobotConstants.POST_C_SK_PO[:2]
        
        # move the robot to the post location
        self.zaber_rail.move(self.assemblyRobotConstants.POST_RAIL_LOCATION)

        # component_set excludes the Anode_case and Spring
        # TODO: Why exclude the Anode_case?
        component_set = set(Components.Anode_Spacer, Components.Anode, Components.Separator, Components.Cathode, Components.Cathode_Spacer, Components.Cathode_Case)
        if component in component_set and auto_calib:
            self.rail_meca500.move_for_snapshot()
            self.logger.info(f"checking alignment on {component.name}'s grab")
            correction, self.grabYes = self.auto_correction.run_autocorrection(state=AssemblySteps.Grab, component=component, nr=nr, show_image=show_image, save_img=save_img)
            if self.grabYes:
                drop_po = drop_po + correction
                self.logger.info("Implementing auto correction...")
            elif grab_check:
                self.logger.info(f"Trying to grab {component.name} for the second time...")
                self.zaber_rail.move_home()
                self.rail_meca500.move_home()
                component_property: ComponentProperty = getattr(self.assemblyRobotConstants, component.name)
                self.grab_component(component_property.railPo[nr-1], component_property.grabPo[nr])
                self.zaber_rail.move(self.assemblyRobotConstants.POST_RAIL_LOCATION)
                self.rail_meca500.move_for_snapshot()
                correction, self.grabYes = self.auto_correction.run_autocorrection(state=AssemblySteps.Grab, component=component, nr=nr, show_image=show_image, save_img=save_img)
                if self.grabYes:
                    drop_po = drop_po + correction
                    self.logger.info("Implementing auto correction...")
                else:
                    self.logger.info(f"No {component.name} detected! Manual check on tray {component.name}_No.[{nr}] required!")
                    # Move the meca500 robotic arm home
                    self.rail_meca500.move_home()
                    self.zaber_rail.move_home()
                    manual_ok = input(f"Make sure vacuum pump is on and {component.name} is on the tray NO.[{nr}], then type in 'ok' and Enter!")
                    if manual_ok == 'ok':
                        self.grab_component(component_property.railPo[nr-1], component_property.grabPo[nr])
                        self.zaber_rail.move(self.assemblyRobotConstants.POST_RAIL_LOCATION)
                        self.rail_meca500.move_for_snapshot()
                        correction, self.grabYes = self.auto_correction.run_autocorrection(state=AssemblySteps.Grab, component=component, nr=nr, show_image=show_image, save_img=save_img)
                        drop_po = drop_po + correction
                    else:
                        # TODO: suction_off
                        self.userInterupt = True
                        return False
        
        self.logger.debug(f"Dropping on rail dropping position: {drop_po}...")
        self.rail_meca500.robot.MovePose(drop_po[0], drop_po[1], 100, drop_po[3], drop_po[4], drop_po[5])
        self.rail_meca500.robot.Delay(0.5)
        self.rail_meca500.robot.SetCartLinVel(self.SLOW_DOWN)
        self.rail_meca500.robot.MoveLin(*drop_po)
        self.rail_meca500.robot.Delay(0.5)
        if component == Components.Cathode_Case:
            self.rail_meca500.robot.SetCartAngVel(5)
            self.rail_meca500.robot.MoveLinRelWRF(0,-0.3,0.5,0,0,0)
            self.rail_meca500.robot.MoveLinRelWRF(0,0,0,2,0,0)
            self.rail_meca500.robot.MoveLinRelWRF(0,0,-1.5,0,0,0)
            self.rail_meca500.robot.MoveLinRelWRF(0,0,0,-2,0,0)
            self.rail_meca500.smart_drop()
            self.rail_meca500.robot.SetCartAngVel(45)
            self.rail_meca500.robot.MoveLinRelWRF(0,0,30,0,0,0)
            # TODO: Tap Press
            self.rail_meca500.move_after_drop()
        
        elif component in (Components.Anode, Components.Separator, Components.Cathode): # Electordes
            self.rail_meca500.robot.SetCartAngVel(90)
            self.rail_meca500.smart_drop()
            if auto_calib:
                self.rail_meca500.robot.MoveLinRelWRF(0,0,10,0,0,0)
                self.rail_meca500.robot.SetCartAngVel(45)
                # Taking a snap shot
                self.rail_meca500.move_for_snapshot()
                # Do the auto correction in Assmebly Robot class
                self.auto_correction.run_autocorrection(state=AssemblySteps.Drop, component=component, nr=nr, save_img=save_img)
            self.rail_meca500.robot.MoveLinRelWRF(0, 0, 30, 0, 0, 0)
            self.rail_meca500.move_after_drop()
        else:
            self.rail_meca500.smart_drop()
            self.rail_meca500.robot.MoveLinRelWRF(0,0,30,0,0,0)
            self.rail_meca500.move_after_drop()

        self.status["Progress"]["LastStep"] = AssemblySteps.Drop
        return True

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