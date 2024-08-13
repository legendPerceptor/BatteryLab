#!/home/yuanjian/Research/BatteryLab/lab_venv/bin/python3
from BatteryLab.robots.Meca500 import Meca500, RobotTool
from BatteryLab.helper.Logger import Logger
from BatteryLab.robots.AutoCorrection import AutoCorrection
from BatteryLab.robots.Constants import AssemblyRobotConstants, Components, AssemblySteps, ComponentProperty

from BatteryLab.robots.ZaberRail import ZaberRail
from linear_rail_control.linear_rail_client import LinearRailClient
import numpy as np

import yaml
from pathlib import Path
from typing import List
from rclpy.node import Node
import rclpy

class AssemblyRobot(Node):

    def __init__(self, logger = None):
        super().__init__('assembly_robot')
        self.dir_name = "experiment_results"
        self.rail_meca500 = Meca500(logger=logger, log_path="./rail_meca500.log", robot_address="192.168.0.101")
        self.status = dict(Progress=dict(Initiate=0, LastStep=None), Meca500Ready=False, ZaberRailReady=False)
        self.logger = Logger(logger_name="Assembly Robot", log_path="logs", logger_filename="assembly_robot.log") if logger is None else logger
        self.zaber_rail = LinearRailClient()
        self.assemblyRobotConstants = AssemblyRobotConstants()
        self.auto_correction = AutoCorrection()
        self.initialize_and_home_robots()

    def initialize_and_home_robots(self):
        ok = self.rail_meca500.initializeRobot()
        if not ok:
            print("The Meca500 cannot be connected")
            exit()

    def grab_component(self, rail_position, grab_position, is_grab=True):
        """Grab a component for battery or return it to the tray.
        This requires a cooperation between the rail and the Meca500 robotic arm on top of it.
        """
        # Move to the rail position
        self.logger.info(f"Assembly Robot Moving to {rail_position}.")
        self.zaber_rail.send_move_request(rail_position)

        # Prepare proper tooling for grabbing components
        # if abs(grab_position[0]) >= 160:
        #     self.rail_meca500.change_tool(RobotTool.GRIPPER)
        # else:
        #     self.rail_meca500.change_tool(RobotTool.SUCTION)
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
        self.zaber_rail.send_move_request(self.assemblyRobotConstants.POST_RAIL_LOCATION)

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
                self.zaber_rail.send_move_request(0) # move home
                self.rail_meca500.move_home()
                component_property: ComponentProperty = getattr(self.assemblyRobotConstants, component.name)
                self.grab_component(component_property.railPo[nr-1], component_property.grabPo[nr])
                self.zaber_rail.send_move_request(self.assemblyRobotConstants.POST_RAIL_LOCATION)
                self.rail_meca500.move_for_snapshot()
                correction, self.grabYes = self.auto_correction.run_autocorrection(state=AssemblySteps.Grab, component=component, nr=nr, show_image=show_image, save_img=save_img)
                if self.grabYes:
                    drop_po = drop_po + correction
                    self.logger.info("Implementing auto correction...")
                else:
                    self.logger.info(f"No {component.name} detected! Manual check on tray {component.name}_No.[{nr}] required!")
                    # Move the meca500 robotic arm home
                    self.rail_meca500.move_home()
                    self.zaber_rail.send_move_request(0)
                    manual_ok = input(f"Make sure vacuum pump is on and {component.name} is on the tray NO.[{nr}], then type in 'ok' and Enter!")
                    if manual_ok == 'ok':
                        self.grab_component(component_property.railPo[nr-1], component_property.grabPo[nr])
                        self.zaber_rail.send_move_request(self.assemblyRobotConstants.POST_RAIL_LOCATION)
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
    
def get_8_8_well_pos(bottom_left_coordinates, bottom_right_coordinates, top_left_coordinates, top_right_coordinates):
    avg_height = np.average([bottom_left_coordinates[2], bottom_right_coordinates[2], top_left_coordinates[2], top_right_coordinates[2]])
    pos_dict: dict[int, List[float]] = {}
    delta_y = np.average([bottom_right_coordinates[1] - bottom_left_coordinates[1], top_right_coordinates[1] - top_left_coordinates[1]]) / 8
    delta_x = np.average([bottom_right_coordinates[0] - top_right_coordinates[0], bottom_left_coordinates[0] - top_left_coordinates[0]]) / 8
    for i in range(64):
        x = i / 8
        y = i % 8
        pos_dict[i] = [top_left_coordinates[0] + x * delta_x, top_left_coordinates[1] + y * delta_y, avg_height] + bottom_left_coordinates[3:]
    return pos_dict

def create_robot_constants_from_manual_positions(manual_positions) -> AssemblyRobotConstants:
    """Obtain the assembly robot constants"""
    constants = AssemblyRobotConstants()

    cartesian_coord_prop = "cartesians"
    rail_pos_prop = "rail_pos"
    bottom_left_prop = "bottom_left"
    bottom_right_prop = "bottom_right"
    top_left_prop = "top_left"
    top_right_prop = "top_right"

    assemble_post = manual_positions["AssemblePost"]
    constants.POST_C_SK_PO = assemble_post[cartesian_coord_prop]
    constants.POST_RAIL_LOCATION = assemble_post[rail_pos_prop]

    separator = manual_positions["Separator"]
    constants.Separator = ComponentProperty()
    constants.Separator.railPo = separator[rail_pos_prop]
    constants.Separator.dropPo = assemble_post[cartesian_coord_prop]
    bottom_left_coordinates = separator[bottom_left_prop][cartesian_coord_prop]
    bottom_right_coordinates = separator[bottom_right_prop][cartesian_coord_prop]
    top_left_coordinates = separator[top_left_prop][cartesian_coord_prop]
    top_right_coordinates = separator[top_right_prop][cartesian_coord_prop]
    constants.Separator.grabPo = get_8_8_well_pos(bottom_left_coordinates, bottom_right_coordinates, top_left_coordinates, top_right_coordinates)

    return constants


def main():
    rclpy.init()
    robot = AssemblyRobot()
    position_file = Path(__file__).parent / ".." / "resource" / "well_positions.yaml"
    with open(position_file, "r") as f:
        try:
            constant_positions = yaml.safe_load(f)
        except yaml.YAMLError as e:
            print("Cannot load the YAML file with error: ", e)
    
    assembly_robot_constants = create_robot_constants_from_manual_positions(constant_positions)
    grabpos = assembly_robot_constants.Separator.grabPo
    railpos = assembly_robot_constants.Separator.railPo
    for i in range(64):
        robot.grab_component(railpos, grabpos[i], is_grab=True)
        robot.grab_component(railpos, grabpos[i], is_grab=False)
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()