#!/home/yuanjian/Research/BatteryLab/lab_venv/bin/python3
from BatteryLab.robots.RailMeca500 import RailMeca500
from BatteryLab.helper.Logger import Logger
from BatteryLab.robots.AutoCorrection import AutoCorrection
from BatteryLab.robots.Constants import AssemblyRobotConstants, AssemblyRobotCameraConstants, Components, AssemblySteps, ComponentProperty, RobotTool
from BatteryLab.helper.utils import create_assembly_robot_constants_from_manual_positions, create_assembly_robot_camera_constants_from_manual_positions
from linear_rail_control.linear_rail_client import LinearRailClient
import numpy as np

import yaml
from pathlib import Path
from rclpy.node import Node
import rclpy
import time

from ament_index_python.packages import get_package_share_path
from camera_service.camera_client import ImageClient

class AssemblyRobot(Node):

    def __init__(self, logger = None, robot_address="192.168.0.100"):
        super().__init__('assembly_robot')
        self.rail_meca500 = RailMeca500(logger=logger, robot_address=robot_address)
        # self.status = dict(Progress=dict(Initiate=0, LastStep=None), Meca500Ready=False, ZaberRailReady=False)
        self.logger = Logger(logger_name="Assembly Robot", log_path="logs", logger_filename="assembly_robot.log") if logger is None else logger
        self.zaber_rail = LinearRailClient()
        self.assemblyRobotConstants = AssemblyRobotConstants()
        self.assemblyRobotCameraConstants = AssemblyRobotCameraConstants()
        self.auto_correction = AutoCorrection(logger=logger)
        self.look_up_camera_client = ImageClient(node_name="assembly_robot_lookup_camera_client", serv_name="/batterylab/lookup_camera")
        self.arm_camera_client = ImageClient(node_name="assembly_robot_arm_camera_client", serv_name="/batterylab/rail_meca500_camera")
        self.initialize_and_home_robots()

    def initialize_and_home_robots(self):
        ok = self.rail_meca500.initializeRobot()
        if not ok:
            print("The Meca500 cannot be connected")
            exit()

    def move_zaber_rail(self, rail_pos: float):
        self.logger.info(f"Assembly Robot Moving to {rail_pos}")
        future = self.zaber_rail.send_move_request(rail_pos)
        while rclpy.ok():
            rclpy.spin_once(self.zaber_rail)
            self.get_logger().info("waiting for the moving request to complete...")
            time.sleep(1)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().error("Service call failed and Zaber rail cannot move")
                else:
                    self.get_logger().info(f"Moving request success: {response.success}")
                break
        # make sure the move is finished
        self.get_logger().debug(f"Assembly Robot move request finished")

    def take_a_look_up_photo(self):
        rail_pos = self.assemblyRobotConstants.LOOKUP_CAM_RAIL_LOCATION
        robot_pos = self.assemblyRobotConstants.LOOKUP_CAM_SK_PO
        self.move_zaber_rail(rail_pos)
        self.rail_meca500.robot.MovePose(*robot_pos)
        self.rail_meca500.robot.WaitIdle()
        self.rail_meca500.robot.Delay(0.2)
        # TODO: use get_image for storing the image or analysis
        self.look_up_camera_client.display_image()
        return self.look_up_camera_client.get_image()    
        
    def take_a_tray_photo(self, component_name: str):
        rail_pos = getattr(self.assemblyRobotCameraConstants, component_name)
        Trf = self.assemblyRobotCameraConstants.TRF
        robot_pos = self.assemblyRobotCameraConstants.RobotPose
        self.move_zaber_rail(rail_pos)
        self.rail_meca500.robot.SetTrf(Trf)
        self.rail_meca500.robot.MovePose(*robot_pos)
        self.rail_meca500.robot.WaitIdle()
        self.rail_meca500.robot.Delay(0.2)
        # TODO: use get_image for storing the image or analysis
        self.arm_camera_client.display_image()
        return self.look_up_camera_client.get_image()

    def grab_component(self, rail_position, grab_position, is_grab=True):
        """Grab a component for battery or return it to the tray.
        This requires a cooperation between the rail and the Meca500 robotic arm on top of it.
        """
        self.move_zaber_rail(rail_position)
        # Prepare proper tooling for grabbing components
        # if abs(grab_position[0]) >= 160:
        #     self.rail_meca500.change_tool(RobotTool.GRIPPER)
        # else:
        #     self.rail_meca500.change_tool(RobotTool.SUCTION)
        # self.rail_meca500.change_tool(RobotTool.SUCTION)

        # Move home based on the tooling
        self.rail_meca500.move_home(tool=RobotTool.SUCTION)

        self.logger.debug(f"Assembly Robot moved home and start picking soon.")
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

        # self.status["Progress"]["LastStep"] = AssemblySteps.Drop
        return True

def main():
    rclpy.init()
    logger = Logger("assembly_robot_test", "/home/yuanjian/Research/BatteryLab/logs", "assembly_robot_test.log")
    robot = AssemblyRobot(logger=logger, robot_address="192.168.0.100")
    position_file = Path(get_package_share_path("assembly_robot")) / "yaml" / "well_positions.yaml"
    with open(position_file, "r") as f:
        try:
            constant_positions = yaml.safe_load(f)
        except yaml.YAMLError as e:
            print("Cannot load the well positions YAML file with error: ", e)

    assembly_robot_constants = create_assembly_robot_constants_from_manual_positions(constant_positions)
    robot.assemblyRobotConstants = assembly_robot_constants
    camera_position_file = Path(get_package_share_path("assembly_robot")) / "yaml" / "arm_camera_positions.yaml"
    with open(camera_position_file, "r") as f:
        try:
            camera_constant_positions = yaml.safe_load(f)
        except yaml.YAMLError as e:
            print("Cannot load the camera positions YAML file with error: ", e)
    assembly_robot_camera_constants = create_assembly_robot_camera_constants_from_manual_positions(camera_manual_positions=camera_constant_positions)

    prompt= """Press [Enter] to quit, [S] to test component suction,
[M] to finish a cycle of assembly, [C] to take a photo of the desired tray,
[L] to grab a component and move to the lookup camera for a picture.
:> """

    component_prompt = """Which type of component do you want to test? Choose from the following
["CathodeCase", "Cathode", "Separator", "Anode", "Spacer", "AnodeCase"]
:> """
    
    while True:
        input_str = input(prompt).strip().upper()
        if input_str == '':
            break
        elif input_str == 'S':
            component_name = input(component_prompt)
            component = getattr(assembly_robot_constants, component_name)
            grabpos = component[0].grabPo
            railpos = component[0].railPo
            for i in range(0, len(grabpos), 4):
                print(f"reaching the position of well {i}: {grabpos[i]}")
                robot.grab_component(railpos, grabpos[i], is_grab=True)
                robot.grab_component(railpos, grabpos[i], is_grab=False)
        elif input_str == 'C':
            component_name = input(component_prompt)
            robot.take_a_tray_photo(component_name)
        elif input_str == 'M':
            component_name = input(component_prompt)
            component = getattr(assembly_robot_constants, component_name)
            grabpos = component[0].grabPo
            railpos = component[0].railPo
            robot.grab_component(railpos, grabpos[0], is_grab=True)
            robot.grab_component(assembly_robot_constants.POST_RAIL_LOCATION, assembly_robot_constants.POST_C_SK_PO, is_grab=False)
        elif input_str == 'L':
            component_name = input(component_prompt)
            component = getattr(assembly_robot_constants, component_name)
            grabpos = component[0].grabPo
            railpos = component[0].railPo
            robot.grab_component(railpos, grabpos[0], is_grab=True)
            image = robot.take_a_look_up_photo()
            robot.grab_component(railpos, grabpos[0], is_grab=False)

    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()