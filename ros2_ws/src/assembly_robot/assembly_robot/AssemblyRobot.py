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
import cv2
from datetime import datetime

from ament_index_python.packages import get_package_share_path
from camera_service.camera_client import ImageClient
from suction_pump.suction_client import SuctionPumpClient

class AssemblyRobot(Node):

    def __init__(self, logger=None, robot_address="192.168.0.100"):
        super().__init__('assembly_robot')
        self.suction_pump_client = SuctionPumpClient()
        self.logger = self.get_logger() if logger is None else logger
        self.rail_meca500 = RailMeca500(logger=self.logger, robot_address=robot_address, suction_pump=self.suction_pump_client)
        # self.status = dict(Progress=dict(Initiate=0, LastStep=None), Meca500Ready=False, ZaberRailReady=False)
        
        self.zaber_rail = LinearRailClient()
        self.assemblyRobotConstants = AssemblyRobotConstants()
        self.assemblyRobotCameraConstants = AssemblyRobotCameraConstants()
        self.auto_correction = AutoCorrection(logger=self.logger)
        self.look_up_camera_client = ImageClient(node_name="assembly_robot_lookup_camera_client", serv_name="/batterylab/lookup_camera")
        self.arm_camera_client = ImageClient(node_name="assembly_robot_arm_camera_client", serv_name="/batterylab/rail_meca500_camera")

    def initialize_and_home_robots(self):
        self.load_position_files()
        ok = self.rail_meca500.initializeRobot()
        if not ok:
            print("The Meca500 cannot be connected")
            exit()

    def move_home_and_out_of_way(self):
        self.rail_meca500.move_home(tool=RobotTool.SUCTION)
        self.move_zaber_rail(0.0)

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
        Trf = self.assemblyRobotConstants.TRF
        self.move_zaber_rail(rail_pos)
        self.rail_meca500.robot.SetTrf(*Trf)
        self.rail_meca500.robot.MovePose(*robot_pos)
        self.rail_meca500.robot.WaitIdle(30)
        self.rail_meca500.robot.Delay(0.2)
        # TODO: use get_image for storing the image or analysis
        self.look_up_camera_client.send_request()
        rclpy.spin_until_future_complete(self.look_up_camera_client, self.look_up_camera_client.future)
        # self.look_up_camera_client.display_image()
        image = self.look_up_camera_client.get_image()
        return image     
        
    def take_a_tray_photo(self, component_name: str):
        rail_pos = getattr(self.assemblyRobotCameraConstants, component_name)
        Trf = self.assemblyRobotCameraConstants.TRF
        robot_pos = self.assemblyRobotCameraConstants.RobotPose
        self.move_zaber_rail(rail_pos)
        self.rail_meca500.robot.SetTrf(*Trf)
        print(f"To take a photo, moving to Robot Pos {robot_pos}")
        self.rail_meca500.robot.MovePose(*robot_pos)
        self.rail_meca500.robot.WaitIdle(30)
        self.rail_meca500.robot.Delay(0.2)
        # TODO: use get_image for storing the image or analysis
        self.arm_camera_client.send_request()
        rclpy.spin_until_future_complete(self.arm_camera_client, self.arm_camera_client.future)
        # self.arm_camera_client.display_image()
        return self.arm_camera_client.get_image()

    def grab_component(self, rail_position, grab_position, is_grab=True, component_name="suction"):
        """Grab a component for battery or return it to the tray.
        This requires a cooperation between the rail and the Meca500 robotic arm on top of it.
        """
        # Move home based on the tooling
        if component_name == 'Washer':
            self.rail_meca500.change_tool(tool_name=RobotTool.GRIPPER)
            self.rail_meca500.move_home(tool=RobotTool.GRIPPER)
        else:
            self.rail_meca500.change_tool(tool_name=RobotTool.SUCTION)
            self.rail_meca500.move_home(tool=RobotTool.SUCTION)
        
        # Move the Zaber Rail
        self.move_zaber_rail(rail_position)
        self.logger.debug(f"Assembly Robot will start picking soon.")
        # Let Meca500 pick up the component and move it home
        self.rail_meca500.pick_place(grab_position, is_grab=is_grab)
    
    def load_position_files(self):
        position_file = Path(get_package_share_path("assembly_robot")) / "yaml" / "well_positions.yaml"
        with open(position_file, "r") as f:
            try:
                constant_positions = yaml.safe_load(f)
            except yaml.YAMLError as e:
                print("Cannot load the well positions YAML file with error: ", e)

        self.assemblyRobotConstants = create_assembly_robot_constants_from_manual_positions(constant_positions)
        camera_position_file = Path(get_package_share_path("assembly_robot")) / "yaml" / "arm_camera_positions.yaml"
        with open(camera_position_file, "r") as f:
            try:
                camera_constant_positions = yaml.safe_load(f)
            except yaml.YAMLError as e:
                print("Cannot load the camera positions YAML file with error: ", e)
        self.assemblyRobotCameraConstants = create_assembly_robot_camera_constants_from_manual_positions(camera_manual_positions=camera_constant_positions)
    
    def auto_grab_a_component_to_assembly_post(self, component_name: str):
        component = getattr(self.assemblyRobotConstants, component_name)
        available_locations = list(component.keys())
        # take a picutre of the component
        tray_photo = self.take_a_tray_photo(component_name)
        # TODO: deal with the photo to find available components
        sublocation = available_locations[0] # the sublocation is default to 0 for now
        location = component[sublocation]
        auto_well_grab_pos = location.grabPo[0] # the grab well index is default to 0 for now
        rail_position = location.railPo
        # grab the decided component
        self.grab_component(rail_position=rail_position, grab_position=auto_well_grab_pos, is_grab=True, component_name=component_name)
        # move the component to the assembly post
        self.grab_component(self.assemblyRobotConstants.POST_RAIL_LOCATION, self.assemblyRobotConstants.POST_C_SK_PO, is_grab=False, component_name=component_name)

    def drop_component(self, drop_po, component:Components, nr:int, auto_calib:bool=True, grab_check:bool=True, save_img:bool=True, show_image:bool=False):
        """Drop the component to the assembly post with autocorrection"""
        # TODO: add autocorrection on top of grab_component
        pass

def get_component_location_from_user(robot, component_prompt):
    component_name = input(component_prompt)
    component = getattr(robot.assemblyRobotConstants, component_name)
    available_locations = list(component.keys())
    if len(available_locations) == 0:
        print(f"The selected component <{component_name}> has not been manually positioned yet!")
        exit()
    sub_location = input(f"Which corner do you want to test ({available_locations}): ")
    if sub_location not in available_locations:
        print("The sublocation you pick is not valid!")
        exit()
    location = component[sub_location]
    return location.grabPo, location.railPo, sub_location, component_name

def assembly_robot_command_loop(robot: AssemblyRobot, image_path="/home/yuanjian/Research/BatteryLab/images/anode_case_photos"):
    prompt= """Press [Enter] to quit, [S] to test component suction,
[M] to move a component to the assembly post, [C] to take a photo of the desired tray,
[L] to grab a component and move to the lookup camera for a picture.
:> """

    component_prompt = """Which type of component do you want to test? Choose from the following
["CathodeCase", "Cathode", "Separator", "Anode", "Washer", "Spacer", "AnodeCase"]
:> """

    while True:
        input_str = input(prompt).strip().upper()
        if input_str == '':
            break
        elif input_str == 'S':
            grabpos, railpos, sub_location, component_name = get_component_location_from_user(robot, component_prompt)
            step = len(grabpos) / 4
            test_range = [0, step, 2*step, 3*step]
            test_all = input(f"Do you want to test the four corners or test all? (all/corners) default is corners:")
            if test_all == "all":
                test_range = range(0, len(grabpos))
            for i in test_range:
                print(f"reaching the position of well {i}: {grabpos[i]}")
                robot.grab_component(railpos, grabpos[i], is_grab=True, component_name=component_name)
                robot.grab_component(railpos, grabpos[i], is_grab=False, component_name=component_name)
        elif input_str == 'C':
            component_name = input(component_prompt)
            image = robot.take_a_tray_photo(component_name)
            cur_time = datetime.now().strftime("%Y-%m-%d-%H-%M:%S")
            image_file = str(Path(image_path) / f"ArmCam-{component_name}-{cur_time}.jpg")
            cv2.imwrite(image_file, image)
        elif input_str == 'M':
            grabpos, railpos, sub_location, component_name = get_component_location_from_user(robot, component_prompt)
            index = int(input(f"which index do you want the robot to reach for {sub_location}, range [0, {len(grabpos)}):"))
            if index >=0 and index < len(grabpos):
                robot.grab_component(railpos, grabpos[index], is_grab=True, component_name=component_name)
                if component_name == 'Washer':
                    pass
                else:
                    robot.grab_component(robot.assemblyRobotConstants.POST_RAIL_LOCATION, robot.assemblyRobotConstants.POST_C_SK_PO, is_grab=False, component_name=component_name)
            else:
                print("The index you give is not valid for the robot to grab!")
                continue
        elif input_str == 'L':
            grabpos, railpos, sub_location, component_name = get_component_location_from_user(robot, component_prompt)
            index = int(input(f"which index do you want the robot to reach for {sub_location}, range [0, {len(grabpos)}):"))
            if not (index >=0 and index < len(grabpos)):
                print("The index you give is not valid for the robot to grab!")
                continue
            robot.grab_component(railpos, grabpos[index], is_grab=True, component_name=component_name)
            image = robot.take_a_look_up_photo()
            cur_time = datetime.now().strftime("$Y-%m-%d-%H-%M:%S")
            image_file = str(Path(image_path) / f"Lookup-{component_name}-{cur_time}.jpg")
            cv2.imwrite(image_file, image)
            robot.rail_meca500.move_home(tool=RobotTool.SUCTION)
            robot.grab_component(railpos, grabpos[index], is_grab=False, component_name=component_name)

def main():
    rclpy.init()
    log_path = "/home/yuanjian/Research/BatteryLab/logs"
    logger = Logger("assembly_robot_test", log_path, "assembly_robot_test.log")
    robot = AssemblyRobot(logger=logger, robot_address="192.168.0.100")
    robot.initialize_and_home_robots()
    image_path = Path("/home/yuanjian/Research/BatteryLab/images/anode_case_photos")
    image_path.mkdir(exist_ok=True)
    assembly_robot_command_loop(robot, image_path=str(image_path))
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()