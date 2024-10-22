from .dobot.dobot_api import DobotApi, DobotApiDashboard, DobotApiMove
from ..helper.Logger import Logger
from .SartoriusRLine import SartoriusRLine
from ..helper.utils import get_proper_port_for_device, SupportedDevices
from ..helper.utils import get_m_n_well_pos
import yaml
from pathlib import Path
from typing import List
import time

class MG400():
    def __init__(self, logger = None, log_path="logs", logger_filename="MG400.log",
                 ip="192.168.0.107", dashboardPort=29999, movePort=30003, feedPort=30004, sartorius_rline = None,
                 mg400_position_file = Path(__file__).parent.parent / "configs" / "MG400positions.yaml"):
        self.ip = ip
        self.dashboardPort = dashboardPort
        self.movePort = movePort
        self.feedPort = feedPort
        self.logger = Logger("MG400", log_path=log_path, logger_filename=logger_filename) if logger is None else logger
        self.dashboard = None
        self.movectl = None
        self.feed = None
        self.sartorius_rline = sartorius_rline if sartorius_rline is not None else SartoriusRLine(port=get_proper_port_for_device(SupportedDevices.SartoriusRLine), logger=self.logger)
        self.position_file = mg400_position_file
        self.home :List[float]= [90, 0, 0, 0]
        self.tip_poses_up: List[List[float]] = []
        self.tip_poses_down: List[List[float]] = []
        self.liquid_poses_up: List[List[float]] = []
        self.liquid_poses_down: List[List[float]] = []
        self.assembly_pose_down: List[float] = []
        self.assembly_pose_up: List[float] = []
        self.liquid_m = None
        self.liquid_n = None
        self.tip_m = None
        self.tip_n = None
    
    def intialize_robot(self) -> bool:
        try:
            self.dashboard = DobotApiDashboard(ip=self.ip, port=self.dashboardPort)
            self.movectl = DobotApiMove(ip=self.ip, port=self.movePort)
            self.feed = DobotApi(ip=self.ip, port=self.feedPort)
            self.logger.info("Created Dobot API control panel!")
        except Exception as e:
            self.logger.error("Cannot connect to the MG400, error: ", e)
            return False
        
        try:
            self.parse_position_file()
            self.logger.info("Finished parsing the MG400 position file!")
        except Exception as e:
            self.logger.error("Failed to load the config file for MG400, error:", e)
            return False
        self.dashboard.EnableRobot(0.145, 50, 0, 0)
        self.logger.info("Finished enabling the robot, initialization succeeded!")
        return True

    def disconnect(self):
        self.dashboard.DisableRobot()

    def move_home(self):
        self.logger.debug("The MG400 is homing itself!")
        self.movectl.JointMovJ(*self.home)
        self.movectl.Sync()
        self.logger.debug("The MG400 has been homed!")

    def move_to_tip_case(self, x, y, level = 1):
        # The level is a percentage of the height, 0 will be at the down pos, 1 will be at the up pos
        self.dashboard.Tool(index=0)
        self.dashboard.SpeedJ(10)
        self.dashboard.SpeedL(3)
        up_pos = self.tip_poses_up[self.get_tip_index(x, y)]
        down_pos = self.tip_poses_down[self.get_tip_index(x, y)]
        move_pos = [down_pos[0], down_pos[1], (up_pos[2]-down_pos[2]) * level + down_pos[2], down_pos[3]]
        self.movectl.MovJ(*up_pos)
        self.movectl.Sync()
        if level < 1:
            self.movectl.MovL(*move_pos)
            self.movectl.Sync()
        self.logger.info(f"finished moving to tipcase at ({x}, {y}).")

    def get_tip(self, x, y):
        self.move_to_tip_case(x, y)
        self.dashboard.SpeedL(3)
        self.movectl.MovL(*self.tip_poses_down[self.get_tip_index(x, y)])
        self.movectl.Sync()
        self.movectl.MovL(*self.tip_poses_up[self.get_tip_index(x, y)])
        self.movectl.Sync()
        self.sartorius_rline.aspirate(10)
        self.logger.info(f"finished getting the tip at ({x}, {y}).")
        self.move_home()

    def drop_tip(self, x, y):
        self.move_to_tip_case(x, y, 0.4)
        self.movectl.Sync()
        self.sartorius_rline.eject_and_home()
        self.move_to_tip_case(x, y)
        self.movectl.Sync()
        self.logger.info(f"The tip should have been ejected")
        self.move_home()

    def move_to_assemble_post(self):
        self.dashboard.SpeedJ(10)
        self.movectl.JointMovJ(-90, 0, 0, 0)
        self.movectl.Sync()
        self.movectl.MovJ(*self.assembly_pose_up)
        self.movectl.Sync()

    def move_to_liquid(self, x, y):
        self.dashboard.Tool(index=0)
        self.dashboard.SpeedJ(10)
        self.movectl.MovJ(*self.liquid_poses_up[self.get_liquid_index(x, y)])
        self.movectl.Sync()
        self.logger.info(f"finished moving for liquid bottle ({x}, {y}).")

    def get_liquid(self, x, y, volume):
        self.move_to_liquid(x, y)
        self.dashboard.SpeedL(3)
        self.movectl.MovL(*self.liquid_poses_down[self.get_liquid_index(x,y)])
        self.movectl.Sync()
        # TODO: level sensing and ensure the liquid is enough
        # self.logger.info(f"The current liquid level: {self.sartorius_rline.tellLevel()}")
        self.sartorius_rline.aspirate(volume)
        self.movectl.MovL(*self.liquid_poses_up[self.get_liquid_index(x, y)])
        self.movectl.Sync()
        self.move_home()

    def return_liquid(self, x, y):
        self.move_to_liquid(x, y)
        self.dashboard.SpeedL(3)
        self.movectl.MovL(*self.liquid_poses_down[self.get_liquid_index(x, y)])
        self.movectl.Sync()
        self.sartorius_rline.blowout()
        self.movectl.MovL(*self.liquid_poses_up[self.get_liquid_index(x, y)])
        self.movectl.Sync()
        self.move_home()

    def add_liquid_to_post(self, volume):
        self.move_to_assemble_post()
        self.dashboard.SpeedL(3)
        self.movectl.MovL(*self.assembly_pose_down)
        self.movectl.Sync()
        self.sartorius_rline.dispense(volume)
        self.movectl.MovL(*self.assembly_pose_up)
        self.movectl.Sync()
        self.move_home()

    def get_tip_index(self, x, y):
        return x * self.tip_n + y
    
    def get_liquid_index(self, x, y):
        return x * self.liquid_n + y

    def parse_position_file(self):
        with open(self.position_file) as f:
            config = yaml.safe_load(f)
        self.home = config["Home"]
        tipcase = config["TipCase"]
        m = int(tipcase["m"])
        n = int(tipcase["n"])
        self.tip_m = m
        self.tip_n = n

        self.tip_poses_down = get_m_n_well_pos(tipcase["bottom_left"]["down"],
                         tipcase["bottom_right"]["down"],
                         tipcase["top_left"]["down"],
                         tipcase["top_right"]["down"], m, n, "mg400-tip-pose-up")
        self.tip_poses_up = get_m_n_well_pos(tipcase["bottom_left"]["up"],
                         tipcase["bottom_right"]["up"],
                         tipcase["top_left"]["up"],
                         tipcase["top_right"]["up"], m, n, "mg400-tip-pose-down")
        
        liquid = config["Liquid"]
        m = int(liquid["m"])
        n = int(liquid["n"])
        self.liquid_m = m
        self.liquid_n = n
        self.liquid_poses_down = get_m_n_well_pos(liquid["bottom_left"]["down"],
                         liquid["bottom_right"]["down"],
                         liquid["top_left"]["down"],
                         liquid["top_right"]["down"], m, n, "mg400-liquid-pose-down")
        self.liquid_poses_up = get_m_n_well_pos(liquid["bottom_left"]["up"],
                         liquid["bottom_right"]["up"],
                         liquid["top_left"]["up"],
                         liquid["top_right"]["up"], m, n, "mg400-liquid-pose-up")
        
        self.assembly_pose_up = config["AssemblyPost"]["prepare_location"]
        self.assembly_pose_down = config["AssemblyPost"]["drop_location"]

def manual_position_loop(mg400: MG400):
    mode = input("do you want to drive in joints (J) or cartesian (C)? Type in J or C: ")
    if mode == 'J':
        parameters_str = input("Please type in the 4 joints [J1, J2, J3, J4]:")
    elif mode =='C':
        parameters_str = input("Please type in the 4 cartesian coordinates: [X, Y, Z, R]:")
    else:
        print("The mode you select does not exist! Please select J or C!")
        return

    parameters_str = parameters_str.strip('[]')
    parameters = [float(x) for x in parameters_str.split(',')]
    if mode == 'J':
        print(f"The robot is moving with JointMovJ to {parameters}")
        mg400.movectl.JointMovJ(*parameters)
    elif mode == 'C':
        print(f"The robot is moving with MovJ to cartesian coordinates {parameters}")
        mg400.movectl.MovJ(*parameters)
    

def main_loop(mg400:MG400):
    prompt="""Press [Enter] to quit, [0] to home the robot, [M] to drive to tip case/liquid case,
[G] to get tip at tipcase case(x,y), [A] to get liquid at liquid case (x,y) with volume, [D] to return tip to tipcase (x,y),
[R] to return liquid to liquidcase(x,y), [J] to dispense liquid with volume to the post.
[Z] to enter manual positioning mode.
:> 
"""
    try:
        while True:
            input_str = input(prompt).strip().upper()
            if input_str == '':
                break
            elif input_str == 'Z':
                manual_position_loop(mg400)
            elif input_str == '0':
                mg400.move_home()
            elif input_str == 'M':
                choice = input("Please select which case to go (tip/liquid):")
                if choice == 'tip':
                    x = int(input("Please input tip index x:").strip())
                    y = int(input("Please input tip index y:").strip())
                    mg400.move_to_tip_case(x, y)
                elif choice == 'liquid':
                    x = int(input("Please input liquid index x:").strip())
                    y = int(input("Please input liquid index y:").strip())
                    mg400.move_to_liquid(x, y)
                else:
                    print("Your choice is invalid!")
            elif input_str == 'G':
                x = int(input("Please input tip index x:").strip())
                y = int(input("Please input tip index y:").strip())
                mg400.get_tip(x, y)
            elif input_str == 'D':
                x = int(input("Please input tip index x:").strip())
                y = int(input("Please input tip index y:").strip())
                mg400.drop_tip(x, y)
            elif input_str == 'R':
                x = int(input("Please input tip index x:").strip())
                y = int(input("Please input tip index y:").strip())
                mg400.return_liquid(x, y)
            elif input_str == 'J':
                volume = int(input("Please input volume:").strip())
                mg400.add_liquid_to_post(volume)
            elif input_str == 'A':
                x = int(input("Please input tip index x:").strip())
                y = int(input("Please input tip index y:").strip())
                volume = int(input("Please input volume:").strip())
                mg400.get_liquid(x, y, volume)
            else:
                print("Invalid input. Please enter a valid option.")
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        mg400.disconnect()
        print("MG400 disconnected safely.")

def mg400_example():
    mg400 = MG400(ip="192.168.0.107")
    ok = mg400.intialize_robot()
    if not ok:
        print("Failed to initialize MG400, program aborted!")
        exit()
    mg400.move_home()
    main_loop(mg400)