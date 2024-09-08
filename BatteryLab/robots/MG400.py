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
    
    def intialize_robot(self) -> bool:
        try:
            self.dashboard = DobotApiDashboard(ip=self.ip, port=self.dashboardPort)
            self.movectl = DobotApiMove(ip=self.ip, port=self.movePort)
            self.feed = DobotApi(ip=self.ip, port=self.feedPort)
        except Exception as e:
            self.logger.error("Cannot connect to the MG400, error: ", e)
            return False
        
        try:
            self.parse_position_file()
        except Exception as e:
            self.logger.error("Failed to load the config file for MG400, error:", e)
            return False
        self.dashboard.EnableRobot()
        return True

    def disconnect(self):
        self.dashboard.DisableRobot()

    def move_home(self):
        self.movectl.JointMovJ(*self.home)

    def move_to_tip_case(self, x, y):
        self.dashboard.Tool(index=0)
        self.dashboard.SpeedJ(10)
        up_pos = self.tip_poses_up[x][y]
        self.movectl.MovJ(*up_pos)
        self.logger.info(f"finished moving to tipcase at ({x}, {y}).")

    def get_tip(self, x, y):
        self.move_to_tip_case(x, y)
        self.dashboard.SpeedL(3)
        self.movectl.MovL(*self.tip_poses_down[x][y])
        time.sleep(5)
        self.movectl.MovL(*self.tip_poses_up[x][y])
        self.logger.info(f"finished getting the tip at ({x}, {y}).")

    def drop_tip(self, x, y):
        self.move_to_tip_case(x, y)
        self.sartorius_rline.eject_and_home()
        self.logger.info(f"The tip should have been ejected")

    def move_to_assemble_post(self):
        self.dashboard.SpeedJ(10)
        self.movectl.JointMovJ(*self.assembly_pose_up)

    def move_to_liquid(self, x, y):
        self.dashboard.Tool(index=0)
        self.dashboard.SpeedJ(10)
        self.movectl.MovJ(*self.liquid_poses_up[x][y])
        self.logger.info(f"finished moving for liquid bottle ({x}, {y}).")

    def get_liquid(self, x, y, volume):
        self.move_to_liquid(x, y)
        self.dashboard.SpeedL(3)
        self.movectl.MovL(*self.liquid_poses_down[x][y])
        time.sleep(3)
        # TODO: level sensing and ensure the liquid is enough
        self.logger.info(f"The current liquid level: {self.sartorius_rline.tellLevel()}")
        self.sartorius_rline.aspirate(volume)
        time.sleep(3)
        self.movectl.MovL(*self.liquid_poses_up[x][y])
        time.sleep(3)

    def return_liquid(self, x, y):
        self.move_to_liquid(x, y)
        self.dashboard.SpeedL(3)
        self.movectl.MovL(*self.liquid_poses_down[x][y])
        time.sleep(3)
        self.sartorius_rline.blowout()

    def add_liquid_to_post(self, volume):
        self.move_to_assemble_post()
        self.dashboard.SpeedL(3)
        self.movectl.MovL(*self.assembly_pose_down)
        self.sartorius_rline.dispense(volume)
        time.sleep(5)
        self.movectl.MovL(*self.assembly_pose_up)

    def parse_position_file(self):
        with open(self.position_file) as f:
            config = yaml.safe_load(f)
        self.home = config["Home"]
        tipcase = config["TipCase"]
        m = int(tipcase["m"])
        n = int(tipcase["n"])

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

def main_loop(mg400:MG400):
    prompt="""Press [Enter] to quit, [0] to home the robot, [M] to drive to tip case,
[G] to get tip at tipcase(x,y), [A] to aspirate liquid with volume, [D] to return tip to tipcase (x,y),
[R] to return liquid to liquidcase(x,y), [J] to dispense liquid with volume to the post.
:> 
"""
    try:
        while True:
            input_str = input(prompt).strip().upper()
            if input_str == '':
                break
            elif input_str == '0':
                mg400.move_home()
            elif input_str == 'M':
                x = int(input("Please input tip index x:").strip())
                y = int(input("Please input tip index y:").strip())
                mg400.move_to_tip_case(x, y) 
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

def mg400_example_debug():
    mg400 = MG400(ip="192.168.0.107")
    ok = mg400.intialize_robot()
    if not ok:
        print("Failed to initialize MG400, program aborted!")
        exit()
    mg400.move_home()
    mg400.dashboard.Tool(index=0)
    mg400.dashboard.SpeedJ(10)
    mg400.dashboard.SpeedL(10)
    up_high_position = [19.88, 282.44, 0.68, 35.36]
    mg400.movectl.MovL(*up_high_position)
    up_position = [19.88, 282.44, -60.68, 35.36]
    mg400.movectl.MovL(*up_position)
    time.sleep(2)
    down_position = [19.88, 282.44, -76.88, 35.36]
    mg400.dashboard.SpeedL(3)
    mg400.movectl.MovL(*down_position)
    time.sleep(5)
    tip_up_position = [19.88, 282.44, -14.28, 35.36]
    mg400.movectl.MovL(*tip_up_position)
    prepare_drain_liquid_position = [6.98, 393.24, -20.28, 35.36]
    mg400.movectl.MovJ(*prepare_drain_liquid_position)
    time.sleep(3)
    mg400.dashboard.SpeedL(3)
    drain_liquid_position = [6.98, 393.24, -72.01, 35.36]
    mg400.movectl.MovL(*drain_liquid_position)
    time.sleep(5)
    mg400.movectl.MovL(*prepare_drain_liquid_position)
    # put the tip back
    assembly_prepare_location = [8.67, -351.02, 58.73, -90.04]
    mg400.movectl.MovJ(*assembly_prepare_location)