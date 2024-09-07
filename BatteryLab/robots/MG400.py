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
                 ip="192.168.0.107", dashboardPort=29999, movePort=30003, feedPort=30004,
                 mg400_position_file = Path(__file__).parent.parent / "configs" / "MG400positions.yaml"):
        self.ip = ip
        self.dashboardPort = dashboardPort
        self.movePort = movePort
        self.feedPort = feedPort
        self.logger = Logger("MG400", log_path=log_path, logger_filename=logger_filename) if logger is None else logger
        self.dashboard = None
        self.movectl = None
        self.feed = None
        self.sartorius_rline = SartoriusRLine(port=get_proper_port_for_device(SupportedDevices.SartoriusRLine), logger=self.logger)
        self.position_file = mg400_position_file
        self.home :List[float]= [90, 0, 0, 0]
        self.well_poses: List[List[float]] = []
    
    def intialize_robot(self) -> bool:
        try:
            self.dashboard = DobotApiDashboard(ip=self.ip, port=self.dashboardPort)
            self.movectl = DobotApiMove(ip=self.ip, port=self.movePort)
            self.feed = DobotApi(ip=self.ip, port=self.feedPort)
        except Exception as e:
            self.logger.error("Cannot connect to the MG400, error: ", e)
            return False
        
        # try:
        #     self.parse_position_file()
        # except Exception as e:
        #     self.logger.error("Failed to load the config file for MG400, error:", e)
        #     return False
        self.dashboard.EnableRobot()
        return True

    def disconnect(self):
        self.dashboard.DisableRobot()

    def move_home(self):
        self.movectl.JointMovJ(*self.home)

    def move_to_tip_case(self, x, y):
        down_pos = self.well_poses[x][y]
        upper_pos = [down_pos[0], down_pos[1], down_pos[2] + 30, down_pos[3]]
        self.movectl.MovJ(*upper_pos)
        self.movectl.wait_reply()
        self.movectl.MovL(*down_pos)
        self.movectl.wait_reply()
        self.movectl.MovL(*upper_pos)
        self.movectl.wait_reply()
        self.logger.info(f"finished moving to tipcase at ({x}, {y}).")

    def get_tip(self):
        pass

    def drop_tip(self):
        pass

    def move_to_assemble_post(self):
        pass

    def add_electrolyte(self, quantity):
        pass

    def parse_position_file(self):
        with open(self.position_file) as f:
            config = yaml.safe_load(f)
        self.home = config["Home"]["joints"]
        tipcase = config["TipCase"]
        m = int(tipcase["m"])
        n = int(tipcase["n"])
        self.well_poses = get_m_n_well_pos(tipcase["bottom_left"]["cartesian"],
                         tipcase["bottom_right"]["cartesian"], tipcase["top_left"]["cartesian"], tipcase["top_right"]["cartesian"], m, n)


def main_loop(mg400):
    try:
        while True:
            input_str = input("Press [Enter] to quit, [0] to home the robot, [M] to drive to tip case: ").strip().lower()
            if input_str == '':
                break
            elif input_str == '0':
                mg400.move_home()
            elif input_str == 'm':
                x = int(input("Please input tip index x:").strip())
                y = int(input("Please input tip index y:").strip())
                mg400.move_to_tip_case(x, y)
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