from .dobot.dobot_api import DobotApi, DobotApiDashboard, DobotApiMove
from ..helper.Logger import Logger
from .SartoriusRLine import SartoriusRLine
from ..helper.utils import get_proper_port_for_device, SupportedDevices
from ..helper.utils import get_m_n_well_pos
import yaml
from pathlib import Path
from typing import List

class MG400():
    def __init__(self, logger = None, log_path="logs", logger_filename="MG400.log",
                 ip="192.168.0.107", dashboardPort=29999, movePort=30003, feedPort=30004,
                 mg400_position_file = Path(__file__).parent.parent / "configs" / "MG400positions.yaml"):
        self.ip = ip
        self.dashboardPort = dashboardPort
        self.movePort = movePort
        self.feedPort = feedPort
        self.logger = Logger("MG400", log_path=log_path, logger_filename=logger_filename) if logger is None else logger
        self.dashboard = DobotApiDashboard(ip=ip, port=dashboardPort)
        self.movectl = DobotApiMove(ip=ip, port=movePort)
        self.feed = DobotApi(ip=ip, port=feedPort)
        self.sartorius_rline = SartoriusRLine(port=get_proper_port_for_device(SupportedDevices.SartoriusRLine), logger=self.logger)
        self.position_file = mg400_position_file
        self.home :List[float]= []
        self.well_poses: List[List[float]] = []

    def stand_by(self):
        pass

    def move_to_tip_case(self, x, y):
        pass

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

def mg400_example():
    position_file = Path(__file__).parent.parent / "configs" / "MG400positions.yaml"
    with open(position_file) as f:
            config = yaml.safe_load(f)
    home = config["Home"]["joints"]
    tipcase = config["TipCase"]
    well_poses = get_m_n_well_pos(tipcase["bottom_left"]["cartesian"],
                         tipcase["bottom_right"]["cartesian"], tipcase["top_left"]["cartesian"], tipcase["top_right"]["cartesian"], 8, 12)
    print("Home:", home)
    print("well_poses:", well_poses)
