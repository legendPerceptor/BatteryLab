from .dobot.dobot_api import DobotApi, DobotApiDashboard, DobotApiMove
from ..helper.Logger import Logger

class MG400():
    def __init__(self, logger = None, log_path="logs", logger_filename="MG400.log",
                 ip="192.168.0.107", dashboardPort=29999, movePort=30003, feedPort=30004):
        self.ip = ip
        self.dashboardPort = dashboardPort
        self.movePort = movePort
        self.feedPort = feedPort
        self.logger = Logger("Meca500", log_path=log_path, logger_filename=logger_filename) if logger is None else logger
        self.dashboard = DobotApiDashboard(ip=ip, port=dashboardPort)
        self.movectl = DobotApiMove(ip=ip, port=movePort)
        self.feed = DobotApi(ip=ip, port=feedPort)

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