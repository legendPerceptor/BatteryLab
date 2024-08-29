from .Constants import RobotTool
from pathlib import Path
from .Meca500 import Meca500

class BreadBoardMeca500(Meca500):

    def __init__(self, logger = None, log_path="logs", logger_filename="Meca500.log", robot_address="192.168.0.101",
                 robot_constants_config_file=Path(__file__).parent.parent / "configs" / "BreadBoardMeca500.yaml"):
        super().__init__(logger, log_path, logger_filename, robot_address, robot_constants_config_file)
    

def breadboard_meca500_example_app():
    robot_address = "192.168.0.101"
    user_provided_address = input(f"Please type in the IP address for the robot (the default is {robot_address}, press enter to use the default): ")
    if user_provided_address != '':
        robot_address = user_provided_address
    meca500_robot = BreadBoardMeca500(robot_address=robot_address)
    meca500_robot.initializeRobot()
    meca500_robot.draw_square()
    meca500_robot.move_home(tool=RobotTool.GRIPPER)
    meca500_robot.exitRobot()