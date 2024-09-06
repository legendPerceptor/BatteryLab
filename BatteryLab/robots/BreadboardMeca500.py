from .Constants import RobotTool, CrimperRobotConstants
from pathlib import Path
from .Meca500 import Meca500
import yaml

class BreadBoardMeca500(Meca500):

    def __init__(self, logger = None, log_path="logs", logger_filename="Meca500.log", robot_address="192.168.0.101",
                 robot_constants_config_file=Path(__file__).parent.parent / "configs" / "BreadBoardMeca500.yaml",
                 crimper_robot_constants_config_file=Path(__file__).parent.parent / "configs" / "CrimperPositions.yaml"):
        super().__init__(logger, log_path, logger_filename, robot_address, robot_constants_config_file)
        try:
            with open(robot_constants_config_file, 'r') as file:
                yaml_data = yaml.safe_load(file)
                self.crimperRobotConstants = CrimperRobotConstants(**yaml_data)
        except Exception as e:
            self.logger.error("Cannot load the crimper robot constants: ", e)
            print("Program will exit because it cannot load crimper robot constants")
            exit()
    
    def pick_up_from_assembly_post(self):
        self.robot.SetTrf(*self.crimperRobotConstants.TRF)
        self.robot.MovePose(*self.crimperRobotConstants.PostReadyPose)
        self.robot.WaitIdle(30)
        self.robot.SetCartLinVel(30)
        self.robot.MoveLin(*self.crimperRobotConstants.PostDownPose)
        self.robot.Delay(0.5)
        self.robot.MoveLin(*self.crimperRobotConstants.GrabReadyPose)
        self.robot.Delay(0.5)
        self.robot.GripperClose()
        self.robot.Delay(0.5)
        self.robot.MoveLin(*self.crimperRobotConstants.GrabbedUpPose)
        self.robot.Delay(0.5)
        # Suction is used for 0 joints in Crimper Robot
        self.move_home(tool=RobotTool.SUCTION)
    
    def drop_to_crimper(self):
        self.robot.SetTrf(*self.crimperRobotConstants.TRF)
        self.robot.MovePose(*self.crimperRobotConstants.CrimperReadyToOperatePose)
        self.robot.WaitIdle(30)
        self.robot.SetCartLinVel(10)
        self.robot.MoveLin(*self.crimperRobotConstants.CrimperDropPose)
        self.robot.Delay(0.5)
        self.robot.GripperOpen()
        self.robot.Delay(1)
        self.robot.MoveLin(*self.crimperRobotConstants.CrimperReadyToOperatePose)
        self.robot.WaitIdle(20)

    def pick_up_from_crimper(self):
        self.robot.SetTrf(*self.crimperRobotConstants.TRF)
        self.robot.MovePose(*self.crimperRobotConstants.CrimperReadyToOperatePose)
        self.robot.WaitIdle(30)
        self.robot.SetCartLinVel(20)
        self.robot.MoveLin(*self.crimperRobotConstants.CrimperReadyToPickPose)
        self.robot.Delay(0.5)
        self.robot.MoveLin(*self.crimperRobotConstants.CrimperPickPressPose)
        self.robot.Delay(0.5)
        self.robot.MoveLin(*self.crimperRobotConstants.CrimperPickPose)
        self.robot.Delay(0.5)
        self.robot.GripperClose()
        self.robot.MoveLin(*self.crimperRobotConstants.CrimperPickedUpPose)
        self.robot.Delay(0.5)
        self.robot.MoveLin(*self.crimperRobotConstants.CrimperReadyToOperatePose)
        self.robot.WaitIdle(10)
        self.move_home(tool=RobotTool.SUCTION)

    def put_to_storage(self):
        self.robot.SetTrf(*self.crimperRobotConstants.TRF)
        self.robot.MovePose(*self.crimperRobotConstants.StorageReadyPose)
        self.robot.WaitIdle(20)
        self.robot.SetCartLinVel(20)
        self.robot.MoveLin(*self.crimperRobotConstants.StorageDropPose)
        self.robot.GripperOpen()
        self.robot.Delay(0.5)
        self.move_home(tool=RobotTool.GRIPPER)


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