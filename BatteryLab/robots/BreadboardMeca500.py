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
            with open(crimper_robot_constants_config_file, 'r') as file:
                yaml_data = yaml.safe_load(file)
                self.crimperRobotConstants = CrimperRobotConstants(**yaml_data)
        except Exception as e:
            self.logger.error("Cannot load the crimper robot constants: ", e)
            print("Program will exit because it cannot load crimper robot constants")
            exit()

    def setup_for_pick_up(self):
        self.robot.SetTrf(*self.crimperRobotConstants.TRF)
        self.robot.SetGripperForce(self.RobotConstants.GRIP_F)
        self.robot.SetGripperVel(self.RobotConstants.GRIP_VEL)
    
    def pick_up_from_assembly_post(self):
        self.setup_for_pick_up()
        self.move_home(tool=RobotTool.SUCTION)
        self.robot.GripperOpen()
        self.robot.WaitGripperMoveCompletion(5)
        self.robot.MovePose(*self.crimperRobotConstants.PostReadyPose)
        self.robot.WaitIdle(30)
        self.robot.SetCartLinVel(20)
        self.robot.MoveLin(*self.crimperRobotConstants.PostDownPose)
        self.robot.Delay(0.5)
        self.robot.MoveLin(*self.crimperRobotConstants.GrabReadyPose)
        self.robot.Delay(0.5)
        self.robot.GripperClose()
        self.robot.WaitGripperMoveCompletion(5)
        self.robot.Delay(0.5)
        self.robot.MoveLin(*self.crimperRobotConstants.GrabbedUpPose)
        self.robot.Delay(0.5)
        # Suction is used for 0 joints in Crimper Robot
        self.move_home(tool=RobotTool.SUCTION)
    
    def drop_to_crimper(self):
        self.robot.SetTrf(*self.crimperRobotConstants.TRF)
        self.robot.MovePose(*self.crimperRobotConstants.CrimperReadyToOperatePose)
        self.robot.WaitIdle(30)
        self.robot.SetCartLinVel(20)
        self.robot.MoveLin(*self.crimperRobotConstants.CrimperDropPose)
        self.robot.WaitIdle(30)
        self.robot.GripperOpen()
        self.robot.WaitGripperMoveCompletion(5)
        self.robot.Delay(1)
        self.robot.MoveLin(*self.crimperRobotConstants.CrimperReadyToOperatePose)
        self.robot.WaitIdle(20)

    def pick_up_from_crimper(self):
        self.robot.SetTrf(*self.crimperRobotConstants.TRF)
        self.robot.GripperOpen()
        self.robot.WaitGripperMoveCompletion(5)
        self.robot.MovePose(*self.crimperRobotConstants.CrimperReadyToOperatePose)
        self.robot.WaitIdle(20)
        self.robot.SetCartLinVel(20)
        self.robot.MoveLin(*self.crimperRobotConstants.CrimperReadyToPickPose)
        self.robot.WaitIdle(20)
        self.robot.MoveLin(*self.crimperRobotConstants.CrimperPickPressPose)
        self.robot.WaitIdle(20)
        self.robot.MoveLin(*self.crimperRobotConstants.CrimperPickPose)
        self.robot.WaitIdle(20)
        self.robot.GripperClose()
        self.robot.WaitGripperMoveCompletion(5)
        self.robot.MoveLin(*self.crimperRobotConstants.CrimperPickedUpPose)
        self.robot.WaitIdle(10)
        self.robot.MoveLin(*self.crimperRobotConstants.CrimperReadyToOperatePose)
        self.robot.WaitIdle(10)
        self.move_home(tool=RobotTool.SUCTION)

    def put_to_storage(self):
        self.robot.SetTrf(*self.crimperRobotConstants.TRF)
        self.robot.MovePose(*self.crimperRobotConstants.StorageReadyPose)
        self.robot.WaitIdle(20)
        self.robot.SetCartLinVel(20)
        self.robot.MoveLin(*self.crimperRobotConstants.StorageDropPose)
        self.robot.WaitIdle(20)
        self.robot.GripperOpen()
        self.robot.WaitGripperMoveCompletion(5)
        self.robot.Delay(0.5)
        self.move_home(tool=RobotTool.SUCTION)
    
    def move_for_photo_check(self):
        self.robot.SetTrf(*self.crimperRobotConstants.TRF)
        self.robot.MovePose(*self.crimperRobotConstants.PhotoCheckPreparePose)
        self.robot.WaitIdle(20)
        self.robot.SetCartLinVel(30)
        self.robot.MoveLin(*self.crimperRobotConstants.PhotoCheckPose)
        self.robot.WaitIdle(20)

    def move_away_from_photo_check(self):
        self.robot.SetTrf(*self.crimperRobotConstants.TRF)
        self.robot.MovePose(*self.crimperRobotConstants.PhotoCheckPreparePose)
        self.robot.WaitIdle(20)
        self.move_home(tool=RobotTool.SUCTION)


def breadboard_meca500_example_app():
    robot_address = "192.168.0.101"
    user_provided_address = input(f"Please type in the IP address for the robot (the default is {robot_address}, press enter to use the default): ")
    if user_provided_address != '':
        robot_address = user_provided_address
    meca500_robot = BreadBoardMeca500(robot_address=robot_address)
    meca500_robot.initializeRobot()
    
    prompt= """Press [Enter] to quit, [P] to pick up from the post,
[C] to drop the battery to the crimper, [D] to pick the battery up from the crimper
[S] to store the battery in the storage post. [A] to run the whole process
:> """

    while True:
        input_str = input(prompt).strip().upper()
        if input_str == '':
            break
        elif input_str == 'P':
            meca500_robot.pick_up_from_assembly_post()
        elif input_str == 'C':
            meca500_robot.drop_to_crimper()
        elif input_str == 'D':
            meca500_robot.pick_up_from_crimper()
        elif input_str == 'S':
            meca500_robot.put_to_storage()
        elif input_str == 'A':
            meca500_robot.pick_up_from_assembly_post()
            meca500_robot.drop_to_crimper()
            meca500_robot.pick_up_from_crimper()
            meca500_robot.put_to_storage()
        else:
            print("The command you gave is not recognized!")
    
    meca500_robot.move_home(tool=RobotTool.GRIPPER)
    meca500_robot.exitRobot()