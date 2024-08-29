from mecademicpy.robot import Robot
from mecademicpy import tools

import mecademicpy.robot as mdr
from ..helper.Logger import Logger

from .Constants import Meca500RobotConstants, RobotTool
import yaml
from pathlib import Path

class Meca500():

    def __init__(self, logger = None, log_path="logs", logger_filename="Meca500.log", robot_address="192.168.0.101",
                 robot_constants_config_file=Path(__file__).parent.parent / "configs" / "BreadBoardMeca500.yaml", robot_name="Meca500"):
        try:
            with open(robot_constants_config_file, 'r') as file:
                yaml_data = yaml.safe_load(file)
                self.RobotConstants = Meca500RobotConstants(**yaml_data)
        except Exception as e:
            self.logger.error("Cannot load the robot constants: ", e)
            print("Program will exit because it cannot load robot constants")
            exit()

        self.robot = Robot()
        self.robot_address = robot_address
        self.logger = Logger("Meca500", log_path=log_path, logger_filename=logger_filename) if logger is None else logger

        self.robot_name = robot_name
    
    def __del__(self):
        if self.robot is not None:
            self.logger.info("Exit the robot in __del__ function")
            self.exitRobot()
    
    def exitRobot(self):
        self.robot.DeactivateRobot()
        self.robot.Disconnect()
        self.robot = None
        self.logger.info("The robot is deactivated and disconnected!")

    def initializeRobot(self) -> bool:
        try:
            self.logger.info(f"Trying to connect to Meca500 with IP Address: {self.robot_address}")
            self.robot.Connect(address=self.robot_address)
        except mdr.CommunicationError as e:
            self.logger.info(f"Robot failed to connect. Check if the IP address {self.robot_address} is correct! Error: {e}")
            return False
        
        try:
            self.logger.info(f"Activating and homing the {self.robot_name} robot!")
            self.robot.ActivateRobot()
            self.robot.Home()
            self.robot.WaitHomed()
            self.logger.info(f"{self.robot_name} is homed and ready!")
            if tools.robot_model_is_meca500(self.robot.GetRobotInfo().robot_model):
                self.logger.info("The robot is confirmed to be Meca500!")
                # TODO: Set Gripper Force, Joint Velocity and other parameters
                self.robot.SetGripperForce(self.RobotConstants.GRIP_F)
                self.robot.SetGripperVel(self.RobotConstants.GRIP_VEL)
                self.robot.SetCartLinVel(self.RobotConstants.L_VEL)
                self.robot.SetJointVel(self.RobotConstants.J_VEL)
                self.robot.SetJointAcc(20)
                self.robot.MoveJoints(self.RobotConstants.HOME_SK_J)
                self.logger.info(f"The {self.robot_name} is initialized!")
        except Exception as exception:
            if self.robot.GetStatusRobot().error_status:
                self.logger.info(exception)
                self.logger.info("Robot has encountered an error, attempting to clear...")
                self.robot.ResetError()
                self.robot.ResumeMotion()
            else:
                raise

    def move_home(self, tool: RobotTool):
        self.logger.debug(f"{self.robot_name} moving to home with tool {tool.name}")
        if tool == RobotTool.SUCTION:
            self.robot.MoveJoints(*self.RobotConstants.HOME_SK_J)
        elif tool == RobotTool.GRIPPER:
            self.robot.MoveJoints(*self.RobotConstants.HOME_GP_J)
        self.robot.WaitIdle(30)

    def move_for_snapshot(self):
        self.robot.MovePose(*self.RobotConstants.SNAP_SHOT_GRAB_PO)
        self.robot.WaitIdle(30)

    def draw_square(self):
        """For development purpose only, don't use in production"""
        try:
            self.robot.MovePose(200, 0, 300, 0, 90, 0)
            self.robot.MovePose(200, 100, 300, 0, 90, 0)
            self.robot.MovePose(200, 100, 100, 0, 90, 0)
            self.robot.MovePose(200, -100, 100, 0, 90, 0)
            self.robot.MovePose(200, -100, 300, 0, 90, 0)
            self.robot.MovePose(200, 0, 300, 0, 90, 0)
            self.logger.info('Commands for drawing a square sent. Robot should now be moving.')
            self.robot.Delay(1)
            self.robot.MoveJoints(0, -60, 60, 0, 0, 0)
            self.logger.info('Waiting for robot to finish moving...')
            self.robot.WaitIdle(60)
            self.logger.info('Robot finished drawing square.')
        except Exception as e:
            self.logger.info(f"Drawing square has an error: {e}")

def test_config_file_location():
    file_path = Path(__file__).parent.parent / "configs" / "BreadBoardMeca500.yaml"
    print(file_path)