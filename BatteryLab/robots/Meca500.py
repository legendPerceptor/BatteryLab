from mecademicpy.robot import Robot
from mecademicpy import tools

import mecademicpy.robot as mdr
from ..helper.Logger import Logger
from ..helper.utils import get_proper_port_for_device, SupportedDevices

from .SuctionPump import SuctionPump
from .Constants import Meca500RobotConstants, RobotTool


class Meca500():
    def __init__(self, logger = None, log_path="logs", logger_filename="Meca500.log", robot_address="192.168.0.101"):
        self.robot = Robot()
        self.robot_address = robot_address
        self.logger = Logger("Meca500", log_path=log_path, logger_filename=logger_filename) if logger is None else logger
        # Status
        self.status = dict(Tool=None, Progress=dict(Initiate=0, LastStep=None), Initiated=False, Vacuumed=False, Grabbed=True, Aborted=False)
        self.suction_pump = SuctionPump(self.logger, self.status, vacuum_port=get_proper_port_for_device(SupportedDevices.SuctionPump))
        self.home = [0, 0, 0, 0, 0, 0]
        # Constants
        self.LIN_SPEED = 50
        self.SLOW_DOWN = 30
        self.RobotConstants = Meca500RobotConstants()

    def __del__(self):
        if self.robot is not None:
            self.logger.info("Exit the robot in __del__ function")
            self.exitRobot()

    def exitRobot(self):
        # self.robot.DeactivateRobot()
        self.robot.Disconnect()
        self.robot = None
        self.logger.info("The robot is deactivated!")

    def initializeRobot(self):
        try:
            self.robot.Connect(address=self.robot_address)
        except mdr.CommunicationError as e:
            self.logger.info(f"Robot failed to connect. Check if the IP address {self.robot_address} is correct! Error: {e}")
            return False
        
        try:
            self.logger.info("Activating and homing the Meca500 robot!")
            self.robot.ActivateRobot()
            self.robot.Home()
            self.robot.WaitHomed()
            self.logger.info("Meca500 is homed and ready!")
            if tools.robot_model_is_meca500(self.robot.GetRobotInfo().robot_model):
                self.logger.info("The robot is confirmed to be Meca500!")
                # TODO: Set Gripper Force, Joint Velocity and other parameters
                self.robot.SetGripperForce(self.RobotConstants.GRIP_F)
                self.robot.SetGripperVel(self.RobotConstants.GRIP_VEL)
                self.robot.SetCartLinVel(self.RobotConstants.L_VEL)
                self.robot.SetJointVel(self.RobotConstants.J_VEL)
                self.robot.SetJointAcc(20)
                self.robot.MoveJoints(0, 0, 0, 0, 0, 0)
                self.logger.info("Assembly Meca500 is initilized!")
        except Exception as exception:
            if self.robot.GetStatusRobot().error_status:
                self.logger.info(exception)
                self.logger.info("Robot has encountered an error, attempting to clear...")
                self.robot.ResetError()
                self.robot.ResumeMotion()
            else:
                raise
        
        ok = self.suction_pump.connect_pump()
        if not ok:
            print("suction pump cannot be connected!")
            self.logger.error("suction pump cannot be connected!")
        else:
            print("suction pump is successfully connected!")
            self.logger.info("suction pump is successfully connected!")

    def change_tool(self, tool_name: RobotTool):
        if tool_name == RobotTool.GRIPPER:
            self.robot.SetTRF(*self.RobotConstants.TCP_GP)
            self.status["Tool"] = RobotTool.GRIPPER
            self.home = self.RobotConstants.HOME_GP_J
        elif tool_name == RobotTool.SUCTION:
            self.robot.SetTRF(*self.RobotConstants.TCP_SK)
            self.status["Tool"] = RobotTool.SUCTION
            self.home = self.RobotConstants.HOME_SK_J
    
    def move_home(self):
        self.robot.MoveJoints(*self.home)
        self.robot.WaitIdle(30)

    def move_for_snapshot(self):
        self.robot.MovePose(*self.RobotConstants.SNAP_SHOT_GRAB_PO)
        self.robot.WaitIdle(30)

    def smart_grab(self):
        if self.status["Tool"] == RobotTool.GRIPPER:
            self.robot.GripperClose()
            self.robot.WaitGripperMoveCompletion()
        elif self.status["Tool"] == RobotTool.SUCTION:
            self.suction_pump.pick()
    
    def smart_drop(self):
        if self.status["Tool"] == RobotTool.GRIPPER:
            self.robot.GripperOpen()
            self.robot.WaitGripperMoveCompletion()
        elif self.status["Tool"] == RobotTool.SUCTION:
            self.suction_pump.drop()

    def move_after_drop(self):
        self.robot.MoveJoints(*self.RobotConstants.HOME_POST_J)
        self.robot.MoveJoints(*self.RobotConstants.HOME_SK_J)

    def pick_place(self, grab_pos, is_grab = True):
        self.robot.MovePose(grab_pos[0], grab_pos[1], grab_pos[2] + 20, grab_pos[3], grab_pos[4], grab_pos[5])
        # Linearly moving down to grab the component and go back
        self.robot.Delay(0.5)
        self.robot.SetCartLinVel(self.SLOW_DOWN)
        self.robot.MoveLin(*grab_pos)
        self.robot.Delay(0.5)
        if is_grab:
            self.smart_grab()
        else:
            self.smart_drop()
        self.robot.Delay(0.2)
        self.robot.MoveLin(grab_pos[0], grab_pos[1], grab_pos[2], grab_pos[3], grab_pos[4], grab_pos[5])
        self.robot.Delay(0.5)
        # Move the component back to home
        self.robot.SetCartLinVel(self.LIN_SPEED)
        self.move_home()
        self.robot.WaitIdle()
        
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

def meca500_example_app():
    robot_address = "192.168.0.101"
    user_provided_address = input(f"Please type in the IP address for the robot (the default is {robot_address}, press enter to use the default): ")
    if user_provided_address != '':
        robot_address = user_provided_address
    meca500_robot = Meca500(robot_address=robot_address)
    meca500_robot.initializeRobot()
    meca500_robot.draw_square()
    meca500_robot.move_home()
    meca500_robot.exitRobot()