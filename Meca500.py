from mecademicpy.robot import Robot
from mecademicpy import tools

import mecademicpy.robot as mdr
from Logger import Logger
from SuctionPump import SuctionPump

from enum import Enum

class RobotTool(Enum):
    GRIPPER = 1
    SUCTION = 2

class Meca500():

    def __init__(self, logger = None, log_path="./Meca500.log", robot_address="192.168.0.101", vacuum_port="COM7"):
        self.robot = Robot()
        self.robot_address = robot_address
        self.logger = Logger("Meca500", log_path=log_path) if logger is None else logger

        # Status
        self.status = dict(Tool=None, Progress=dict(Initiate=0, LastStep=None), Initiated=False, Vacuumed=False, Grabbed=True, Aborted=False)
        self.home = [0, 0, 0, 0, 0, 0]
        self.suction_pump = SuctionPump(self.logger, self.status, vacuum_port)
        # Constants
        self.LIN_SPEED = 50
        self.SLOW_DOWN = 30
        self.RobotConstants = {
            "GRIP_F": 10,
            "GRIP_VEL": 20,
            "L_VEL": 10,
            "J_VEL": 10,
            "TCP_GP": [49.5, 0, 13.95, 45, 0, 0],
            "TCP_SK": [34.479, 11.2, 84.3116, 0, 30, 0],
            "HOME_GP_J": [0, 0, 0, 0, 0, 0],
            "HOME_SK_J": [-90, 0, 0, 0, 60, 0],
        }

    def __del__(self):
        if self.robot is not None:
            self.logger.info("Exit the robot in __del__ function")
            self.exitRobot()

    def exitRobot(self):
        self.robot.DeactivateRobot()
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
                self.robot.SetGripperForce(self.RobotConstants["GRIP_F"])
                self.robot.SetGripperVel(self.RobotConstants["GRIP_VEL"])
                self.robot.SetCartLinVel(self.RobotConstants["L_VEL"])
                self.robot.SetJointVel(self.RobotConstants["J_VEL"])
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
        # Connect the suction pump
        ok = self.suction_pump.connect_pump()
        if ok:
            self.logger.info("Suction pump is sucessfully connected!")
        else:
            self.logger.error("Cannot connect to the suction pump")

    def change_tool(self, tool_name: RobotTool):
        if tool_name == RobotTool.GRIPPER:
            self.robot.SetTRF(*self.RobotConstants["TCP_GP"])
            self.status["Tool"] = RobotTool.GRIPPER
            self.home = self.RobotConstants["HOME_GP_J"]
        elif tool_name == RobotTool.SUCTION:
            self.robot.SetTRF(*self.RobotConstants["TCP_SK"])
            self.status["Tool"] = RobotTool.SUCTION
            self.home = self.RobotConstants["HOME_SK_J"]
    
    def move_home(self):
        self.robot.MoveJoints(*self.home)

    def smart_grab(self):
        if self.status["Tool"] == RobotTool.GRIPPER:
            self.robot.GripperClose()
            self.robot.WaitGripperMoveCompletion()
        elif self.status["Tool"] == RobotTool.SUCTION:
            self.suction_pump.suction_on()
    
    def smart_drop(self):
        if self.status["Tool"] == RobotTool.GRIPPER:
            self.robot.GripperOpen()
            self.robot.WaitGripperMoveCompletion()
        elif self.status["Tool"] == RobotTool.SUCTION:
            self.suction_pump.suction_off()

    def pick_place(self, grab_pos, is_grab = True):
        self.robot.MovePose(grab_pos[0], grab_pos[1], 40, grab_pos[3], grab_pos[4], grab_pos[5])
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
        self.robot.MoveLin(grab_pos[0], grab_pos[1], 40, grab_pos[3], grab_pos[4], grab_pos[5])
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

if __name__ == "__main__":
    meca500_robot = Meca500()
    meca500_robot.initializeRobot()
    
    # Do movements here

    meca500_robot.draw_square()
    # meca500_robot.pick_piece()
    # meca500_robot.place_piece()

    # Exit robot

    meca500_robot.exitRobot()