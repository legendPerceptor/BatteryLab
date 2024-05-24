from mecademicpy.robot import Robot
from mecademicpy import tools

import mecademicpy.robot as mdr
from Logger import Logger

class Meca500():

    def __init__(self, log_path="./Meca500.log", robot_address="192.168.0.100"):
        self.robot = Robot()
        self.robot_address = robot_address
        self.logger = Logger("Meca500", log_path=log_path)

    def __del__(self):
        if self.robot is not None:
            self.logger.info("Exit the robot in __del__ function")
            self.exitRobot()

    def exitRobot(self):
        self.robot.DeactivateRobot()
        self.robot.Disconnect()
        self.robot = None
        self.logger.info("The robot is deactivated!")

    def draw_square(self):
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

    def initializeRobot(self):
        try:
            self.robot.Connect(address=self.robot_address)
        except mdr.CommunicationError as e:
            self.logger.info(f"Robot failed to connect. Check if the IP address {self.robot_address} is correct! Error: {e}")
            return
        
        try:
            self.logger.info("Activating and homing the Meca500 robot!")
            self.robot.ActivateRobot()
            self.robot.Home()
            self.robot.WaitHomed()
            self.logger.info("Meca500 is homed and ready!")
            if tools.robot_model_is_meca500(self.robot.GetRobotInfo().robot_model):
                self.logger.info("The robot is confirmed to be Meca500!")
        except Exception as exception:
            if self.robot.GetStatusRobot().error_status:
                self.logger.info(exception)
                self.logger.info("Robot has encountered an error, attempting to clear...")
                self.robot.ResetError()
                self.robot.ResumeMotion()
            else:
                raise

if __name__ == "__main__":
    meca500_robot = Meca500()
    meca500_robot.initializeRobot()
    meca500_robot.draw_square()
    meca500_robot.exitRobot()