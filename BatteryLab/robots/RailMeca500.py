from ..helper.utils import get_proper_port_for_device, SupportedDevices
from .SuctionPump import SuctionPump
from .Constants import RobotTool
from .Meca500 import Meca500

from pathlib import Path

class RailMeca500(Meca500):

    def __init__(self, logger = None, log_path="logs", logger_filename="Meca500.log", robot_address="192.168.0.101",
                 suction_pump = None,
                 robot_constants_config_file=Path(__file__).parent.parent / "configs" / "RailMeca500.yaml"):
        super().__init__(logger, log_path, logger_filename, robot_address, robot_constants_config_file)
        self.status = {}
        self.suction_pump = suction_pump if suction_pump is not None else SuctionPump(self.logger, self.status, vacuum_port=get_proper_port_for_device(SupportedDevices.SuctionPump))
        ok = self.suction_pump.connect_pump()
        if not ok:
            self.logger.error("Cannot connect to the suction pump!")
            print("Cannot connect to the suction pump!")
        self.tool = RobotTool.SUCTION

    def change_tool(self, tool_name: RobotTool):
        if tool_name == RobotTool.GRIPPER:
            self.robot.SetTRF(*self.RobotConstants.TCP_GP)
            self.home = self.RobotConstants.HOME_GP_J
        elif tool_name == RobotTool.SUCTION:
            self.robot.SetTRF(*self.RobotConstants.TCP_SK)
            self.home = self.RobotConstants.HOME_SK_J
        self.tool = tool_name


    def smart_grab(self):
        if self.tool == RobotTool.GRIPPER:
            self.robot.SetGripperForce(5)
            self.robot.GripperClose()
            self.robot.WaitGripperMoveCompletion()
        elif self.tool == RobotTool.SUCTION:
            self.suction_pump.continues_pick()
    
    def smart_drop(self):
        if self.tool == RobotTool.GRIPPER:
            self.robot.SetGripperForce(5)
            self.robot.GripperOpen()
            self.robot.WaitGripperMoveCompletion()
        elif self.tool== RobotTool.SUCTION:
            self.suction_pump.drop()

    def move_after_drop(self):
        self.robot.MoveJoints(*self.RobotConstants.HOME_POST_J)
        self.robot.MoveJoints(*self.RobotConstants.HOME_SK_J)

    def pick_place(self, grab_pos, is_grab = True):
        self.logger.info(f"Starting picking component at {grab_pos}")
        self.robot.SetJointVel(self.RobotConstants.J_VEL)
        self.robot.MovePose(grab_pos[0], grab_pos[1], grab_pos[2] + 30, grab_pos[3], grab_pos[4], grab_pos[5])
        # Linearly moving down to grab the component and go back
        self.robot.Delay(0.5)
        self.robot.SetCartLinVel(self.RobotConstants.L_VEL)
        self.robot.MoveLin(*grab_pos)
        self.robot.WaitIdle()
        if is_grab:
            self.smart_grab()
        else:
            self.smart_drop()
        self.robot.Delay(0.2)
        self.robot.MoveLin(grab_pos[0], grab_pos[1], grab_pos[2] + 30, grab_pos[3], grab_pos[4], grab_pos[5])
        self.robot.Delay(0.5)
        # Move the component back to home
        self.robot.SetCartLinVel(self.RobotConstants.L_VEL)
        self.move_home(tool=RobotTool.SUCTION)
        self.robot.WaitIdle()

def rail_meca500_example_app():
    robot_address = "192.168.0.100"
    user_provided_address = input(f"Please type in the IP address for the robot (the default is {robot_address}, press enter to use the default): ")
    if user_provided_address != '':
        robot_address = user_provided_address
    meca500_robot = RailMeca500(robot_address=robot_address)
    meca500_robot.initializeRobot()
    meca500_robot.draw_square()
    meca500_robot.move_home(tool=RobotTool.SUCTION)
    meca500_robot.exitRobot()