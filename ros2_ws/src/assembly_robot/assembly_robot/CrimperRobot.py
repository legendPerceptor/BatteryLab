#!/home/yuanjian/Research/BatteryLab/lab_venv/bin/python3
from BatteryLab.robots.BreadboardMeca500 import BreadBoardMeca500
from BatteryLab.helper.Logger import Logger
from BatteryLab.robots.Constants import RobotTool
import rclpy
from rclpy.node import Node
from camera_service.camera_client import ImageClient
import cv2


class CrimperRobot(Node):

    def __init__(self, logger=None, robot_address="192.168.0.101"):
        super().__init__("crimper_robot")
        self.logger = self.get_logger() if logger is None else logger
        self.crimper_robot = BreadBoardMeca500(
            logger=self.logger, robot_address=robot_address
        )
        self.tower_camera_client = ImageClient(
            node_name="crimper_tower_camera", serv_name="/batterylab/lookup_camera"
        )

    def initialize_and_home_robots(self):
        ok = self.crimper_robot.initializeRobot()
        if not ok:
            print("The Crimper Meca500 cannot be connected")
            exit()
        self.crimper_robot.move_home(tool=RobotTool.GRIPPER)

    def move_home(self):
        self.crimper_robot.move_home(tool=RobotTool.GRIPPER)

    def exitRobot(self):
        self.crimper_robot.exitRobot()

    def crimp_a_battery(self, use_camera_check: bool = False):
        self.crimper_robot.pick_up_from_assembly_post()
        if use_camera_check:
            ok = self.check_battery_is_picked()
            if not ok:
                print("The battery is not picked up!")
                self.logger.error("The battery is not picked up properly")
                return
        self.crimper_robot.drop_to_crimper()
        # TODO: Call the crimper to do the crimping work
        self.crimper_robot.pick_up_from_crimper()
        if use_camera_check:
            ok = self.check_battery_is_picked()
            if not ok:
                print("The battery is not picked up!")
                self.logger.error("The battery is not picked up properly")
                return
        self.logger.info("Finished crimping a battery!")

    def put_to_storage(self):
        self.crimper_robot.put_to_storage()

    def drop_back_to_assembly_post(self):
        self.crimper_robot.drop_back_to_assembly_post()

    def check_battery_is_picked(self) -> bool:
        self.crimper_robot.move_for_photo_check()
        image = self.tower_camera_client.get_image()
        # TODO: Analyze the image to determine if the battery is in there
        self.tower_camera_client.display_image()
        human_decision = input("Is the battery there (yes/no), default is yes:")
        self.crimper_robot.move_away_from_photo_check()
        if human_decision == "yes" or human_decision == "":
            return True
        else:
            return False


def crimper_robot_command_loop(robot: CrimperRobot):
    prompt = """Press [Enter] to quit.
[P] to pick up from the post.
[C] to drop the battery to the crimper.
[D] to pick the battery up from the crimper
[S] to store the battery in the storage post.
[B] to put the battery back to the assembly post.
[G] to grab from the post, do the crimping and put back to the post.
[A] to run the whole process and store the battery in storage post.
[T] to move to the camera tower and see the picture.
:> """

    while True:
        input_str = input(prompt).strip().upper()
        if input_str == "":
            break
        elif input_str == "P":
            robot.crimper_robot.pick_up_from_assembly_post()
        elif input_str == "C":
            robot.crimper_robot.drop_to_crimper()
        elif input_str == "D":
            robot.crimper_robot.pick_up_from_crimper()
        elif input_str == "S":
            robot.crimper_robot.put_to_storage()
        elif input_str == "G":
            robot.crimp_a_battery(False)
            robot.drop_back_to_assembly_post()
            robot.move_home()
        elif input_str == "T":
            is_picked = robot.check_battery_is_picked()
            print(f"The battery is picked: {is_picked}")
        elif input_str == "A":
            use_camera = input(
                "Do you want to use camera_check? (yes/no), default is yes:"
            )
            if use_camera == "" or use_camera == "yes":
                robot.crimp_a_battery(use_camera_check=True)
            else:
                robot.crimp_a_battery(use_camera_check=False)

            robot.put_to_storage()
            robot.move_home()
        else:
            print("The command you gave is not recognized!")
    robot.crimper_robot.move_home(tool=RobotTool.GRIPPER)


def main():
    rclpy.init()
    log_path = "/home/yuanjian/Research/BatteryLab/logs"
    logger = Logger("assembly_robot_test", log_path, "assembly_robot_test.log")
    robot = CrimperRobot(logger, robot_address="192.168.0.101")
    robot.initialize_and_home_robots()
    crimper_robot_command_loop(robot=robot)
    robot.exitRobot()
    robot.destroy_node()
    rclpy.shutdown()
