#!/home/yuanjian/Research/BatteryLab/lab_venv/bin/python3
import rclpy
from rclpy.node import Node
from BatteryLab.robots.MG400 import MG400, main_loop
from sartorius.sartorius_client import SartoriusClient


class LiquidRobot(Node):
    def __init__(self, ip="192.168.0.107", logger=None):
        super().__init__("liquid_robot")
        self.logger = self.get_logger() if logger is None else logger
        self.sartorius = SartoriusClient()
        self.MG400 = MG400(logger=self.logger, ip=ip, sartorius_rline=self.sartorius)

    def initialize_robot(self):
        ok = self.MG400.intialize_robot()
        if not ok:
            print("Failed to initialize MG400, program aborted!")
            return False
        return True

    def __del__(self):
        self.disconnect()

    def move_home(self):
        self.MG400.move_home()

    def disconnect(self):
        self.MG400.disconnect()
        self.logger.info("The MG400 robot has successfully disconnected!")


def manual_position_loop(liquid_robot: LiquidRobot):
    cartesian = liquid_robot.MG400.dashboard.GetPose()
    joints = liquid_robot.MG400.dashboard.GetAngle()
    print(f"The current cartesian coordinates are [{cartesian}]")
    print(f"The current joint angles are [{joints}]")
    mode = input(
        "do you want to drive in joints (J) or cartesian (C)? Type in J or C: "
    )
    if mode == "J":
        parameters_str = input("Please type in the 4 joints [J1, J2, J3, J4]:")
    elif mode == "C":
        parameters_str = input(
            "Please type in the 4 cartesian coordinates: [X, Y, Z, R]:"
        )
    else:
        print("The mode you select does not exist! Please select J or C!")
        return

    parameters_str = parameters_str.strip("[]")
    parameters = [float(x) for x in parameters_str.split(",")]
    if mode == "J":
        print(f"The robot is moving with JointMovJ to {parameters}")
        liquid_robot.MG400.movectl.JointMovJ(*parameters)
    elif mode == "C":
        print(f"The robot is moving with MovJ to cartesian coordinates {parameters}")
        liquid_robot.MG400.movectl.MovJ(*parameters)


def liquid_robot_command_loop(liquidRobot: LiquidRobot):
    prompt = """Press [Enter] to quit,
[0] to home the robot,
[M] to drive to tip case/liquid case,
[G] to get tip at tipcase (x,y),
[A] to get liquid at liquid case (x,y) with volume,
[D] to return tip to tipcase (x,y),
[R] to return liquid to liquidcase(x,y),
[J] to dispense liquid with volume to the post.
[S] to move to the assembly post.
[Z] to enter manual positioning mode.
:> 
"""
    try:
        while True:
            input_str = input(prompt).strip().upper()
            if input_str == "":
                break
            elif input_str == "Z":
                manual_position_loop(liquidRobot)
            elif input_str == "0":
                liquidRobot.MG400.move_home()
            elif input_str == "M":
                choice = input("Please select which case to go (tip/liquid):")
                if choice == "tip":
                    x = int(input("Please input tip index x:").strip())
                    y = int(input("Please input tip index y:").strip())
                    liquidRobot.MG400.move_to_tip_case(x, y)
                elif choice == "liquid":
                    x = int(input("Please input liquid bottle index x:").strip())
                    y = int(input("Please input liquid bottle index y:").strip())
                    liquidRobot.MG400.move_to_liquid(x, y)
                else:
                    print("Your choice is invalid!")
            elif input_str == "G":
                x = int(input("Please input tip index x:").strip())
                y = int(input("Please input tip index y:").strip())
                liquidRobot.MG400.get_tip(x, y)
            elif input_str == "D":
                x = int(input("Please input tip index x:").strip())
                y = int(input("Please input tip index y:").strip())
                liquidRobot.MG400.drop_tip(x, y)
            elif input_str == "R":
                x = int(input("Please input liquid bottle index x:").strip())
                y = int(input("Please input liquid bottle index y:").strip())
                liquidRobot.MG400.return_liquid(x, y)
            elif input_str == "J":
                volume = int(input("Please input volume:").strip())
                liquidRobot.MG400.add_liquid_to_post(volume)
            elif input_str == "S":
                liquidRobot.MG400.move_to_assemble_post()
            elif input_str == "A":
                x = int(input("Please input liquid bottle index x:").strip())
                y = int(input("Please input liquid bottle index y:").strip())
                volume = int(input("Please input volume:").strip())
                liquidRobot.MG400.get_liquid(x, y, volume)
            else:
                print("Invalid input. Please enter a valid option.")
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        liquidRobot.disconnect()
        print("MG400 disconnected safely.")


def liquid_robot_example():
    liquid_robot = LiquidRobot(ip="192.168.0.107")
    ok = liquid_robot.initialize_robot()
    if not ok:
        print("Failed to initialize the Liquid Robot, program aborted!")
        exit()
    liquid_robot_command_loop(liquid_robot)


def main():
    rclpy.init()
    liquid_robot_example()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
