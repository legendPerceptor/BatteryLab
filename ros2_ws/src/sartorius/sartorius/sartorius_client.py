#!/home/yuanjian/Research/BatteryLab/lab_venv/bin/python3
import rclpy
from rclpy.node import Node
from battery_lab_custom_msg.srv import SartoriusCtrl
from BatteryLab.robots.SartoriusRLine import SartoriusRLineInterface

class SartoriusClient(Node, SartoriusRLineInterface):
    def __init__(self):
        super().__init__("sartorius_client")
        self.sartorius_ctrl_cli = self.create_client(srv_type=SartoriusCtrl, srv_name='/sartorius')
        while not self.sartorius_ctrl_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Sartorius Ctrl Service not available, waiting...')
        self.sartorius_ctrl_request = SartoriusCtrl.Request()
    
    def send_request(self, command, volume=0) -> rclpy.task.Future:
        self.sartorius_ctrl_request.command = command
        self.sartorius_ctrl_request.volume = volume
        return self.sartorius_ctrl_cli.call_async(self.sartorius_ctrl_request)
    
    def handle_request_result(self, future):
        if future is not None:
            print("future is ", future)
            while rclpy.ok():
                rclpy.spin_once(self)
                if future.done():
                    try:
                        response = future.result()
                    except Exception as e:
                        self.get_logger().info(f'Service call failed {e}')
                    else:
                        self.get_logger().info(f'The sartorius request result: {response.status}')
                        return response
                    break
        else:
            self.get_logger().error("The sartorius future to parse is None!")
        return None

    def tellPosition(self):
        future = self.send_request(command="tellPosition")
        return self.handle_request_result(future)

    def tellLevel(self):
        future = self.send_pump_ctrl_request(command="tellLevel")
        return self.handle_request_result(future)

    def initiate_rline(self):
        future = self.send_request(command="initiate")
        return self.handle_request_result(future)

    def aspirate(self, volume):
        future = self.send_request(command="aspirate", volume=volume)
        return self.handle_request_result(future)

    def dispense(self,volume):
        future = self.send_request(command="dispense", volume=volume)
        return self.handle_request_result(future)
    
    def blowout(self):
        future = self.send_request(command="blowout")
        return self.handle_request_result(future)

    def clear_and_reset(self):
        future = self.send_request(command="clear_and_reset")
        return self.handle_request_result(future)

    def reset(self):
        future = self.send_request(command="reset")
        return self.handle_request_result(future)
    
    def eject(self):
        future = self.send_request(command="eject")
        return self.handle_request_result(future)
    
    def eject_and_home(self):
        future = self.send_request(command="eject_and_home")
        return self.handle_request_result(future)
    
    def disconnect(self):
        future = self.send_request(command="disconnect")
        return self.handle_request_result(future)


def cli_app():
    sartorius_rline = SartoriusClient()
    prompt = """Press [Enter] to quit, [B] to blow out all liquid, [E] to eject pipette
[A] to aspirate liquid, [D] to dispense liquid, [T] to tell level:
:>
"""
    try:
        while True:
            input_str = input(prompt).strip().upper()
            if input_str == '':
                break
            elif input_str == 'B':
                sartorius_rline.blowout()
            elif input_str == 'E':
                sartorius_rline.eject_and_home()
            elif input_str == 'A':
                volume = float(input("Please input the volume to aspirate: ").strip())
                sartorius_rline.aspirate(volume)
            elif input_str == 'D':
                volume = float(input("Please input the volume to dispense: ").strip())
                sartorius_rline.dispense(volume)
            elif input_str == 'T':
                response = sartorius_rline.tellLevel()
                if response is not None:
                    print("The current level:", response.float_result)
                else:
                    print("The tell level command failed!")
            else:
                print("Invalid input. Please enter a valid option.")
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        sartorius_rline.disconnect()
        print("Sartorius rLine disconnected safely.")

def main():
    rclpy.init()
    cli_app()
    rclpy.shutdown()

if __name__ == '__main__':
    main()