#!/home/yuanjian/Research/BatteryLab/lab_venv/bin/python3
import rclpy
from rclpy.node import Node
from battery_lab_custom_msg.srv import SuctionPumpCtrl
from BatteryLab.robots.SuctionPump import SuctionPumpInterface

class SuctionPumpClient(Node, SuctionPumpInterface):
    def __init__(self):
        super().__init__("suction_pump_client")
        self.suction_pump_ctrl_cli = self.create_client(srv_type=SuctionPumpCtrl, srv_name='/suction_pump_ctrl')
        while not self.suction_pump_ctrl_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Suction Pump Ctrl Service not available, waiting...')
        self.pump_ctrl_request = SuctionPumpCtrl.Request()
    
    def send_pump_ctrl_request(self, command) -> rclpy.task.Future:
        self.pump_ctrl_request.command = command
        return self.suction_pump_ctrl_cli.call_async(self.pump_ctrl_request)
    
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
                        self.get_logger().info(f'The pump control request result: {response.status}')
                        return response.status
                    break
        else:
            self.get_logger().error("The pump control future to parse is None!")
        return None

    def connect_pump(self):
        future = self.send_pump_ctrl_request(command="connect")
        return self.handle_request_result(future)

    def check_connection(self):
        future = self.send_pump_ctrl_request(command="check_connection")
        return self.handle_request_result(future)

    def continuous_pick(self):
        future = self.send_pump_ctrl_request(command="C")
        return self.handle_request_result(future)

    def pick(self):
        future = self.send_pump_ctrl_request(command="P")
        return self.handle_request_result(future)
    
    def drop(self):
        future = self.send_pump_ctrl_request(command="D")
        return self.handle_request_result(future)

    def off(self):
        future = self.send_pump_ctrl_request(command="O")
        return self.handle_request_result(future)

    def disconnect_pump(self):
        future = self.send_pump_ctrl_request(command="disconnect")
        return self.handle_request_result(future)


def cli_app():
    suctionPump = SuctionPumpClient()
    prompt = """Press [Enter] to reset the pump, [C] to continuously suck, [P] to pick up an item,
[D] to drop an item, [Q] to check connection, [exit] to exit the program.
:>
"""
    try:
        while True:
            input_str = input(prompt).strip().lower()
            if input_str == '':
                suctionPump.off()
            elif input_str == 'p':
                suctionPump.pick()
            elif input_str == 'd':
                suctionPump.drop()
            elif input_str == 'q':
                print(f"The connection status: {suctionPump.check_connection()}")
            elif input_str == 'c':
                suctionPump.continues_pick()
            elif input_str == 'exit':
                break
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        suctionPump.disconnect_pump()
        print("Suction pump disconnected safely.")
        suctionPump.destroy_node()
        print("Suction pump client node destroyed.")

def main():
    rclpy.init()
    cli_app()
    rclpy.shutdown()

if __name__ == '__main__':
    main()