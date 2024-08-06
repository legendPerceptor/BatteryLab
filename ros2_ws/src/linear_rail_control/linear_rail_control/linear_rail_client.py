#!/home/yuanjian/Research/BatteryLab/lab_venv/bin/python3
import sys
import rclpy
from rclpy.node import Node
from battery_lab_custom_msg.srv import MoveLinearRail, GetAbsRailPos

class LinearRailClient(Node):
    def __init__(self):
        super().__init__('linear_rail_client')
        self.move_linear_rail_cli = self.create_client(srv_type=MoveLinearRail, srv_name='/zaber/move_linear_rail')
        while not self.move_linear_rail_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Move Linear Rail Service not available, waiting...')

        self.get_pos_cli = self.create_client(srv_type=GetAbsRailPos, srv_name='/zaber/get_rail_pose')
        while not self.get_pos_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get Pos Service not available, waiting...')
        self.move_req = MoveLinearRail.Request()
        self.get_pos_req = GetAbsRailPos.Request()

    def send_move_request(self, position) -> rclpy.task.Future:
        self.move_req.target_position = position
        return self.move_linear_rail_cli.call_async(self.move_req)
    
    def send_get_pos_request(self) -> rclpy.task.Future:
        self.get_pos_req.get = 10
        return self.get_pos_cli.call_async(self.get_pos_req)


def drive_rail(zaber_rail_client):
    while True:
        pos = input("Type in Abs position, Press [Enter] to quit: ")
        if pos == '':
            break
        try:
            pos = float(pos)
            return zaber_rail_client.send_move_request(pos)
        except ValueError:
            print("Invalid input. Please enter a valid position value.")
    return None

def cli_app():
    zaber_rail_client = LinearRailClient()
    try:
        while True:
            future = None
            mode = ""
            input_str = input("Press [Enter] to quit, [0] to home the rail, [M] to drive rail, [P] to get curent location: ").strip().lower()
            if input_str == '':
                break
            elif input_str == '0':
                future = zaber_rail_client.send_move_request(0)
                mode = "move"
            elif input_str == 'm' or input_str == 'M':
                future = drive_rail(zaber_rail_client)
                mode = "move"
            elif input_str == 'p' or input_str == 'P':
                future = zaber_rail_client.send_get_pos_request()
                mode = "pos"
            else:
                print("Invalid input. Please enter a valid option.")
            if future is not None:
                print("future is ", future)
                while rclpy.ok():
                    rclpy.spin_once(zaber_rail_client)
                    if future.done():
                        try:
                            response = future.result()
                        except Exception as e:
                            zaber_rail_client.get_logger().info(f'Service call failed {e}')
                        else:
                            if mode == 'move':
                                zaber_rail_client.get_logger().info(f'Move call result: {response.success}')
                            elif mode == 'pos':
                                zaber_rail_client.get_logger().info(f'The current position of Zaber rail: {response.current_pos}')
                        future = None
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        zaber_rail_client.destroy_node()
        print("Zaber Rail disconnected safely.")

def main(args=None):
    rclpy.init(args=args)
    cli_app()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
