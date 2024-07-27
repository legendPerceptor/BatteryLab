import sys
import rclpy
from rclpy.node import Node
from linear_rail_control.srv import MoveLinearRail, GetAbsRailPos

class LinearRailClient(Node):
    def __init__(self):
        super().__init__('linear_rail_client')
        self.move_linear_rail_cli = self.create_client(srv_type=MoveLinearRail, srv_name='zaber/move_linear_rail', qos_profile=10)
        while not self.move_linear_rail_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Move Linear Rail Service not available, waiting...')

        self.get_pos_cli = self.create_client(srv_type=GetAbsRailPos, srv_name='zaber/get_rail_pos', qos_profile=10)
        self.move_req = MoveLinearRail.Request()
        self.get_pos_req = GetAbsRailPos.Request()

    def send_move_request(self, position) -> rclpy.task.Future:
        self.move_req.target_position = position
        return self.move_linear_rail_cli.call_async(self.move_req)
    
    def send_get_pos_request(self) -> rclpy.task.Future:
        return self.get_pos_cli.call_async(self.get_pos_req)

def main(args=None):
    rclpy.init(args=args)
    client = LinearRailClient()
    position = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    move_future = client.send_move_request(position)
    get_pos_future = client.send_get_pos_request()
    
    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = move_future.result()
                pos_response = get_pos_future.result()
            except Exception as e:
                client.get_logger().info(f'Service call failed {e}')
            else:
                client.get_logger().info(f'Move Result: {response.success}, Pos Result: pos: {pos_response.current_pos}')
            break

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
