import sys
import rclpy
from rclpy.node import Node
from linear_rail_control.srv import MoveLinearRail

class LinearRailClient(Node):
    def __init__(self):
        super().__init__('linear_rail_client')
        self.cli = self.create_client(MoveLinearRail, 'zaber/move_linear_rail')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = MoveLinearRail.Request()

    def send_request(self, position):
        self.req.target_position = position
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    client = LinearRailClient()
    position = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    client.send_request(position)
    
    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().info(f'Service call failed {e}')
            else:
                client.get_logger().info(f'Result: {response.success}')
            break

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
