import rclpy
from rclpy.node import Node
from linear_rail_control.srv import MoveLinearRail
from BatteryLab.ZaberRail import ZaberRail

class LinearRailServer(Node):
    def __init__(self):
        super().__init__('linear_rail_server')
        self.srv = self.create_service(MoveLinearRail, 'move_linear_rail', self.move_linear_rail_callback)
        self.get_logger().info('Linear Rail Service is ready')
        self.linear_rail = ZaberRail()

    def move_linear_rail_callback(self, request, response):
        target_position = request.target_position
        self.get_logger().info(f'Received request to move to position: {target_position} cm')
        
        # Here you would add the code to move the linear rail to the target_position
        # For example: linear_rail.move_to(target_position)
        
        response.success = True  # or False based on the result of the movement
        return response

def main(args=None):
    rclpy.init(args=args)
    linear_rail_server = LinearRailServer()
    rclpy.spin(linear_rail_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()