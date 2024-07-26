import rclpy
from rclpy.node import Node
from linear_rail_control.srv import MoveLinearRail
from BatteryLab.robots.ZaberRail import ZaberRail, ZaberRailConnectionError
from BatteryLab.helper.utils import get_proper_port_for_device, SupportedDevices

class LinearRailServer(Node):
    def __init__(self):
        super().__init__('linear_rail_server')
        self.srv = self.create_service(MoveLinearRail, 'zaber/move_linear_rail', self.move_linear_rail_callback)
        self.get_logger().info('Linear Rail Service is ready')
        
        selected_port = get_proper_port_for_device(SupportedDevices.ZaberLinearRail)
        self.zaber_rail = ZaberRail(port=selected_port)
        ok = self.zaber_rail.connect()
        if not ok:
            print("Failed to connect to the Zaber linear rail!")
        else:
            print("The server has successfully connected to the Zaber linear rail!")

    def move_linear_rail_callback(self, request, response):
        target_position = request.target_position
        self.get_logger().info(f'Received request to move to position: {target_position} cm')
        
        # Here you would add the code to move the linear rail to the target_position
        # For example: linear_rail.move_to(target_position)
        try:
            self.zaber_rail.move(target_position)
            response.success = True
        except ZaberRailConnectionError as e:
            print(f"Caught an error in LinearRailServer: {e}")
            response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    linear_rail_server = LinearRailServer()
    rclpy.spin(linear_rail_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()