#!/home/yuanjian/Research/BatteryLab/lab_venv/bin/python3
import rclpy
from rclpy.node import Node
from battery_lab_custom_msg.srv import MoveLinearRail, GetAbsRailPos
from BatteryLab.robots.ZaberRail import ZaberRail, ZaberRailConnectionError
from BatteryLab.helper.utils import get_proper_port_for_device, SupportedDevices

class LinearRailServer(Node):
    def __init__(self):
        super().__init__('linear_rail_server')
        selected_port = get_proper_port_for_device(SupportedDevices.ZaberLinearRail)
        self.zaber_rail = ZaberRail(port=selected_port)
        ok = self.zaber_rail.connect()
        if not ok:
            print("Failed to connect to the Zaber linear rail!")
            self.get_logger().error("Failed to connect to the Zaber linear rail!")
        else:
            print("The server has successfully connected to the Zaber linear rail!")
            self.get_logger().info("The server has successfully connected to the Zaber linear rail!")
        
        self.move_linear_rail_srv = self.create_service(MoveLinearRail, '/zaber/move_linear_rail', self.move_linear_rail_callback)
        self.get_linear_rail_pos_srv = self.create_service(GetAbsRailPos, '/zaber/get_rail_pose', self.get_rail_pos_callback)
        self.get_logger().info('Linear Rail Service is ready')

    def get_rail_pos_callback(self, request, response):
        self.get_logger().info(f'Received request to get current position, {request.get}')
        try:
            pos = self.zaber_rail.get_cur_position()
            response.current_pos = pos
            response.connected = True
        except ZaberRailConnectionError as e:
            print(f"Caught an error in LinearRailServer: {e}")
            response.current_pos = -1
            response.connected = False
        return response

    def move_linear_rail_callback(self, request, response):
        target_position = request.target_position
        self.get_logger().info(f'Received request to move to position: {target_position} cm')
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