#!/home/yuanjian/Research/BatteryLab/lab_venv/bin/python3
import rclpy
from rclpy.node import Node
from BatteryLab.robots.SuctionPump import SuctionPump
from BatteryLab.helper.utils import get_proper_port_for_device, SupportedDevices
from battery_lab_custom_msg.srv import SuctionPumpCtrl

class SuctionPumpServer(Node):
    def __init__(self):
        super().__init__('suction_pump_server')
        self.status = {}
        self.suction_pump = SuctionPump(logger=self.get_logger(), status=self.status, vacuum_port=get_proper_port_for_device(SupportedDevices.SuctionPump))
        ok = self.suction_pump.connect_pump()
        if not ok:
            self.get_logger().error("Cannot connect to the suction pump!")
        else:
            self.get_logger().info("Connected to the suction pump!")
    
        self.get_linear_rail_pos_srv = self.create_service(SuctionPumpCtrl, '/suction_pump_ctrl', self.suction_pump_ctrl_callback)
        self.get_logger().info('Linear Rail Service is ready')
    
    def suction_pump_ctrl_callback(self, request, response):
        command = request.command
        self.get_logger().info(f"Received suction request with command: {command}")
        if command == 'C':
            ok = self.suction_pump.continues_pick()
        elif command == 'D':
            ok = self.suction_pump.drop()
        elif command == 'P':
            ok = self.suction_pump.pick()
        elif command == 'O':
            ok = self.suction_pump.off()
        elif command == 'connect':
            ok = self.suction_pump.connect_pump()
        elif command == 'disconnect':
            ok = self.suction_pump.disconnect_pump()
        elif command == 'check_connection':
            ok = self.suction_pump.check_connection()
        if ok:
            response.status = 'ok'
        else:
            response.status = 'error'
        return response
    
def main(args=None):
    rclpy.init(args=args)
    suction_pump_server = SuctionPumpServer()
    rclpy.spin(suction_pump_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()