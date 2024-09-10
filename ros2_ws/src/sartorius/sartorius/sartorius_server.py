#!/home/yuanjian/Research/BatteryLab/lab_venv/bin/python3
import rclpy
from rclpy.node import Node
from BatteryLab.robots.SartoriusRLine import SartoriusRLine
from BatteryLab.helper.utils import get_proper_port_for_device, SupportedDevices
from battery_lab_custom_msg.srv import SartoriusCtrl # we reuse the service message for simplicity

class SartoriusServer(Node):
    def __init__(self):
        super().__init__('suction_pump_server')
        self.sartorius = SartoriusRLine(logger=self.get_logger(), port=get_proper_port_for_device(SupportedDevices.SartoriusRLine))
        ok = self.sartorius.initiate_rline()
        if not ok:
            self.get_logger().error("Cannot connect to the Sartorius RLine")
        else:
            self.get_logger().info("Connected to the Sartorius RLine.")
    
        self.sartorius_control_service = self.create_service(SartoriusCtrl, '/sartorius', self.sartorius_ctrl_callback)
        self.get_logger().info('Linear Rail Service is ready')
    
    def sartorius_ctrl_callback(self, request, response):
        command = request.command
        volume = request.volume
        self.get_logger().info(f"Received suction request with command: {command}")
        response.float_result = 0.0
        response.int_result = 0
        if command == 'tellPosition':
            ok, positions = self.sartorius.tellPosition()
            print(positions)
        elif command == 'tellLevel':
            ok = True
            response.float_result = self.sartorius.tellLevel()
        elif command == 'initiate':
            ok = self.sartorius.initiate_rline()
        elif command == 'aspirate':
            ok = self.sartorius.aspirate(volume)
        elif command == 'dispense':
            ok = self.sartorius.dispense(volume)
        elif command == 'blowout':
            ok = self.sartorius.blowout()
        elif command == 'eject':
            ok = self.sartorius.blowout()
        elif command == 'eject_and_home':
            ok = self.sartorius.eject_and_home()
        elif command == 'clear_and_reset':
            ok = self.sartorius.clear_and_reset()
        elif command == 'reset':
            ok = self.sartorius.reset()
        elif command == 'disconnect':
            ok = self.sartorius.disconnect()
        if ok:
            response.status = 'ok'
        else:
            response.status = 'error'
        return response
    
def main(args=None):
    rclpy.init(args=args)
    sartorius_server = SartoriusServer()
    rclpy.spin(sartorius_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()