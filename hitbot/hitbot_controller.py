import sys
import os
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Int64
from hitbot_msgs.srv import SetGPIO

os.chdir(os.path.expanduser('~'))
sys.path.append("./hitbot_ws/src/hitbot")  ## get import pass: hitbot_interface.py
from hitbot_interface import HitbotInterface

class HitbotController(Node):
    def __init__(self):
        super().__init__('hitbot_controller')

        self.srv = self.create_service(SetGPIO, 'set_gpio', self.set_gpio_callback)

        self.hitbot_x = 0
        self.hitbot_y = 0
        self.hitbot_z = 0

        self.subscription = self.create_subscription(
            Int64,
            '/hitbot_x',
            self.hitbot_x_callback,
            10
        )

        self.subscription = self.create_subscription(
            Int64,
            '/hitbot_y',
            self.hitbot_y_callback,
            10
        )

        self.subscription = self.create_subscription(
            Int64,
            '/hitbot_z',
            self.hitbot_z_callback,
            10
        )

        self.robot_id = 123  ## 123 is robot_id, Modify it to your own
        self.robot = HitbotInterface(self.robot_id)

        self.init_robot()

    def hitbot_x_callback(self, msg):
        self.hitbot_x = msg.data

    def hitbot_y_callback(self, msg):
        self.hitbot_y = msg.data

    def hitbot_z_callback(self, msg):
        self.hitbot_z = msg.data

    def set_gpio_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.set_digital_out(request.gpio_number, request.set_on)
                response.success = True
                self.get_logger().info('GPIO setting successful: gpio_number=%d, set_on=%r' % (request.gpio_number, request.set_on))
                break
            except Exception as e:
                self.get_logger().error('Failed to set GPIO: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying GPIO setting (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to set GPIO.')
                    break

        return response

    def init_robot(self):
        self.robot.net_port_initial()
        time.sleep(1)
        is_connected = self.robot.is_connect()

        if is_connected != 1:
            print('No robot connection!!!')
            raise RuntimeError('No robot connection!!!')

        print('Robot connected.')

        init = self.robot.initial(1, 1000) ## 1000 is z-axis parameter, Modify it to your own

        if init != 1:
            print('Robot initialization failed!!!')
            raise RuntimeError('Robot initialization failed!!!')

        print('unlock Robot')
        self.robot.unlock_position()
        print('Robot position initialized.')
        self.robot.new_movej_angle(0, 0, 0, 0, 100, 1)
        self.robot.wait_stop()
        print('Robot I/O output initialized.')
        for i in range(12):
            self.robot.set_digital_out(i, False)
        time.sleep(1)
        print('Robot initialized.')

    def run(self):
        print("hello hibot")

        while rclpy.ok():
            try:
                rclpy.spin_once(self)
                self.robot.new_movej_xyz_lr(self.hitbot_x, self.hitbot_y, self.hitbot_z, 0, 100, 1, 1)
                self.robot.wait_stop()
            except ValueError as e:
                print("Error:", str(e))
            except RuntimeError as e:
                print("Error:", str(e))

        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    hitbot_controller = HitbotController()

    try:
        hitbot_controller.run()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    finally:
        hitbot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()