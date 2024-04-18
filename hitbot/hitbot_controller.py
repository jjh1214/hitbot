import sys
import rclpy
import time
from std_msgs.msg import Int64, Bool
from rclpy.node import Node

sys.path.append("./hitbot_ws/src/hitbot")  ## get import pass: hitbot_interface.py
from hitbot_interface import HitbotInterface

class HitbotController(Node):
    def __init__(self):
        super().__init__('hitbot_controller')

        self.hitbot_x = 0
        self.hitbot_y = 0
        self.hitbot_z = 0
        self.io_set = False

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

        self.subscription = self.create_subscription(
            Bool,
            '/io_set',
            self.io_set_callback,
            10
        )

        self.robot_id = 123  ## Modify your robot id
        self.robot = HitbotInterface(self.robot_id)

        self.init_robot()

    def hitbot_x_callback(self, msg):
        self.hitbot_x = msg.data

    def hitbot_y_callback(self, msg):
        self.hitbot_y = msg.data

    def hitbot_z_callback(self, msg):
        self.hitbot_z = msg.data

    def io_set_callback(self, msg):
        self.io_set = msg.data

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
                self.robot.set_digital_out(5, self.io_set)
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