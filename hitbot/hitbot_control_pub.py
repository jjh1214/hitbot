import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Bool

class HitbotControl(Node):
    def __init__(self):
        super().__init__('hitbot_control_pub')

        self.hitbot_x_publisher = self.create_publisher(Int64, '/hitbot_x', 10)
        self.hitbot_y_publisher = self.create_publisher(Int64, '/hitbot_y', 10)
        self.hitbot_z_publisher = self.create_publisher(Int64, '/hitbot_z', 10)
        self.hitbot_r_publisher = self.create_publisher(Int64, '/hitbot_r', 10)
        self.io_set_publisher = self.create_publisher(Bool, '/io_set', 10)

    def publish_hitbot_x(self, data):
        msg = Int64()
        msg.data = data
        self.hitbot_x_publisher.publish(msg)

    def publish_hitbot_y(self, data):
        msg = Int64()
        msg.data = data
        self.hitbot_y_publisher.publish(msg)

    def publish_hitbot_z(self, data):
        msg = Int64()
        msg.data = data
        self.hitbot_z_publisher.publish(msg)

    def publish_hitbot_r(self, data):
        msg = Int64()
        msg.data = data
        self.hitbot_r_publisher.publish(msg)

    def publish_io_set(self, data):
        msg = Bool()
        msg.data = data
        self.io_set_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    hitbot_publisher = HitbotControl()

    try:
        while rclpy.ok():

            hitbot_x_data = int(input("Enter hitbot_x (mm): "))
            hitbot_y_data = int(input("Enter hitbot_y (mm): "))
            hitbot_z_data = int(input("Enter hitbot_z -(mm): "))
            hitbot_r_data = int(input("Enter hitbot_r (degree): "))
            io_set_data = bool(input("Enter io_set data (True/False): "))

            hitbot_publisher.publish_hitbot_x(hitbot_x_data)
            hitbot_publisher.publish_hitbot_y(hitbot_y_data)
            hitbot_publisher.publish_hitbot_z(hitbot_z_data)
            hitbot_publisher.publish_hitbot_z(hitbot_r_data)
            hitbot_publisher.publish_io_set(io_set_data)
            
    except KeyboardInterrupt:
        pass

    hitbot_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()