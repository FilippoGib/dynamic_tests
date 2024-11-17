import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from mmr_base.msg import VehicleState

class DynamicTestsNode(Node):
    def __init__(self):
        super().__init__('dynamic_tests_node')

        self.drive_msg = AckermannDrive()

        self.declare_parameter('radius', 1.0)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('steering_angle_velocity', 0.0)
        self.declare_parameter('acceleration', 0.0)
        self.declare_parameter('jerk', 0.0)

        self.radius = self.get_parameter('radius').value
        self.speed = self.get_parameter('speed').value
        self.steering_angle_velocity = self.get_parameter('steering_angle_velocity').value
        self.acceleration = self.get_parameter('acceleration').value
        self.jerk = self.get_parameter('jerk').value

        self.publisher_ = self.create_publisher(AckermannDrive, '/sim/drive_parameters', 10)

        self.subscriber = self.create_subscription(VehicleState, '/vehicle/actualState', self.vehicle_actual_state_cb, 10)

        self.timer = self.create_timer(0.05, self.publish_drive_command) #20Hz


    def publish_drive_command(self):
    
        # Create and publish the AckermannDrive message
        self.drive_msg.steering_angle = self.calculate_steering_angle_from_radius(self.radius)
        self.drive_msg.speed = self.speed
        self.drive_msg.steering_angle_velocity = self.steering_angle_velocity
        self.drive_msg.acceleration = self.acceleration
        self.drive_msg.jerk = self.jerk

        self.get_logger().info(f'Publishing: {self.drive_msg}')
        self.publisher_.publish(self.drive_msg)


    def vehicle_actual_state_cb(self, vehicle_state):
        self.get_logger().info('Callback entered: /vehicle/actualState')

    def calculate_steering_angle_from_radius(self, radius):
        #TODO
        return radius

def main(args=None):
    rclpy.init(args=args)
    node = DynamicTestsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
