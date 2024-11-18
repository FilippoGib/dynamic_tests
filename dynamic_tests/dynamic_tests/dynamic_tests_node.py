import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from dynamic_tests.PIDcontroller import PIDController
import math
from mmr_base.msg import VehicleState

class DynamicTestsNode(Node):
    def __init__(self):
        super().__init__('dynamic_tests_node')

        self.drive_msg = AckermannDrive()
        self.controller = PIDController(   
            kp=1.0,  # Proportional gain
            ki=0.1,  # Integral gain
            kd=0.05, # Derivative gain
            dt=0.05 # Delta time
            )
        
        # modify from config.yaml
        self.declare_parameter('radius', 0.0)
        self.declare_parameter('speed', 0.0) # speed we want to keep -> DO NOT CHANGE THIS DYNAMICALLY
        self.declare_parameter('steering_angle_velocity', 0.0)
        self.declare_parameter('acceleration', 0.0)
        self.declare_parameter('jerk', 0.0)

        self.radius = self.get_parameter('radius').value
        self.tartget_speed = self.get_parameter('speed').value # speed we want to keep -> DO NOT CHANGE THIS DYNAMICALLY
        self.steering_angle_velocity = self.get_parameter('steering_angle_velocity').value
        self.acceleration = self.get_parameter('acceleration').value
        self.jerk = self.get_parameter('jerk').value
        self.speed = self.tartget_speed # speed message we send to the simulator -> CHANGE THIS DYNAMICALLY
        self.counter = 0 # i want the actual_speed to be equal to the target speed for a certain number of iterations
        self.target_speed_reached = False

        # publish drive parameters to simulator
        self.publisher_ = self.create_publisher(AckermannDrive, '/sim/drive_parameters', 10)

        # listen to actual car state -> only interested in "actual_speed"
        self.subscriber = self.create_subscription(VehicleState, '/vehicle/actualState', self.vehicle_actual_state_cb, 10)

        self.timer = self.create_timer(0.05, self.publish_drive_command) #20Hz


    def publish_drive_command(self):
        
        #Keep the steering to 0 until the target speed is reached
        if(self.target_speed_reached == True):
            self.drive_msg.steering_angle = self.calculate_steering_angle_from_radius(self.radius)
        else:
            self.drive_msg.steering_angle = 0.0
        self.drive_msg.speed = self.speed # THE SPEED MESSAGE IS ACTUALLY INTERPRETED AS ACCELERATION
        self.drive_msg.steering_angle_velocity = self.steering_angle_velocity # should stay 0
        self.drive_msg.acceleration = self.acceleration # should stay 0
        self.drive_msg.jerk = self.jerk # should stay 0 (jerk is acceleration rate over time btw)

        self.get_logger().info(f'Publishing: {self.drive_msg}')
        self.publisher_.publish(self.drive_msg)



    def vehicle_actual_state_cb(self, vehicle_state):

        self.get_logger().info('Callback entered: /vehicle/actualState')

        target_speed = self.tartget_speed # the speed we want to keep
        actual_speed = vehicle_state.actual_speed #actual vehicle speed feedback we get from the sim

        if(abs(target_speed - actual_speed) <= 0.3): # 0.3 m/s = 1km/h
            self.counter += 1
            if(self.counter >= 5): # at least 5 iterations
                self.target_speed_reached = True

        # use PID controller to calculate the new speed input we have to send to keep the target speed based on the current error
        new_speed = self.controller.compute(target=target_speed, current=actual_speed) 

        self.speed = new_speed # set the speed message we send to the simulator to the output of the controller

        self.get_logger().info(
        f'Actual Speed: {actual_speed:.2f}, Target Speed: {target_speed:.2f}, '
        f'New Speed (PID Output): {new_speed:.2f}'
        )


    def calculate_steering_angle_from_radius(self, radius):
        #TODO: need formula
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
