import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        
        # Publisher for the joint velocity controller
        self.velocity_pub = self.create_publisher(Float64MultiArray, '/joint_group_velocity_controller/commands', 10)
        
        # Subscriber for cmd_vel
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Robot parameters
        self.wheel_radius = 0.1  # Example: 10 cm
        self.wheel_separation = 0.5  # Example: 50 cm (distance between left and right wheels)
        self.wheel_base = 0.5  # Example: 50 cm (distance between front and rear wheels)

    def cmd_vel_callback(self, msg):
        # Create a Float64MultiArray to publish joint velocities
        joint_velocities = Float64MultiArray()
        
        # Extract linear and angular velocities from the message
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Calculate wheel velocities based on kinematic equations
        V_x = linear_x
        V_theta = angular_z
        
        V_fl = (V_x - self.wheel_separation / 2 * V_theta) / self.wheel_radius  # Front Left
        V_fr = (V_x + self.wheel_separation / 2 * V_theta) / self.wheel_radius  # Front Right
        V_rl = (V_x - self.wheel_separation / 2 * V_theta) / self.wheel_radius  # Rear Left
        V_rr = (V_x + self.wheel_separation / 2 * V_theta) / self.wheel_radius  # Rear Right
        
        # Set the joint velocities into the data array
        joint_velocities.data = [V_fr, V_fl, V_rr, V_rl]
        
        # Publish the joint velocities
        self.velocity_pub.publish(joint_velocities)

def main(args=None):
    rclpy.init(args=args)
    robot_mover = RobotMover()
    
    rclpy.spin(robot_mover)
    
    robot_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
