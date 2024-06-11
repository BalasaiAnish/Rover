import rclpy 
from rclpy.node import Node

# Import odometry to get feedback from the ekf
from nav_msgs.msg import Odometry

# Import twist to read cmd_vel and provide final output to the STM32
from geometry_msgs.msg import Twist

class PID_Control(Node):
    
    def __init__(self,kp,kd,ki):
        super().__init__('pid_control')

        # Initialise PID constants, change in main function
        self.kp = kp
        self.kd = kd
        self.ki - ki

        # Used for calculating derivative and integral terms
        self.error_sum = 0
        self.error_prev = 0

        # Used fro storing cmd_vel values and calcualting error
        self.target_vel = Twist()
        self.target_vel.linear.x = 0
        self.target_vel.angular.z = 0

        # Subscriber to read /odom
        self.odom_subscriber = self.create_subscription(Odometry,'/odom',self.update_pid_output,10)

        # Subscriber to read /cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(Twist,'/cmd_vel',self.update_target,10)

        # Publisher to create a PID output
        self.pid_publisher = self.create_publisher(Twist,'pid_output',10)

        '''
        Callback function that calculates the PID output using error. Error is calculated using
        the information from /odom and the target velocity in /cmd_vel. PID constants are initialised 
        in the constructor. Automatically updates the derivative and integral terms. Includes publishing
        to the /pid topic which is read by the STM32
        '''
        def update_pid_output(self,odometry):

            error  = odometry.twist.linear.x - self.target_vel.linear.x

            pid_msg = Twist()

            pid_msg.linear.x = kp*error + kd*(error-self.error_prev) + ki*self.error_sum

            self.pid_publisher.publish(pid_msg)

            self.error_prev = error
            self.error_sum += error

        # Updates the target velocity based on the data in /cmd_vel
        def update_target(self,cmd_vel):
            self.target_vel.linear.x = cmd_vel.linear.x
            self.target_vel.angular.z = cmd_vel.angular.z

# Standard main function
def main(args=None):
    rclpy.init(args=args)

    node = PID_Control(kp=1,kd=0,ki=0)

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

