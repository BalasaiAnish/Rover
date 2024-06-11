import rclpy 
from rclpy.node import Node

# Import odometry to get feedback from the ekf
from nav_msgs.msg import Odometry

# Import twist to read cmd_vel and provide final output to the STM32
from geometry_msgs.msg import Twist

# Defining a class to store PID constants, keep track of errors, and calculate PID output
class PID():

    # Initialise PID constants, derivative, and integral terms
    def __init__(self,kp:float,kd:float,ki:float):
        self.kp = kp
        self.kd = kd
        self.ki = ki

        self.error_prev = 0.0
        self.error_sum = 0.0

    # Define a function to calculate a PID output and update derivative and integral terms
    def compute(self,target:float,current:float) ->float:
        error = target - current

        pid_output = self.kp*error + self.kd*(error-self.error_prev) + self.ki*self.error_sum

        self.error_prev = error
        self.error_sum += error

        return pid_output
    

class PID_Control(Node):
    '''
    Initialises the following
    -> A seperate set of PID constants for linear and angular velocity control
    -> A publlisher for PID output and subscribers for /odom and /cmd_vel
    -> A self.target term  to store the value in /cmd_vel
    -> Two PID objects to handle the linear and angular PID
    '''
    def __init__(self,kp_linear,kd_linear,ki_linear,kp_angular,kd_angular,ki_angular):
        super().__init__('pid_controller')
        self.pid_linear = PID(kp=kp_linear,kd=kd_linear,ki=ki_linear)
        self.pid_angular = PID(kp=kp_angular,kd=kd_angular,ki=ki_angular)

        self.pid_publisher = self.create_publisher(Twist,'/pid_output',10)

        self.odom_subscriber = self.create_subscription(Odometry,'/odom',self.update_pid_output,10)

        self.cmd_vel_subscriber = self.create_subscription(Twist,'/cmd_vel',self.update_target,10)

        self.target = Twist()
    
    '''
    Callback function for /odom
    Performs the following operations
    -> Compute PID output for both linear and angular velocity
    -> Publish the same to the /pid_output topic
    -> Any additional PID tasks are handled by the PID objects
    '''
    def update_pid_output(self,odometry:Odometry):
        pid_output = Twist()

        pid_output.linear.x = self.pid_linear.compute(self.target.linear.x,odometry.twist.linear.x)
        pid_output.angular.z = self.pid_angular.compute(self.target.angular.z,odometry.twist.angular.z)

        self.pid_publisher.publish(pid_output)

    '''
    Callback function for /cmd_vel
    Performs the following operations
    -> Uses the data in cmd_vel to update the targets, 
    targets are stored as part of the PID_Control() class
    '''    
    def update_target(self,cmd_vel:Twist):
        self.target.linear.x = cmd_vel.linear.x
        self.target.angular.z = cmd_vel.angular.z
    
    

class PID_Control_Old(Node):
    
    def __init__(self,kp,kd,ki):
        super().__init__('pid_control')

        # Initialise PID constants, change in main function
        self.kp = kp
        self.kd = kd
        self.ki = ki

        # Used for calculating derivative and integral terms
        self.error_sum = 0.0
        self.error_prev = 0.0

        # Used fro storing cmd_vel values and calcualting error
        self.target_vel = Twist()
        self.target_vel.linear.x = 0.0
        self.target_vel.angular.z = 0.0

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

            pid_msg.linear.x = self.kp*error + self.kd*(error-self.error_prev) + self.ki*self.error_sum

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

    node = PID_Control(kp_linear=1,kd_linear=0,ki_linear=0,kp_angular=0.1,kd_angular=0,ki_angular=0)

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

