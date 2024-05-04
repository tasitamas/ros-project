import rclpy
import math

from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from .logger import Logger
from .koch import Koch

LINEAR_SPEED = 5.0
ANGULAR_SPEED = 2.0
WAITING_MESSAGE = 'Waiting for the turtlesim\'s starting position...'

class TurtlesimController(Node):
    def __init__(self, logger):
        super().__init__('turtlesim_controller')
        
        self.speed = LINEAR_SPEED
        self.turn_speed = ANGULAR_SPEED
        
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, 'turtle1/pose', self.update_pose, 10)
        
        self.pose = Pose()
        self.logger = logger
        
        self.logger.log_message(WAITING_MESSAGE)

    def move(self, linear, angular):
        msg = Twist()
        
        msg.linear.x = linear * self.speed
        msg.angular.z = angular * self.turn_speed
        
        self.publisher.publish(msg)

    def turn(self, radians, tol=0.01):
        target_theta = self.pose.theta + radians
        
        while not self.has_reached_angle(target_theta, tol):
            self.move(0.0, self.calculate_turning_speed(target_theta))
            rclpy.spin_once(self)
        
        self.move(0.0, 0.0)
        
        self.logger.log_turn(radians)
    
    def move_forward(self, distance):
        start_pose = self.pose
        self.move(1.0, 0.0)
        
        while not self.has_reached_distance(start_pose, distance):
            rclpy.spin_once(self)
        
        self.move(0.0, 0.0)
        
        self.logger.log_message(f'Moved forward by {round(distance,4)} units.')
    
    def calculate_turning_speed(self, target_theta):
        error = self.normalize_angle(target_theta - self.pose.theta)
        return 0.5 * math.copysign(1, error)

    def update_pose(self, msg):
        self.pose = msg
        
        self.logger.log_pose(self.pose)

    def has_reached_distance(self, start_pose, distance):
        return math.pow(self.pose.x - start_pose.x, 2) + math.pow(self.pose.y - start_pose.y, 2) >= math.pow(distance, 2)

    def has_reached_angle(self, target_theta, tol):
        error = self.normalize_angle(target_theta - self.pose.theta)
        return abs(error) <= tol

    @staticmethod
    def normalize_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
def main(args=None):
    try:
        rclpy.init(args=args)
        
        logger = Logger()
        turtlesim_controller = TurtlesimController(logger)
        koch_drawer = Koch(turtlesim_controller)
        
        koch_drawer.draw_koch_curve(1,1.0)
        #koch_drawer.draw_koch_snowflake(3, 1.0)
        
        logger.log_success("Koch curve/snowflake drawn without errors.")
    except Exception as e:
        logger.log_error(e)
    finally:
        turtlesim_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()