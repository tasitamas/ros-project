import logging
import os

class Logger:
    def __init__(self):
        log_file = os.path.expanduser("~/ros2_ws/src/ros2_course/ros2_course/test.log")
        logging.basicConfig(filename=log_file, 
                            format='[%(asctime)s] %(message)s',
                            datefmt='%Y.%m.%d %H:%M:%S', 
                            filemode='w') 
        self.logger = logging.getLogger()
        self.logger.setLevel(logging.INFO)
        

    def log_pose(self, pose):
        x = pose.x
        y = pose.y
        theta = pose.theta
        self.logger.info(f'[INFO]: Pose: x={round(x, 4)}, y={round(y, 4)}, theta={round(theta, 4)}')

    def log_turn(self, radians):
        self.logger.info(f'[INFO]: Turned by {round(radians, 4)} radians.')
    
    def log_message(self, message):
        self.logger.info(f'[INFO]: {message}')
    
    def log_error(self, message):
        self.logger.error(message)
    
    def log_success(self, message):
        self.logger.info(f'[SUCCESS]: {message}')