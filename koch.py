import math

DEGREE_60 = math.pi / 3
DEGREE_MINUS_120 = -2 * math.pi / 3

class Koch:
    def __init__(self, turtlesim_controller):
        self.turtle = turtlesim_controller
    
    def draw_koch_curve(self, steps, length):
        if steps == 0:
            self.turtle.move_forward(length)
        else:
            for angle in [DEGREE_60, DEGREE_MINUS_120, DEGREE_60, 0]:
                self.draw_koch_curve(steps - 1, length / 3)
                self.turtle.turn(angle)

    def draw_koch_snowflake(self, steps, length):
        for _ in range(3):
            self.draw_koch_curve(steps, length)
            self.turtle.turn(DEGREE_MINUS_120)