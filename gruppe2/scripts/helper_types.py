import numpy as np


class AnimalProperties:
    def __init__(self, max_linear_vel, max_omega) -> None:
        self.max_linear_vel: float = max_linear_vel
        self.max_omega: float = max_omega


class Positition2D:
    def __init__(self, x, y):
        self.x: float = x
        self.y: float = y


class PosAndOrientation:
    def __init__(self, x, y, orientation):
        self.pos = Positition2D(x, y)
        self.orientation: float = orientation


def get_angle_between_positions(pos_one: PosAndOrientation, pow_two: PosAndOrientation):
    x_diff = pow_two.pos.x - pos_one.pos.x
    y_diff = pow_two.pos.y - pos_one.pos.y
    angle_towards_second_pos = np.arctan2(y_diff, x_diff)

    return np.arctan2(np.sin(angle_towards_second_pos - pos_one.orientation), np.cos(angle_towards_second_pos - pos_one.orientation))


def get_distance_between_positions(pos_one: Positition2D, pos_two: Positition2D):
    return np.sqrt((pos_two.x - pos_one.x) ** 2 + (pos_two.y - pos_one.y) ** 2)
