import numpy as np

class AnimalProperties:
    def __init__(self, max_linear_vel, max_omega) -> None:
        self.max_linear_vel: float = max_linear_vel
        self.max_omega: float = max_omega

class Positition2D:
    def __init__(self, x, y):
        self.x: float = x
        self.y: float = y

class AnimalPosAndOrientation:
    def __init__(self, x, y, orientation):
        self.pos = Positition2D(x, y)
        self.orientation: float = orientation

def get_angle_to_other_robot(own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation):
    own_orientation_vec = np.array([np.cos(own_pos.orientation), np.sin(own_pos.orientation)])
    own_orientation_vec /= np.linalg.norm(own_orientation_vec)

    other_to_own_vector = np.array([other_pos.pos.x - own_pos.pos.x, other_pos.pos.y - own_pos.pos.y])
    other_to_own_vector /= np.linalg.norm(other_to_own_vector)

    angle_to_other_robot = np.arccos(np.dot(own_orientation_vec, other_to_own_vector))

    return angle_to_other_robot