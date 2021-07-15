class AnimalProperties:
    def __init__(self, linear_vel, max_omega) -> None:
        self.max_linear_vel = linear_vel
        self.max_omega = max_omega

class Positition2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class AnimalPosAndOrientation:
    def __init__(self, x, y, orientation):
        self.pos = Positition2D(x, y)
        self.orientation = orientation