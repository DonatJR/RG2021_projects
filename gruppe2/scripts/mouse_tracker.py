from helper_types import AnimalPosAndOrientation, AnimalProperties

import numpy as np

# keeps track of what the mouse can do, currently only its max velocity and max turning angle (omega) 
class MouseTracker():
    """Keeps track of mouse capabilities and properties"""
    def __init__(self) -> None:
        self.mouse_properties = AnimalProperties(0, 0)
        self.last_five_velocities = np.zeros(5)
        self.last_velocity_index = 0

    def update_capabilities(self, mouse_velocity: int, mouse_omega: int):
        if (mouse_velocity > self.mouse_properties.max_linear_vel):
            self.mouse_properties.max_linear_vel = mouse_velocity

        if (np.abs(mouse_omega) > self.mouse_properties.max_omega):
            self.mouse_properties.max_omega = np.abs(mouse_omega)

        self.last_five_velocities[self.last_velocity_index] = mouse_velocity
        self.last_velocity_index += 1
        self.last_velocity_index = self.last_velocity_index % 5

    def update_position_and_orientation(self, pos_and_orientation: AnimalPosAndOrientation):
        self.mouse_pos = pos_and_orientation
    
    def update_cheese_target(self):
        pass

    def get_mouse_capabilities(self) -> AnimalProperties:
        return self.mouse_properties

    def get_mouse_position_and_orientation(self) -> AnimalPosAndOrientation:
        return self.mouse_pos

    def get_cheese_target(self):
        pass

    def get_mean_velocity(self):
        return np.mean(self.last_five_velocities)