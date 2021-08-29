from helper_types import PosAndOrientation, AnimalProperties, Positition2D, get_distance_between_positions

import numpy as np

# keeps track of what the mouse can do, currently only its max velocity and max turning angle (omega)


class MouseTracker():
    """Keeps track of mouse capabilities and properties"""

    def __init__(self, cheese_pos) -> None:
        self.mouse_properties = AnimalProperties(0, 0)
        self.last_five_velocities = np.zeros(5)
        self.last_velocity_index = 0
        self.cheese_pos = cheese_pos

    def update_capabilities(self, mouse_velocity: int, mouse_omega: int):
        if (mouse_velocity > self.mouse_properties.max_linear_vel):
            self.mouse_properties.max_linear_vel = mouse_velocity

        if (np.abs(mouse_omega) > self.mouse_properties.max_omega):
            self.mouse_properties.max_omega = np.abs(mouse_omega)

        self.last_five_velocities[self.last_velocity_index] = mouse_velocity
        self.last_velocity_index += 1
        self.last_velocity_index = self.last_velocity_index % 5

    def update_position_and_orientation(self, pos_and_orientation: PosAndOrientation):
        self.mouse_pos = pos_and_orientation

        # find cheese target
        min_dist = np.inf
        min_dist_idx = 0
        for idx, cheese in enumerate(self.cheese_pos):
            if get_distance_between_positions(self.mouse_pos.pos, Positition2D(cheese[0], cheese[1])) < min_dist:
                min_dist_idx = idx

        # TODO future: maybe get other nearby cheese positions and decide based on viewing angle??

        self.cheese_target = Positition2D(
            self.cheese_pos[min_dist_idx][0], self.cheese_pos[min_dist_idx][1])

    def get_mouse_capabilities(self) -> AnimalProperties:
        return self.mouse_properties

    def get_mouse_position_and_orientation(self) -> PosAndOrientation:
        return self.mouse_pos

    def get_cheese_target(self):
        return self.cheese_target

    def get_mean_velocity(self):
        return np.mean(self.last_five_velocities)
