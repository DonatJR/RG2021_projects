from numpy.core.numeric import NaN
from helper_types import PosAndOrientation, AnimalProperties

import numpy as np


class CollisionAvoidanceBehaviour():
    def __init__(self, animal_properties: AnimalProperties, small_fov=False):
        self.animal_properties = animal_properties
        self.small_fov = small_fov

        if small_fov:
            self.ahead_bounds = (np.radians(330), np.radians(30))
            self.distance_to_obstacle = 0.4
        else:
            self.ahead_bounds = (np.radians(290), np.radians(70))
            self.distance_to_obstacle = 0.4

    def get_velocity_and_omega(self, scan: tuple):
        ranges = scan[0]
        angles = scan[1]
        return self.animal_properties.max_linear_vel, self.__collision_avoidance(ranges, angles)

    def __collision_avoidance(self, ranges, angles):
        fov_ranges = ranges[np.logical_or(
            angles > self.ahead_bounds[0], angles < self.ahead_bounds[1])]
        fov_angles = angles[np.logical_or(
            angles > self.ahead_bounds[0], angles < self.ahead_bounds[1])]

        if len(ranges) == 0:  # can happen when all obstacles are further away than laser can scan
            omega = 0
        # if distance to obstacle is big, just drive forward
        elif (fov_ranges > self.distance_to_obstacle).all():
            omega = 0
        else:
            # normalize ranges
            norm_ranges = (fov_ranges - np.min(fov_ranges)) / \
                (np.max(fov_ranges) - np.min(fov_ranges) + 0.01)

            # compute the angular command velocity omega (turning rate)
            k = 0.001
            force = (4/(norm_ranges + 0.01) - 1) * k
            # opposite force direction than sensor direction
            force_angles = fov_angles - np.pi
            force_y = np.sin(force_angles) * force
            force_y = np.sum(force_y)

            c = 10 if self.small_fov else 3
            omega_max = 2.84

            omega = np.clip(force_y * c, -omega_max, omega_max)

        return omega
