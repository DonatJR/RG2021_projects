from helper_types import AnimalPosAndOrientation, AnimalProperties

import numpy as np

class CollisionAvoidanceBehaviour():
    def __init__(self, animal_properties: AnimalProperties):
        self.animal_properties = animal_properties
        self.distance_to_obstacle = 1
        self.ahead_bounds = (np.radians(290), np.radians(70))

    def get_velocity_and_omega(self, scan: tuple):
        ranges = scan[0]
        angles = scan[1]
        return self.animal_properties.max_linear_vel, self.__collision_avoidance(ranges, angles)        

    def __collision_avoidance(self, ranges, angles):
        if len(ranges) == 0: # can happen when all obstacles are further away than laser can scan
            omega = 0
        elif (ranges[np.logical_or(angles > np.radians(self.ahead_bounds[0]), angles < self.ahead_bounds[1])] > self.distance_to_obstacle).all(): # if distance to obstacle is big, just drive forward
            omega = 0
        else:
            # normalize ranges 
            norm_ranges = (ranges - np.min(ranges)) / (np.max(ranges) - np.min(ranges))

            # compute the angular command velocity omega (turning rate)
            k = 0.001
            force = (4/norm_ranges - 1) * k 
            force_angles = angles - np.pi # opposite force direction than sensor direction
            force_y = np.sin(force_angles) * force
            force_y = np.sum(force_y) # or use np.mean()
            c = 3.0
            omega_max = 2.84
            omega = np.clip(force_y * c, -omega_max, omega_max)

        return omega
