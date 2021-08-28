from helper_types import AnimalProperties

import numpy as np

class FreeSpaceBehaviour():
    def __init__(self, animal_properties: AnimalProperties):
        self.animal_properties = animal_properties

    def get_velocity_and_omega(self, scan: tuple):
        ranges = scan[0]
        angles = scan[1]
        return self.animal_properties.max_linear_vel, self.__free_space(ranges, angles)        

    def __free_space(self, ranges, angles):
        n = 50
        ids = np.arange(len(ranges)) // n
        ranges_average = np.bincount(ids, ranges) / np.bincount(ids)
        angles_average = np.bincount(ids, angles) / np.bincount(ids)
        i = np.argmax(ranges_average)
        freespace_range = ranges_average[i]
        freespace_angle = angles_average[i]

        F_y = np.sin(freespace_angle) * freespace_range
        omega = F_y * 1.0  # scaling force to rotational speed
        return omega