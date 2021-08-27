from abstract_behaviour import AbstractBehaviour
from minimax_behaviour import MinimaxBehaviour
from collision_avoidance_behaviour import CollisionAvoidanceBehaviour
from freespace_behaviour import FreeSpaceBehaviour
from animal_types import AnimalPosAndOrientation, AnimalProperties

import numpy as np
import behavior_gates

class CombineMinimaxCa(AbstractBehaviour):
    def __init__(self, animal_properties: AnimalProperties):
        self.animal_properties = animal_properties
        self.minimax = MinimaxBehaviour(animal_properties)
        self.collision_avoidance = CollisionAvoidanceBehaviour(animal_properties)
        self.free_space = FreeSpaceBehaviour(animal_properties)

        self.distance_to_obstacle = 1
        self.ahead_bounds = (np.radians(290), np.radians(70))

    def get_velocity_and_omega(self, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation, scan: tuple, enemy_capabilities: AnimalProperties):
        return self.animal_properties.max_linear_vel, self.__combine(own_pos, other_pos, scan, enemy_capabilities)

    def __combine(self, own_pos, other_pos, scan, enemy_capabilities):
        _, omega_ca = self.collision_avoidance.get_velocity_and_omega(own_pos, other_pos, scan, enemy_capabilities)
        _, omega_minimax = self.minimax.get_velocity_and_omega(own_pos, other_pos, scan, enemy_capabilities)
        _, omega_fs = self.free_space.get_velocity_and_omega(own_pos, other_pos, scan, enemy_capabilities)
        ranges = scan[0]
        angles = scan[1]

        # return self.__utility_function(own_pos, other_pos, ranges, omega_minimax, omega_ca)
        return self.__utility_function_behaviour_gates(omega_minimax, omega_ca, omega_fs)
        # return self.__base_function(ranges, angles, omega_minimax, omega_ca)

    def __base_function(self, ranges, angles, omega_minimax, omega_ca):
        # use minimax and ca only when wall is near
        # Problem: he drives towards the wall, then collision avoidance takes effect, he drives away from the wall, then minimax takes effect again, which drives him towards the wall again.
        if (ranges[np.logical_or(angles > np.radians(self.ahead_bounds[0]), angles < self.ahead_bounds[1])] > self.distance_to_obstacle).all(): # if distance to obstacle is big, use minimax
            omega = omega_minimax
        else:
            omega = omega_ca
        return omega

    def __utility_function(self, own_pos, other_pos, ranges, omega_minimax, omega_ca):
        # doesn't work at all
        util_ca = 2/np.min(ranges) * 0.02  
        util_ca = np.clip(util_ca, 0, 1)

        euclid_dist = np.sqrt((other_pos.pos.x - own_pos.pos.x)**2 + (other_pos.pos.y - own_pos.pos.y)**2)
        util_ho = 3/euclid_dist * 0.01 + 0.08 
        util_ho = np.clip(util_ho, 0, 1)

        print(util_ho)
        print(util_ca)

        omega = (util_ho * omega_minimax) *  (util_ca * omega_ca)
        omega_max = 2.84
        omega = np.clip(omega, -omega_max, omega_max)
        return omega

    def __utility_function_behaviour_gates(self, omega_minimax, omega_ca, omega_fs):
        gate1 = behavior_gates.INVOKE(omega_minimax, omega_fs)
        gate2 = behavior_gates.PREVAIL(omega_minimax, omega_ca)
        gate3 = behavior_gates.PREVAIL(gate1, omega_ca)
        return behavior_gates.OR(gate2, gate3)