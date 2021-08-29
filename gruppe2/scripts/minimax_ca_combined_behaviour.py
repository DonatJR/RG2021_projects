from mouse_tracker import MouseTracker
from minimax_behaviour import MinimaxBehaviour
from collision_avoidance_behaviour import CollisionAvoidanceBehaviour
from helper_types import PosAndOrientation, AnimalProperties

import numpy as np
import behavior_gates

class CombineMinimaxCaBehaviour():
    """Combines minimax behaviour and collision avoidance and combines them appropriately"""
    def __init__(self, animal_properties: AnimalProperties):
        self.animal_properties = animal_properties
        self.minimax = MinimaxBehaviour(animal_properties)
        self.collision_avoidance = CollisionAvoidanceBehaviour(animal_properties)


    def get_velocity_and_omega(self, own_pos: PosAndOrientation, mouse_tracker: MouseTracker, scan: tuple):
        return self.animal_properties.max_linear_vel, self.__combine(own_pos, mouse_tracker, scan)


    def __combine(self, own_pos, mouse_tracker: MouseTracker, scan):
        _, omega_ca = self.collision_avoidance.get_velocity_and_omega(scan)
        _, omega_minimax = self.minimax.get_velocity_and_omega(own_pos, mouse_tracker)
    
        return self.__utility_function_behaviour_gates(omega_minimax, omega_ca)


    def __utility_function_behaviour_gates(self, omega_minimax, omega_ca):
        return behavior_gates.PREVAIL(omega_minimax, omega_ca)