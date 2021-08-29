from mouse_tracker import MouseTracker
from follow_plan_behaviour import FollowPlanBehaviour
from collision_avoidance_behaviour import CollisionAvoidanceBehaviour
from helper_types import PosAndOrientation, AnimalProperties

import behavior_gates
import numpy as np

class CombineFollowPlanCaBehaviour():
    """Combines follow plan behaviour and collision avoidance and combines them appropriately"""
    def __init__(self, animal_properties: AnimalProperties):
        self.animal_properties = animal_properties
        self.follow_plan = FollowPlanBehaviour(animal_properties)
        self.collision_avoidance = CollisionAvoidanceBehaviour(animal_properties, small_fov = True)


    def get_velocity_and_omega(self, own_pos: PosAndOrientation, other_pos: PosAndOrientation, scan: tuple):
        return self.__combine(own_pos, other_pos, scan)


    def __combine(self, own_pos, other_pos: PosAndOrientation, scan):
        _, omega_follow_plan = self.follow_plan.get_velocity_and_omega(own_pos, other_pos, scan)
        if omega_follow_plan is None:
            return None, None

        _, omega_ca = self.collision_avoidance.get_velocity_and_omega(scan)
    
        omega_combined = self.__utility_function_behaviour_gates(omega_follow_plan, omega_ca)
        return self.animal_properties.max_linear_vel if omega_combined != omega_ca else 0, omega_combined


    def __utility_function_behaviour_gates(self, omega_follow_plan, omega_ca):
        if omega_ca == 0:
            return omega_follow_plan

        if np.abs(omega_ca) > 2.0:
            return omega_ca

        if np.abs(np.pi - omega_follow_plan) < 0.1:
            if np.sign(omega_follow_plan) != np.sign(omega_ca):
                omega_follow_plan *= -1

        return behavior_gates.PREVAIL(omega_follow_plan, omega_ca)