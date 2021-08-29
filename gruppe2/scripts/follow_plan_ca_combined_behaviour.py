from mouse_tracker import MouseTracker
from follow_plan_behaviour import FollowPlanBehaviour
from collision_avoidance_behaviour import CollisionAvoidanceBehaviour
from helper_types import PosAndOrientation, AnimalProperties

import behavior_gates

class CombineFollowPlanCaBehaviour():
    """Combines follow plan behaviour and collision avoidance and combines them appropriately"""
    def __init__(self, animal_properties: AnimalProperties):
        self.animal_properties = animal_properties
        self.follow_plan = FollowPlanBehaviour(animal_properties)
        self.collision_avoidance = CollisionAvoidanceBehaviour(animal_properties)


    def get_velocity_and_omega(self, own_pos: PosAndOrientation, other_pos: PosAndOrientation, scan: tuple):
        return self.animal_properties.max_linear_vel, self.__combine(own_pos, other_pos, scan)


    def __combine(self, own_pos, other_pos: PosAndOrientation, scan):
        _, omega_ca = self.collision_avoidance.get_velocity_and_omega(scan)
        _, omega_follow_plan = self.follow_plan.get_velocity_and_omega(own_pos, other_pos, scan)
    
        return self.__utility_function_behaviour_gates(omega_follow_plan, omega_ca)


    def __utility_function_behaviour_gates(self, omega_follow_plan, omega_ca):
        return behavior_gates.PREVAIL(omega_follow_plan, omega_ca)