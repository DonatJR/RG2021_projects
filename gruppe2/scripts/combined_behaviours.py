from math import pi
from minimax_behaviour import MinimaxBehaviour
from mouse_tracker import MouseTracker
from follow_plan_ca_combined_behaviour import CombineFollowPlanCaBehaviour
from minimax_ca_combined_behaviour import CombineMinimaxCaBehaviour
from recovery_behaviours import RecoveryBehaviours
from helper_types import AnimalProperties, PosAndOrientation, get_angle_between_positions, get_distance_between_positions

import numpy as np

class CombinedBehaviours():
    """Switches between and selects the best behaviour to use depending on the game state / current situation"""
    def __init__(self, animal_properties: AnimalProperties):
        self.minimax_behaviour = MinimaxBehaviour(animal_properties)
        self.minimax_ca_behaviour = CombineMinimaxCaBehaviour(animal_properties)
        self.follow_plan_ca_behaviour = CombineFollowPlanCaBehaviour(animal_properties)
        # self.recovery_behaviour = RecoveryBehaviours(animal_properties)

    def get_velocity_and_omega(self, own_pos: PosAndOrientation, mouse_tracker: MouseTracker, scan: tuple):
        other_pos = mouse_tracker.get_mouse_position_and_orientation()

        # if self.__needs_recovery():
        #     return self.recovery_behaviour()

        if self.__is_mouse_very_close_and_visible(own_pos, other_pos, scan):
            return self.minimax_behaviour.get_velocity_and_omega(own_pos, mouse_tracker)
        elif self.__is_mouse_close_and_visible(own_pos, other_pos, scan):
            return self.minimax_ca_behaviour.get_velocity_and_omega(own_pos, mouse_tracker, scan)
        else:
            return self.follow_plan_ca_behaviour.get_velocity_and_omega(own_pos, other_pos, scan)

    def __is_mouse_very_close_and_visible(self, own_pos, other_pos, scan):
        distance = get_distance_between_positions(own_pos.pos, other_pos.pos)
        return distance < 0.8 and self.__is_other_robot_visible(own_pos, other_pos, scan, distance)

    def __is_mouse_close_and_visible(self, own_pos, other_pos, scan):
        distance = get_distance_between_positions(own_pos.pos, other_pos.pos)
        return distance < 1.8 and self.__is_other_robot_visible(own_pos, other_pos, scan, distance)

    def __is_other_robot_visible(self, own_pos, other_pos, scan, distance):
        scan_pointing_to_other_robot = self.__find_scan_pointing_to_other_robot(own_pos, other_pos, scan[1])
        scan_distance_to_other_robot = scan[0][scan_pointing_to_other_robot]
        other_robot_visible = np.abs(scan_distance_to_other_robot - distance) < 0.1 # if real distance and distance of laser are close to each other, there can be no obstacle between them

        return other_robot_visible

    def __find_scan_pointing_to_other_robot(self, own_pos, other_pos, angles):
        angle_to_other_robot = get_angle_between_positions(own_pos, other_pos)
        smallest_diff = np.inf

        for idx, angle in enumerate(angles):
            clockwise_angle = (2*np.pi - angle) # laserscans on turtlebot are ccw
            new_diff = np.abs(clockwise_angle - angle_to_other_robot)
            if new_diff < smallest_diff:
                smallest_diff = new_diff
                best_idx = idx

        return best_idx

    # def __needs_recovery(self):
    #     return False # recovery is not implemented and also not really useful for winning the game: if we need it we have lost anyway
