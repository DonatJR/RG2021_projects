from minimax_behaviour import MinimaxBehaviour
from animal_types import AnimalPosAndOrientation, AnimalProperties

import numpy as np

class PursuitMinimaxBehaviour(MinimaxBehaviour):
    def __init__(self, animal_properties: AnimalProperties):
        super().__init__(animal_properties)

    def get_velocity_and_omega(self, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation, scan: tuple, enemy_capabilities: AnimalProperties):
        self.current_enemy_properties = enemy_capabilities # enemy capabilities could incorporate more info than just properties in the future, but for now just save it as is
        self.current_enemy_choices = np.linspace(-enemy_capabilities.max_omega, enemy_capabilities.max_omega, self.choices)

        all_move_values = [self.__minimax(choice, own_pos, other_pos, self.tree_depth, False) for choice in self.strategy_choices]

        # TODO: when do we want linear velocity not to be the max value?
        return self.animal_properties.max_linear_vel, self.strategy_choices[np.argmin(all_move_values)]

    def __minimax(self, angular_vel: float, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation, depth: int, optimize_for_cat: bool, alpha: float = -np.inf, beta: float = np.inf):
        return super().__minimax(angular_vel, own_pos, other_pos, depth, optimize_for_cat, alpha, beta)

    def __cat_metric(self, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation):
        # TODO: we need to include the cheese positions in here as well somehow
        # maybe adopt some guarding behaviour if both cat and mouse are near a cheese
        # and the mouse wants to reach it (cheese guarding could also be another MiniMax implementation
        # which we can then switch to if we want!)
        own_orientation_vec = np.array([np.cos(own_pos.orientation), np.sin(own_pos.orientation)])
        own_orientation_vec /= np.linalg.norm(own_orientation_vec)

        other_to_own_vector = np.array([other_pos.pos.x - own_pos.pos.x, other_pos.pos.y - own_pos.pos.y])
        other_to_own_vector /= np.linalg.norm(other_to_own_vector)

        angle = np.rad2deg(np.arccos(np.dot(own_orientation_vec, other_to_own_vector)))
        
        # angle > 270 or < 90 means other animal (mouse) is behind ourselves (cat)
        # use a little less than 180 deg to mean 'in front', i.e. angle > 290 or angle < 70
        other_behind_own = angle > 290 or angle < 70

        if other_behind_own:
            distance_weight = 0.1
            angle_weight = 0.9
        else:
            distance_weight = 0.9
            angle_weight = 0.1

        distance = np.sqrt((other_pos.pos.x - own_pos.pos.x) ** 2 + (other_pos.pos.y - own_pos.pos.y) ** 2)

        # emphasize distance and a good angle and weight it based on the fact if other animal (mouse) is behind or in front of ourselves (cat)
        return distance_weight * distance + angle_weight * np.abs(180 - angle)
    
    def __mouse_metric(self, own_pos, other_pos):
        # TODO: it could be (i.e. it is pretty much given) that the mouse uses another system than we are, 
        # so we might try to figure out what this system is and model it here as best as we can
        # for now, just return the same metric as for ourselves 
        return self.__cat_metric(own_pos, other_pos)