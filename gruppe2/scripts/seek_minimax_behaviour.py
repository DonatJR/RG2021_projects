from minimax_behaviour import MinimaxBehaviour
from animal_types import AnimalPosAndOrientation, AnimalProperties

import numpy as np

class SeekMinimaxBehaviour(MinimaxBehaviour):
    def __init__(self, animal_properties: AnimalProperties):
        super().__init__(animal_properties)

    def get_velocity_and_omega(self, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation, scan: tuple, enemy_capabilities: AnimalProperties):
        self.current_enemy_properties = enemy_capabilities # enemy capabilities could incorporate more info than just properties in the future, but for now just save it as is
        self.current_enemy_choices = np.linspace(-enemy_capabilities.max_omega, enemy_capabilities.max_omega, self.choices)

        all_move_values = [self._minimax(choice, own_pos, other_pos, self.tree_depth, True) for choice in self.strategy_choices]

        # TODO: when do we want linear velocity not to be the max value?
        return self.animal_properties.max_linear_vel, self.strategy_choices[np.argmin(all_move_values)]

    def _cat_metric(self, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation):
        # TODO: we need to include the cheese positions in here as well somehow
        # maybe adopt some guarding behaviour if both cat and mouse are near a cheese
        # and the mouse wants to reach it (cheese guarding could also be another MiniMax implementation
        # which we can then switch to if we want!)
        own_orientation_vec = np.array([np.cos(own_pos.orientation), np.sin(own_pos.orientation)])
        own_orientation_vec /= np.linalg.norm(own_orientation_vec)

        other_to_own_vector = np.array([other_pos.pos.x - own_pos.pos.x, other_pos.pos.y - own_pos.pos.y])
        other_to_own_vector /= np.linalg.norm(other_to_own_vector)

        angle = np.rad2deg(np.arccos(np.dot(own_orientation_vec, other_to_own_vector)))
        
        # check whether other animal is in front or behind us
        # use a little less than 180 deg to mean 'in front'
        other_behind_own = angle < 290 and angle > 70
        
        if other_behind_own:
            distance_weight = 0.3
            angle_weight = 0.7
        else:
            distance_weight = 0.7
            angle_weight = 0.3

        distance = np.sqrt((other_pos.pos.x - own_pos.pos.x) ** 2 + (other_pos.pos.y - own_pos.pos.y) ** 2)

        # normalize distance (0 / 25 min / max distance)
        distance = (distance - 0) / (25 - 0)

        # normalize angle
        angle = angle / 360

        # emphasize distance and a good angle and weight it based on the fact if other animal (mouse) is behind or in front of ourselves (cat)
        #return np.abs(180 - angle)/-180
        return distance_weight * distance + angle_weight * (1 - np.abs(0.5 - angle))
    
    def _mouse_metric(self, own_pos, other_pos):
        # TODO: it could be (i.e. it is pretty much given) that the mouse uses another system than we are, 
        # so we might try to figure out what this system is and model it here as best as we can
        # for now, just return the same metric as for ourselves 
        return self._cat_metric(own_pos, other_pos)