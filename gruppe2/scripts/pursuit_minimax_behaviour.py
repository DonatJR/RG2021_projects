from minimax_behaviour import MinimaxBehaviour
from animal_types import AnimalPosAndOrientation, AnimalProperties

import numpy as np
import inspect

class PursuitMinimaxBehaviour(MinimaxBehaviour):
    def __init__(self, animal_properties: AnimalProperties):
        super().__init__(animal_properties)

    def get_velocity_and_omega(self, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation, scan: tuple, enemy_capabilities: AnimalProperties):
        self.current_enemy_properties = enemy_capabilities # enemy capabilities could incorporate more info than just properties in the future, but for now just save it as is
        self.current_enemy_choices = np.linspace(-enemy_capabilities.max_omega, enemy_capabilities.max_omega, self.choices)

        all_move_values = [self._minimax(choice, own_pos, other_pos, self.tree_depth, False) for choice in self.strategy_choices]

        # TODO: when do we want linear velocity be another value? use half of max speed for better mobility for now (!! should also be used in system update !!)
        return self.animal_properties.max_linear_vel // 2, self.strategy_choices[np.argmin(all_move_values)]

    def _cat_metric(self, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation):
        raise NotImplementedError(f'{inspect.stack()[0][3]} is not implemented, do not use abstract behaviour class directly')
    
    def _mouse_metric(self, own_pos, other_pos):
        raise NotImplementedError(f'{inspect.stack()[0][3]} is not implemented, do not use abstract behaviour class directly')
