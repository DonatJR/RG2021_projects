from abstract_behaviour import AbstractBehaviour
from animal_types import AnimalPosAndOrientation, AnimalProperties

import numpy as np
import inspect

class MinimaxBehaviour(AbstractBehaviour):
    def __init__(self, animal_properties: AnimalProperties):
        self.animal_properties = animal_properties

        # TODO: which depth do we want?
        self.tree_depth = 3

        # TODO: how many choices do we want?
        self.choices = 7

        self.strategy_choices = np.linspace(-self.animal_properties.max_omega, self.animal_properties.max_omega, self.choices)

    def get_velocity_and_omega(self, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation, scan: tuple, enemy_capabilities: AnimalProperties):
        # TODO: these next two lines are just here so the minimax function does not show errors in the editor, maybe find a nicer solution later on
        self.current_enemy_properties = enemy_capabilities # enemy capabilities could incorporate more info than just properties in the future, but for now just save it as is
        self.current_enemy_choices = np.linspace(-enemy_capabilities.max_omega, enemy_capabilities.max_omega, self.choices)

        raise NotImplementedError(f'{inspect.stack()[0][3]} is not implemented, do not use abstract behaviour class directly')

    def _minimax(self, angular_vel: float, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation, depth: int, optimize_for_cat: bool, alpha: float = -np.inf, beta: float = np.inf):
        if depth == 0:
            return self._cat_metric(own_pos, other_pos) if optimize_for_cat else self._mouse_metric(own_pos, other_pos)

        new_own_pos, new_other_pos = self.__system_update(angular_vel, own_pos, other_pos, optimize_for_cat)

        if optimize_for_cat:            
            value = np.inf
            for choice in self.strategy_choices:
                new_value = self._minimax(choice, new_own_pos, new_other_pos, depth-1, False, alpha, beta)
                value = min(value, new_value)
                beta = min(beta, value)

                if value <= alpha:
                    break

        else:
            value = -np.inf
            for choice in self.current_enemy_choices:
                new_value = self._minimax(choice, new_own_pos, new_other_pos, depth-1, True, alpha, beta)
                value = max(value, new_value)
                alpha = max(alpha, value)

                if value >= beta:
                    break

        return value


    def _cat_metric(self, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation):
        raise NotImplementedError(f'{inspect.stack()[0][3]} is not implemented, do not use abstract behaviour class directly')

    def _mouse_metric(self, own_pos, other_pos):
        raise NotImplementedError(f'{inspect.stack()[0][3]} is not implemented, do not use abstract behaviour class directly')
    
    def __system_update(self, angular_vel: float, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation, optimize_for_cat: bool):
        iterations = 10
        dt = 1.0 / iterations

        if optimize_for_cat:
            new_x = own_pos.pos.x
            new_y = own_pos.pos.y
            new_orientation = own_pos.orientation
            speed = self.animal_properties.max_linear_vel

            for _ in range(iterations):
                new_x += np.cos(new_orientation) * speed * dt
                new_y += np.sin(new_orientation) * speed * dt
                new_orientation += angular_vel * dt

            return AnimalPosAndOrientation(new_x, new_y, new_orientation), other_pos
        
        else:
            new_x = other_pos.pos.x
            new_y = other_pos.pos.y
            new_orientation = other_pos.orientation
            speed = self.current_enemy_properties.max_linear_vel # TODO: maybe the mouse does not alwayse use max speed? make a rolling average over the last few velocities?

            for _ in range(iterations):
                new_x += np.cos(new_orientation) * speed * dt
                new_y += np.sin(new_orientation) * speed * dt
                new_orientation += angular_vel * dt

            return own_pos, AnimalPosAndOrientation(new_x, new_y, new_orientation)