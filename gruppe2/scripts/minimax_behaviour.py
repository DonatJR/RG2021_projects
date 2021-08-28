from helper_types import AnimalPosAndOrientation, AnimalProperties, get_angle_to_other_robot

import numpy as np

class MinimaxBehaviour():
    """Uses minimax algorithm to find best cmd_vel properties to minimize distance to mouse and keep mouse away from cheese (based on a simple assumption that the mouse wants to run away from the cat)"""
    def __init__(self, animal_properties: AnimalProperties):
        self.animal_properties = animal_properties

        # anymore and calculations take too long
        self.tree_depth = 3

        # anymore and calculations take too long
        self.choices = 7

        self.strategy_choices = np.linspace(-self.animal_properties.max_omega, self.animal_properties.max_omega, self.choices)

    def get_velocity_and_omega(self, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation, enemy_capabilities: AnimalProperties, mean_enemy_vel: float):
        self.current_enemy_properties = enemy_capabilities # enemy capabilities could incorporate more info than just properties in the future, but for now just save it as is
        self.current_enemy_choices = np.linspace(-enemy_capabilities.max_omega, enemy_capabilities.max_omega, self.choices)

        all_move_values = [self._minimax(choice, own_pos, other_pos, self.tree_depth, mean_enemy_vel, True) for choice in self.strategy_choices]

        return self.animal_properties.max_linear_vel, self.strategy_choices[np.argmin(all_move_values)]

    def _minimax(self, angular_vel: float, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation, mean_enemy_vel: float, depth: int, optimize_for_cat: bool, alpha: float = -np.inf, beta: float = np.inf):
        if depth == 0:
            return self._cat_metric(own_pos, other_pos) if optimize_for_cat else self._mouse_metric(own_pos, other_pos)

        new_own_pos, new_other_pos = self.__system_update(angular_vel, own_pos, other_pos, mean_enemy_vel, optimize_for_cat)

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
        # TODO: we need to include the cheese positions in here as well somehow
        own_orientation_vec = np.array([np.cos(own_pos.orientation), np.sin(own_pos.orientation)])
        own_orientation_vec /= np.linalg.norm(own_orientation_vec)

        other_to_own_vector = np.array([other_pos.pos.x - own_pos.pos.x, other_pos.pos.y - own_pos.pos.y])
        other_to_own_vector /= np.linalg.norm(other_to_own_vector)

        angle = np.rad2deg(get_angle_to_other_robot(own_pos, other_pos))

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

        # normalize distance (0 / 25 for min / max distance)
        distance = (distance - 0) / (25 - 0)

        # normalize angle
        angle = angle / 360

        # emphasize distance and a good angle and weight it based on the fact if other animal (mouse) is behind or in front of ourselves (cat)
        #return np.abs(180 - angle)/-180
        return distance_weight * distance + angle_weight * (1 - np.abs(0.5 - angle))
    
    def _mouse_metric(self, own_pos, other_pos):
        # it could be (i.e. it is pretty much given) that the mouse uses another system than we are, 
        # so we might try to figure out what this system is and model it here as best as we can
        # for now, just return the same metric as for ourselves 
        return self._cat_metric(own_pos, other_pos)
    
    def __system_update(self, angular_vel: float, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation, mean_enemy_vel:float, optimize_for_cat: bool):
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
            speed = mean_enemy_vel # TODO: maybe the mouse does not alwayse use max speed? make a rolling average over the last few velocities?

            for _ in range(iterations):
                new_x += np.cos(new_orientation) * speed * dt
                new_y += np.sin(new_orientation) * speed * dt
                new_orientation += angular_vel * dt

            return own_pos, AnimalPosAndOrientation(new_x, new_y, new_orientation)