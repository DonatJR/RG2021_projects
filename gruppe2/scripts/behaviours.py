from numpy.lib.function_base import angle
from animal_types import AnimalPosAndOrientation, AnimalProperties, Positition2D
import inspect
import numpy as np

class AbstractBehaviour():
    def get_velocity_and_omega(self, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation,  scan: tuple):
        raise NotImplementedError(f'{inspect.stack()[0][3]} is not implemented, do not use abstract behaviour class directly')

class MinimaxBehaviour(AbstractBehaviour):
    def __init__(self, animal_properties: AnimalProperties):
        self.animal_properties = animal_properties

        # TODO: how many choices do we want?
        self.strategy_choices = np.linspace(-self.animal_properties.max_omega, self.animal_properties.max_omega, 8)

    def get_velocity_and_omega(self, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation, scan: tuple):
        # TODO: which depth do we want?
        all_move_values = [self.__minimax(choice, own_pos, other_pos, depth = 3) for choice in self.strategy_choices]

        return self.animal_properties.max_linear_vel, self.strategy_choices[np.argmin(all_move_values)]

    def __minimax(self, angular_vel: float, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation, depth: int, alpha: float = -np.inf, beta: float = np.inf):
        if depth == 0:
            return self.__metric(own_pos, other_pos)

        # TODO: system update did never update the position of the other animal (even in original code), but shouldn't it do that also for minimax to work? :thinking: 
        new_own_pos = self.__system_update(angular_vel, own_pos)
            
        value = np.inf
        for choice in self.strategy_choices:
            new_value = self.__minimax(choice, new_own_pos, other_pos, depth-1, alpha, beta)
            value = min(value, new_value)
            beta = min(beta, value)

            if value <= alpha:
                break

        return value

    def __metric(self, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation):
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
    
    def __system_update(self, angular_vel: float, own_pos: AnimalPosAndOrientation):
        new_x = own_pos.pos.x
        new_y = own_pos.pos.y
        new_orientation = own_pos.orientation

        speed = self.animal_properties.max_linear_vel
        iterations = 10
        dt = 1.0 / iterations

        for _ in range(iterations):
            new_x += np.cos(new_orientation) * speed * dt
            new_y += np.sin(new_orientation) * speed * dt
            new_orientation += angular_vel * dt

        return AnimalPosAndOrientation(new_x, new_y, new_orientation)

class CollisionAvoidanceBehaviour(AbstractBehaviour):
    def __init__(self, animal_properties: AnimalProperties):
        self.animal_properties = animal_properties

    def get_velocity_and_omega(self, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation, scan: tuple):
        ranges = scan[0]
        angles = scan[1]
        return self.animal_properties.max_linear_vel, self.__collision_avoidance(ranges, angles)        

    def __collision_avoidance(self, ranges, angles):
        if len(ranges) == 0: # can happen when all obstacles are further away than laser can scan
            omega = 0
        else:
            # normalize ranges 
            norm_ranges = (ranges - np.min(ranges)) / (np.max(ranges) - np.min(ranges))

            # compute the angular command velocity omega (turning rate)
            k = 0.001
            force = (4/norm_ranges - 1) * k 
            force_angles = angles - np.pi # opposite force direction than sensor direction
            force_y = np.sin(force_angles) * force
            force_y = np.sum(force_y) # or use np.mean()
            c = 3.0
            omega_max = 2.84
            omega = np.clip(force_y * c, -omega_max, omega_max)

        return omega



def get_all_behaviours():
    return { 'minimax': MinimaxBehaviour,
             'collision_avoidance': CollisionAvoidanceBehaviour}