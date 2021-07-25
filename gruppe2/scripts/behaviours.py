from numpy.lib.function_base import angle
from animal_types import AnimalPosAndOrientation, AnimalProperties, Positition2D
import inspect
import numpy as np
import behavior_gates

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
        self.distance_to_obstacle = 1
        self.ahead_bounds = (np.radians(290), np.radians(70))

    def get_velocity_and_omega(self, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation, scan: tuple):
        ranges = scan[0]
        angles = scan[1]
        return self.animal_properties.max_linear_vel, self.__collision_avoidance(ranges, angles)        

    def __collision_avoidance(self, ranges, angles):
        if len(ranges) == 0: # can happen when all obstacles are further away than laser can scan
            omega = 0
        elif (ranges[np.logical_or(angles > np.radians(self.ahead_bounds[0]), angles < self.ahead_bounds[1])] > self.distance_to_obstacle).all(): # if distance to obstacle is big, just drive forward
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

class FreeSpaceBehaviour(AbstractBehaviour):
    # added to try behaviour gates
    def __init__(self, animal_properties: AnimalProperties):
        self.animal_properties = animal_properties

    def get_velocity_and_omega(self, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation, scan: tuple):
        ranges = scan[0]
        angles = scan[1]
        return self.animal_properties.max_linear_vel, self.__free_space(ranges, angles)        

    def __free_space(self, ranges, angles):
        n = 50
        ids = np.arange(len(ranges)) // n
        ranges_average = np.bincount(ids, ranges) / np.bincount(ids)
        angles_average = np.bincount(ids, angles) / np.bincount(ids)
        i = np.argmax(ranges_average)
        freespace_range = ranges_average[i]
        freespace_angle = angles_average[i]

        F_y = np.sin(freespace_angle) * freespace_range
        omega = F_y * 1.0  # scaling force to rotational speed
        return omega

class combineMinimaxCa(AbstractBehaviour):
    def __init__(self, animal_properties: AnimalProperties):
        self.animal_properties = animal_properties
        self.minimax = MinimaxBehaviour(animal_properties)
        self.collision_avoidance = CollisionAvoidanceBehaviour(animal_properties)
        self.free_space = FreeSpaceBehaviour(animal_properties)

        self.distance_to_obstacle = 1
        self.ahead_bounds = (np.radians(290), np.radians(70))

    def get_velocity_and_omega(self, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation, scan: tuple):
        return self.animal_properties.max_linear_vel, self.__combine(own_pos, other_pos, scan)

    def __combine(self, own_pos, other_pos, scan):
        _, omega_ca = self.collision_avoidance.get_velocity_and_omega(own_pos, other_pos, scan)
        _, omega_minimax = self.minimax.get_velocity_and_omega(own_pos, other_pos, scan)
        _, omega_fs = self.free_space.get_velocity_and_omega(own_pos, other_pos, scan)
        ranges = scan[0]
        angles = scan[1]

        # return self.__utility_function(own_pos, other_pos, ranges, omega_minimax, omega_ca)
        return self.__utility_function_behaviour_gates(omega_minimax, omega_ca, omega_fs)
        # return self.__base_function(ranges, angles, omega_minimax, omega_ca)
    
    def __base_function(self, ranges, angles, omega_minimax, omega_ca):
        # use minimax and ca only when wall is near
        # Problem: he drives towards the wall, then collision avoidance takes effect, he drives away from the wall, then minimax takes effect again, which drives him towards the wall again.
        if (ranges[np.logical_or(angles > np.radians(self.ahead_bounds[0]), angles < self.ahead_bounds[1])] > self.distance_to_obstacle).all(): # if distance to obstacle is big, use minimax
            omega = omega_minimax
        else:
            omega = omega_ca
        return omega

    def __utility_function(self, own_pos, other_pos, ranges, omega_minimax, omega_ca):
        # doesn't work at all
        util_ca = 2/np.min(ranges) * 0.02  
        util_ca = np.clip(util_ca, 0, 1)

        euclid_dist = np.sqrt((other_pos.pos.x - own_pos.pos.x)**2 + (other_pos.pos.y - own_pos.pos.y)**2)
        util_ho = 3/euclid_dist * 0.01 + 0.08 
        util_ho = np.clip(util_ho, 0, 1)

        print(util_ho)
        print(util_ca)

        omega = (util_ho * omega_minimax) *  (util_ca * omega_ca)
        omega_max = 2.84
        omega = np.clip(omega, -omega_max, omega_max)
        return omega

    def __utility_function_behaviour_gates(self, omega_minimax, omega_ca, omega_fs):
        gate1 = behavior_gates.INVOKE(omega_minimax, omega_fs)
        gate2 = behavior_gates.PREVAIL(omega_minimax, omega_ca)
        gate3 = behavior_gates.PREVAIL(gate1, omega_ca)
        return behavior_gates.OR(gate2, gate3)

def get_all_behaviours():
    return { 'minimax': MinimaxBehaviour,
             'collision_avoidance': CollisionAvoidanceBehaviour,
             'combine_minimax_ca': combineMinimaxCa}