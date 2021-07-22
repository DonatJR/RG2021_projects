from animal_types import AnimalPosAndOrientation, AnimalProperties
import inspect

class AbstractBehaviour():
    def get_velocity_and_omega(self,own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation, scan: tuple, enemy_capabilities: AnimalProperties):
        raise NotImplementedError(f'{inspect.stack()[0][3]} is not implemented, do not use abstract behaviour class directly')
