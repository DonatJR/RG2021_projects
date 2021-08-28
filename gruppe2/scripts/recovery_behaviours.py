# TODO maybe future: write recovery behaviour if necessary (drive towards the most open space around us -> drive backwards for a bit -> rotate 360 degrees and then drive backwards for a bit)
# realistically, if we need to do this we probably lost the game anyway, so what is the point?! ;)
from helper_types import AnimalProperties
import inspect

class RecoveryBehaviours():
    def __init__(self, animal_properties: AnimalProperties):
        self.animal_properties = animal_properties

    def get_velocity_and_omega(self):
        raise NotImplementedError(f"{inspect.stack()[0][3]} is not implemented yet")
        # return self.animal_properties.max_linear_vel, 0