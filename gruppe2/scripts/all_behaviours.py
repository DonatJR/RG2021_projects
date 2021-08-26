from collision_avoidance_behaviour import CollisionAvoidanceBehaviour
from seek_minimax_behaviour import SeekMinimaxBehaviour
from pursuit_minimax_behaviour import PursuitMinimaxBehaviour
from seek_minimax_ca_combined_behaviour import CombineSeekMinimaxCa

def get_all_behaviours():
    return { 
        'collision_avoidance': CollisionAvoidanceBehaviour,
        'seek_minimax': SeekMinimaxBehaviour,
        'pursuit_minimax': PursuitMinimaxBehaviour,
        'combine_seek_ca': CombineSeekMinimaxCa
    }
