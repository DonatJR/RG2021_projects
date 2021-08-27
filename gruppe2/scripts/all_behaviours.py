from collision_avoidance_behaviour import CollisionAvoidanceBehaviour
from minimax_behaviour import MinimaxBehaviour
from minimax_ca_combined_behaviour import CombineMinimaxCa

def get_all_behaviours():
    return { 
        'collision_avoidance': CollisionAvoidanceBehaviour,
        'minimax': MinimaxBehaviour,
        'minimax_ca': CombineMinimaxCa
    }
