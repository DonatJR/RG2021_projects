from collision_avoidance_behaviour import CollisionAvoidanceBehaviour
from protect_minimax_behaviour import ProtectMinimaxBehaviour
from pursuit_minimax_behaviour import PursuitMinimaxBehaviour
from pursuit_minimax_ca_combined_behaviour import CombinePursuitMinimaxCa

def get_all_behaviours():
    return { 
        'collision_avoidance': CollisionAvoidanceBehaviour,
        'protect_minimax': ProtectMinimaxBehaviour,
        'pursuit_minimax': PursuitMinimaxBehaviour,
        'combine_pursuit_ca': CombinePursuitMinimaxCa
    }
