from collision_avoidance_behaviour import CollisionAvoidanceBehaviour
from protect_minimax_behaviour import ProtectMinimaxBehaviour
from pursuit_minimax_behaviour import PursuitMinimaxBehaviour

def get_all_behaviours():
    return { 
        'collision_avoidance': CollisionAvoidanceBehaviour,
        'protect_minimax': ProtectMinimaxBehaviour,
        'pursuit_minimax': PursuitMinimaxBehaviour
    }
