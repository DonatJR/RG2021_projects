from gruppe2.scripts.freespace_behaviour import FreeSpaceBehaviour
from collision_avoidance_behaviour import CollisionAvoidanceBehaviour
from freespace_behaviour import FreeSpaceBehaviour
from minimax_behaviour import MinimaxBehaviour
from minimax_ca_combined_behaviour import CombineMinimaxCa

def get_all_behaviours():
    return { 
        'collision_avoidance': CollisionAvoidanceBehaviour,
        'freespace': FreeSpaceBehaviour,
        'minimax': MinimaxBehaviour,
        'minimax_ca': CombineMinimaxCa
    }
