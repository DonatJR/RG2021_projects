from combined_behaviours import CombinedBehaviours
from collision_avoidance_behaviour import CollisionAvoidanceBehaviour
from freespace_behaviour import FreeSpaceBehaviour
from follow_plan_behaviour import FollowPlanBehaviour
from minimax_behaviour import MinimaxBehaviour
from minimax_ca_combined_behaviour import CombineMinimaxCaBehaviour

def get_all_behaviours():
    return { 
        'collision_avoidance': CollisionAvoidanceBehaviour,
        'freespace': FreeSpaceBehaviour,
        'follow_plan': FollowPlanBehaviour,
        'minimax': MinimaxBehaviour,
        'minimax_ca': CombineMinimaxCaBehaviour,
        'combined': CombinedBehaviours
    }
