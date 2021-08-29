from combined_behaviours import CombinedBehaviours
from collision_avoidance_behaviour import CollisionAvoidanceBehaviour
from follow_plan_behaviour import FollowPlanBehaviour
from follow_plan_ca_combined_behaviour import CombineFollowPlanCaBehaviour
from minimax_behaviour import MinimaxBehaviour
from minimax_ca_combined_behaviour import CombineMinimaxCaBehaviour


def get_all_behaviours():
    return {
        'collision_avoidance': CollisionAvoidanceBehaviour,
        'follow_plan': FollowPlanBehaviour,
        'follow_plan_ca': CombineFollowPlanCaBehaviour,
        'minimax': MinimaxBehaviour,
        'minimax_ca': CombineMinimaxCaBehaviour,
        'combined': CombinedBehaviours
    }
