import rospy
from helper_types import AnimalProperties, AnimalPosAndOrientation
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

class FollowPlanBehaviour():
    """Calculates a plan with navigation/global_planner from current cat position to the mouse and gives cmd_vel properties to follow the generated path"""
    def __init__(self, animal_properties: AnimalProperties):
        self.animal_properties = animal_properties
        service_name = "/cat_planner/planner/make_plan" # TODO: should the service name be configurable?

        rospy.wait_for_service(service_name)
        self.get_plan_proxy = rospy.ServiceProxy(service_name, GetPlan)

        self.last_plan_time = None
        self.generate_plan_every_nth_sec = 2
        self.pos_tolerance = 0.1

    def __generate_plan(self, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation):
        def request_plan(from_pos: AnimalPosAndOrientation, to_pos: AnimalPosAndOrientation):
            request = GetPlanRequest()

            start = PoseStamped()
            start.header.frame_id = 'map'
            start.header.stamp = rospy.Time.now()
            start.pose.position.x = from_pos.pos.x
            start.pose.position.y = from_pos.pos.y

            from_orientation = from_pos.orientation
            from_quat = quaternion_from_euler(0, 0, from_orientation)
            
            start.pose.orientation.x = from_quat[0]
            start.pose.orientation.y = from_quat[1]
            start.pose.orientation.z = from_quat[2]
            start.pose.orientation.w = from_quat[3]
            request.start = start

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = rospy.Time.now()
            
            # TODO: the offset could be in the direction of the cat, but it should not make much of a difference
            goal.pose.position.x = to_pos.pos.x + 0.1 # needs to be slightly offset, otherwise no plan will be created once the scanner sees the other robot (as goal would be inside of it)
            goal.pose.position.y = to_pos.pos.y
            
            to_orientation = to_pos.orientation
            to_quat = quaternion_from_euler(0, 0, to_orientation)

            goal.pose.orientation.x = to_quat[0]
            goal.pose.orientation.y = to_quat[1]
            goal.pose.orientation.z = to_quat[2]
            goal.pose.orientation.w = to_quat[3]
            request.goal = goal

            request.tolerance = 0.5 # not implemented: https://github.com/ros-planning/navigation/pull/1041 (•ˋ _ ˊ•)

            # this should throw an exception on error, but sadly it does not if no plan can be found (even though the service itself logs an error)
            response: GetPlanResponse = self.get_plan_proxy(request)
            return response.plan

        plan = request_plan(own_pos, other_pos)

        self.last_plan_time = rospy.Time.now()
        return plan

    def __velocity_and_orientation_generator(self, plan: Path):
        pose_index = 0
        
        while True:
            if pose_index >= len(plan.poses):
                yield 0, 0 # TODO future: yield None instead and generate a new plan if this is the case?
            else:
                next_goal_pose: PoseStamped  = plan.poses[pose_index]
            
                orientation = euler_from_quaternion([next_goal_pose.pose.orientation.x,
                                                next_goal_pose.pose.orientation.y,
                                                next_goal_pose.pose.orientation.z,
                                                next_goal_pose.pose.orientation.w])[2]

                yield self.animal_properties.max_linear_vel, orientation-self.latest_own_pos.orientation

                # find the next pose not inside the tolerance radius of our own robot
                while abs(next_goal_pose.pose.position.x - self.latest_own_pos.pos.x) < self.pos_tolerance and abs(next_goal_pose.pose.position.y - self.latest_own_pos.pos.y) < self.pos_tolerance and pose_index < len(plan.poses):
                    next_goal_pose = plan.poses[pose_index]
                    pose_index += 1
                    

    def get_velocity_and_omega(self, own_pos: AnimalPosAndOrientation, other_pos: AnimalPosAndOrientation):
        if self.last_plan_time is None or (rospy.Time.now() - self.last_plan_time).to_sec() > self.generate_plan_every_nth_sec:
            plan = self.__generate_plan(own_pos, other_pos)
            self.generator = self.__velocity_and_orientation_generator(plan)

        
        self.latest_own_pos = own_pos
        return next(self.generator)