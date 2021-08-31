import rospy
from helper_types import AnimalProperties, PosAndOrientation, get_distance_between_positions, get_angle_between_positions
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rospy.exceptions import ROSException
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import os


class FollowPlanBehaviour():
    """Calculates a plan with navigation/global_planner from current cat position to the mouse and gives cmd_vel properties to follow the generated path"""

    def __init__(self, animal_properties: AnimalProperties):
        self.animal_properties = animal_properties
        # should the service name be configurable?
        self.planner_service_name = "/cat_planner/planner/make_plan"

        self.last_plan_own_pos = None
        self.last_plan_other_pos = None
        self.first_plan_time = None
        self.beginning_plan_regenerated = False
        self.pos_tolerance = 0.4
        self.path_driven = []
        self.goal_pub = rospy.Publisher(
            "/cat_goal", PoseStamped, queue_size=10)
        self.cat_to_goal_pub = rospy.Publisher(
            "/cat_to_goal", PoseStamped, queue_size=10)
        self.current_goal = None
        self.generator = None

    def get_velocity_and_omega(self, own_pos: PosAndOrientation, other_pos: PosAndOrientation, scan: tuple):
        if self.__new_plan_needed(own_pos, other_pos, scan):
            plan = self.__generate_plan(own_pos, other_pos)

            if plan is None or len(plan.poses) == 0:
                self.generator = None
            else:
                self.generator = self.__velocity_and_orientation_generator(
                    plan)
            self.path_driven = []

        if self.generator is None:
            return None, None

        self.path_driven += [own_pos]
        return next(self.generator)

    def __generate_plan(self, own_pos: PosAndOrientation, other_pos: PosAndOrientation):
        def request_plan(from_pos: PosAndOrientation, to_pos: PosAndOrientation):
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

            # the offset could be in the direction of the cat, but it should not make much of a difference
            # needs to be slightly offset, otherwise no plan will be created once the scanner sees the other robot (as goal would be inside of it)
            goal.pose.position.x = to_pos.pos.x + 0.1
            goal.pose.position.y = to_pos.pos.y

            to_orientation = to_pos.orientation
            to_quat = quaternion_from_euler(0, 0, to_orientation)

            goal.pose.orientation.x = to_quat[0]
            goal.pose.orientation.y = to_quat[1]
            goal.pose.orientation.z = to_quat[2]
            goal.pose.orientation.w = to_quat[3]
            request.goal = goal

            # not implemented: https://github.com/ros-planning/navigation/pull/1041 (•ˋ _ ˊ•)
            request.tolerance = 0.5

            # this should throw an exception on error, but sadly it does not if no plan can be found (even though the service itself logs an error)
            response: GetPlanResponse = self.get_plan_proxy(request)

            return response.plan

        # wait for service to be available, if it is not yet ready just use fallback
        try:
            rospy.wait_for_service(self.planner_service_name, 0.05)
            # sometimes the planner just dies :( wait until it is restarted
            try:
                self.get_plan_proxy = rospy.ServiceProxy(
                    self.planner_service_name, GetPlan, persistent=False)
                plan = request_plan(own_pos, other_pos)
            except rospy.service.ServiceException:
                print("Encountered an error while making plan, waiting for respawn")
                plan = None
        except ROSException:
            plan = None

        if self.first_plan_time is None:
            self.first_plan_time = rospy.Time.now()

        self.last_plan_own_pos = own_pos
        self.last_plan_other_pos = other_pos
        return plan

    def __velocity_and_orientation_generator(self, plan: Path):
        pose_index = 0

        while True:
            if pose_index >= len(plan.poses):
                yield 0, 0  # TODO future: yield None instead and generate a new plan if this is the case?
            else:
                goal_pos = self.__get_pos_from_pose_stamed(
                    plan.poses[pose_index])
                last_own_pos = self.path_driven[len(self.path_driven)-1]

                # TODO future: this needs a visibility check if we want to use it
                # find the nearest pos to robot
                # alternatively, if a far future point in path is nearer to robot than the current pos
                # it might be a good idea to generate a new path?

                # future_index = pose_index
                # nearest_dist = np.inf

                # while future_index < len(plan.poses):
                #     future_index_pos = self.__get_pos_from_pose_stamed(plan.poses[future_index])
                #     future_dist = get_distance_between_positions(last_own_pos.pos, future_index_pos.pos)
                #     if future_dist < nearest_dist:
                #         nearest_dist = future_dist
                #         pose_index = future_index

                #     future_index += 1

                # find the next pose not inside the tolerance radius of the driven path up until now
                while self.__pos_inside_driven_path(goal_pos):
                    pose_index += 1
                    if pose_index >= len(plan.poses):
                        break

                    goal_pos = self.__get_pos_from_pose_stamed(
                        plan.poses[pose_index])

                # drive towards goal pos
                angle_to_turn = get_angle_between_positions(
                    last_own_pos, goal_pos)

                # this is for RVIZ visualization only
                try:
                    self.goal_pub.publish(plan.poses[pose_index])
                    cat_pose = PoseStamped()
                    cat_pose.header.frame_id = "map"
                    cat_pose.header.stamp = rospy.Time.now()
                    cat_pose.pose.position.x = last_own_pos.pos.x
                    cat_pose.pose.position.y = last_own_pos.pos.y
                    [a, b, c, d] = quaternion_from_euler(
                        0, 0, angle_to_turn + last_own_pos.orientation)
                    cat_pose.pose.orientation.x = a
                    cat_pose.pose.orientation.y = b
                    cat_pose.pose.orientation.z = c
                    cat_pose.pose.orientation.w = d
                    self.cat_to_goal_pub.publish(cat_pose)
                except Exception as ecx:
                    print(ecx)

                self.current_goal = goal_pos
                self.current_angle_to_turn = angle_to_turn
                yield self.animal_properties.max_linear_vel, angle_to_turn

    def __pos_inside_driven_path(self, pos):
        for previous_pos in self.path_driven:
            if get_distance_between_positions(pos.pos, previous_pos.pos) < self.pos_tolerance:
                return True

        return False

    def __get_pos_from_pose_stamed(self, pose_stamped: PoseStamped):
        orientation = euler_from_quaternion([pose_stamped.pose.orientation.x,
                                             pose_stamped.pose.orientation.y,
                                             pose_stamped.pose.orientation.z,
                                             pose_stamped.pose.orientation.w])[2]

        return PosAndOrientation(pose_stamped.pose.position.x, pose_stamped.pose.position.y, orientation)

    def __new_plan_needed(self, own_pos, other_pos, scan):
        if self.generator is None:
            return True

        # no plan was generated yet
        if self.last_plan_own_pos is None or self.last_plan_other_pos is None:
            return True

        # the first plan can sometimes not have much costmap data available, generate a new one after half a second
        if not self.beginning_plan_regenerated and (self.first_plan_time is not None and (rospy.Time.now() - self.first_plan_time).to_sec() > 0.5):
            self.beginning_plan_regenerated = True
            return True

        # either the enemy has moved some distance
        if get_distance_between_positions(self.last_plan_other_pos.pos, other_pos.pos) > 0.3:
            return True

        # or we have moved some distance
        if get_distance_between_positions(self.last_plan_own_pos.pos, own_pos.pos) > 2:
            return True

        # check if goal is obstructed
        if self.current_goal is not None:
            angle_to_turn = self.current_angle_to_turn
            if angle_to_turn < 0:
                angle_to_turn += 2*np.pi

            smallest_diff = np.inf
            for idx, angle in enumerate(scan[1]):
                new_diff = np.abs(angle - angle_to_turn)
                if new_diff < smallest_diff:
                    smallest_diff = new_diff
                    best_idx = idx

            distance_to_laser_obstacle = scan[0][best_idx]
            distance_to_goal = get_distance_between_positions(
                own_pos.pos, self.current_goal.pos)
            if distance_to_laser_obstacle < distance_to_goal - 0.2:
                return True

        return False
