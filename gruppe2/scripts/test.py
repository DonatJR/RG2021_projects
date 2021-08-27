#!/usr/bin/env python3

import rospy
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion #, quaternion_from_euler

rospy.init_node('test')

rospy.wait_for_service('/cat_planner/planner/make_plan')
print('service available')
get_plan = rospy.ServiceProxy('/cat_planner/planner/make_plan', GetPlan)
request = GetPlanRequest()

#robot_orientation = 5
#robot_quat = quaternion_from_euler(0, 0, robot_orientation)

start = PoseStamped()
start.header.frame_id = 'cat/base_link'
start.header.stamp = rospy.Time.now()
start.pose.position.x = 0
start.pose.position.y = 0
# start.pose.orientation = robot_quat
request.start = start

goal = PoseStamped()
goal.header.frame_id = 'mouse/base_link'
goal.header.stamp = rospy.Time.now()
goal.pose.position.x = 5
goal.pose.position.y = 0
# goal.pose.orientation = robot_quat # which orientation do we want?
request.goal = goal

request.tolerance = .5

response: GetPlanResponse = get_plan(request)
plan = response.plan

print (plan.header)
print(plan)
for pose in plan.poses:
    print(pose)
    orientation = euler_from_quaternion([pose.pose.orientation.x,
                                                pose.pose.pose.orientation.y,
                                                pose.pose.pose.orientation.z,
                                                pose.pose.pose.orientation.w])[2]
    

