#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import rospkg
import rogata_library as rgt
import numpy as np
import os


rospack    = rospkg.RosPack()
catch_path = rospack.get_path('catch')



cheese_contours =[]
for i in [5,6,7,8]:
    filepath   = os.path.join(catch_path, 'maps/cheese_'+str(i)+'.npy')
    cheese     = np.load(filepath)
    cheese_contours.append(cheese)
cheese_obj = rgt.game_object('cheese_obj', cheese_contours, np.ones(4))

name      = "cat_obj"
cat_id    = 1
hit_box   = {"type":"rectangle","height":2,"width":2}
cat_obj   = rgt.dynamic_object(name,hit_box,cat_id)

name      = "mouse_obj"
mouse_id  = 1
hit_box   = {"type":"rectangle","height":2,"width":2}
mouse_obj = rgt.dynamic_object(name,hit_box,mouse_id)


if __name__ == '__main__':
    rospy.init_node("rogata_engine")
    try:
        example_scene = rgt.scene([cheese_obj,cat_obj,mouse_obj])
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
