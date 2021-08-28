#!/usr/bin/env python3

from mouse_tracker import MouseTracker
from helper_types import AnimalProperties, PosAndOrientation
import sys
import os

from all_behaviours import get_all_behaviours

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import rospkg

from tf.transformations import euler_from_quaternion
import numpy as np

from rogata_library import rogata_helper 

assert len(sys.argv) >= 6

class Cat:
    def __init__(self, self_cmd_vel_topic, self_odom_topic, self_scan, enemy_odom_topic, Behaviour) -> None:

        # Cheese
        self.cheese_pos = self.__get_cheese_pos()

        # keep track of enemy capabilities (currently only max velocity and max omega)
        self.mouse_tracker = MouseTracker(self.cheese_pos)

        # Properties
        self.__init_properties()

        self.behavior = Behaviour(self.properties)

        # Topics
        rospy.Subscriber(self_odom_topic, Odometry, lambda odom: self.__odom_callback(odom, True))
        rospy.Subscriber(enemy_odom_topic, Odometry, lambda odom: self.__odom_callback(odom, False))
        rospy.Subscriber(self_scan, LaserScan, self.__scan_callback)

        self.cmd_vel_pub = rospy.Publisher(self_cmd_vel_topic, Twist, queue_size=10)

        self.self_odom_callback_received = False
        self.enemy_odom_callback_received = False
        self.scan_callback_received = False

    def __get_cheese_pos(self):
        rospack    = rospkg.RosPack()
        catch_path = rospack.get_path('catch')
        cheese_positions = []
        # as both map 1 and map 2 setup nodes are initialized as rogata_engine, we have to get creative to find the relevant cheese
        # test if the first cheese middle point is inside of cheese object, if yes we are dealing with map 1, else with map 2
        cheese_1_middle = np.mean(np.load(os.path.join(catch_path, 'maps/cheese_' + str(1) + '.npy')), axis=0)[0]
        cheese_indices = [1,2,3,4] if rogata_helper().inside("cheese_obj",cheese_1_middle) else [5,6,7,8]

        for i in cheese_indices:
            filepath   = os.path.join(catch_path, 'maps/cheese_'+str(i)+'.npy')
            cheese     = np.load(filepath)
            pos = np.mean(cheese, axis=0)[0]
            # negative transform from `catch_tracker.py` odom callback, we guess these are the transformations between ros and rogata coordinates
            pos = pos-np.array([500,500])
            pos /= 100

            cheese_positions.append(pos)
        
        return cheese_positions

    def __scan_callback(self, scan_msg):
        ranges = scan_msg.ranges
        ranges = np.array(ranges)
        angle_min = scan_msg.angle_min  # start angle of the scan [rad]
        angle_max = scan_msg.angle_max  # end angle of the scan [rad]
        angle_increment = scan_msg.angle_increment  # angular distance between measurements [rad]
        angles = np.arange(angle_min, angle_max + angle_increment, angle_increment)

        angles = angles[np.isfinite(ranges)]  # delete 'inf'
        ranges = ranges[np.isfinite(ranges)]
        self.scan = (ranges, angles)
        self.scan_callback_received = True

    def __init_properties(self):
        random_velocity_factor = np.random.uniform(0, 0.2)
        random_omega_factor = np.random.uniform(0, 1.2)
        self.properties = AnimalProperties(0.2 + random_velocity_factor, 0.8 + random_omega_factor)

    def __odom_callback(self, odom: Odometry, self_odom: bool):
        orientation = euler_from_quaternion([odom.pose.pose.orientation.x,
                                                odom.pose.pose.orientation.y,
                                                odom.pose.pose.orientation.z,
                                                odom.pose.pose.orientation.w])[2]
        if self_odom:
            self.self_pos = PosAndOrientation(odom.pose.pose.position.x, odom.pose.pose.position.y, orientation)
            self.self_odom_callback_received = True
        else:
            self.mouse_tracker.update_position_and_orientation(PosAndOrientation(odom.pose.pose.position.x, odom.pose.pose.position.y, orientation))
            self.mouse_tracker.update_capabilities(odom.twist.twist.linear.x, odom.twist.twist.linear.z)
            self.enemy_odom_callback_received = True

    def spin(self):
        while not rospy.is_shutdown():            
            # we cannot calculate anything without having received our and the enemy position
            if not self.self_odom_callback_received or not self.enemy_odom_callback_received or not self.scan_callback_received:
                continue

            velocity, angle = self.behavior.get_velocity_and_omega(self.self_pos, self.mouse_tracker, self.scan)

            out = Twist()
            out.linear.x = np.clip(velocity, 0.2, self.properties.max_linear_vel)
            out.angular.z = np.clip(angle, -self.properties.max_omega, self.properties.max_omega)

            self.cmd_vel_pub.publish(out)

# ===== SCRIPT =====
if __name__ == '__main__':
    self_cmd_vel_topic = sys.argv[1]
    self_odom_topic = sys.argv[2]
    self_scan = sys.argv[3]
    enemy_odom_topic = sys.argv[4]
    behaviour_to_use = 'combined' #sys.argv[5] # behaviour does only need to change for testing, so setting it here in the script directly is easier

    try:
        rospy.init_node('cat')
        cat = Cat(self_cmd_vel_topic, self_odom_topic, self_scan, enemy_odom_topic, get_all_behaviours()[behaviour_to_use])
        cat.spin()
    except rospy.ROSInterruptException:
        pass
