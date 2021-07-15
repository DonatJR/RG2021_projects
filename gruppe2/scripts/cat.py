#!/usr/bin/env python3

from gruppe2.scripts.types import AnimalPosAndOrientation
import sys

from behaviours import get_all_behaviours
from types import AnimalProperties

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import numpy as np

from rogata_library import rogata_helper 

# TODO: remove or comment in again once we have the arguments locked down
# assert len(sys.argv) >= 6

class Cat:
    def __init__(self, self_cmd_vel_topic, self_odom_topic, self_scan, enemy_odom_topic, Behaviour) -> None:

        # TODO: weiß nicht, ob wir das brauchen
        # rospy.Subscriber("game_state", Int32, lambda game_state: self.__game_state_callback(game_state))

        # TODO: Cheese. Warum bekomme ich hier nur ein Punkt zurück. Hat doch mehrere? Vermutlich deswegen? The center in this case refers to the mean position of the object. For a disjointed area this center can be outside of the object itself.
        self.rogata_helper = rogata_helper() 
        self.cheese_pos = self.rogata_helper.get_pos('cheese_obj')

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

    def __scan_callback(self, msg):
        ranges = msg.ranges
        ranges = np.array(ranges)
        # TODO: why are we not doing anything with these?

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
            self.self_pos = AnimalPosAndOrientation(odom.pose.pose.position.x, odom.pose.pose.position.y, orientation)
            self.self_odom_callback_received = True
        else:
            self.enemy_pos = AnimalPosAndOrientation(odom.pose.pose.position.x, odom.pose.pose.position.y, orientation)
            self.enemy_odom_callback_received = True

    def start(self):
        while not rospy.is_shutdown():            
            # we cannot calculate anything without having received our and the enemy position
            if not self.self_odom_callback_received or not self.enemy_odom_callback_received:
                continue
            
            velocity, angle = self.behavior.get_velocity_and_omega(self.self_pos, self.enemy_pos)

            out = Twist()
            out.linear.x = velocity
            out.angular.z = angle

            self.cmd_vel_pub.publish(out)

# ===== SCRIPT =====
if __name__ == '__main__':
    self_cmd_vel_topic = sys.argv[1]
    self_odom_topic = sys.argv[2]
    self_scan = sys.argv[3]
    # TODO: can we use this? shouldn't it be the perception of the other animal instead of odom directly?
    enemy_odom_topic = sys.argv[4]
    behaviour_to_use = sys.argv[5]

    try:
        # TODO: is this necessay? does it maybe have to be another value instead of 'cat'?
        rospy.init_node('cat')
        cat = Cat(self_cmd_vel_topic, self_odom_topic, self_scan, enemy_odom_topic, get_all_behaviours()[behaviour_to_use])
        cat.start()
    except rospy.ROSInterruptException:
        pass
