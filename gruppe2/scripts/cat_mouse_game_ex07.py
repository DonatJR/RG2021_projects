#!/usr/bin/env python3

import sys
import os
from enum import IntEnum

from numpy.lib.function_base import angle
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import timeit
import numpy as np
import matplotlib.pyplot as plt

assert len(sys.argv) >= 6

############################################################
# We have a bug where sometimes the behaviour of both bots
# seems to be like intended, but other times they just
# move around almost randomly, which is why we could not
# complete the last task (testing different positions)
############################################################

# ===== CLASSES =====

class AnimalType(IntEnum):
    MOUSE = 1
    CAT = 2

class AnimalProperties:
    def __init__(self, linear_vel, max_omega) -> None:
        self.linear_vel = linear_vel
        self.max_omega = max_omega

animal_properties = { AnimalType.MOUSE: AnimalProperties(linear_vel = 0.18, max_omega = 0.8), AnimalType.CAT: AnimalProperties(linear_vel = 0.22, max_omega = 2.84) }

class AnimalBehaviour:
    def __init__(self, animal_type, self_cmd_vel_topic, self_odom_topic, enemy_cmd_vel_topic, enemy_odom_topic) -> None:
        self.type = animal_type
        self.enemy_type = AnimalType.MOUSE if self.type == AnimalType.CAT else AnimalType.CAT
        self.properties = animal_properties[self.type]
        self.enemy_properties = animal_properties[self.enemy_type]

        rospy.Subscriber(self_odom_topic, Odometry, lambda odom: self.__odom_callback(odom, True))
        rospy.Subscriber(enemy_odom_topic, Odometry, lambda odom: self.__odom_callback(odom, False))

        # cmd_vel_callback = functools.partial(self.__cmd_vel_callback, self)
        # rospy.Subscriber(self_cmd_vel_topic, Twist, lambda cmd_vel: cmd_vel_callback(cmd_vel, True))
        # rospy.Subscriber(enemy_cmd_vel_topic, Twist, lambda cmd_vel: cmd_vel_callback(cmd_vel, False))

        self.cmd_vel_pub = rospy.Publisher(self_cmd_vel_topic, Twist, queue_size=10)

        self.strategy_choices = np.linspace(-self.properties.max_omega, self.properties.max_omega, 8)
        self.self_odom_callback_received = False
        self.enemy_odom_callback_received = False

    def __odom_callback(self, odom: Odometry, self_odom: bool):
        orientation = euler_from_quaternion([odom.pose.pose.orientation.x,
                                                odom.pose.pose.orientation.y,
                                                odom.pose.pose.orientation.z,
                                                odom.pose.pose.orientation.w])[2]

        if self_odom:
            self.orientation = orientation
            self.x = odom.pose.pose.position.x
            self.y = odom.pose.pose.position.y
            self.self_odom_callback_received = True
        else:
            self.enemy_orientation = orientation
            self.enemy_x = odom.pose.pose.position.x
            self.enemy_y = odom.pose.pose.position.y
            self.enemy_odom_callback_received = True

    # def __cmd_vel_callback(self, twist: Twist, self_cmd_vel: bool):
    #     if self_cmd_vel:
    #         self.cmd_vel = twist.linear.x
    #         self.omega = twist.angular.z
    #     else:
    #         self.enemy_cmd_vel = twist.linear.x
    #         self.enemy_omega = twist.angular.z

    def __system_update(self, angular_vel: float, current_state, animal_type: AnimalType):
        if animal_type == AnimalType.MOUSE:
            new_x = current_state[0]
            new_y = current_state[1]
            new_orientation = current_state[2]
        else:
            new_x = current_state[3]
            new_y = current_state[4]
            new_orientation = current_state[5]

        speed = animal_properties[animal_type].linear_vel
        iterations = 10
        dt = 1.0 / iterations

        for _ in range(iterations):
            new_x += np.cos(new_orientation) * speed * dt
            new_y += np.sin(new_orientation) * speed * dt
            new_orientation += angular_vel * dt


        if animal_type == AnimalType.MOUSE:
            return (new_x, new_y, new_orientation, current_state[3], current_state[4], current_state[5])
        else:
            return (current_state[0], current_state[1], current_state[2], new_x, new_y, new_orientation)

    def __metric(self, current_state):
        mouse_x, mouse_y, mouse_orientation, cat_x, cat_y, cat_orientation = current_state
        
        cat_orientation_vec = np.array([np.cos(cat_orientation), np.sin(cat_orientation)])
        cat_orientation_vec /= np.linalg.norm(cat_orientation_vec)

        mouse_cat_vector = np.array([mouse_x - cat_x, mouse_y - cat_y])
        mouse_cat_vector /= np.linalg.norm(mouse_cat_vector)

        angle = np.rad2deg(np.arccos(np.dot(cat_orientation_vec, mouse_cat_vector)))
        
        # angle > 270 or < 90 means mouse is behind cat
        # use a little less than 180 deg to mean 'in front', i.e. angle > 290 or angle < 70
        mouse_behind_cat = angle > 290 or angle < 70

        if mouse_behind_cat:
            distance_weight = 0.1
            angle_weight = 0.9
        else:
            distance_weight = 0.9
            angle_weight = 0.1

        
        distance = np.sqrt((mouse_x - cat_x) ** 2 + (mouse_y - cat_y) ** 2)
        
        # if self.type == AnimalType.MOUSE:
        #     print(f"distance: {distance}, angle: {angle} ({np.abs(180 - angle)}), d_weight: {distance_weight}, angle_weight: {angle_weight}")

        # emphasize distance and a good angle and weight it based on the fact if mouse is behind or in front of cat
        return distance_weight * distance + angle_weight * np.abs(180 - angle)

    def __minimax(self, angular_vel: float, current_state, depth: int, animal_type: AnimalType, alpha: float = -np.inf, beta: float = np.inf):
        if depth == 0:
            return self.__metric(current_state)

        new_state = self.__system_update(angular_vel, current_state, animal_type)
            
        if animal_type == AnimalType.MOUSE: # mouse maximizes time
            value = -np.inf
            for choice in self.strategy_choices:
                new_value = self.__minimax(choice, new_state, depth-1, animal_type, alpha, beta)
                value = max(value, new_value)
                alpha = max(alpha, value)

                if value >= beta:
                    break

            return value

        else:
            value = np.inf
            for choice in self.strategy_choices:
                new_value = self.__minimax(choice, new_state, depth-1, animal_type, alpha, beta)
                value = min(value, new_value)
                beta = min(beta, value)

                if value <= alpha:
                    break

            return value

    def measure_minimax(self):
        # we cannot calculate anything without having received our and the enemy position
        while not self.self_odom_callback_received or not self.enemy_odom_callback_received:
            print('waiting')
            continue

        print('measuring')
        depths = np.arange(1, 7, 1) # higher depths took too long
        measured_time = []
        
        for depth in depths:
            print(f'measuring depth {depth}')
            start = timeit.default_timer()

            current_state = (self.x, self.y, self.orientation, self.enemy_x, self.enemy_y, self.enemy_orientation) if self.type == AnimalType.MOUSE else (self.enemy_x, self.enemy_y, self.enemy_orientation, self.x, self.y, self.orientation)

            [self.__minimax(choice, current_state = current_state, depth = depth, animal_type = self.type) for choice in self.strategy_choices]

            elapsed_time = timeit.default_timer() - start
            measured_time += [elapsed_time]


        plt.figure()
        plt.plot(depths, measured_time)
        plt.xlabel('depth')
        plt.ylabel('elapsed time')
        plt.savefig(f'{os.path.dirname(os.path.realpath(sys.argv[0]))}/../minimax.png')

        print('finished measuring')

    def start(self):
        while not rospy.is_shutdown():            
            # we cannot calculate anything without having received our and the enemy position
            if not self.self_odom_callback_received or not self.enemy_odom_callback_received:
                continue

            out = Twist()
            out.linear.x = self.properties.linear_vel
            
            current_state = (self.x, self.y, self.orientation, self.enemy_x, self.enemy_y, self.enemy_orientation) if self.type == AnimalType.MOUSE else (self.enemy_x, self.enemy_y, self.enemy_orientation, self.x, self.y, self.orientation)
            
            # a depth higher than 3 leads to too much thinking time, where the bots stay in one configuration
            # for too long and cannot react fast enough to the environment / the other bot
            move_values = [self.__minimax(choice, current_state = current_state, depth = 3, animal_type = self.type) for choice in self.strategy_choices]
        
            out.angular.z = self.strategy_choices[np.argmax(move_values)] if self.type == AnimalType.MOUSE else self.strategy_choices[np.argmin(move_values)]

            self.cmd_vel_pub.publish(out)

# ===== SCRIPT =====
if __name__ == '__main__':

    self_cmd_vel_topic = sys.argv[1]
    is_mouse = sys.argv[2] == 'True'
    animal_type = AnimalType.MOUSE if is_mouse else AnimalType.CAT
    self_odom_topic = sys.argv[3]
    enemy_cmd_vel_topic = sys.argv[4]
    enemy_odom_topic = sys.argv[5]

    try:
        if animal_type == AnimalType.MOUSE:
            rospy.init_node('mouse')
        else:
            rospy.init_node('cat')
        animal = AnimalBehaviour(animal_type, self_cmd_vel_topic, self_odom_topic, enemy_cmd_vel_topic, enemy_odom_topic)
        # if animal_type == AnimalType.MOUSE:
        #     animal.measure_minimax()
        animal.start()
    except rospy.ROSInterruptException:
        pass
