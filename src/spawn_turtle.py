#!/usr/bin/python

import rospy
from turtlesim.srv import Spawn
from class_turtle import Agent


if __name__ == "__main__":
    rospy.init_node("~turtle", anonymous=True)
    ID = rospy.get_param("~ID")
    goal_ID = rospy.get_param("/goal_ID")
    angular = rospy.get_param("/angular_vel")
    linear = rospy.get_param("/linear_vel")
    k_p = rospy.get_param("/k_p")
    k_d = rospy.get_param("/k_d")
    max_distance = rospy.get_param("/max_distance")
    rospy.wait_for_service('spawn')
    if ID != 1:
        x = rospy.get_param("~x_coordinate")
        y = rospy.get_param("~y_coordinate")
        theta = rospy.get_param("~theta_coordinate")
        spawn_turtle_x = rospy.ServiceProxy('/spawn', Spawn)
        spawn_turtle_x(x, y, theta, '')
    x = Agent(ID, angular, linear, max_distance, goal_ID, k_p, k_d)
    x.start()
    rospy.spin()
