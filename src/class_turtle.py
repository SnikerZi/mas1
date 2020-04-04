#!/usr/bin/python

import rospy
import math
import random
import geometry_msgs.msg
from mas_framework.msg import base
from mas_framework.msg import status
from mas_framework.msg import trade
import turtlesim.msg as turtle


class Agent:
    def __init__(self, ID, angular, linear, max_distance, goal_ID, k_p, k_d):
        self.ID = ID # unique number of the agent
        self.name = 'turtle'
        self.status = "wait"
        self.general_goal_ID = goal_ID
        self.local_goal_ID = self.general_goal_ID
        self.x = 0.0 # x coordinate of the agent
        self.y = 0.0 # y coordinate of the agent
        self.theta = 0.0 # in [0, 2*Pi]
	    self.price = 0.0 # the price of the agent
	    self.max_distance = max_distance
        self.max_linear_velocity = linear
        self.max_angular_velocity = angular
        self.max_angle_error = 0.3
        self.angle_past_error = 0
        self.angle_present_error = 0
        self.distance_past_error = 0
        self.distance_present_error = 0
        self.k_p = k_p # the coefficient for the proportional terms
        self.k_d = k_d # the coefficient for the derivative term
        self.agents = {self.ID: [self.x, self.y, self.theta]}
	    self.agents_prices = {self.ID: self.price} # contain prices of all agents
        self.borders_points = {"A": [2, 8], "B": [2, 2], "C": [8, 2], "D": [8, 8]}
        self.base_msg = base()
	    self.trade_msg = trade()
        self.geometry_msg = geometry_msgs.msg.Twist()
        # register pub to send twist velocity
        self.velocity_publisher = rospy.Publisher(self.name + str(self.ID) + "/cmd_vel",
                                                  geometry_msgs.msg.Twist, queue_size=1)
        # register pub to send agent coordinates
        self.coordinate_publisher = rospy.Publisher("/environment", base, queue_size=1)
        # register pub to send the agent price
	    self.trade_publisher = rospy.Publisher("/trade", trade, queue_size=1)
        # register sub to get the agent pose
        self.pose_subscriber = rospy.Subscriber(self.name + str(self.ID) + '/pose',
                                                turtle.Pose, self.callback_pose, queue_size=1)
        # register sub to get coordinates of other agents
        self.coordinate_subscriber = rospy.Subscriber("/environment", base,
                                                      self.callback_coordinate, queue_size=6)
        # register sub to get status msg ("move", "wait" and etc)
        self.status_msg_subscriber = rospy.Subscriber("/status", status,
                                                      self.callback_status, queue_size=1)
        # register sub to get prices of other agents
	    self.trade_subscriber = rospy.Subscriber("/trade", trade, 
                                                 self.callback_trade, queue_size=6)

    def start(self):
        # waiting
        if self.ID == self.general_goal_ID:
            rospy.sleep(1)
        # waiting for a msg from the turtle leader
        while self.general_goal_ID not in self.agents:
            rospy.sleep(1)
        rospy.sleep(1)
        self.status = "move"
        # if the agent isn't general goal
        if self.ID != self.general_goal_ID:
            self.trade()
            rospy.sleep(1)
            while not rospy.is_shutdown():
                if self.status == "move":
                    self.move()

    def move(self):
        if self.detect_border():
            # go to the centre of the polygon ABCD
	    rospy.loginfo('border')
            A = self.borders_points["A"]
            C = self.borders_points["C"]
            y_goal = 0.5*(A[0]+C[0])
            x_goal = 0.5*(A[1]+C[1])
        else:
            # go to the agent local goal
            x_goal = self.agents[self.local_goal_ID][0]
            y_goal = self.agents[self.local_goal_ID][1]
        angular, linear = self.get_velocity(x_goal, y_goal)
	self.geometry_msg.angular.z = angular
        self.geometry_msg.linear.x = linear
        self.velocity_publisher.publish(self.geometry_msg)

    def detect_border(self):
        A = self.borders_points["A"]
        B = self.borders_points["B"]
        C = self.borders_points["C"]
        D = self.borders_points["D"]
	polygon = [A, B, C, D]
        # return true if the agent is inside the borders
        return self.is_point_in_polygon(polygon)

    def is_point_in_polygon(self, polygon):
        for i in range(len(polygon)):
            if (((polygon[i][0] <= self.y and self.y < polygon[i - 1][0]) or
                 (polygon[i - 1][0] <= self.y and self.y < polygon[i][0])) and
                    (self.x > (polygon[i - 1][1] - polygon[i][1]) *
                     (self.y - polygon[i][0]) / (polygon[i - 1][0] -
                     polygon[i][0]) + polygon[i][1])):
                            return False
        return True

    def trade(self):
        price = self.get_price()
        self.trade_msg.price = price
        self.trade_msg.ID = self.ID
        pub = False
        # publish while dont get msg from all turtles
        while len(self.agents_prices) < len(self.agents) - 1 or pub == False:
            self.trade_publisher.publish(self.trade_msg)
            pub = True

	self.local_goal_ID = self.get_goal_ID()

    def sort_by_price_and_id(self, element):
        return element[1], element[0]

    def get_price(self):
        # price is turtle distance form the goal
        x_goal = self.agents[self.general_goal_ID][0]
        y_goal = self.agents[self.general_goal_ID][1]
        x = self.x
        y = self.y
        return self.get_distance(x_goal, y_goal, self.x, self.y)

    def get_goal_ID(self):
        # convert dictionary (self.agent_prices) to list of tuples (agent ID, price)
        prices = [(ID, price) for ID, price in self.agents_prices.items()]
        prices.sort(key=self.sort_by_price_and_id)
        # find a number of the turtle
        for i in range(len(prices)):
            if prices[i][0] == self.ID:
                number = i
                break
        # find the goal of the agent and return it
	if number == 0:
            goal_ID = self.general_goal_ID
        else:
            goal_ID = prices[number - 1][0]
        return goal_ID

    def get_distance(self, x1, y1, x2, y2):
        # the distance between two points (X1, y1) and (x2, y2)
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def get_theta(self):
        # get theta of the agent
        theta = self.theta
        if self.theta < 0:
            return (theta+2*math.pi)
        else:
            return self.theta

    def get_angle(self, x, y):
        # get the angle between OX
        # and the line agent and goal points
        phi = math.atan2(y-self.y, x-self.x)
        theta = self.get_theta()
        if phi < 0:
            phi += math.pi*2
        angle = self.get_error_angle(theta, phi)
        return angle

    def get_error_angle(self, theta, phi):
        # get the real angle between two angle
        return math.atan2(math.sin(phi-theta), math.cos(phi-theta))

    def get_velocity(self, x_goal, y_goal):
        distance = self.get_distance(x_goal, y_goal, self.x, self.y)
        # stop if the agent is close to the goal
        if distance < self.max_distance:
           self.angle_past_error = 0
           self.distance_past_error = distance
           return 0, 0
        angle = self.get_angle(x_goal, y_goal)
        if abs(angle) < self.max_angle_error:
            angular = 0
            linear = self.k_p*(distance)
            self.angle_past_error = 0
            self.distance_past_error = distance
	else:
            # angular = K*error + D*(present_error - past_error)
	    angular = angle*self.k_p + self.k_d*(angle-self.angle_past_error)
            linear = self.k_p*(distance) + self.k_d*abs(distance-self.distance_past_error)
            if linear > self.max_linear_velocity:
                linear = self.max_linear_velocity
            if angular > self.max_angular_velocity:
                angular = self.max_angular_velocity
            self.angle_past_error = angle
            self.distance_past_error = distance
        return angular, linear

    def callback_pose(self, data):
        self.x = data.x
        self.y = data.y
        self.theta = data.theta
        self.base_msg.x = data.x
        self.base_msg.y = data.y
        self.base_msg.theta = data.theta
        self.base_msg.load = str(self.ID)
        self.coordinate_publisher.publish(self.base_msg)

    def callback_coordinate(self, data):
        self.agents[int(data.load)] = [data.x, data.y, data.theta]

    def callback_status(self, data):
        if self.ID != self.general_goal_ID and self.general_goal_ID in self.turtles:
                self.trade()
        self.status = data.status
        rospy.sleep(1)

    def callback_trade(self, data):
        self.agents_prices[data.ID] = [data.price]
