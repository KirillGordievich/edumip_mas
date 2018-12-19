#!/usr/bin/python

import rospy
import math
import geometry_msgs.msg
from edumip_mas.msg import base_message
from edumip_mas.msg import status
import turtlesim.msg
import time


class Edumip:

    def __init__(self, number):

        self.number = number
        self.name = 'robot' + str(number)
        self.status = "wait"
	self.base_msg = base_message()
	self.geometry_msg = geometry_msgs.msg.Twist()
        self.subscriber_pose = rospy.Subscriber("/" + str(self.name) + "/edumip/pose", turtlesim.msg.Pose, self.callback_pose, queue_size = 1)
        self.velocity_publisher = rospy.Publisher("/robot" + str(self.number) + "/edumip/cmd_vel", geometry_msgs.msg.Twist, queue_size = 1)
	self.coordinate_publisher = rospy.Publisher("/environment", base_message, queue_size=1)
	self.subscriber_coordinate = rospy.Subscriber("/environment", base_message, self.callback_coordinate, queue_size = 10)
	self.subscriber_status_msg = rospy.Subscriber("/status", status, self.callback_status, queue_size = 1)
	self.general_goal_name = "robot0"
	self.local_goal_name = None
	self.x = 0
	self.y = 0
	self.theta = 0
	self.edumips = {self.name: [self.x, self.y, self.theta]}
        
    def move(self):

	rospy.sleep(3)

	if self.name != self.general_goal_name:
		self.trade()

	self.status = "move"

	while not rospy.is_shutdown():

		if self.name != self.general_goal_name:

			if self.status == "move":
					
				x_goal = self.edumips[self.local_goal_name][0]
				y_goal = self.edumips[self.local_goal_name][1]

				if self.get_distance(x_goal, y_goal, self.edumips[self.name][0], self.edumips[self.name][1]) > 0.5:
					
        				self.geometry_msg.angular.z = 6 * (math.atan2(y_goal - self.y, x_goal - self.x) - self.edumips[self.name][2])
        				self.geometry_msg.linear.x = 1.5 * self.get_distance(x_goal, y_goal, self.turtles[self.name][0], self.edumips[self.name][1])
				else:
					self.geometry_msg.angular.z = 0
        				self.geometry_msg.linear.x = 0

        			self.velocity_publisher.publish(self.geometry_msg)
				rospy.sleep(0.2)

				

    def callback_pose(self, data):

	self.x = data.x
	self.y = data.y
	self.theta = data.theta
	self.base_msg.x = data.x
	self.base_msg.y = data.y
	self.base_msg.theta = data.theta
	self.base_msg.load = self.name
	self.coordinate_publisher.publish(self.base_msg)

    def callback_coordinate(self, data):

        self.edumips[data.load] = [data.x, data.y, data.theta]

    def trade(self):

        edumips = (sorted(self.edumips.items(), key=lambda item: (item[1][0] - self.edumips[self.general_goal_name][0]) ** 2 + (item[1][1]- self.edumips[self.general_goal_name][1]) ** 2))
        edumips_order = dict([(edumips[i][0], i) for i in range(0, len(self.edumips))])
	inv_edumips_order = {v: k for k, v in edumips_order.items()}
        self.local_goal_name = inv_edumips_order[edumips_order[self.name]-1]

    def callback_status(self, data):

	if self.name != self.general_goal_name:
		self.trade()
	self.status = data.status
	rospy.sleep(3)
    
    def get_distance(self, x1, y1, x2, y2):
        
        return math.sqrt((x1- x2) ** 2 + (y1- y2) ** 2)

if __name__ == "__main__":

    rospy.init_node("~edumip", anonymous=True)
    number = rospy.get_param("~number")
    x = Edumip(number)
    x.move()
    rospy.spin()
