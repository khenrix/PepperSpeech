import rospy

from std_msgs.msg import String
from std_srvs.srv import *

import os
import commands


class PepperResponse(object):

	def __init__(self):
		rospy.init_node("pepper_speech_node")
		self.pub_ = rospy.Publisher('/interaction/status', String, queue_size=10)
		self.speak = rospy.Publisher('speech', String, queue_size=10)

		self.orders = ["coke", "wine", "tequila", "beer", "sprite"]
		self.greetings = ["hello", "greetings"]
		self.accept = ["yes", "agreed", "ok", "correct"]
		self.decline = ["no", "disagree", "not"]	

	def publish_order(self, order):
		self.pub_.publish(String(msg))

	def respond(self, msg):
		msg = msg.lower()

		if msg in self.accept:
			return True
		if msg in self.decline:
			return False

		if msg in self.orders:
			self.publish_order(msg)
		if msg in self.greetings:
			self.speak.publish(String("Hello, what would you like to drink?"))

		return True

if __name__ == "__main__":
	print("RUNNINGS PEPPER RESPONSE SCRIPT")
