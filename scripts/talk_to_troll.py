#!/usr/bin/env python

import rospy
import random
from trollnode.msg import Expression

def talker(speech, expression):
	pub = rospy.Publisher('trollExpression', Expression, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(0.2) # 10hz
	while not rospy.is_shutdown():
		#TODO fix .msg!!
		expr_msg = Expression()s
		expr_msg.expression = expression
		expr_msg.speech = speech
		pub.publish(expr_msg)
		rate.sleep()

def talk_random_expression(speech):
	expr = ["happy", "angry", "smile", "sad", "disgust", "surprise", "fear", "suspicious",
		"blink", "pain", "duckface"]
	talker(speech, expr[random.randint(0, len(expr)-1)])

def talk_default_expression(speech):
	talker(speech, "happy")

if __name__ == '__main__':
	try:
		#talker("hello", "smile")
		talk_random_expression("hello")
	except rospy.ROSInterruptException:
		pass

