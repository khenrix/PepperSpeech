#!/usr/bin/env python
import argparse
import sys
import time
import rospkg

import qi
import imp
import pyaudio
import rospy
from std_msgs.msg import String

class StreamAudio(object):
	"""
	Stream audio from pepper with pyaudio and naoqi.

	TO RUN: python stream_audio.py --ip [YOUR PEPEPR IP] --port [YOUR PEPPER PORT]
	EX: python stream_audio.py --ip 192.168.xx.xx --port 9559
	"""

	def __init__(self, app):
		super(StreamAudio, self).__init__()
		# Using session as input, if app is input uncomment this (session can be initiated more than once without causing error)
		# app.start();
		# self.session = app.session

		self.robot_audio = app.service("ALAudioDevice")
		self.speak = rospy.Publisher('speech', String, queue_size=10) # Node that published to pepper robot, make pepper talk. can be done through Naoqi as well.

	@qi.nobind
	def start(self, serviceName):

		self.speak.publish(String("Hello, can I take your order?"))

		rospack = rospkg.RosPack()
		package_path = rospack.get_path('pepperspeech') + '/scripts/speech_recognition.py'
		sr = imp.load_source('SpeechRecognizer', package_path)
		self.recog = sr.SpeechRecognizer()

		self.pyaudio = pyaudio.PyAudio()
		self.stream = self.pyaudio.open(format=pyaudio.paInt16, channels=1, rate=16000, output=True, frames_per_buffer=1024)
		self.robot_audio.setClientPreferences(serviceName, 16000, 3, 0)
		self.robot_audio.subscribe(serviceName)
		self.stop_listening = False

		try:
			print(self.stop_listening)
			while not self.stop_listening:
				print(self.stop_listening)
				time.sleep(1)
		except KeyboardInterrupt:
			pass

		self.robot_audio.unsubscribe(serviceName)
		self.stream.close()
		self.pyaudio.terminate()

	def processRemote(self, nbOfChannels, nbOfSamplesByChaneel, timeStamp, inputBuffer):
		if not self.stop_listening:
			self.stop_listening = self.recog.decode(str(inputBuffer))


if __name__ == "__main__":
	print("Main")
	parser = argparse.ArgumentParser()
	parser.add_argument("--ip", type=str, default="192.168.1.131", help="PEPPERS IP ADRESS, just ask pepper")
	parser.add_argument("--port", type=int, default=9559, help="Naoqi port number")
	args = parser.parse_args()

	try:
		connection_url = "tcp://{0}:{1}".format(args.ip, args.port)
		app = qi.Session()
		app.connect(connection_url)
	except RuntimeError:
		print "Can't connect to Naoqi at ip \" {0} \" on port {1}. \nPlease check your script arguments. Run with -h option for help.".format(args.ip, args.port)
		sys.exit(1)

	player = StreamAudio(app)
	app.registerService("MyStreamAudio", player)
	player.start("MyStreamAudio")
