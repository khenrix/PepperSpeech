import argparse
import sys
import time

import qi
from pyaudio import PyAudio

class StreamAudio(object):
	"""
	Stream audio from pepper with pyaudio and naoqi.

	TO RUN: python pepper_sound.py --ip [YOUR PEPEPR IP] --port [YOUR PEPPER PORT]
	EX: python pepper_sound.py --ip 192.168.xx.xx --port 9559
	"""

	def __init__(self, app):
		super(StreamAudio, self).__init__()
		app.start();
		session = app.session

		self.robot_audio = session.service("ALAudioDevice")

	@qi.nobind
	def start(self, serviceName):
		self.pyaudio = PyAudio()
		self.stream = self.pyaudio.open(format=self.pyaudio.get_format_from_width(2), channels=1, rate=16000, output=True)
		self.robot_audio.setClientPreferences(serviceName, 16000, 3, 0)
		self.robot_audio.subscribe(serviceName)

		try:
			while True:
				time.sleep(1)
		except KeyboardInterrupt:
			pass

		self.robot_audio.unsubscribe(serviceName)
		self.stream.close()
		self.pyaudio.terminate()

	def processRemote(self, nbOfChannels, nbOfSamplesByChaneel, timeStamp, inputBuffer):
		self.stream.write(str(inputBuffer))

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("--ip", type=str, default="192.168.1.131", help="PEPPERS IP ADRESS, just ask pepper")
	parser.add_argument("--port", type=int, default=9559, help="Naoqi port number")
	args = parser.parse_args()

	try:
		connection_url = "tcp://{0}:{1}".format(args.ip, args.port)
		app = qi.Application(["MyStreamAudio", "--qi-url=" + connection_url])
	except RuntimeError:
		print "Can't connect to Naoqi at ip \" {0} \" on port {1}. \nPlease check your script arguments. Run with -h option for help.".format(args.ip, args.port)
		sys.exit(1)

	player = StreamAudio(app)
	app.session.registerService("MyStreamAudio", player)
	player.start("MyStreamAudio")
