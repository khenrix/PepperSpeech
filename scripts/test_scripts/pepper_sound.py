import argparse
import sys
import time

import qi
from pyaudio import PyAudio

class SoundDownloadPlayer(object):
	"""
	GET SOUND FROM PEPPER AND PLAY IT IN REAL TIME
	"""

	def __init__(self, app):
		super(SoundDownloadPlayer, self).__init__()
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
		connection_url = "tcp://" + args.ip + ":" + str(args.port)
		app = qi.Application(["MySoundDownloadPlayer", "--qi-url=" + connection_url])
	except RuntimeError:
		print "CANNOT CONNECT"
		sys.exit(1)

	player = SoundDownloadPlayer(app)
	app.session.registerService("MySoundDownloadPlayer", player)
	player.start("MySoundDownloadPlayer")
