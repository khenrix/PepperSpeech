#!/usr/bin/env python
import rospy
import rospkg

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *

import os
import commands
import imp

class SpeechRecognizer(object):

	def __init__(self):
		print("TEST!!!")
		rospy.loginfo("Loading language model and vocabulary files")
		rospack = rospkg.RosPack()
		package_path = rospack.get_path('pepperspeech')
		self.lm = package_path + '/language_model'
		self.lexicon = package_path + '/vocab/voice_cmd.dic'
		self.kw_list = package_path + '/vocab/voice_cmd.kwlist'
		rospy.loginfo("Done loading language model and vocabulary files")

		rospy.loginfo("Initializing pocketsphinx")
		config = Decoder.default_config()
		rospy.loginfo("Done initializing pocketsphinx")

		# Hidden Markov model: The model which has been used
		config.set_string('-hmm', self.lm)
		#Pronunciation dictionary used
		config.set_string('-dict', self.lexicon)
		#Keyword list file for keyword searching
		config.set_string('-kws', self.kw_list)

		rospy.loginfo("Starting the decoder")
		self.decoder = Decoder(config)
		self.decoder.start_utt()
		rospy.loginfo("Done starting the decoder")

		rospack = rospkg.RosPack()
		package_path = rospack.get_path('pepperspeech') + '/scripts/pepper_response.py'
		pr = imp.load_source('PepperResponse', package_path)
		self.response = pr.PepperResponse()

	def decode(self, buffer, output=True):
		self.decoder.process_raw(buffer, False, False)
		if self.decoder.hyp() != None:
			recognized_segments = [(seg.word) for seg in self.decoder.seg()]

			if output:
				print(recognized_segments)

			isOrder = self.response.respond(recognized_segments[0])

			self.decoder.end_utt()
			self.decoder.start_utt()
			return isOrder

		else:
			return False

if __name__ == "__main__":
	print("RUNNINGS SPEECH RECOGNITION SCRIPT")
