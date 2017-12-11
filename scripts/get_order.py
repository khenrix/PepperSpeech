#!/usr/bin/env python

import argparse
import rospy
import rospkg

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio

from std_msgs.msg import String
from std_srvs.srv import *
import os
import commands

class recognizer(object):

    def __init__(self):

        # initialize ROS
        self.speed = 0.2

        # Start node
        #rospy.init_node("recognizer")
        #rospy.on_shutdown(self.shutdown)

        # you may need to change publisher destination depending on what you run
        self.pub_ = rospy.Publisher('/interaction/status', String, queue_size=10)
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('pepperspeech')
        self.lm = package_path + '/language_model'
        self.lexicon = package_path + '/vocab/voice_cmd.dic'
        self.kw_list = package_path + '/vocab/voice_cmd.kwlist'

    def start_recognizer(self):
        # initialize pocketsphinx. As mentioned in python wrapper
        rospy.loginfo("Initializing pocketsphinx")
        config = Decoder.default_config()
        rospy.loginfo("Done initializing pocketsphinx")

        # Hidden Markov model: The model which has been used
        config.set_string('-hmm', self.lm)
        #Pronunciation dictionary used
        config.set_string('-dict', self.lexicon)
        #Keyword list file for keyword searching
        config.set_string('-kws', self.kw_list)

        rospy.loginfo("Opening the audio channel")

	# Pocketsphinx requires 16kHz, mono, 16-bit little-Endian audio.
	# See http://cmusphinx.sourceforge.net/wiki/tutorialtuning
        stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
                        rate=16000, input=True, frames_per_buffer=1024)
        stream.start_stream()
        rospy.loginfo("Done opening the audio channel")

        #decoder streaming data
        rospy.loginfo("Starting the decoder")
        self.decoder = Decoder(config)
        self.decoder.start_utt()
        rospy.loginfo("Done starting the decoder")

        found_order = False

        # Main loop
        while not found_order:
            # taken as is from python wrapper
            buf = stream.read(1024)
            if buf:
                self.decoder.process_raw(buf, False, False)
            else:
                break
            found_order = self.publish_result()

        return True

    def publish_result(self):
        """
        Publish the words
        """
        if self.decoder.hyp() != None:
            print ([(seg.word)
                for seg in self.decoder.seg()])
            seg.word = seg.word.lower()
            self.decoder.end_utt()
            self.decoder.start_utt()
            self.pub_.publish(seg.word)
            return True
        else:
            return False


if __name__ == "__main__":
    #r = recognizer()
    #r.start_recognizer()
