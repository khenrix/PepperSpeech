#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""
Connects to the pepper robot and records audio from the pepper robot.
"""

import qi
import argparse
import sys
import time
import numpy as np

import rospy
import rospkg

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *

from std_msgs.msg import String
from std_srvs.srv import *


class SoundProcessingModule(object):
    """
    A simple get signal from the front microphone of Nao & calculate its rms power.
    It requires numpy.
    """

    def __init__( self, app):
        """
        Initialise services and variables.
        """
        super(SoundProcessingModule, self).__init__()
        app.start()
        session = app.session

        rospy.init_node("test_pepper_node")

        # Get the service ALAudioDevice.
        self.audio_service = session.service("ALAudioDevice")
        self.isProcessingDone = False
        self.nbOfFramesToProcess = 20
        self.framesCount=0
        self.micFront = []
        self.module_name = "SoundProcessingModule"

        #self.pub_ = rospy.Publisher('/interaction/status', String, queue_size=10)
        self.pub_ = rospy.Publisher('~output', String, queue_size=1)
        
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('pepperspeech')
        self.lm = package_path + '/language_model'
        self.lexicon = package_path + '/vocab/voice_cmd.dic'
        self.kw_list = package_path + '/vocab/voice_cmd.kwlist'

    def startProcessing(self):
        """
        Start processing
        """

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

        # ask for the front microphone signal sampled at 16kHz
        # if you want the 4 channels call setClientPreferences(self.module_name, 48000, 0, 0)
        self.audio_service.setClientPreferences(self.module_name, 16000, 3, 0)
        self.audio_service.subscribe(self.module_name)

        rospy.loginfo("Done opening the audio channel")

        rospy.loginfo("Starting the decoder")
        self.decoder = Decoder(config)
        self.decoder.start_utt()
        rospy.loginfo("Done starting the decoder")

        while self.isProcessingDone == False:
            time.sleep(1)

        self.audio_service.unsubscribe(self.module_name)

    def processRemote(self, nbOfChannels, nbOfSamplesByChannel, timeStamp, inputBuffer):
        """
        Try to match the inputBuffer to one of our sentences.
        Tries "nbOfFramesToProcess" times before quitting or until finding a match.
        """
        self.framesCount = self.framesCount + 1

        if (self.framesCount <= self.nbOfFramesToProcess):
            self.isProcessingDone = self.match_sentence(inputBuffer)
        else :
            self.isProcessingDone=True

    def match_sentence(self, buffer):
        self.decoder.process_raw(buffer, False, False)
        if self.decoder.hyp() != None:
            sentences = [(seg.word) for seg in self.decoder.seg()]
            print (sentences)
            res = sentences[0].lower()
            self.decoder.end_utt()
            self.decoder.start_utt()
            self.pub_.publish(res)
            return True
        else:
            return False

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.1.131",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    try:
        # Initialize qi framework.
        connection_url = "tcp://" + args.ip + ":" + str(args.port)
        app = qi.Application(["SoundProcessingModule", "--qi-url=" + connection_url])
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    MySoundProcessingModule = SoundProcessingModule(app)
    app.session.registerService("SoundProcessingModule", MySoundProcessingModule)
    MySoundProcessingModule.startProcessing()
