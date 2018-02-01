# PepperSpeech :robot: :speech_balloon:
PepperSpeech is a simple base for speech recognition for the pepper robot. 
The ROS package mainly consists of three parts (python scripts) and is ment to be used as a quick start/guide for people wanting to implement speech recognition functionality in their own projects. 
Please take a look at the wiki for further details on troubleshooting/modifications etc. 
<br>  
  
### :grey_question: &nbsp; Current functionality
We are trying to simulate a scenario where pepper is gathering drink orders from customers. 
To begin with pepper streams audio and then tries to recognize what people are saying, we have some basic things which can be recognized (like beer, wine, coke, yes, no).
If pepper recognizes a order the process stops and we publish the order to a ROS node called "pepper_speech_node" and peppers says something witty.  
  
### :file_folder: &nbsp; FILES
<b>stream_audio.py</b> - establishes a link between pepper and the computer, then reads audio.  
<b>speech_recogntion.py</b> - takes audio and matches it to predefined sentences in the corpus using pocketsphinx.  
<b>pepper_response</b> - decides what to do depending on what sentence is recognized.  
  
Getting everything to work together might be quite tricky and if something is not working I would recommend taking a look at the dependencies section in the wiki.
I tried to gather some guides and useful pages for troubleshooting purposes, please take note that everything is setup on Linux Ubuntu 16.04.
I also assume that you have general knowledge of ROS and are connected to peppers network. 
  
I recommend to get the tests working first and then continue working with the base. See the tests section of the wiki.

I wish you happy coding and good luck with your projects!  
<br>
# Quick start
### Installation
Run the following bash script. Take a look at the [dependencies](https://github.com/khenrix/PepperSpeech/wiki/Dependencies) section in the wiki if something is not working properly.
```
./install_dep.sh
```
  
### Usage
1. Through roslaunch
```
roslaunch pepperspeech pepperspeech.launch
```

2. Through python command line
```
python stream_audio.py --ip YOUR PEPPER IP ADRESS --port YOUR PEPPER PORT
```

3. Through another python script in ROS  
```
    import roslib
    import rospy
    import actionlib
    import rospkg
    import imp
    import sys
    import time
    import qi
    
    rospack = rospkg.RosPack() 
    package_path = rospack.get_path('pepperspeech') + '/scripts/stream_audio.py' # Quite crude, os.path might be good to use here.
    speech = imp.load_source('StreamAudio', package_path)
    
    pepper_ip = "192.168.1.131" # Your pepper IP adress
    pepper_port = 9559 # Your pepper Port
    
    # Using session instead of application as it can be initialized more than once. 
    
    try:
      connection_url = "tcp://{0}:{1}".format(pepper_ip, pepper_port)
      app = qi.Session()
      app.connect(connection_url)
    except RuntimeError:
      print "Can't connect to Naoqi at ip \" {0} \" on port {1}. \nPlease check your script arguments.".format(pepper_ip, pepper_port)
      sys.exit(1)
    
    player = speech.StreamAudio(app)
    app.registerService("MyStreamAudio", player)
    player.start("MyStreamAudio")
```
