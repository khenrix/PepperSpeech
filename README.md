# PepperSpeech :robot: :speech_balloon:
PepperSpeech is a simple base for speech recognition for the pepper robot. 
The ROS package mainly consists of three parts (python scripts) and is ment to be used as a quick start guide for people wanting to implement speech recognition functionality in their own projects. 
Please take a look at the wiki for further details on installation/troubleshooting/modifications etc.   
<br>
<b>stream_audio.py</b> - establishes a link between peppers and the computer, then reads audio (1024 bytes).  
<b>speech_recogntion.py</b> - takes audio and matches it to predefined sentences in the corpus using pocketsphinx.  
<b>pepper_response</b> - decides what to do depending on what sentence is recognized.  
<br>
Getting everything to work together might be quite tricky and if something is not working I would recommend taking a look at the dependencies section in the wiki where I tried to gather some guides and useful pages for troubleshooting purposes.   
<br>
I recommend to get the tests working first and then continue working with the base. See the tests section of the wiki.

I wish you happy coding and good luck with your projects!  
<br>
# Quick start
### Installation
Take a look at the [dependencies section in the wiki](https://gitlab.ida.liu.se/TDDE19athome/pepperspeech/wikis/dependencies) if something is not working properly.
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
    
    # Using session instead of application as it can be initialized more then once. 
    
    try:
      connection_url = "tcp://{0}:{1}".format("pepper_ip, pepper_port)
      app = qi.Session()
      app.connect(connection_url)
    except RuntimeError:
      print "Can't connect to Naoqi at ip \" {0} \" on port {1}. \nPlease check your script arguments.".format(pepper_ip, pepper_port)
      sys.exit(1)
    
    player = speech.StreamAudio(app)
    app.registerService("MyStreamAudio", player)
    player.start("MyStreamAudio")
```