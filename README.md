# PepperSpeech :robot: :speech_balloon:
Speech recognition package for the pepper robot. Makes use of PocketSphinx and GStreamer to translate speach to text and map them to sentences.  
This package is a modifcation of [UTNuclearRoboticsPublic](https://github.com/UTNuclearRoboticsPublic/pocketsphinx) package and will not be maintained further by me.
It would therefore be wise to take a look  
at that repository in case of any big updates, it probably will be more up to date.  
<br>
### Installation
Take a look at the dependencies section in the wiki if something is not working properly.
```
./install_dep.sh
```
  
### Usage

```
roslaunch pepperspeech pepperspeech.launch
```
OR
```
python stream_audio.py --ip YOUR PEPPER IP ADRESS --port YOUR PEPPER PORT
```