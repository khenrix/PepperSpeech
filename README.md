# PepperSpeech :robot: :speech_balloon:
Speech recognition package for the pepper robot. Makes use of PocketSphinx and GStreamer to translate speach to text and map them to sentences.  
This package is a modifcation of [UTNuclearRoboticsPublic](https://github.com/UTNuclearRoboticsPublic/pocketsphinx) package and will not be maintained further by me.
It would therefore be wise to take a look  
at that repository in case of any big updates, it probably will be more up to date.  
<br>
### Installation
Take a look at the dependencies section if something is not working properly.
```
./install_dep.sh
```
  
### Usage

```
roslaunch pocketsphinx pocketsphinx.launch
```
  
<br>
# Modifications
  
### Create your own vocabulary files
If you want to be able to recognize other sentences you need to create your own .dic and .kwlist files.
These are models which provide the system with a mapping of vocabulary words to sequences of phonemes.
<br>  
:gear: &nbsp; [lmtool (currently only supports english)](http://www.speech.cs.cmu.edu/tools/lmtool.html)

> The lmtool builds a consistent set of lexical and language model files for decoders. The target decoders are the Sphinx family, though any system that can read ARPA-format files can use them.
  Create a corpus with the sentences you want to be able to recognize and run them through the lmtool which generates the files for you, example below.
  
##### :memo: &nbsp; Corpus.txt
```
Hello
How are you?
Thank you
Good bye
```
  
<br>
### Change language
If you want to recognize another language than english you need to change the speach recognition model. Here are some models from cmusphinx trained on different languages. 
Download the model you want and extract the files. Then replace the content in the "language_model" folder with the extracted files.  
<br>
:gear: &nbsp; [Model Downloads](https://sourceforge.net/projects/cmusphinx/files/Acoustic%20and%20Language%20Models/) 
<br>  
<br>
# Dependencies
    
## General
> There are some general dependencies which the package is based on. Run the command below and you should be good to go. 
  
:book: &nbsp; [General dependencies and CMU Sphinx guide](http://jrmeyer.github.io/asr/2016/01/08/Installing-CMU-Sphinx-on-Ubuntu.html)
  
<br>

### Requirements

* gcc
* automake
* autoconf
* libtool
* bison
* swig
* python-dev
* libpulse-dev

  
<br>

```shell
sudo apt-get install gcc automake autoconf libtool bison swig python-dev libpulse-dev
```
  
<br>
## CMU Sphinx (speech recognition)
> CMU Sphinx is a toolkit with a number of packages for different tasks and applications. We will use Pocketsphinx and Sphinxbase for speech recognition.

:book: &nbsp; [PocketSphinx guide by cmusphinx](https://cmusphinx.github.io/wiki/tutorialpocketsphinx/)
  
<br>
### Requirements

* Pocketsphinx — recognizer library written in C.
* Sphinxbase — support library required by Pocketsphinx

<br>
  
```shell
echo "Installing sphinxbase"
mkdir sphinx-source
cd sphinx-source
git clone https://github.com/cmusphinx/sphinxbase.git
cd sphinxbase
  
echo "Compiling and installing"
./autogen.sh
make
sudo make install
  
echo "Adding (/usr/local/lib) to library search path"
echo "/usr/local/lib" | sudo tee -a /etc/ld.so.conf
sudo ldconfig
  
echo "Check if computer is finding shared libraries"
ldconfig -p | grep local
  
echo "Sphinxbase installed"
cd ..
  
echo "Installing PocketSphinx"
git clone https://github.com/cmusphinx/pocketsphinx.git
cd pocketsphinx
  
echo "Compiling and installing"
./autogen.sh
make
sudo make install
  
echo "Updating search path (ldconfig)"
sudo ldconfig
  
echo "PocketSphinx installed"
```
  
<br>
## GStreamer (audio streaming framework)
> PocketSphinx support for the GStreamer streaming media framework. What this means is that the PocketSphinx decoder can be treated as an element in a media processing pipeline, specifically, one which filters audio into text.

:book: &nbsp; [GStreamer guide by cmusphinx](https://cmusphinx.github.io/wiki/gstreamer/)
  
<br>
### Requirements
* gstreamer-1.0.pc
* gstreamer-base-1.0.pc
* gstreamer-plugins-base-1.0.pc

<br>
:book: &nbsp; [Here is a full installation guide](https://gstreamer.freedesktop.org/documentation/installing/on-linux.html)
<br>
```
sudo apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools
```
  
<br>
# Further reading
1. [ROS pocketsphinx speech recognition tutorial](https://code.google.com/archive/p/ros-pocketsphinx-speech-recognition-tutorial/)  
2. [ROS actionlib server/client communcation](http://wiki.ros.org/actionlib)