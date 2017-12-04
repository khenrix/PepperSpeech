# Pepperspeech
> Speech recognition package for the pepper robot. Makes use of PocketSphinx and GStreamer to translate speach to text and map them to sentences. This package is a modifcation of[UTNuclearRoboticsPublic](https://github.com/UTNuclearRoboticsPublic/pocketsphinx) package and will not be maintained further by me. It would therefore be wise to take a look at that repository in case of changes as it probably will be maintained better.
### Installation
Take a look at the dependencies section if something is not working properly.
```
./install_dep.sh
```

### Usage

```
roslaunch pocketsphinx pocketsphinx.launch
```

### Create your own vocabulary files
If you want pepper to recognize other sentences you need to create your own .dic and .kwlist files.
These are models which provide the system with a mapping of vocabulary words to sequences of phonemes.

![alt text](https://assets-cdn.github.com/images/icons/emoji/unicode/2699.png?v7, "Modification") [lmtool (currently only support english)](http://www.speech.cs.cmu.edu/tools/lmtool.html)

> The lmtool builds a consistent set of lexical and language model files for decoders. The target decoders are the Sphinx family, though any system that can read ARPA-format files can use them.

Create a corpus with the sentences you want to be able to recognize and run them through the lmtool which generates the files for you, example below.
```
Hello
How are you?
Thank you
Good bye
```

### Change language
If you want to recognize another language than english you need to change the speach recognition model. Here are some models from cmusphinx trained on different languages.

![alt text](https://assets-cdn.github.com/images/icons/emoji/unicode/2699.png?v7, "Modification") [Model Downloads](https://sourceforge.net/projects/cmusphinx/files/Acoustic%20and%20Language%20Models/)

Download the model you want and extract the files. Then replace the content in the "language_model" folder with the extracted files.

# Dependencies

## Python

### Requirements
```shell
sudo apt-get install gcc automake autoconf libtool bison swig python-dev libpulse-dev
```

## CMU Sphinx (speech recognition)
> CMU Sphinx is a toolkit with a number of packages for different tasks and applications. We will use Pocketsphinx and Sphinxbase for speech recognition.

![alt text](https://assets-cdn.github.com/images/icons/emoji/unicode/1f4d6.png?v7, "Guide") [Recommended CMU Sphinx guide by cmusphinx](https://cmusphinx.github.io/wiki/tutorialpocketsphinx/)


### Requirements

* Pocketsphinx — recognizer library written in C.
* Sphinxbase — support library required by Pocketsphinx
  
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

## GStreamer (audio streaming framework)
> PocketSphinx support for the GStreamer streaming media framework. What this means is that the PocketSphinx decoder can be treated as an element in a media processing pipeline, specifically, one which filters audio into text.

![alt text](https://assets-cdn.github.com/images/icons/emoji/unicode/1f4d6.png?v7, "Guide") [Recommended GStreamer guide by cmusphinx](https://cmusphinx.github.io/wiki/gstreamer/)

### Requirements
* gstreamer-1.0.pc
* gstreamer-base-1.0.pc
* gstreamer-plugins-base-1.0.pc

```
apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools
```

![alt text](https://assets-cdn.github.com/images/icons/emoji/unicode/1f4d6.png?v7, "Guide") [Here is full installation guide](https://gstreamer.freedesktop.org/documentation/installing/on-linux.html)

# Further reading
1. [ROS pocketsphinx speech recognition tutorial](https://code.google.com/archive/p/ros-pocketsphinx-speech-recognition-tutorial/)