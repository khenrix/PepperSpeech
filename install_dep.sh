 #!/bin/bash

echo "Installing general dependencies"
sudo apt-get install gcc automake autoconf libtool bison swig python-dev libpulse-dev python-pyaudio

echo "Installing GStreamer dependencies"
sudo apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools

echo "Installing CMU Sphinx dependencies"
mkdir sphinx-source
cd sphinx-source
git clone https://github.com/cmusphinx/sphinxbase.git
cd sphinxbase

./autogen.sh
make
sudo make install

echo "/usr/local/lib" | sudo tee -a /etc/ld.so.conf
sudo ldconfig

ldconfig -p | grep local

cd ..

git clone https://github.com/cmusphinx/pocketsphinx.git
cd pocketsphinx
./autogen.sh

make
sudo make install

sudo ldconfig

cd ..

echo "Installing Naoqi Python SDK"
wget https://developer.softbankrobotics.com/Software/Python/2.5.5/Linux/pynaoqi-python2.7-2.5.5.5-linux64.tar.gz
sudo tar --directory=/opt -xvf pynaoqi-python2.7-2.5.5.5-linux64.tar.gz
echo "export PYTHONPATH=${PYTHONPATH}:/opt/pynaoqi-python2.7-2.5.5.5-linux64/lib/python2.7/site-packages" | sudo tee -a ~/.bashrc
sudo ldconfig

echo "All dependencies should now be installed"
