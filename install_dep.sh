 #!/bin/bash

# Created from guide
# http://jrmeyer.github.io/installation/2016/01/08/Installing-CMU-Sphinx-on-Ubuntu.html

echo "Installing Dependencies"
sudo apt-get install gcc automake autoconf libtool bison swig python-dev libpulse-dev

# TODO: add install for these Dependencies
#gstreamer-1.0.pc
#gstreamer-base-1.0.pc
#gstreamer-plugins-base-1.0.pc

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
./autogen.sh

echo "Compiling and installing"
make
sudo make install

echo "Updating search path (ldconfig)"
sudo ldconfig

echo "PocketSphinx installed"
