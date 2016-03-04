


This FroboMind component depends on a third party driver. To install the
driver execute the following shell commands:

mkdir -p ~/frobomind-temp/ros-deps
cd ~/frobomind-temp/ros-deps 
git clone http://github.com/konradb3/libLMS1xx.git 
cd libLMS1xx
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr 
make 
sudo make install
cd ~/
rm -rf ~/frobomind-temp


