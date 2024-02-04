sudo apt-get update
sudo apt-get upgrade
sudo apt-get -y install build-essential cmake git unzip pkg-config
sudo apt-get -y install libjpeg-dev libpng-dev
sudo apt-get -y install libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get -y install libgtk2.0-dev libcanberra-gtk* libgtk-3-dev
sudo apt-get -y install libgstreamer1.0-dev gstreamer1.0-gtk3
sudo apt-get -y install libgstreamer-plugins-base1.0-dev gstreamer1.0-gl
sudo apt-get -y install libxvidcore-dev libx264-dev
sudo apt-get -y install python3-dev python3-numpy python3-pip
sudo apt-get -y install libtbb2 libtbb-dev libdc1394-22-dev
sudo apt-get -y install libv4l-dev v4l-utils
sudo apt-get -y install libopenblas-dev libatlas-base-dev libblas-dev
sudo apt-get -y install liblapack-dev gfortran libhdf5-dev
sudo apt-get -y install libprotobuf-dev libgoogle-glog-dev libgflags-dev
sudo apt-get -y install protobuf-compiler
pip3 install opencv-contrib-python
python -m pip install --upgrade tflite-support==0.4.3
pip install matplotlib