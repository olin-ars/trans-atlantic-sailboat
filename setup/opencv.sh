echo Removing any previous installations of x264...
sudo apt remove -y x264 libx264-dev
echo Installing OpenCV dependencies...
sudo apt install -y checkinstall cmake pkg-config yasm gfortran \
  libjpeg8-dev libjasper-dev libpng12-dev
sudo apt install -y libtiff5-dev # For Ubuntu 16.04
sudo apt install -y libavcodec-dev libavformat-dev libswscale-dev \
  libdc1394-22-dev libxine2-dev libv4l-dev libgstreamer0.10-dev \
  libgstreamer-plugins-base0.10-dev qt5-default libgtk2.0-dev libtbb-dev \
  libatlas-base-dev libfaac-dev libmp3lame-dev libtheora-dev libvorbis-dev \
  libxvidcore-dev libopencore-amrnb-dev libopencore-amrwb-dev x264 v4l-utils

echo Installing necessary Python libraries...
pip install numpy scipy matplotlib scikit-image scikit-learn ipython

echo Downloading OpenCV source...
cd $oars_root
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout $opencv_version

echo Downloading OpenCV contrib source...
cd $oars_root
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout $opencv_version

echo Building OpenCV with contrib modules...
cd ../opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D INSTALL_C_EXAMPLES=ON \
  -D INSTALL_PYTHON_EXAMPLES=ON \
  -D WITH_TBB=ON \
  -D WITH_V4L=ON \
  -D WITH_QT=ON \
  -D WITH_OPENGL=ON \
  -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
  -D BUILD_EXAMPLES=ON ..
num_cpu_cores=`nproc`
make -j$num_cpu_cores
sudo make install
sudo sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig

echo OpenCV setup complete