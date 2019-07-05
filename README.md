## Prerequisite

### 0. Dependencies

```
sudo apt-get install qt5-default
```

### 1. Install OpenCV 3.4.6

```
wget https://github.com/opencv/opencv/archive/3.4.6.zip
unzip opencv-3.4.6.zip
cd opencv-3.4.6/
mkdir build/
cd build/
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local -DFORCE_VTK=ON \
-DWITH_TBB=ON -DWITH_V4L=ON -DWITH_QT=ON -DWITH_OPENGL=ON -DWITH_CUBLAS=ON \
-DCUDA_NVCC_FLAGS="-D_FORCE_INLINES" -DWITH_GDAL=ON -DWITH_XINE=ON -DBUILD_EXAMPLES=ON ..
make -j4
sudo make install
```

### 2. Install Raspicam

```
git clone https://github.com/cedricve/raspicam.git
cd raspicam/
mkdir build
cd build
cmake ..
sudo make install
```

### 3. Install WiringPi

```
git clone git://git.drogon.net/wiringPi
cd WiringPi
./build
```
