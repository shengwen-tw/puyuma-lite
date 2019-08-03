# Puyuma-core

Core components of self-driving car system, building a complete open source solution
for low-cost hardware along with Linux real-time extensions.

## Demo videos

[![lane_following](https://github.com/ncku-ros2-research/xenobot/blob/master/materials/demo_video1.jpeg?raw=true)](https://www.youtube.com/watch?v=84MXc0_F61o)

[![rviz](https://github.com/ncku-ros2-research/xenobot/blob/master/materials/demo_video2.jpeg?raw=true)](https://www.youtube.com/watch?v=XK602hzbORY&feature=youtu.be)

Licensing
---------
`puyuma-core` is freely redistributable under the two-clause BSD License.
Use of this source code is governed by a BSD-style license that can be found
in the `LICENSE` file.

## Prerequisite

### 0. Dependencies

```
sudo apt-get install qt5-default cmake vim libyaml-cpp-dev
```
### 1. Setup swap memory (highly recommended for Raspberry Pi)

Install dphys-swapfile

```
sudo apt install dphys-swapfile
```

Setup the swap memory size

```
sudo vim /etc/dphys-swapfile
```

add (or edit)

```
CONF_SWAPSIZE=4096
```

restart swap memory service

```
/etc/init.d/dphys-swapfile restart
```

### 2. Install OpenCV 3.4.6

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

### 3. Install Raspicam

```
git clone https://github.com/cedricve/raspicam.git
cd raspicam/
mkdir build
cd build
cmake ..
sudo make install
```

### 4. Install WiringPi

```
git clone git://git.drogon.net/wiringPi
cd WiringPi
./build
```

## Installation

```
git clone https://github.com/Puyuma/puyuma-core.git

cd puyuma_core/

make -j4
```

## Calibration

### 1. Intrinsic calibration

print out [intrinsic calibrating pattern](https://drive.google.com/open?id=0B2DQhcp-s6aoQ2J3LUVPR3FVZTA) in A4 and run the following cammands:

```
ssh -Y -C pi@HOSTNAME #login to Pi

cd puyuma/

./puyuma -c intrinsic
```


### 2. Extrinsic calibration

print out [extrinsic calibrating pattern](https://drive.google.com/open?id=1J0H0wnwJJ62ytl2PEVEy6D1bAt3FOtRG) in A4 and run the following cammands:

```
ssh -Y -C pi@HOSTNAME #login to Pi

cd puyuma/

./puyuma -c extrinsic
```


### 3. Color thresholding calibration (lane mark identification)

```
ssh -Y -C pi@HOSTNAME #login to Pi

cd puyuma/

./puyuma -c color
```

## Activate self-driving system

```
ssh -Y -C pi@HOSTNAME #login to Pi

cd puyuma/

./puyuma run
```
