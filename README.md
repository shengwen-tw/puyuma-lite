## Prerequisite

### 0. Dependencies

```
sudo apt-get install qt5-default cmake vim libyaml-cpp-dev
```
### 1. Setup swap memory (recommendation)

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


## Calibration

### 1. Intrinsic calibration

```
ssh -Y -C pi@HOSTNAME

cd puyuma/

./puyuma -c intrinsic
```
