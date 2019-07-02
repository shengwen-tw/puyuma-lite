### 1. Install OpenCV 3.4

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
