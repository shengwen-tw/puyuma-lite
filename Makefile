EXECUTABLE=puyuma
CC=g++

#OpenCV 3
CFLAGS=`pkg-config opencv --cflags` `pkg-config opencv --libs`
CFLAGS+=-lyaml-cpp

CFLAGS+=-I./src/core
CFLAGS+=-I./src/calibration

SRC+=src/calibration/intrinsic_calibration.cpp \
	src/calibration/extrinsic_calibration.cpp \
	src/calibration/color_calibration.cpp
SRC+=src/core/main.cpp \
	src/core/lane_detector.cpp \
	src/core/common.cpp \
	src/core/self_driving.cpp

all:$(EXECUTABLE)

$(EXECUTABLE):$(SRC)
	$(CC) $(SRC) $(LDFLAGS) $(CFLAGS) -o $@

clean:
	rm -rf $(EXECUTABLE)
