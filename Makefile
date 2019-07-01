EXECUTABLE=puyuma
CC=g++

#OpenCV 3
CFLAGS=`pkg-config opencv --cflags` `pkg-config opencv --libs`

CFLAGS+=-I./src
SRC+=src/main.cpp \
	src/lane_detector.cpp \
	src/common.cpp \
	src/self_driving.cpp

#################################################################
all:$(EXECUTABLE)

$(EXECUTABLE):$(SRC)
	$(CC) $(SRC) $(LDFLAGS) $(CFLAGS) -o $@

clean:
	rm -rf $(EXECUTABLE)
