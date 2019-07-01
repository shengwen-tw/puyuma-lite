EXECUTABLE=puyuma
CC=g++

#OpenCV 3
CFLAGS=`pkg-config opencv --cflags` `pkg-config opencv --libs`
LDFLAGS:=

CFLAGS+=-I./src
SRC+=src/main.cpp \
	src/lane_detector.cpp

#################################################################
all:$(EXECUTABLE)

$(EXECUTABLE):$(SRC)
	$(CC) $< $(LDFLAGS) $(CFLAGS) -o $@ $(SRC)

clean:
	rm -rf $(EXECUTABLE)
