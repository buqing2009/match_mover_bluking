CC=gcc
CXX=g++
RM=rm -f


#INCLUDE
OPENCVINC=-I/home/manu/Downloads/opencv/OpenCV-2.3.1/include/
OPENCVINC+=-I/home/manu/Downloads/opencv/OpenCV-2.3.1/include/opencv 
OPENCVINC+=-I/home/manu/Downloads/opencv/OpenCV-2.3.1/include/opencv2

CPPFLAGS=-g $(shell pkg-config gtkmm-2.4 --cflags)
CPPFLAGS+=$(shell pkg-config opencv --cflags)
CPPFLAGS+=${OPENCVINC}
CPPFLAGS+=-I/usr/include/SDL 

#LIBS
LDLIBS=$(shell pkg-config gtkmm-2.4 --libs)
LDLIBS+=$(shell pkg-config opencv --libs)
LDLIBS+=-lglut -lGLU -lGL
LDLIBS+=-L/usr/lib -lSDL

SRCS= main.cpp arcball.cpp calibration.cpp camera.cpp multiview.cpp params.cpp rendering.cpp stereroview.cpp videodevice.cpp videoplayer.cpp

OBJS=$(subst .cpp,.o,$(SRCS))

all: tool

tool: $(OBJS)
	g++ $(LDFLAGS) -o matchmover $(OBJS) $(LDLIBS) 

depend: .depend

.depend: $(SRCS)
	rm -f ./.depend
	$(CXX) $(CPPFLAGS) -MM $^>>./.depend;

clean:
	$(RM) $(OBJS)

dist-clean:	clean
	$(RM) *~ .dependtool

include .depend