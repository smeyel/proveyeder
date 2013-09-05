CC = g++
AR = ar
CFLAGS = 	-c -Wall \
			-I include/ \
			-I ../Framework/libCommunication/include \
			-I ../Framework/libLogConfigTime/include \
			-I ../Framework/libPlatformSpecifics/include \
			-I ../Framework/libCameraAbstraction/include \
			-I ../Tracking/lib3dWorld/include/ \
			-I ../Framework/libVideoInput/include \
			-I ../ImageProcessing/libTwoColorCircleMarker/include/


LIBS =	../Framework/libCameraAbstraction/libCameraAbstraction.a \
		../Framework/libLogConfigTime/libLogConfigTime.a \
		../Framework/libCommunication/libCommunication.a \
		../Framework/libPlatformSpecifics/libPlatformSpecifics.a \
		../Tracking/lib3dWorld/lib3dWorld.a \
		../Framework/libVideoInput/libVideoInput.a \
		../ImageProcessing/libTwoColorCircleMarker/libTwoColorCircleMarker.a
								
LDFLAGS = `pkg-config --libs opencv`

all: CamClient

CAMCLIENT_OBJECTS =  src/main.o src/MyPhoneServer.o
								

CamClient: $(CAMCLIENT_OBJECTS)
	$(CC) $(CAMCLIENT_OBJECTS) $(LIBS) $(LDFLAGS) -o CamClient

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@
	
clean:
	rm -rf src/*.o CamClient
	
.PHONY: all clean
