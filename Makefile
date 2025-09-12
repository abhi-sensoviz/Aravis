cc=g++

flags= -Wall -O2 -I./header $(shell pkg-config --cflags opencv4 aravis-0.10)

libs = $(shell pkg-config --libs opencv4 aravis-0.10)

Source = src/CameraHandler.cpp 
Header = header/CameraHandler.h 

Main= cam.cpp
Target= cam


all: $(Target)

$(Target) : $(Source) $(Main)
	$(cc) $(flags)  $(Source) $(Main) -o $(Target) $(libs)


clean:
	rm $(Target) *.png
	


.PHONY: clean all






