cc = g++

# Compiler flags
flags = -Wall -O2 -I./header $(shell pkg-config --cflags opencv4 aravis-0.8)

# Libraries (pkg-config + explicit libusb)
libs = $(shell pkg-config --libs opencv4 aravis-0.8) -lusb-1.0

# Source and header files
Source = src/ExtTrig.cpp
Header = header/ExtTrig.h
Main = cam.cpp
Target = cam

# Default target
all: $(Target)

# Build target
$(Target): $(Source) $(Main)
	$(cc) $(flags) $(Source) $(Main) -o $(Target) $(libs)

# Clean
clean:
	rm -f $(Target) *.png

.PHONY: clean all
