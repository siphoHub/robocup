# Build ball_predictor.so via pybind11 on macOS
CXX ?= clang++
SRC := $(wildcard *.cpp)
OBJ := $(SRC:.cpp=.o)

SDKROOT ?= $(shell xcrun --sdk macosx --show-sdk-path)
STDINC  := -I$(SDKROOT)/usr/include/c++/v1

CXXFLAGS := -O3 -std=c++11 -fPIC -Wall -stdlib=libc++ $(STDINC) $(PYBIND_INCLUDES)
LDFLAGS  := -shared -undefined dynamic_lookup -stdlib=libc++ -isysroot $(SDKROOT)

TARGET := ball_predictor.so

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(LDFLAGS) -o $@ $(OBJ)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

debug: $(filter-out lib_main.cpp,$(OBJ))
	$(CXX) -O0 -std=c++14 -Wall -g -stdlib=libc++ -isysroot $(SDKROOT) -o debug.bin debug_main.cc $^

.PHONY: clean
clean:
	rm -f $(OBJ) $(TARGET) debug.bin
