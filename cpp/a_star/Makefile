# Build a_star.so via pybind11 on macOS
CXX := g++
SRC := $(wildcard *.cpp)
OBJ := $(SRC:.cpp=.o)

CXXFLAGS := -O3 -std=c++11 -fPIC -Wall $(PYBIND_INCLUDES)
LDFLAGS  := -shared -undefined dynamic_lookup

TARGET := a_star.so

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(LDFLAGS) -o $@ $(OBJ)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

debug: $(filter-out lib_main.cpp,$(OBJ))
	$(CXX) -O0 -std=c++14 -Wall -g -o debug.bin debug_main.cc $^

.PHONY: clean
clean:
	rm -f $(OBJ) $(TARGET) debug.bin
