# Build localization.so via pybind11 on macOS + link GSL
CXX := g++
SRC := $(wildcard *.cpp)
OBJ := $(SRC:.cpp=.o)

GSL_CFLAGS := $(shell pkg-config --cflags gsl 2>/dev/null)
GSL_LIBS   := $(shell pkg-config --libs gsl 2>/dev/null)

ifeq ($(strip $(GSL_CFLAGS)),)
GSL_CFLAGS := $(shell gsl-config --cflags 2>/dev/null)
endif
ifeq ($(strip $(GSL_LIBS)),)
GSL_LIBS := $(shell gsl-config --libs 2>/dev/null)
endif

ifeq ($(strip $(GSL_CFLAGS)),)
$(error GSL headers not found. Install GSL (e.g. `brew install gsl` or `conda install -c conda-forge gsl`))
endif

CXXFLAGS := -O3 -std=c++11 -fPIC -Wall $(PYBIND_INCLUDES) $(GSL_CFLAGS)
LDFLAGS  := -shared -undefined dynamic_lookup
LDLIBS   := $(if $(strip $(GSL_LIBS)),$(GSL_LIBS),-lgsl -lgslcblas)

TARGET := localization.so

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(LDFLAGS) -o $@ $(OBJ) $(LDLIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

debug: $(filter-out lib_main.cpp,$(OBJ))
	$(CXX) -O0 -std=c++14 -Wall -g -o debug.bin debug_main.cc $^ $(LDLIBS)

.PHONY: clean
clean:
	rm -f $(OBJ) $(TARGET) debug.bin
