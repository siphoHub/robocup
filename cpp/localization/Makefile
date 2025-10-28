# Build localization.so via pybind11 on macOS + link GSL
CXX ?= clang++
SRC := $(wildcard *.cpp)
OBJ := $(SRC:.cpp=.o)

SDKROOT ?= $(shell xcrun --sdk macosx --show-sdk-path)
STDINC  := -I$(SDKROOT)/usr/include/c++/v1

# Prefer pkg-config; fallback to gsl-config
GSL_CFLAGS := $(shell pkg-config --cflags gsl 2>/dev/null)
GSL_LIBS   := $(shell pkg-config --libs   gsl 2>/dev/null)

ifeq ($(strip $(GSL_CFLAGS)),)
GSL_CFLAGS := $(shell gsl-config --cflags 2>/dev/null)
endif
ifeq ($(strip $(GSL_LIBS)),)
GSL_LIBS := $(shell gsl-config --libs 2>/dev/null)
endif

ifeq ($(strip $(GSL_CFLAGS)),)
$(error GSL headers not found. Install GSL (`brew install gsl`) and ensure pkg-config path is set)
endif

CXXFLAGS := -O3 -std=c++17 -fPIC -Wall -stdlib=libc++ $(STDINC) $(PYBIND_INCLUDES) $(GSL_CFLAGS)
LDFLAGS  := -shared -undefined dynamic_lookup -stdlib=libc++ -isysroot $(SDKROOT)
LDLIBS   := $(if $(strip $(GSL_LIBS)),$(GSL_LIBS),-lgsl -lgslcblas)

TARGET := localization.so

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(LDFLAGS) -o $@ $(OBJ) $(LDLIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

debug: $(filter-out lib_main.cpp,$(OBJ))
	$(CXX) -O0 -std=c++17 -Wall -g -stdlib=libc++ -isysroot $(SDKROOT) -o debug.bin debug_main.cc $^ $(LDLIBS)

.PHONY: clean
clean:
	rm -f $(OBJ) $(TARGET) debug.bin
