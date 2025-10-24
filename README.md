# How to run
1. Activate environment in terminal
```bash
source ~/robocup-venv/bin/activate
```

2. Start server (in new terminal tab)

```bash
docker run --rm -it -p 3100:3100 -p 3200:3200 rcss3d/rcssserver3d:latest rcssserver3d
```

3. Start visualiser (app)

```bash
Open RoboViz.app → connect to localhost:3200
```

4. Run team in (firobocup-venv)  terminal tab

```bash
cd ~/WitsFcCodebase2025/WitsFcCodebase
for i in 1 2 3 4 5; do (python3 Run_Player.py &); done
wait
```

5. In RoboViz: press O → Play On


# RoboCup Set-up on macOS (Monterey) Guide

* **Server (rcssserver3d)** → in **Docker** on your Mac
* **Agents (Python)** → natively on macOS inside a **venv**
* **Viewer (RoboViz)** → **RoboViz.app** (Java 17) on macOS

---

## 0) One-time Mac prep (compilers & Homebrew)

Install Apple’s toolchain:

```bash
xcode-select --install
```

Install Homebrew (if you don’t have it):

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Update brew:

```bash
brew update
```

---

## 1) Docker Desktop (to run the Linux server)

Docker lets us run the Linux-only server on macOS.

**Download (Intel Mac) — exact build used:**

```
https://desktop.docker.com/mac/main/amd64/131620/Docker.dmg
```

1. Mount the `.dmg`, drag **Docker** into **/Applications**
2. Launch **Docker Desktop** (allow permissions)
3. Ensure the 🐳 whale icon is visible in the menu bar

Verify:

```bash
docker --version
# Example:
# Docker version 24.0.7, build afdd53b
```

---

## 2) Java 17 (for RoboViz)

RoboViz requires Java 17+.

**Option A (recommended — Temurin JDK 17)**
Download from Adoptium and install the `.pkg` (choose macOS, Intel/arm64):

```
https://adoptium.net/temurin/releases/
```

**Option B (Homebrew):**

```bash
brew install openjdk@17
sudo ln -sfn /usr/local/opt/openjdk@17/libexec/openjdk.jdk \
  /Library/Java/JavaVirtualMachines/openjdk-17.jdk
```

Verify:

```bash
java -version
# Example:
# openjdk version "17.0.8-beta" 2023-07-18
# OpenJDK Runtime Environment Temurin-17.0.8+6-202307161209
# OpenJDK 64-Bit Server VM Temurin-17.0.8+6-202307161209
```

---

## 3) Python + virtual environment (for your agents)

We’ll keep conda out of the way and use a plain `venv`.

**Check Python & install if needed**

```bash
python3 -V
# Example (brew’s Python): Python 3.13.7
```

If you don’t have a recent Python:

```bash
brew install python
```

**(If you use conda) prevent auto-activation of base:**

```bash
conda config --set auto_activate_base false
exec zsh   # restart shell so change takes effect
```

**Create and activate venv:**

```bash
/usr/local/bin/python3 -m venv ~/robocup-venv
source ~/robocup-venv/bin/activate
which python3
python3 -V
# Expect: /Users/<you>/robocup-venv/bin/python3
#         Python 3.13.7 (or your version)
```

**Install Python deps:**

```bash
pip install --upgrade pip
pip install numpy pybind11 psutil
pip list
# Expect to see: numpy, pybind11, psutil
```

---

## 4) Get the team code

Download & unzip:

```bash
cd ~
curl -LO https://lamp.ms.wits.ac.za/robocup/WitsFcCodebase.zip
unzip WitsFcCodebase.zip
```

You should now have:

```
~/WitsFcCodebase2025/WitsFcCodebase
```

Sanity check:

```bash
ls ~/WitsFcCodebase2025/WitsFcCodebase
# Expect: start.sh, start_debug.sh, Run_Player.py, agent/, world/, cpp/, ...
```

---

## 5) Start the 3D server in Docker (publish ports)

Open a **new terminal** (leave the agents terminal for later), and run:

```bash
docker pull rcss3d/rcssserver3d:latest
docker run --rm -it -p 3100:3100 -p 3200:3200 rcss3d/rcssserver3d:latest rcssserver3d
```

* `3100` → agents connect here
* `3200` → RoboViz connects here

Leave this window open (you’ll see server logs).

> If ports are busy, stop the old container (Ctrl+C) or use different host ports:
> `-p 3101:3100 -p 3201:3200` (and remember them).

---

## 6) Install RoboViz & run it (macOS)

Download a Mac build (2.0.0):

```
https://github.com/magmaOffenburg/RoboViz/releases/tag/2.0.0
```

You’ll get `RoboViz-2.0.0-macos.zip` → unzip → **RoboViz.app**.

Move & run:

```bash
mv ~/Downloads/RoboViz.app /Applications/
open /Applications/RoboViz.app
```

If macOS blocks it: **System Settings → Privacy & Security → Allow Anyway**, then run again.

Connect RoboViz to the server:

```
Server → Connect
Host: localhost
Port: 3200
```

Status should show **Connected** and you’ll see the field.

---

## 7) Build C++ helper modules (pybind11) & run agents

These modules live in `WitsFcCodebase/cpp/` and must compile for your Python version.

**Install math libs (GSL) and CMake:**

```bash
brew install gsl cmake
```

(If needed, export paths on Intel macOS:)

```bash
export CPPFLAGS="-I/usr/local/opt/gsl/include $CPPFLAGS"
export LDFLAGS="-L/usr/local/opt/gsl/lib $LDFLAGS"
```

### 7.2 Fix the Makefiles (drop-in replacements)

**`cpp/a_star/Makefile`**

```make
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
```

**`cpp/ball_predictor/Makefile`**

```make
# Build ball_predictor.so via pybind11 on macOS
CXX := g++
SRC := $(wildcard *.cpp)
OBJ := $(SRC:.cpp=.o)

CXXFLAGS := -O3 -std=c++11 -fPIC -Wall $(PYBIND_INCLUDES)
LDFLAGS  := -shared -undefined dynamic_lookup

TARGET := ball_predictor.so

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
```

**`cpp/localization/Makefile`** *(robust GSL detection)*

```make
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
```

### 7.3 Fix Python math calls (NumPy → `math`)

Some code used `np.math.*` which does not exist. Replace with `math.*`.

Files edited:

* `math_ops/Matrix_3x3.py`
* `math_ops/Matrix_4x4.py`

Add to imports:

```python
from math import asin, atan2, pi, sqrt, cos, sin
```

Replace all:

* `np.math.cos(...)` → `cos(...)`
* `np.math.sin(...)` → `sin(...)`

*(You can do this by hand, or via a quick find/replace. You already did this in your working setup.)*

### 7.4 Run a single player (first run will compile C++)

Back in the **agents** terminal:

```bash
source ~/robocup-venv/bin/activate
cd ~/WitsFcCodebase2025/WitsFcCodebase
python3 Run_Player.py
```

What you should see (first time):

* C++ modules compile:

  * `a_star... success!`
  * `ball_predictor... success!`
  * `localization... success!`
* Then:

  * `Connected agent 1 ('127.0.0.1', <port>)`
* RoboViz shows one player on the field.

> If you see “undefined symbols: _Py… for x86_64”, `-undefined dynamic_lookup` is missing from `LDFLAGS` (fixed in the Makefiles above).

---

## 8) Run all five players

```bash
source ~/robocup-venv/bin/activate
cd ~/WitsFcCodebase2025/WitsFcCodebase
for i in 1 2 3 4 5; do
  (python3 Run_Player.py &)
done
wait
```

You’ll see 5 connections, e.g.:

```
Connected agent 1 ('127.0.0.1', 60864)
Connected agent 1 ('127.0.0.1', 60865)
...
```

---

## 9) Start the match (RoboViz)

In RoboViz:

* Press **O** (letter O) → **Play On**

You should see players move (or at least stand at initial spots, depending on logic).

---

## 10) Troubleshooting quick notes

* **“zsh: command not found: #”**
  A pasted line started with `#` mid-block; ignore it (comments only work at the start of commands you type).

* **Conda mixing with venv**
  If your prompt shows `(base)` next to `(robocup-venv)`, disable conda base:

  ```bash
  conda config --set auto_activate_base false
  exec zsh
  source ~/robocup-venv/bin/activate
  ```

* **GSL not found**
  Install via Homebrew and ensure headers/libs are visible:

  ```bash
  brew install gsl
  export CPPFLAGS="-I/usr/local/opt/gsl/include $CPPFLAGS"
  export LDFLAGS="-L/usr/local/opt/gsl/lib $LDFLAGS"
  ```

* **RoboViz won’t open (security)**
  System Settings → Privacy & Security → **Allow Anyway** → re-open:

  ```bash
  open /Applications/RoboViz.app
  ```

* **RoboViz doesn’t connect**
  Ensure the Docker server is running with:

  ```bash
  -p 3100:3100 -p 3200:3200
  ```

  and connect to `localhost:3200`.

---

## 11) Optional quality-of-life

Simple Mac launcher to start 5 players:

```bash
cat > run_five.sh <<'SH'
#!/usr/bin/env bash
source ~/robocup-venv/bin/activate
cd ~/WitsFcCodebase2025/WitsFcCodebase
for i in 1 2 3 4 5; do
  (python3 Run_Player.py &) 
done
wait
SH
chmod +x run_five.sh
```

Run with:

```bash
./run_five.sh
```

> If you change Python versions later, delete the cached `.so` files in `cpp/*/` and re-run to rebuild.

---

## 12) What to hand in / show

Short note for your report:

> “Server run via Docker (rcssserver3d), agents in macOS Python venv, RoboViz.app on Java 17. C++ pybind modules compiled locally with Makefile fixes for macOS (`-undefined dynamic_lookup`) and GSL via Homebrew. Replaced `np.math.*` with `math.*` in Matrix_3x3/4x4.”

---

### ✅ Final “it’s working” checklist

* Docker server window shows logs and is listening on **3100/3200**
* RoboViz.app is **Connected** to `localhost:3200` and shows the field
* Running `python3 Run_Player.py` connects an agent to **localhost:3100**
* Running the loop starts **5 agents**
* Press **O → Play On** in RoboViz to start play

---

## Citing the Project

```bibtex
@article{abreu2023designing,
  title={Designing a Skilled Soccer Team for RoboCup: Exploring Skill-Set-Primitives through Reinforcement Learning},
  author={Abreu, Miguel and Reis, Luis Paulo and Lau, Nuno},
  journal={arXiv preprint arXiv:2312.14360},
  year={2023}
}
```

---
# FC Portugal Codebase <br> for RoboCup 3D Soccer Simulation League

![](https://s5.gifyu.com/images/Siov6.gif)

## About

The FC Portugal Codebase was mainly written in Python, with some C++ modules. It was created to simplify and speed up the development of a team for participating in the RoboCup 3D Soccer Simulation League. We hope this release helps existing teams transition to Python more easily, and provides new teams with a robust and modern foundation upon which they can build new features.

## Documentation

The documentation is available [here](https://docs.google.com/document/d/1aJhwK2iJtU-ri_2JOB8iYvxzbPskJ8kbk_4rb3IK3yc/edit)

## Features

* The team is ready to play!

  * Sample Agent – the active agent attempts to score with a kick, while the others maintain a basic formation
    Launch team with: **start.sh**
  * Sample Agent supports [Fat Proxy](https://github.com/magmaOffenburg/magmaFatProxy)
    Launch team with: **start_fat_proxy.sh**
  * Sample Agent Penalty – a striker performs a basic kick and a goalkeeper dives to defend
    Launch team with: **start_penalty.sh**
* Skills

  * Get Ups (latest version)
  * Walk (latest version)
  * Dribble v1 (version used in RoboCup 2022)
  * Step (skill-set-primitive used by Walk and Dribble)
  * Basic kick
  * Basic goalkeeper dive
* Features

  * Accurate localization based on probabilistic 6D pose estimation [algorithm](https://doi.org/10.1007/s10846-021-01385-3) and IMU
  * Automatic head orientation
  * Automatic communication with teammates to share location of all visible players and ball
  * Basics: common math ops, server communication, RoboViz drawings (with arrows and preset colors)
  * Behavior manager that internally resets skills when necessary
  * Bundle script to generate a binary and the corresponding start/kill scripts
  * C++ modules are automatically built into shared libraries when changes are detected
  * Central arguments specification for all scripts
  * Custom A* pathfinding implementation in C++, optimized for the soccer environment
  * Easy integration of neural-network-based behaviors
  * Integration with Open AI Gym to train models with reinforcement learning

    * User interface to train, retrain, test & export trained models
    * Common features from Stable Baselines were automated, added evaluation graphs in the terminal
    * Interactive FPS control during model testing, along with logging of statistics
  * Interactive demonstrations, tests and utilities showcasing key features of the team/agents
  * Inverse Kinematics
  * Multiple agents can be launched on a single thread, or one agent per thread
  * Predictor for rolling ball position and velocity
  * Relative/absolute position & orientation of every body part & joint through forward kinematics and vision
  * Sample train environments
  * User-friendly interface to check active arguments and launch utilities & gyms

