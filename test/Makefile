CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O1 -g -Wall -Wno-unused-variable -Wno-unused-function -Wno-unused-but-set-variable -DFIRMWARE_TEST

# Include order: mocks first (so <Arduino.h> etc hit the host stubs),
# then firmware sources, then the test/ folder for test.h and sim.h.
INC = -Imocks -I../src -I.

FW_SRC   = ../src/planner.cpp ../src/stepper.cpp ../src/arc.cpp ../src/gcode.cpp ../src/protocol.cpp ../src/settings.cpp
MOCK_SRC = mocks/mock_impl.cpp
TEST_SRC = sim.cpp \
           test_main.cpp \
           test_kinematics.cpp \
           test_planner.cpp \
           test_arc.cpp \
           test_gcode.cpp \
           test_stepper.cpp \
           test_integration.cpp \
           test_protocol.cpp \
           test_settings.cpp \
           test_arc_timing.cpp

SRC = $(FW_SRC) $(MOCK_SRC) $(TEST_SRC)
OBJ = $(SRC:.cpp=.o)
BIN = run_tests

.PHONY: all test clean

all: $(BIN)

$(BIN): $(OBJ)
	$(CXX) $(CXXFLAGS) $(INC) -o $@ $(OBJ)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INC) -c -o $@ $<

test: $(BIN)
	./$(BIN)

clean:
	rm -f $(OBJ) $(BIN)
