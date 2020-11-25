CXXFLAGS = -std=c++17 -O3

INCLUDE = -Iinclude/
SOURCES = *.cpp
LIBS = -framework glut -framework opengl

EXENAME = single_pendulum

all: $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $(EXENAME) $(INCLUDE) $(SOURCES) $(LIBS)

clean:
	rm -f *.o $(EXENAME)

.PHONY: all clean
