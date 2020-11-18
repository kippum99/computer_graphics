CXXFLAGS = -std=c++17 -O3

INCLUDE = -Iinclude/
SOURCES = src/*.cpp
LIBS = -framework glut -framework opengl

EXENAME = smooth

all: $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $(EXENAME) $(INCLUDE) $^ $(LIBS)

clean:
	rm -f *.o src/*.o $(EXENAME)

.PHONY: all clean
