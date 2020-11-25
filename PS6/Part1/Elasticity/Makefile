CXXFLAGS = -std=c++17 -O3 -Wno-deprecated

INCLUDE = -I../
SOURCES = *.cpp
LIBS = -framework glut -framework opengl

EXENAME = simulate

all: $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $(EXENAME) $(INCLUDE) $(SOURCES) $(LIBS)

clean:
	rm -f *.o $(EXENAME)

.PHONY: all clean
