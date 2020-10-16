CXXFLAGS = -std=c++17
# SOURCES = src/object.o src/transformation.o src/data_loader.o
SOURCES = src/*.hpp src/*.cpp

EXENAME = wireframe_renderer

all: $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $(EXENAME) $(INCLUDE) $^

clean:
	rm -f *.o src/*.o $(EXENAME)

.PHONY: all clean
