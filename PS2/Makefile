CXXFLAGS = -std=c++17 -O3

INCLUDE = -Iinclude/
SOURCES = src/*.cpp

EXENAME = shaded_renderer

all: $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $(EXENAME) $(INCLUDE) $^

clean:
	rm -f *.o src/*.o $(EXENAME)

.PHONY: all clean
