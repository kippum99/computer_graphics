CXXFLAGS = -std=c++17
SOURCES = ppm_circle.cpp

EXENAME = ppm_circle

all: $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $(EXENAME) $(INCLUDE) $^

clean:
	rm -f *.o $(EXENAME)

.PHONY: all clean
