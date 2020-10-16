CXXFLAGS = -std=c++17
SOURCES = object.o transformation.o data_loader.o

EXENAME = data_loader

all: $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $(EXENAME) $(INCLUDE) $^

clean:
	rm -f *.o $(EXENAME)

.PHONY: all clean
