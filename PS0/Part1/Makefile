CXXFLAGS = -Wall -Werror -Wpedantic -Werror=sign-compare -std=c++14
SOURCES = *.cpp

EXENAME = obj_parser

all: $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $(EXENAME) $(SOURCES)

clean:
	rm -f *.o $(EXENAME)

.PHONY: all clean
