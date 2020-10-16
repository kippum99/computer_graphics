CXXFLAGS = -std=c++17
# SOURCES = src/object.o src/transformation.o src/data_loader.o
SOURCES = src/*.hpp src/*.cpp

EXENAME = wireframe_renderer

all: $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $(EXENAME) $(INCLUDE) $^

clean:
	rm -f *.o src/*.o $(EXENAME)

.PHONY: all clean


# ###############################################################################
# # CS/CNS 171 Fall 2015
# #
# # This is a template Makefile for the assignment. Edit it however you find
# # convenient.
# ###############################################################################
# CC = g++
# # FLAGS = -g -std=c++17
# FLAGS = -std=c++17
#
# # The following line is a relative directory reference that assumes the Eigen
# # folder--which your program will depend on--is located one directory above the
# # directory that contains this Makefile.
# # INCLUDE = -I../
# # SOURCES = *.h *.cpp
# SOURCES = src/object.o src/transformation.o src/data_loader.o
#
# EXENAME = wireframe
#
# all: $(SOURCES)
# 	$(CC) $(FLAGS) -o $(EXENAME) $(INCLUDE) $(SOURCES)
#
# clean:
# 	rm -f *.o $(EXENAME)
#
# .PHONY: all clean
