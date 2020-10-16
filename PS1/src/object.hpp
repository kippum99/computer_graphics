#ifndef OBJECT_H
#define OBJECT_H

#include <string>
#include <vector>


using namespace std;


struct Vertex {
    double x;
    double y;
    double z;
};

struct Face {
    int v1;
    int v2;
    int v3;
};


class Object {
public:
    vector<Vertex> vertices;
    vector<Face> faces;

    Object() {};
    Object(string filename);
};

#endif
