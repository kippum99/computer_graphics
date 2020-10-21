#ifndef OBJECT_H
#define OBJECT_H

#include "color.hpp"

#include <string>
#include <vector>


using namespace std;


struct Vertex {
    double x;
    double y;
    double z;
};

typedef Vertex Normal;

struct Face {
    // Vertex indices
    int v1;
    int v2;
    int v3;

    // Surface normal indices
    int n1;
    int n2;
    int n3;
};


class Object {
public:
    vector<Vertex> vertices;
    vector<Normal> normals;     // Surface normals
    vector<Face> faces;

    Color ambient;
    Color diffuse;
    Color specular;
    double shininess;

    Object() {};
    Object(string filename);
};

#endif
