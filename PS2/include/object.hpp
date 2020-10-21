#ifndef OBJECT_H
#define OBJECT_H

#include "color.hpp"

#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>


using namespace Eigen;
using namespace std;


typedef Vector3f Vertex;
typedef Vector3f Normal;

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

struct Material {
    Color ambient;
    Color diffuse;
    Color specular;
    float shininess;
};


class Object {
public:
    vector<Vertex> vertices;
    vector<Normal> normals;     // Surface normals
    vector<Face> faces;

    Material material;

    Object() {};
    Object(const string &filename);
};

#endif
