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

struct Object {
    vector<Vertex> vertices;
    vector<Face> faces;
};


Object load_object(string filename);
