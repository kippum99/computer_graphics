#ifndef SCENE_H
#define SCENE_H


#include "light.hpp"
#include "object.hpp"

#include <eigen3/Eigen/Dense>


using namespace Eigen;
using namespace std;


class Scene {
public:
    Scene(const string &filename);
    void render_scene(int xres, int yres);

private:
    Matrix4f camera_transformation;
    Vector3f camera_pos;
    Matrix4f inv_camera_transformation;
    Matrix4f perspective_projection;
    vector<Object> objects;
    vector<Light> lights;

    void _render_object(Object &obj, int xres, int yres, int **grid);
    Vertex _get_NCD(const Vertex &v) const;
};

#endif
