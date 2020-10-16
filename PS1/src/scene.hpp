#ifndef SCENE_H
#define SCENE_H


#include "object.hpp"

#include <eigen3/Eigen/Dense>


using namespace Eigen;
using namespace std;


class Scene {
public:
    Scene(string filename);
    void render_scene(int xres, int yres);

private:
    Matrix4d camera_transformation;
    Matrix4d inv_camera_transformation;
    Matrix4d perspective_projection;
    vector<Object> objects;

    void _render_object(Object &obj, int xres, int yres, int **grid);
    Vertex _get_NCD(const Vertex &v) const;
};

#endif
