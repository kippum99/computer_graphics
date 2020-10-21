#ifndef SCENE_H
#define SCENE_H


#include "image.hpp"
#include "light.hpp"
#include "object.hpp"

#include <eigen3/Eigen/Dense>


using namespace Eigen;
using namespace std;


class Scene {
public:
    Scene(const string &filename);
    void render_scene(Image &image, int mode);

private:
    Matrix4f camera_transformation;
    Vector3f camera_pos;
    Matrix4f inv_camera_transformation;
    Matrix4f perspective_projection;
    vector<Object> objects;
    vector<Light> lights;

    void _render_object(Object &obj, Image &image, int mode);
    Vertex _get_NDC(const Vertex &v) const;
};

#endif
