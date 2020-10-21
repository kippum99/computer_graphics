#ifndef RASTERIZATION_H
#define RASTERIZATION_H

#include "light.hpp"
#include "object.hpp"

#include <vector>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

void rasterize_object(Object &obj, vector<Light> &lights, Vector3f &camera_pos,
                        int xres, int yres, int **grid);


#endif
