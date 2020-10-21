#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <eigen3/Eigen/Dense>

using namespace Eigen;

Matrix4f get_translation_matrix(float tx, float ty, float tz);
Matrix4f get_rotation_matrix(float rx, float ry, float rz, float angle);
Matrix4f get_scaling_matrix(float sx, float sy, float sz);

#endif
