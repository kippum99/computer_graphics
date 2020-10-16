#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <eigen3/Eigen/Dense>

using namespace Eigen;

Matrix4d get_translation_matrix(double tx, double ty, double tz);
Matrix4d get_rotation_matrix(double rx, double ry, double rz, double angle);
Matrix4d get_scaling_matrix(double sx, double sy, double sz);

#endif
