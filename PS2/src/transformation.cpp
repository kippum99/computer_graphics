#include "transformation.hpp"

Matrix4f get_translation_matrix(float tx, float ty, float tz) {
    Matrix4f m = MatrixXf::Identity(4, 4);

    m(0, 3) = tx;
    m(1, 3) = ty;
    m(2, 3) = tz;

    return m;
}

Matrix4f get_rotation_matrix(float rx, float ry, float rz, float angle) {
    float norm = Vector3f{rx, ry, rz}.norm();
    float ux = rx / norm;
    float uy = ry / norm;
    float uz = rz / norm;

    Matrix4f m = MatrixXf::Zero(4, 4);

    m(0, 0) = pow(ux, 2) + (1 - pow(ux, 2)) * cos(angle);
    m(0, 1) = ux * uy * (1 - cos(angle)) - uz * sin(angle);
    m(0, 2) = ux * uz * (1 - cos(angle)) + uy * sin(angle);
    m(1, 0) = uy * ux * (1 - cos(angle)) + uz * sin(angle);
    m(1, 1) = pow(uy, 2) + (1 - pow(uy, 2)) * cos(angle);
    m(1, 2) = uy * uz * (1 - cos(angle)) - ux * sin(angle);
    m(2, 0) = uz * ux * (1 - cos(angle)) - uy * sin(angle);
    m(2, 1) = uz * uy * (1 - cos(angle)) + ux * sin(angle);
    m(2, 2) = pow(uz, 2) + (1 - pow(uz, 2)) * cos(angle);
    m(3, 3) = 1;

    return m;
}

Matrix4f get_scaling_matrix(float sx, float sy, float sz) {
    Matrix4f m = MatrixXf::Identity(4, 4);
    m(0, 0) = sx;
    m(1, 1) = sy;
    m(2, 2) = sz;

    return m;
}
