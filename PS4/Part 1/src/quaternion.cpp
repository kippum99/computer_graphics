#include "quaternion.hpp"

#include <iostream>

Quaternion Quaternion::Identity() {
    return Quaternion{1, 0, 0, 0};
}

Quaternion::Quaternion(float r, float i, float j, float k) {
    real = r;
    imag << i, j, k;
}

Quaternion::Quaternion(float r, Eigen::Vector3f i) {
    real = r;
    imag = i;
}

Quaternion Quaternion::operator*(const Quaternion &q2) const {
    float r = real * q2.real - imag.dot(q2.imag);
    Eigen::Vector3f i = real * q2.imag + q2.real * imag + imag.cross(q2.imag);

    return Quaternion{r, i};
}

Eigen::Matrix4d Quaternion::get_rotation_matrix() const {
    float i = imag(0);
    float j = imag(1);
    float k = imag(2);

    Eigen::Matrix4d m = Eigen::MatrixXd::Identity(4, 4);

    m(0, 0) = 1 - 2 * pow(j, 2) - 2 * pow(k, 2);
    m(0, 1) = 2 * (i * j - k * real);
    m(0, 2) = 2 * (i * k + j * real);
    m(1, 0) = 2 * (i * j + k * real);
    m(1, 1) = 1 - 2 * pow(i, 2) - 2 * pow(k, 2);
    m(1, 2) = 2 * (j * k - i * real);
    m(2, 0) = 2 * (i * k - j * real);
    m(2, 1) = 2 * (j * k + i * real);
    m(2, 2) = 1 - 2 * pow(i, 2) - 2 * pow(j, 2);

    return m;
}
