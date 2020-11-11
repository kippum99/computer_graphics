#include <Eigen/Dense>

class Quaternion {
private:
    float real;
    Eigen::Vector3f imag;

public:
    static Quaternion Identity();

    Quaternion() {};
    Quaternion(float r, float i, float j, float k);
    Quaternion(float r, Eigen::Vector3f i);

    Quaternion operator*(const Quaternion &q2) const;
    Eigen::Matrix4d get_rotation_matrix() const;
};
