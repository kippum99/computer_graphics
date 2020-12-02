#include "object.h"
#include "scene.h"

#include <iostream>

#include "image.h"

using namespace Eigen;
using namespace std;

const int MAX_ITERS = 10000;
const int XRES = 500;
const int YRES = 500;

/**
 * IOTest Code
 */

bool Superquadric::IOTest(const Vector3d &point) {
	// Convert point from world space to body space
	Matrix4d transform_mat = Matrix4d::Identity();

	for (const unique_ptr<Transformation> &trans: transforms) {
		transform_mat = trans->GetMatrix() * transform_mat;
	}

	transform_mat = transform_mat.inverse().eval();

	Vector4d point_world(point.x(), point.y(), point.z(), 1);
	Vector4d point_body = transform_mat * point_world;


	// Evaluate inside outside function
	double s = -1 + pow(pow(point_body.z(), 2), 1 / exp1)
				+ pow(pow(pow(point_body.x(), 2), 1 / exp0) 
						+ pow(pow(point_body.y(), 2), 1 / exp0)
						, exp0 / exp1);

    return s < 0;
}

bool Assembly::IOTest(const Vector3d &point) {
	// Convert point from world space to body space
	Matrix4d transform_mat = Matrix4d::Identity();

	for (const unique_ptr<Transformation> &trans: transforms) {
		transform_mat = trans->GetMatrix() * transform_mat;
	}

	transform_mat = transform_mat.inverse().eval();

	Vector4d point_world(point.x(), point.y(), point.z(), 1);
	Vector4d point_body = transform_mat * point_world;

	// Recurse on children 
	for (const shared_ptr<Object> &obj : children) {
		if (obj->IOTest(Vector3d{point_body.x(), point_body.y(), point_body.z()} / point_body.w()
			)) {
			return true;
		}
	}
    
	return false;
}

/**
 * Closest Intersection Code
 */

pair<double, Intersection> Superquadric::ClosestIntersection(const Ray &ray) {
    /**
     * PART 1
     * TODO: Implement a ray-superquadric intersection using Newton's method.
     *       Make sure to apply any transformations to the superquadric before
     *       performing Newton's method.
     */
    pair<double, Intersection> closest = make_pair(INFINITY, Intersection());
    return closest;
}

pair<double, Intersection> Assembly::ClosestIntersection(const Ray &ray) {
    /**
     * PART 1
     * TODO: Implement a ray-assembly intersection by recursively finding
     *       intersection with the assembly's children. Make sure to apply any
     *       transformations to the assembly before calling ClosestIntersection
     *       on the children.
     */
    pair<double, Intersection> closest = make_pair(INFINITY, Intersection());
    return closest;
}

/**
 * Raytracing Code
 */

void Scene::Raytrace() {
    Image img = Image(XRES, YRES);

    for (int i = 0; i < XRES; i++) {
        for (int j = 0; j < YRES; j++) {
            /**
             * PART 2
             * TODO: Implement raytracing using the code from the first part
             *       of the assignment. Set the correct color for each pixel
             *       here.
             */
            img.SetPixel(i, j, Vector3f::Ones());
        }
    }

    // Outputs the image.
    if (!img.SaveImage("rt.png")) {
        cerr << "Error: couldn't save PNG image" << std::endl;
    } else {
        cout << "Done!\n";
    }
}
