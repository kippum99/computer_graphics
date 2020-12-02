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
 * Helper functions 
 */

/* Evaluates the inside outside function. */
double S(double x, double y, double z, double e, double n) {
	return -1 + pow(pow(z, 2), 1 / n) 
			+ pow(pow(pow(x, 2), 1 / e) + pow(pow(y, 2), 1 / e), e / n);
}

/* Evaluates the gradient of the inside outside function. */
Vector3d grad_S(double x, double y, double z, double e, double n) {
    Vector3d grad;

    grad(0) = (x == 0.0) ? 0.0 :
        2.0 * x * pow(x * x, 1.0 / e - 1.0) *
        pow(pow(x * x, 1.0 / e) + pow(y * y, 1.0 / e),
            e / n - 1.0) / n;
    grad(1) = (y == 0.0) ? 0.0 :
        2.0 * y * pow(y * y, 1.0 / e - 1.0) *
        pow(pow(x * x, 1.0 / e) + pow(y * y, 1.0 / e),
            e / n - 1.0) / n;
    grad(2) = (z == 0.0) ? 0.0 :
        2.0 * z * pow(z * z, 1.0 / n - 1.0) / n;

    return grad;
}


/**
 * IOTest Code
 */

bool Superquadric::IOTest(const Vector3d &point) {
	// Convert point from parent space to body space
	Matrix4d transform_mat = Matrix4d::Identity();

	for (const unique_ptr<Transformation> &trans: transforms) {
		transform_mat = trans->GetMatrix() * transform_mat;
	}

	Matrix4d inv_transform_mat = transform_mat.inverse();

	Vector4d point_world(point.x(), point.y(), point.z(), 1);
	Vector4d point_body = inv_transform_mat * point_world;

    return S(point_body.x(), point_body.y(), point_body.z(), exp0, exp1) < 0;
}

bool Assembly::IOTest(const Vector3d &point) {
	// Convert point from parent space to body space
	Matrix4d transform_mat = Matrix4d::Identity();

	for (const unique_ptr<Transformation> &trans: transforms) {
		transform_mat = trans->GetMatrix() * transform_mat;
	}

	Matrix4d inv_transform_mat = transform_mat.inverse();

	Vector4d point_world(point.x(), point.y(), point.z(), 1);
	Vector4d point_body = inv_transform_mat * point_world;

	// Recurse on children 
	for (const shared_ptr<Object> &obj : children) {
		if (obj->IOTest(Vector3d{point_body.x(), point_body.y(), point_body.z()} 
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
	cout << "ClosestInterseciton sq" << endl;


	pair<double, Intersection> closest = make_pair(INFINITY, Intersection());

	// Convert ray from parent space to body space
	Matrix4d transform_mat = Matrix4d::Identity();

	for (const unique_ptr<Transformation> &trans: transforms) {
		transform_mat = trans->GetMatrix() * transform_mat;
	}

	Matrix4d inv_transform_mat = transform_mat.inverse();

	Ray ray_body = ray.Transformed(inv_transform_mat);

	// Compute initial t for Newton's method
	double a = ray_body.direction.dot(ray_body.direction);
	double b = 2 * ray_body.direction.dot(ray_body.origin);
	double c = ray_body.origin.dot(ray_body.origin) - 3;

	double discriminant = b * b - 4 * a * c;

	if (discriminant < 0) {
		// Ray misses the bounding box 
		return closest;
	}

	double t = (-b - sqrt(discriminant)) / (2 * a);

	if (t < 0) {
		return closest;
	}

	// Newton's method 
	const double eps = 0.001;	// Stopping condition threshold

	while (true) {
		Vector3d ray_t = ray_body.At(t);

		double x = ray_t.x();
		double y = ray_t.y();
		double z = ray_t.z();

		double g = S(x, y, z, exp0, exp1);
		cout << "g: " << g << endl;

		if (abs(g) < eps) {
			// origin is the location of the intersection in parent space
			// direction is normal of the superquadric at the origin in parent 
			// space
			Ray location;
			location.origin = ray_t;
			location.direction = GetNormal(ray_t);
			location.Transform(transform_mat);
			location.Normalize();

			return make_pair(t, Intersection(location, this));
		}

		double g_prime = ray_body.direction.dot(grad_S(x, y, z, exp0, exp1));

		if (g_prime >= 0) {
			// TODO: DIVISION BY ZERO ??
			cout << "DIVISION BY ZERO!" << endl;

			// We're moving away from superquadric 
			return closest;
		}

		t = t - (g / g_prime);
	}

    
    return closest;
}

pair<double, Intersection> Assembly::ClosestIntersection(const Ray &ray) {
	cout << "ClosestInterseciton assembly" << endl;
    /**
     * PART 1
     * TODO: Implement a ray-assembly intersection by recursively finding
     *       intersection with the assembly's children. Make sure to apply any
     *       transformations to the assembly before calling ClosestIntersection
     *       on the children.
     */

	// Convert point from parent space to body space
	Matrix4d transform_mat = Matrix4d::Identity();

	for (const unique_ptr<Transformation> &trans: transforms) {
		transform_mat = trans->GetMatrix() * transform_mat;
	}

	Matrix4d inv_transform_mat = transform_mat.inverse();
	Ray ray_body = ray.Transformed(inv_transform_mat);

	pair<double, Intersection> closest = make_pair(INFINITY, Intersection());

	// Recurse on children and find the closest intersection
	for (const shared_ptr<Object> &obj : children) {
		pair<double, Intersection> child_intersect 
			= obj->ClosestIntersection(ray_body);
		
		cout << "Finished" << endl;
		if (child_intersect.first < closest.first) {
			closest = child_intersect;
		}
	}

	// Normalize location Ray in Intersection
	Ray location = closest.second.location;
	location.Transform(transform_mat);
	location.Normalize();
	Intersection inter(location, closest.second.obj);
    
	return make_pair(closest.first, inter);
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
