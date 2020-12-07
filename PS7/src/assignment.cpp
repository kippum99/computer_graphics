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

	// Newton's method iteration
	const double eps = 0.001;	// Stopping condition threshold

	while (true) {
		Vector3d ray_t = ray_body.At(t);

		double x = ray_t.x();
		double y = ray_t.y();
		double z = ray_t.z();

		double g = S(x, y, z, exp0, exp1);

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
			// We're moving away from superquadric 
			return closest;
		}

		t = t - (g / g_prime);
	}

    
    return closest;
}

pair<double, Intersection> Assembly::ClosestIntersection(const Ray &ray) {
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

/* Returns the color in a Vector3f {r, g, b} of a point v given in world 
 * coordinates using the Phong lighting model.
 */
Vector3f Scene::Lighting(const Vector3d &v, const Vector3d &n,
					const Material &material, const vector<Light> &lights,
					const Vector3d &e) const {
	Vector3f diffuse_sum(0, 0, 0);
	Vector3f specular_sum(0, 0, 0);
	Vector3d e_direction = (e - v).normalized();

	for (const Light &l : lights) {
		Vector3d l_position(l.position.x(), l.position.y(), l.position.z());
		l_position /= l.position.w();

		// Check if light is obstructed by another object on the path to the obj 
		Ray ray;	// Ray from light to obj 
		ray.origin = l_position;
		ray.direction = v - l_position;

		// cout << "our vertex v " << v << endl;
		// cout << "computed v " << ray.At(1) << endl;

		// assert(v == ray.At(1));

		double obstruct_t = ClosestIntersection(ray).first;

		// cout << "t " << obstruct_t << endl;

		if (obstruct_t < 1.d - 0.001) {	// Our object is intersected at t = 1
			continue;	// Light is obstructed (apply shadow)
		}

		//Attenuation
		float d = (v - l_position).norm();
		Vector3f l_c = l.color.ToVector() / (1 + l.attenuation * d * d);

		Vector3d l_direction = (l_position - v).normalized();

		Vector3f l_diffuse = l_c * max(0.d, n.dot(l_direction));
		diffuse_sum += l_diffuse;

		Vector3f l_specular = l_c 
			* pow(max(0.d, n.dot((e_direction + l_direction).normalized())),
			material.shininess);
		specular_sum += l_specular;
	}

	Vector3f color = material.ambient.ToVector()
						+ diffuse_sum.cwiseProduct(material.diffuse.ToVector())
						+ specular_sum.cwiseProduct(
							material.specular.ToVector());

	color(0) = min(1.f, color(0));
	color(1) = min(1.f, color(1));
	color(2) = min(1.f, color(2));

	return color;
}

void Scene::Raytrace() {
	// TODO: Check coordinates

    Image img = Image(XRES, YRES);

    // TODO: CHECK COORDINATES

    const Camera cam = GetCamera();
   	const Frustum frustum = cam.frustum;

    // Get width and height of the front plane of camera frustum
    const double fov_rad = frustum.fov * M_PI / 180;
    const double h = 2 * frustum.near * tan(fov_rad / 2);
    const double w = frustum.aspect_ratio * h;

    // Transform matrix to convert from camera space to world space 
    const Matrix4d cam_trans_mat = (cam.translate.GetMatrix() 
	    							* cam.rotate.GetMatrix()
	    							).inverse();

    for (int i = 0; i < XRES; i++) {
        for (int j = 0; j < YRES; j++) {
            // Get the ray from the camera through this pixel (i, j)
            Ray ray;
            ray.origin = Vector3d(0, 0, 0);
            ray.direction = cam.frustum.near * Vector3d(0, 0, -1)
            				+ (i - XRES / 2) * w / XRES * Vector3d(1, 0, 0)
            				+ (j - YRES / 2) * h / YRES * Vector3d(0, 1, 0);

            // Convert ray to world coords 
            ray.Transform(cam_trans_mat);

            // Find the closest intersection of the ray with an object
            // TODO: what to do with the t value?
            // pair<double, Intersection> closest = ClosestIntersection(ray);
            // cout << closest.first << endl;

            Intersection inter = ClosestIntersection(ray).second;

            if (inter.obj) {
            	// Check if light is 



	            Vector3f color = Lighting(inter.location.origin, 
	            							inter.location.direction,
	            							inter.obj->GetMaterial(), 
	            							GetLights(), ray.origin);
	            // cout << "Setting color to " << color.x() << " " << color.y() << " " << color.z()<<endl;
	            img.SetPixel(i, j, color);
	        }
	        else {
	        	// No object at the pixel
	        	img.SetPixel(i, j, Vector3f::Zero());
	        }
        }
    }

    // Outputs the image.
    if (!img.SaveImage("rt.png")) {
        cerr << "Error: couldn't save PNG image" << std::endl;
    } else {
        cout << "Done!\n";
    }
}
