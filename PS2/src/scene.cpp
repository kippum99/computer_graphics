#include "scene.hpp"

#include "color.hpp"
#include "light.hpp"
#include "rasterization.hpp"
#include "transformation.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>


Scene::Scene(const string &filename) {
    ifstream infile(filename);
    string line;

    // Skip "camera:" token
    getline(infile, line);
    getline(infile, line);

    // Read camera and perspective paramters
    camera_transformation = MatrixXf::Identity(4, 4);
    float n, f, l, r, t, b;

    while (!line.empty()) {
        Matrix4f trans;
        istringstream iss(line);
        string label;
        iss >> label;

        if (label == "position") {
            float tx, ty, tz;
            iss >> tx >> ty >> tz;
            camera_pos << tx, ty, tz;
            trans = get_translation_matrix(tx, ty, tz);
            camera_transformation *= trans;
        }
        else if (label == "orientation") {
            float rx, ry, rz, angle;
            iss >> rx >> ry >> rz >> angle;
            trans = get_rotation_matrix(rx, ry, rz, angle);
            camera_transformation *= trans;
        }
        else if (label == "near") {
            iss >> n;
        }
        else if (label == "far") {
            iss >> f;
        }
        else if (label == "left") {
            iss >> l;
        }
        else if (label == "right") {
            iss >> r;
        }
        else if (label == "top") {
            iss >> t;
        }
        else if (label == "bottom") {
            iss >> b;
        }

        getline(infile, line);
    }

    inv_camera_transformation = camera_transformation.inverse();

    perspective_projection = MatrixXf::Zero(4, 4);
    perspective_projection(0, 0) = 2 * n / (r - l);
    perspective_projection(0, 2) = (r + l) / (r - l);
    perspective_projection(1, 1) = 2 * n / (t - b);
    perspective_projection(1, 2) = (t + b) / (t - b);
    perspective_projection(2, 2) = -(f + n) / (f - n);
    perspective_projection(2, 3) = -2 * f * n / (f - n);
    perspective_projection(3, 2) = -1;

    getline(infile, line);

    // Read point light sources
    while (!line.empty()) {
        istringstream iss(line);
        string _;
        float x, y, z, r, g, b, k;

        iss >> _ >> x >> y >> z >> _ >> r >> g >> b >> _ >> k;

        Light light{Vector3f{x, y, z}, Color{r, g, b}, k};
        lights.push_back(light);

        getline(infile, line);
    }

    // Skip empty line and "objects:" token
    getline(infile, line);
    getline(infile, line);

    // Read objects and filenames

    // Mapping from object label to original objects
    map<string, Object> object_map;

    while (!line.empty()) {
        int break_idx = line.find(" ");
        string obj_label = line.substr(0, break_idx);
        string filename = line.substr(break_idx + 1, line.length());
        object_map[obj_label] = Object("data/" + filename);
        getline(infile, line);
    }

    // Read and store material fields and apply transformations
    while (getline(infile, line)) {
        string obj_label = line;
        Object obj = object_map[obj_label];

        getline(infile, line);

        // Read material fields for the object
        Material mat;

        while (!line.empty()) {
            istringstream iss(line);
            string t;
            iss >> t;

            if (t == "ambient") {
                float r, g, b;
                iss >> r >> g >> b;
                mat.ambient = Color{r, g, b};
            }
            else if (t == "diffuse") {
                float r, g, b;
                iss >> r >> g >> b;
                mat.diffuse = Color{r, g, b};
            }
            else if (t == "specular") {
                float r, g, b;
                iss >> r >> g >> b;
                mat.specular = Color{r, g, b};
            }
            else if (t == "shininess") {
                iss >> mat.shininess;
                getline(infile, line);
                break;
            }

            getline(infile, line);
        }

        obj.material = mat;

        // Read all transformations for the object and store in m
        Matrix4f m = MatrixXf::Identity(4, 4);

        while (!line.empty()) {
            Matrix4f trans;
            istringstream iss(line);
            string t;
            iss >> t;

            if (t == "t") {
                float tx, ty, tz;
                iss >> tx >> ty >> tz;
                trans = get_translation_matrix(tx, ty, tz);
            }
            else if (t == "r") {
                float rx, ry, rz, angle;
                iss >> rx >> ry >> rz >> angle;
                trans = get_rotation_matrix(rx, ry, rz, angle);
            }
            else if (t == "s") {
                float sx, sy, sz;
                iss >> sx >> sy >> sz;
                trans = get_scaling_matrix(sx, sy, sz);
            }

            m = trans * m;

            if (infile.eof()) {
                break;
            }

            getline(infile, line);
        }

        // Apply transformation to every vertex in object
        for (size_t i = 0; i < obj.vertices.size(); i++) {
            Vertex v = obj.vertices[i];
            Vector4f vec{v(0), v(1), v(2), 1};
            vec = m * vec;
            obj.vertices[i] = Vertex{vec(0), vec(1), vec(2)};
        }

        // Apply transformation to every surface normal in object
        Matrix3f x = MatrixXf::Zero(3, 3);

        for (size_t i = 0; i < 3; i++) {
            for (size_t j = 0; j < 3; j++) {
                x(i, j) = m(i, j);
            }
        }

        x = x.inverse().transpose();

        for (size_t i = 0; i < obj.normals.size(); i++) {
            Normal n = obj.normals[i];
            n = x * n;
            n.normalize();
            obj.normals[i] = Vertex{n(0), n(1), n(2)};
        }

        // Store transformed copy
        objects.push_back(obj);
    }
}

void Scene::render_scene(Image &image, int mode) {
    Matrix4f ndc_transform = perspective_projection * inv_camera_transformation;

    for (Object &obj : objects) {
        rasterize_object(obj, lights, camera_pos, ndc_transform, image, mode);
    }
}
