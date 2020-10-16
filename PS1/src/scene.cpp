#include "scene.hpp"

#include "rasterization.hpp"
#include "transformation.hpp"

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>


Scene::Scene(string filename) {
    ifstream infile(filename);
    string line;

    // Skip "camera:" token
    getline(infile, line);
    getline(infile, line);

    // Read camera and perspective paramters
    camera_transformation = MatrixXd::Identity(4, 4);
    double n, f, l, r, t, b;

    while (!line.empty()) {
        Matrix4d trans;
        istringstream iss(line);
        string label;
        iss >> label;

        if (label == "position") {
            double tx, ty, tz;
            iss >> tx >> ty >> tz;
            trans = get_translation_matrix(tx, ty, tz);
            camera_transformation = trans * camera_transformation;
        }
        else if (label == "orientation") {
            double rx, ry, rz, angle;
            iss >> rx >> ry >> rz >> angle;
            trans = get_rotation_matrix(rx, ry, rz, angle);
            camera_transformation = trans * camera_transformation;
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

    perspective_projection = MatrixXd::Zero(4, 4);
    perspective_projection(0, 0) = 2 * n / (r - l);
    perspective_projection(0, 2) = (r + l) / (r - l);
    perspective_projection(1, 1) = 2 * n / (t - b);
    perspective_projection(1, 2) = (t + b) / (t - b);
    perspective_projection(2, 2) = -(f + n) / (f - n);
    perspective_projection(2, 3) = -2 * f * n / (f - n);
    perspective_projection(3, 2) = -1;

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

    // Read and apply transformations
    while (getline(infile, line)) {
        string obj_label = line;
        Object obj = object_map[obj_label];
        Matrix4d m = MatrixXd::Identity(4, 4);

        getline(infile, line);

        // Read all transformations for the object and store in m
        while (!line.empty()) {
            Matrix4d trans;
            istringstream iss(line);
            string t;
            iss >> t;

            if (t == "t") {
                double tx, ty, tz;
                iss >> tx >> ty >> tz;
                trans = get_translation_matrix(tx, ty, tz);
            }
            else if (t == "r") {
                double rx, ry, rz, angle;
                iss >> rx >> ry >> rz >> angle;
                trans = get_rotation_matrix(rx, ry, rz, angle);
            }
            else if (t == "s") {
                double sx, sy, sz;
                iss >> sx >> sy >> sz;
                trans = get_scaling_matrix(sx, sy, sz);
            }

            m = trans * m;

            if (infile.eof()) {
                break;
            }

            getline(infile, line);
        }

        // Apply transformation to object
        for (int i = 0; i < obj.vertices.size(); i++) {
            Vertex v = obj.vertices[i];
            Vector4d vec;
            vec << v.x, v.y, v.z, 1;
            vec = m * vec;
            obj.vertices[i] = Vertex{vec(0), vec(1), vec(2)};
        }

        // Store transformed copy
        objects.push_back(obj);
    }
}

void Scene::render_scene(int xres, int yres) {
    // Initialize grid
    int **grid = new int*[yres];

    for (size_t i = 0; i < yres; i++) {
        grid[i] = new int[xres]{0};
    }

    for (Object &obj : objects) {
        _render_object(obj, xres, yres, grid);
    }

    // Output grid

    // Print header, resolution, and max pixel intensity
    cout << "P3" << endl;
    cout << xres << " " << yres << endl;
    cout << 255 << endl;

    for (int i = 0; i < yres; i++) {
        for (int j = 0; j < xres; j++) {
            if (grid[i][j] == 1) {
                cout << 255 << " " << 255 << " " << 255 << endl;
            }
            else {
                cout << 0 << " " << 0 << " " << 0 << endl;
            }
        }
    }

    // Free grid
}

// Converts object's vertices to Cartesian NCD coordinates and rasterizes the
// object as a wireframe model.
void Scene::_render_object(Object &obj, int xres, int yres, int **grid) {
    // Convert each vertex to a Cartesian NCD coordinate
    for (int i = 0; i < obj.vertices.size(); i++) {
        Vertex v = obj.vertices[i];
        obj.vertices[i] = _get_NCD(v);
    }

    rasterize_object(obj, xres, yres, grid);
}

// Returns the Cartesian NCD coordinates of the given vertex origianlly in world
// coordinates.
Vertex Scene::_get_NCD(const Vertex &v) const {
    Vector4d vec;
    vec << v.x, v.y, v.z, 1;
    vec = perspective_projection * inv_camera_transformation * vec;

    return Vertex{vec(0) / vec(3), vec(1) / vec(3), vec(2) / vec(3)};
}
