#include "object.hpp"
#include "transformation.hpp"

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>

using namespace std;

int main(int argc, char *argv[]) {
    // Mapping from object label to original objects
    map<string, Object> object_map;

    // Mapping from object label to transformed object
    map<string, Object> t_object_map;

    // Mapping of object label to number of its copies
    unordered_map<string, int> num_copies;

    ifstream infile(argv[1]);
    string line;
    getline(infile, line);

    // Read objects and filenames
    while (!line.empty()) {
        int break_idx = line.find(" ");
        string obj_label = line.substr(0, break_idx);
        string filename = line.substr(break_idx + 1, line.length());
        object_map[obj_label] = load_object(filename);
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
            istringstream iss{line};
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
        if (num_copies.count(obj_label) == 0) {
            num_copies[obj_label] = 1;
        }
        else {
            num_copies[obj_label]++;
        }

        string copy_label = obj_label +
            "_copy" + to_string(num_copies[obj_label]);

        t_object_map[copy_label] = obj;
    }

    for (const auto & [copy_label, obj] : t_object_map) {
        cout << copy_label << endl;

        for (int i = 1; i < obj.vertices.size(); i++) {
            Vertex v = obj.vertices[i];
            cout << v.x << " " << v.y << " " << v.z << endl;
        }

        cout << endl;
    }
}
