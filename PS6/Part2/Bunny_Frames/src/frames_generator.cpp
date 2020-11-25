#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

////////////////////////////////////////////////////////////////////////////////

struct Vertex
{
    float x;
    float y;
    float z;
};

////////////////////////////////////////////////////////////////////////////////
/* Global variables */

// A list of keyframe objects, where each keyframe object is a list of vertices
// for that frame
vector<vector<Vertex>> keyframe_objects;

// Basis matrix for Catmull-Rom splines
Eigen::Matrix4f B;

////////////////////////////////////////////////////////////////////////////////
/* Function implementations */

/* Returns the value of f(u) where f(u) is the Catmull-Rom spline between
 * two control points p_i and p_i+1.
 *
 * vec_p = [p_i-1, p_i, p_i+1, p_i+2]
 */
float compute_f(const float u, const Eigen::Vector4f &vec_p) {
    Eigen::Vector4f vec_u{1, u, pow(u, 2), pow(u, 3)};

    return vec_u.dot(B * vec_p);
}

/* Interpolate vertex between points v2 and v3, given four control points
 * v1, v2, v3, and v4, and u in the unit domain.
 */
Vertex interpolate_vertex(const float u, const Vertex &v1, const Vertex &v2,
                            const Vertex &v3, const Vertex &v4) {
    // Interpolate each component of the vertex
    Vertex v;
    v.x = compute_f(u, Eigen::Vector4f{v1.x, v2.x, v3.x, v4.x});
    v.y = compute_f(u, Eigen::Vector4f{v1.y, v2.y, v3.y, v4.y});
    v.z = compute_f(u, Eigen::Vector4f{v1.z, v2.z, v3.z, v4.z});

    return v;
}

/* Writes faces to the .obj file (outfile) by simply copying from any keyframe
 * .obj file.
 */
void write_faces(ofstream &outfile) {
    ifstream infile("keyframes/bunny00.obj");

    string line;

    do {
        getline(infile, line);

        if (line[0] == 'f') {
            outfile << line << endl;
        }

    } while (!line.empty());
}

/* Generates missing frames by interpolating vertices.
 */
void generate_frames() {
    // Initialize basis matrix B for Catmull-Rom splines
    B << 0, 2, 0, 0,
        -1, 0, 1, 0,
        2, -5, 4, -1,
        -1, 3, -3, 1;
    B /= 2;

    int num_vertices = keyframe_objects[0].size();

    // For every pair of keyframes, interpolate transformation values for
    // each frame between the two keyframes.
    for (int keyframe = 0; keyframe < 20; keyframe += 5) {
        // Index used for finding keyframe object in keyframe_objects
        int keyframe_idx = keyframe / 5;

        for (int frame = keyframe + 1; frame < keyframe + 5; frame++) {
            ofstream outfile("output/bunny" + to_string(frame) + + ".obj");

            // Compute normalized value 0 < u < 1
            float u = (float)(frame - keyframe) / 5;
            assert(u > 0 && u < 1);

            // Interpolate vertices
            for (int i = 0; i < num_vertices; i++) {
                // Get four control points, where v_2 is in the current keyframe
                Vertex v1 = keyframe_objects[max(0, keyframe_idx - 1)][i];
                Vertex v2 = keyframe_objects[keyframe_idx][i];
                Vertex v3 = keyframe_objects[keyframe_idx + 1][i];
                Vertex v4 = keyframe_objects[min(4, keyframe_idx + 2)][i];

                Vertex v = interpolate_vertex(u, v1, v2, v3, v4);

                outfile << "v " << v.x << " " << v.y << " " << v.z << endl;
            }

            write_faces(outfile);
        }
    }
}

/* Reads the given .obj file and stores the list of vertices in
 * keyframe_objects.
 */
void parse_obj(const string &filename) {
    ifstream infile(filename);
    string t;

    vector<Vertex> vertices;

    while (infile >> t) {
        if (t == "v") {
            float x, y, z;
            infile >> x >> y >> z;

            vertices.push_back(Vertex{x, y, z});
        }
        else if (t == "f") {
            break;
        }
    }

    keyframe_objects.push_back(vertices);
}

/* Reads the five keyframe obj files and store in keyframe_objects
 */
void parse_keyframes() {
    parse_obj("keyframes/bunny00.obj");
    parse_obj("keyframes/bunny05.obj");
    parse_obj("keyframes/bunny10.obj");
    parse_obj("keyframes/bunny15.obj");
    parse_obj("keyframes/bunny20.obj");
}

int main() {
    parse_keyframes();
    generate_frames();
}
