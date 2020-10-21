#include "rasterization.hpp"

#include "color.hpp"
#include "light.hpp"

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>


using namespace std;


struct VertexInfo {
    Vertex vertex;
    Normal normal;
    Color color;
};


// Returns true if the point is in perspective (NDC) cube, false otherwise.
bool is_in_perspective(Vertex &v) {
    return (v(0) >= -1 && v(0) <= 1 && v(1) >= -1 && v(1) <= 1);
}

/* Barycentric coordinates helper functions  */

// Helper function that computes f_ij(x, y) needed for computing barycentric
// coordinate coefficients alpha, beta, and gamma.
int barycentric_helper(int x, int y, int x1, int y1, int x2, int y2) {
    return ((y1 - y2) * x + (x2 - x1) * y + x1 * y2 - x2 * y1);
}

float compute_alpha(int x1, int y1, int x2, int y2, int x3, int y3, int x,
                        int y) {
    int numer = barycentric_helper(x, y, x2, y2, x3, y3);
    int denom = barycentric_helper(x1, y1, x2, y2, x3, y3);

    return float(numer) / denom;
}

float compute_beta(int x1, int y1, int x2, int y2, int x3, int y3, int x,
                        int y) {
    int numer = barycentric_helper(x, y, x1, y1, x3, y3);
    int denom = barycentric_helper(x2, y2, x1, y1, x3, y3);

    return float(numer) / denom;
}

float compute_gamma(int x1, int y1, int x2, int y2, int x3, int y3, int x,
                        int y) {
    int numer = barycentric_helper(x, y, x1, y1, x2, y2);
    int denom = barycentric_helper(x3, y3, x1, y1, x2, y2);

    return float(numer) / denom;
}

// Returns the (x, y) screen coordinate of an NDC coordinate. Note that
// y grows downwards.
Vector2i get_screen_coordinates(Vertex &v, int xres, int yres) {
    int x = int((v(0) + 1) / 2 * xres);
    int y = yres - int((v(1) + 1) / 2 * yres);

    return Vector2i{x, y};
}

// Returns the color of a point p using the Phong lighting model.
Color lighting(Vertex &v, Normal &n, Material &material, vector<Light> &lights,
                Vector3f &e) {
    Color diffuse_sum{0, 0, 0};
    Color specular_sum{0, 0, 0};
    Vector3f e_direction = (e - v).normalized();

    for (const Light &l : lights) {
        Vector3f l_direction = (l.position - v).normalized();

        Color l_diffuse = l.color * max(0.f, n.dot(l_direction));
        diffuse_sum += l_diffuse;

        Color l_specular = l.color
            * pow(max(0.f, n.dot((e_direction + l_direction).normalized())),
                    material.shininess);
        specular_sum += l_specular;
    }

    Color color = material.ambient + diffuse_sum.cwiseProduct(material.diffuse)
                    + specular_sum.cwiseProduct(material.specular);

    float r = min(1.f, color(0));
    float g = min(1.f, color(1));
    float b = min(1.f, color(2));

    return Color{r, g, b};
}

// Rasterizes colored triangle with depth buffering and backface culling.
void rasterize_colored_triangle(Vertex &v1, Vertex &v2, Vertex &v3,
                                int xres, int yres, int **grid, int **buffer) {
    Vector3f cross = (v3 - v2).cross(v1 - v2);

    if (cross(2) < 0) {
        return;
    }

    // Screen coordinates of vertices
    Vector2i coord1 = get_screen_coordinates(v1, xres, yres);
    Vector2i coord2 = get_screen_coordinates(v2, xres, yres);
    Vector2i coord3 = get_screen_coordinates(v3, xres, yres);

    int x1 = coord1(0);
    int y1 = coord1(1);
    int x2 = coord2(0);
    int y2 = coord2(1);
    int x3 = coord3(0);
    int y3 = coord3(1);

    int x_min = min({x1, x2, x3});
    int x_max = max({x1, x2, x3});
    int y_min = min({y1, y2, y3});
    int y_max = max({y1, y2, y3});

    for (int x = x_min; x <= x_max; x++) {
        for (int y = y_min; y <= y_max; y++) {
            float alpha = compute_alpha(x1, y1, x2, y2, x3, y3, x, y);
            float beta = compute_beta(x1, y1, x2, y2, x3, y3, x, y);
            float gamma = compute_gamma(x1, y1, x2, y2, x3, y3, x, y);

            if (alpha >= 0 && alpha <= 1 && beta >= 0 && beta <= 1
                && gamma >= 0 && gamma <= 1) {
                Vertex v = alpha * v1 + beta * v2 + gamma * v3;

                if (is_in_perspective(v) && v(2) < buffer[y][x]) {
                    buffer[y][x] = v(2);
                    break;

                    // float r = alpha *
                }
                // grid[y][x]
            }
        }
    }

}

void gouraud_shading(VertexInfo &a, VertexInfo &b, VertexInfo &c,
                        Material &mat, vector<Light> &lights, Vector3f &e,
                        int **grid) {
    a.color = lighting(a.vertex, a.normal, mat, lights, e);
}

void rasterize_object(Object &obj, vector<Light> &lights, Vector3f &camera_pos,
                        int xres, int yres, int **grid) {
    for (Face f : obj.faces) {
        // Vertices in NCD coordinates
        Vertex v1 = obj.vertices[f.v1];
        Vertex v2 = obj.vertices[f.v2];
        Vertex v3 = obj.vertices[f.v3];

        // Surface normals
        Normal n1 = obj.normals[f.n1];
        Normal n2 = obj.normals[f.n2];
        Normal n3 = obj.normals[f.n3];

        // Create VertexInfo structs
        VertexInfo a = {v1, n1, Color{0, 0, 0}};
        VertexInfo b = {v2, n2, Color{0, 0, 0}};
        VertexInfo c = {v2, n2, Color{0, 0, 0}};

        gouraud_shading(a, b, c, obj.material, lights, camera_pos, grid);

        // pair<int, int> coords1 = get_screen_coordinates(v1, xres, yres);
        // int x1 = coords1.first;
        // int y1 = coords1.second;
        //
        // pair<int, int> coords2 = get_screen_coordinates(v2, xres, yres);
        // int x2 = coords2.first;
        // int y2 = coords2.second;
        //
        // pair<int, int> coords3 = get_screen_coordinates(v3, xres, yres);
        // int x3 = coords3.first;
        // int y3 = coords3.second;
        //
        // if (is_in_perspective(v1) || is_in_perspective(v2)) {
        //     rasterize_line(x1, y1, x2, y2, xres, yres, grid);
        // }
        // if (is_in_perspective(v2) || is_in_perspective(v3)) {
        //     rasterize_line(x2, y2, x3, y3, xres, yres, grid);
        // }
        // if (is_in_perspective(v1) || is_in_perspective(v3)) {
        //     rasterize_line(x1, y1, x3, y3, xres, yres, grid);
        // }
    }
}
