#include "rasterization.hpp"

#include "color.hpp"
#include "light.hpp"

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
    return (v(0) >= -1 && v(0) < 1 && v(1) >= -1 && v(1) < 1);
}

/* Barycentric coordinates helper functions  */

// Helper function that computes f_ij(x, y) needed for computing barycentric
// coordinate coefficients alpha, beta, and gamma.
int barycentric_helper(int x, int y, int x1, int y1, int x2, int y2) {
    return (y1 - y2) * x + (x2 - x1) * y + x1 * y2 - x2 * y1;
}

float compute_alpha(int x1, int y1, int x2, int y2, int x3, int y3, int x,
                        int y) {
    int numer = barycentric_helper(x, y, x2, y2, x3, y3);
    int denom = barycentric_helper(x1, y1, x2, y2, x3, y3);

    return (float)numer / denom;
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

// Returns the Cartesian NDC coordinates of the given vertex origianlly in world
// coordinates.
Vector3f get_NDC(const Vertex &v, Matrix4f &ndc_transform) {
    Vector4f vec{v(0), v(1), v(2), 1};
    vec = ndc_transform * vec;

    return Vector3f{vec(0) / vec(3), vec(1) / vec(3), vec(2) / vec(3)};
}

// Returns the (x, y) screen coordinate of an NDC coordinate. Note that
// y grows downwards.
Vector2i get_screen_coordinates(Vector3f &v, int xres, int yres) {
    int x = int((v(0) + 1) / 2 * xres);
    int y = yres - int((v(1) + 1) / 2 * yres);

    return Vector2i{x, y};
}

// Returns the color of a point v given in world coordinates using the Phong
// lighting model.
Color lighting(Vertex &v, Normal &n, Material &material, vector<Light> &lights,
                Vector3f &e) {
    Color diffuse_sum{0, 0, 0};
    Color specular_sum{0, 0, 0};
    Vector3f e_direction = (e - v).normalized();

    for (const Light &l : lights) {
        // Attenuation
        float d = (v - l.position).norm();
        Color l_c = l.color / (1 + l.k * pow(d, 2));

        Vector3f l_direction = (l.position - v).normalized();

        Color l_diffuse = l_c * max(0.f, n.dot(l_direction));
        diffuse_sum += l_diffuse;

        Color l_specular = l_c
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
void rasterize_colored_triangle(VertexInfo &a, VertexInfo &b, VertexInfo &c,
                                Matrix4f &ndc_transform, Image &image) {
    Vector3f ndc1 = get_NDC(a.vertex, ndc_transform);
    Vector3f ndc2 = get_NDC(b.vertex, ndc_transform);
    Vector3f ndc3 = get_NDC(c.vertex, ndc_transform);

    Vector3f cross = (ndc3 - ndc2).cross(ndc1 - ndc2);

    if (cross(2) < 0) {
        return;
    }

    // Screen coordinates of vertices
    Vector2i coord1 = get_screen_coordinates(ndc1, image.xres, image.yres);
    Vector2i coord2 = get_screen_coordinates(ndc2, image.xres, image.yres);
    Vector2i coord3 = get_screen_coordinates(ndc3, image.xres, image.yres);

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
                Vector3f ndc = alpha * ndc1 + beta * ndc2
                            + gamma * ndc3;

                if (is_in_perspective(ndc) && ndc(2) < image.buffer[y][x]) {
                    image.buffer[y][x] = (float) ndc(2);

                    float c_r = alpha * (a.color)(0) + beta * (b.color)(0)
                                + gamma * (c.color)(0);
                    float c_g = alpha * (a.color)(1) + beta * (b.color)(1)
                                + gamma * (c.color)(1);
                    float c_b = alpha * (a.color)(2) + beta * (b.color)(2)
                                + gamma * (c.color)(2);

                    image.grid[y][x] = Color{c_r, c_g, c_b};
                }
            }
        }
    }
}

void gouraud_shading(VertexInfo &a, VertexInfo &b, VertexInfo &c,
                        Material &mat, vector<Light> &lights, Vector3f &e,
                        Matrix4f &ndc_transform, Image &image) {
    a.color = lighting(a.vertex, a.normal, mat, lights, e);
    b.color = lighting(b.vertex, b.normal, mat, lights, e);
    c.color = lighting(c.vertex, c.normal, mat, lights, e);

    rasterize_colored_triangle(a, b, c, ndc_transform, image);
}

void phong_shading(VertexInfo &a, VertexInfo &b, VertexInfo &c,
                    Material &mat, vector<Light> &lights, Vector3f &e,
                    Matrix4f &ndc_transform, Image &image) {
    Vector3f ndc1 = get_NDC(a.vertex, ndc_transform);
    Vector3f ndc2 = get_NDC(b.vertex, ndc_transform);
    Vector3f ndc3 = get_NDC(c.vertex, ndc_transform);

    Vector3f cross = (ndc3 - ndc2).cross(ndc1 - ndc2);

    if (cross(2) < 0) {
        return;
    }

    // Screen coordinates of vertices
    Vector2i coord1 = get_screen_coordinates(ndc1, image.xres, image.yres);
    Vector2i coord2 = get_screen_coordinates(ndc2, image.xres, image.yres);
    Vector2i coord3 = get_screen_coordinates(ndc3, image.xres, image.yres);

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
                Vector3f ndc = alpha * ndc1 + beta * ndc2
                            + gamma * ndc3;

                if (is_in_perspective(ndc) && ndc(2) < image.buffer[y][x]) {
                    image.buffer[y][x] = (float) ndc(2);

                    Vector3f n = alpha * a.normal + beta * b.normal
                                    + gamma * c.normal;
                    Vector3f v = alpha * a.vertex + beta * b.vertex
                                    + gamma * c.vertex;

                    image.grid[y][x] = lighting(v, n, mat, lights, e);
                }
            }
        }
    }
}

void rasterize_object(Object &obj, vector<Light> &lights, Vector3f &camera_pos,
                        Matrix4f &ndc_transform, Image &image, int mode) {
    for (const Face &f : obj.faces) {
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
        VertexInfo c = {v3, n3, Color{0, 0, 0}};

        if (mode == 0) {
            gouraud_shading(a, b, c, obj.material, lights, camera_pos,
                            ndc_transform, image);
        }
        else if (mode == 1) {
            phong_shading(a, b, c, obj.material, lights, camera_pos,
                            ndc_transform, image);
        }
    }
}
