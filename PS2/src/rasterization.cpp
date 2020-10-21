#include "rasterization.hpp"

#include "color.hpp"
#include "light.hpp"

#include <cmath>
#include <eigen3/Eigen/Dense>


// // Returns true if the point is in perspective cube, false otherwise.
// bool is_in_perspective(Vertex v) {
//     return (v.x >= -1 && v.x <= 1 && v.y >= -1 && v.y <= 1
//             && v.z <= 1);
// }

// // Returns the (x, y) screen coordinate of an NDC coordinate. Note that
// // y grows downwards.
// pair<int, int> get_screen_coordinates(Vertex v, int xres, int yres) {
//     int x = int((v.x + 1) / 2 * xres);
//     int y = yres - int((v.y + 1) / 2 * yres);
//
//     return make_pair(x, y);
// }

// Returns the color of a point p using the lighting model.
Color lighting(Vertex v, Normal n, Material material, vector<Light> lights,
                Vector3d e) {

    return Color{};
}

// Rasterizes colored triangle with depth buffering and backface culling.
void rasterize_colored_triangle(Vertex v1, Vertex v2, Vertex v3, int **grid) {

}

void rasterize_object(Object &obj, int xres, int yres, int **grid) {
    for (Face f : obj.faces) {
        // Vertices in NCD coordinates
        Vertex v1 = obj.vertices[f.v1];
        Vertex v2 = obj.vertices[f.v2];
        Vertex v3 = obj.vertices[f.v3];

        rasterize_colored_triangle(v1, v2, v3, grid);

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
