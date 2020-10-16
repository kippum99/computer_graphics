#include "rasterization.hpp"

// Returns true if the point is in perspective cube, false otherwise.
bool is_in_perspective(Vertex v) {
    return (v.x >= -1 && v.x <= 1 && v.y >= -1 && v.y <= 1 && v.z >= -1
            && v.z <= 1);
}

// Returns the (x, y) screen coordinate of an NDC coordinate.
pair<int, int> get_screen_coordinates(Vertex v, int xres, int yres) {
    int x = int((v.x + 1) / 2 * xres);
    int y = int((v.y + 1) / 2 * yres);

    return make_pair(x, y);
}

void rasterize_object(Object &obj, int xres, int yres, int **grid) {
    for (Face f : obj.faces) {
        Vertex v1 = obj.vertices[f.v1];
        Vertex v2 = obj.vertices[f.v2];
        Vertex v3 = obj.vertices[f.v3];

        if (is_in_perspective(v1)) {
            pair<int, int> coords = get_screen_coordinates(v1, xres, yres);
            int x = coords.first;
            int y = coords.second;
            grid[y][x] = 1;
        }

        // if (is_in_perspective(f.v1) || is_in_perspective(f.v2)) {
        //     // Draw a line between v1 and v2.
        // }
    }
}

void rasterize_line(int x1, int x2, int y1, int y2, int **grid) {

}
