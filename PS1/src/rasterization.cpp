#include "rasterization.hpp"

#include <cmath>

#include <iostream>


// Returns true if the point is in perspective cube, false otherwise.
bool is_in_perspective(Vertex v) {
    return (v.x >= -1 && v.x <= 1 && v.y >= -1 && v.y <= 1 && v.z >= -1
            && v.z <= 1);
}

// Returns the (x, y) screen coordinate of an NDC coordinate. Note that
// y grows downwards.
pair<int, int> get_screen_coordinates(Vertex v, int xres, int yres) {
    int x = int((v.x + 1) / 2 * xres);
    int y = yres - int((v.y + 1) / 2 * yres);

    return make_pair(x, y);
}

void rasterize_object(Object &obj, int xres, int yres, int **grid) {
    for (Face f : obj.faces) {
        Vertex v1 = obj.vertices[f.v1];
        Vertex v2 = obj.vertices[f.v2];
        Vertex v3 = obj.vertices[f.v3];

        pair<int, int> coords1 = get_screen_coordinates(v1, xres, yres);
        int x1 = coords1.first;
        int y1 = coords1.second;

        pair<int, int> coords2 = get_screen_coordinates(v2, xres, yres);
        int x2 = coords2.first;
        int y2 = coords2.second;

        pair<int, int> coords3 = get_screen_coordinates(v3, xres, yres);
        int x3 = coords3.first;
        int y3 = coords3.second;

        // if (!is_in_perspective(v1)) {
        //     cout << "v1 not in perspective " << v1.x << " " << v1.y << endl;
        // }
        // if (!is_in_perspective(v2)) {
        //     cout << "v2 not in perspective " << v2.x << " " << v2.y << endl;
        // }
        // if (!is_in_perspective(v3)) {
        //     cout << "v3 not in perspective " << v3.x << " " << v3.y << endl;
        // }
        if (is_in_perspective(v1) || is_in_perspective(v2)) {
            rasterize_line(x1, y1, x2, y2, xres, yres, grid);
        }
        if (is_in_perspective(v2) || is_in_perspective(v3)) {
            rasterize_line(x2, y2, x3, y3, xres, yres, grid);
        }
        if (is_in_perspective(v1) || is_in_perspective(v3)) {
            rasterize_line(x1, y1, x3, y3, xres, yres, grid);
        }
    }
}

// Works for first and eigth octants.
void low_slope_bresenham(int x1, int y1, int x2, int y2, int xres, int yres,
                        int **grid) {
    int eps = 0;
    int y = y1;
    int dx = x2 - x1;
    int dy = abs(y2 - y1);

    for (size_t x = x1; x <= x2; x++) {
        if (x >= 0 && x < xres && y >= 0 && y < yres) {
            grid[y][x] = 1;
        }

        if (2 * (eps + dy) < dx) {
            eps += dy;
        }
        else {
            eps += dy - dx;
            y += (y2 >= y1) ? 1 : -1;
        }
    }
}

// Works for 2nd and 7th octants.
void high_slope_bresenham(int x1, int y1, int x2, int y2, int xres, int yres,
                        int **grid) {
    int eps = 0;
    int x = x1;
    int dx = abs(x2 - x1);
    int dy = y2 - y1;

    for (size_t y = y1; y <= y2; y++) {
        if (x >= 0 && x < xres && y >= 0 && y < yres) {
            grid[y][x] = 1;
        }

        if (2 * (eps + dx) < dy) {
            eps += dx;
        }
        else {
            eps += dx - dy;
            x += (x2 >= x1) ? 1 : -1;
        }
    }
}

void rasterize_line(int x1, int y1, int x2, int y2, int xres, int yres,
                    int **grid) {
    if (abs(y2 - y1) <= abs(x2 - x1)) {
        if (x1 <= x2) {
            low_slope_bresenham(x1, y1, x2, y2, xres, yres, grid);
        }
        else {
            low_slope_bresenham(x2, y2, x1, y1, xres, yres, grid);
        }
    }
    else {
        if (y1 <= y2) {
            high_slope_bresenham(x1, y1, x2, y2, xres, yres, grid);
        }
        else {
            high_slope_bresenham(x2, y2, x1, y1, xres, yres, grid);
        }
    }
}
