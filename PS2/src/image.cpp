#include "image.hpp"

#include <cfloat>
#include <iostream>


using namespace std;


Image::Image(int xres, int yres) {
    this->xres = xres;
    this->yres = yres;

    // Initialize grid
    grid = new Color*[yres];

    for (size_t i = 0; i < yres; i++) {
        grid[i] = new Color[xres];

        for (size_t j = 0; j < xres; j++) {
            grid[i][j] = Color{0, 0, 0};
        }
    }

    // Initialize buffer
    buffer = new float*[yres];

    for (size_t i = 0; i < yres; i++) {
        buffer[i] = new float[xres];

        for (size_t j = 0; j < xres; j++) {
            buffer[i][j] = FLT_MAX;
        }
    }
}

Image::~Image() {
    // Free grid and buffer
    for (size_t i = 0; i < yres; i++) {
        delete [] grid[i];
        delete [] buffer[i];
    }

    delete [] grid;
    delete [] buffer;
}

void Image::output_image() {
    // Print header, resolution, and max pixel intensity
    cout << "P3" << endl;
    cout << xres << " " << yres << endl;
    cout << 255 << endl;

    for (int i = 0; i < yres; i++) {
        for (int j = 0; j < xres; j++) {
            int r = int(255 * grid[i][j](0));
            int g = int(255 * grid[i][j](1));
            int b = int(255 * grid[i][j](2));

            cout << r << " " << g << " " << b << endl;
        }
    }
}
