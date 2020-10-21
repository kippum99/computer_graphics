#ifndef IMAGE_H
#define IMAGE_H

#include "color.hpp"


class Image {
public:
    int xres;
    int yres;
    Color **grid;
    float **buffer;

    Image(int xres, int yres);
    ~Image();

    void output_image();
};

#endif
