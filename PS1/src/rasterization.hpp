#ifndef RASTERIZATION_H
#define RASTERIZATION_H

#include "object.hpp"

void rasterize_object(Object &obj, int xres, int yres, int **grid);
void rasterize_line(int x1, int x2, int y1, int y2, int **grid);

#endif
