#ifndef LIGHT_H
#define LIGHT_H


#include "color.hpp"


struct Light {
    double x;
    double y;
    double z;
    Color c;
    double k;   // Attenuation parameter
};

#endif
