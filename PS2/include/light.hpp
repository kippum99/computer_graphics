#ifndef LIGHT_H
#define LIGHT_H


#include "color.hpp"


struct Light {
    Vector3f position;
    Color color;
    float k;   // Attenuation parameter
};

#endif
