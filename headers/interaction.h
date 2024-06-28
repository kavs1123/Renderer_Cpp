#pragma once

#include "vec.h"

// Forward declaration of BSDF class
class BSDF;

struct Interaction {
    // Position of interaction
    Vector3f p;
    // Normal of the surface at interaction
    Vector3f n;
    // The uv co-ordinates at the intersection point
    Vector2f uv;
    // The viewing direction in local shading frame
    Vector3f wi; 
    // Distance of intersection point from origin of the ray
    float t = 1e30f; 
    // Used for light intersection, holds the radiance emitted by the emitter.
    Vector3f emissiveColor = Vector3f(0.f, 0.f, 0.f);
    // BSDF at the shading point
    BSDF* bsdf;
    // Vectors defining the orthonormal basis
    Vector3f a, b, c;

    bool didIntersect = false;

    Vector3f toWorld(Vector3f w) {
        Vector3f world;
        world.x = a.x * w.x + b.x * w.y + c.x * w.z;
        world.y = a.y * w.x + b.y * w.y + c.y * w.z;
        world.z = a.z * w.x + b.z * w.y + c.z * w.z;
        return world;

    }

    Vector3f toLocal(Vector3f w) {
        Vector3f local;
        local.x = Dot(w, a);
        local.y = Dot(w, b);
        local.z =  Dot(w, c);
        return local;

    }
};