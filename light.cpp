#include "light.h"

Light::Light(LightType type, nlohmann::json config) {
    switch (type) {
        case LightType::POINT_LIGHT:
            this->position = Vector3f(config["location"][0], config["location"][1], config["location"][2]);
            break;
        case LightType::DIRECTIONAL_LIGHT:
            this->direction = Vector3f(config["direction"][0], config["direction"][1], config["direction"][2]);
            break;
        case LightType::AREA_LIGHT:
            this->center = Vector3f(config["center"][0], config["center"][1], config["center"][2]);
            this->normal = Vector3f(config["normal"][0], config["normal"][1], config["normal"][2]);
            this->vx = Vector3f(config["vx"][0], config["vx"][1], config["vx"][2]);
            this->vy = Vector3f(config["vy"][0], config["vy"][1], config["vy"][2]);
            
            break;
        default:
            std::cout << "WARNING: Invalid light type detected";
            break;
    }

    this->radiance = Vector3f(config["radiance"][0], config["radiance"][1], config["radiance"][2]);
    this->type = type;
}

std::pair<Vector3f, LightSample> Light::sample(Interaction *si) {
    LightSample ls;
    memset(&ls, 0, sizeof(ls));

    Vector3f radiance;
    switch (type) {
        case LightType::POINT_LIGHT:
            ls.wo = (position - si->p);
            ls.d = ls.wo.Length();
            ls.wo = Normalize(ls.wo);
            radiance = (1.f / (ls.d * ls.d)) * this->radiance;
            break;
        case LightType::DIRECTIONAL_LIGHT:
            ls.wo = Normalize(direction);
            ls.d = 1e10;
            radiance = this->radiance;
            break;
        case LightType::AREA_LIGHT:
            float u = next_float() * 2 - 1;
            float v = next_float() * 2 - 1;
            ls.wo = this->center + u * this->vx + v * this->vy - si->p;
            ls.d = ls.wo.Length();
            ls.wo = Normalize(ls.wo);
            ls.n = this->normal;

            radiance = Dot(ls.wo, this->normal) < 0 ? ((4 * Cross(this->vx, this->vy).Length()) / (ls.d * ls.d)) * this->radiance : Vector3f(0, 0, 0);
    }
    return { radiance, ls };
}

Interaction Light::intersectLight(Ray *ray) {
    Interaction si;
    memset(&si, 0, sizeof(si));
    si.didIntersect = false;
    si.t = 1e30;

    if (type == LightType::AREA_LIGHT) {
        if (Dot(ray->d, this->normal) >= 0) {
            return si;
        }
        Vector3f dir_c = Normalize(center - ray->o);
        float plane_distance = -Dot((center - ray->o), this->normal);
        Vector3f plane_pos = ray->o - ray->d * plane_distance / Dot(ray->d, this->normal);
        if (AbsDot((plane_pos - center), Normalize(vx)) > vx.Length()) {
            return si;
        }
        if (AbsDot((plane_pos - center), Normalize(vy)) > vy.Length()) {
            return si;
        }
        si.t = -Dot(dir_c, this->normal);
        si.didIntersect = true;
        si.emissiveColor = radiance;
        
    }

    return si;
}