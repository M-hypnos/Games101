//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

Vector3f Scene::shade(Intersection& intersection, Vector3f wo) const {
    if (intersection.obj->hasEmit()) {
        return intersection.emit;
    }

    Intersection light;
    float pdf;
    sampleLight(light, pdf);
    Vector3f obj2Light = light.coords - intersection.coords;
    Ray ray(intersection.coords, obj2Light.normalized());
    Intersection inter = intersect(ray);
    Vector3f Ldir;
    if (inter.distance + 0.001 > obj2Light.norm()) {
        Vector3f fr = intersection.m->eval(-obj2Light.normalized(), wo, intersection.normal);
        float costheta = std::max(0.f, dotProduct(obj2Light.normalized(), intersection.normal));
        float costheta1 = std::max(0.f, dotProduct(-obj2Light.normalized(), light.normal));
        Ldir = light.emit * fr * costheta * costheta1 / dotProduct(obj2Light, obj2Light) / pdf;
    }

    Vector3f Lindir;
    if (get_random_float() < RussianRoulette) {
        Vector3f wi = intersection.m->sample(-wo, intersection.normal).normalized();
        float pdf1 = intersection.m->pdf(-wo, wi, intersection.normal);
        if (pdf1 > 0.001) {
            Ray ray1(intersection.coords, wi);
            Intersection inter1 = intersect(ray1);
            if (inter1.happened && !inter1.obj->hasEmit()) {
                Vector3f fr = intersection.m->eval(-wi, wo, intersection.normal);
                float costheta = std::max(0.f, dotProduct(wi, intersection.normal));
                Lindir = shade(inter1, -wi) * fr * costheta / pdf1 / RussianRoulette; 
            }
        }
    }

    return Ldir + Lindir;
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection intersection = Scene::intersect(ray);
    if (intersection.happened) {
        return shade(intersection, -ray.direction);
    }
    else {
        return {};
    }
}