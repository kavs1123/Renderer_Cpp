#include "render.h"

Integrator::Integrator(Scene &scene)
{
    this->scene = scene;
    
    this->outputImage.allocate(TextureType::UNSIGNED_INTEGER_ALPHA, this->scene.imageResolution);
}

long long Integrator::render(int spp,int samplingtype)
{
    auto startTime = std::chrono::high_resolution_clock::now();
    for (int x = 0; x < this->scene.imageResolution.x; x++) {
        for (int y = 0; y < this->scene.imageResolution.y; y++) {

            Vector3f result(0, 0, 0);

            if (samplingtype == 0) {
                for (int i = 0; i < spp; i++) {
                    Ray cameraRay = this->scene.camera.generateRay(x, y);
                    Interaction si = this->scene.rayIntersect(cameraRay);
                    Interaction light_interaction = this->scene.rayEmitterIntersect(cameraRay);

                    if (light_interaction.didIntersect && light_interaction.t < si.t) {
                        result += light_interaction.emissiveColor;
                        continue;
                    }

                    else if (si.didIntersect) {
                        Vector3f hemi_point = this->scene.hemispheresampling();
                        Vector3f global_hemi = si.toWorld(hemi_point);
                        if (Dot(global_hemi, si.n) < 0) {
                            global_hemi *= -1;
                        }
                        Ray newRay(si.p + 1e-3 * si.n, global_hemi);
                        Interaction lighting = this->scene.rayEmitterIntersect(newRay);
                        Interaction si2 = this->scene.rayIntersect(newRay);
                        if (lighting.didIntersect && (!si2.didIntersect || lighting.t < si2.t)) {
                            result += si.bsdf->eval(&si, global_hemi) * lighting.emissiveColor * Dot(global_hemi, si.n) * 2.f * M_PI;
                        }
                    }
                }

            }

            if (samplingtype == 1) {
                for (int i = 0; i < spp; i++) {
                    Ray cameraRay = this->scene.camera.generateRay(x, y);
                    Interaction si = this->scene.rayIntersect(cameraRay);
                    Interaction light_interaction = this->scene.rayEmitterIntersect(cameraRay);

                    if (light_interaction.didIntersect && light_interaction.t < si.t) {
                        result += light_interaction.emissiveColor;
                        continue;
                    }

                    else if (si.didIntersect) {

                        float e1 = next_float();
                        float e2 = next_float();
                        float theta = acos(sqrt(e1));
                        Vector3f cosi_point = this->scene.cosine_sampling(e1,e2);
                        Vector3f global_cosi = si.toWorld(cosi_point);
                        if (Dot(global_cosi, si.n) < 0) {
                            global_cosi *= -1;
                        }
                        Ray newRay(si.p + 1e-3 * si.n, global_cosi);
                        Interaction lighting = this->scene.rayEmitterIntersect(newRay);
                        Interaction si2 = this->scene.rayIntersect(newRay);
                        if (lighting.didIntersect && (!si2.didIntersect || lighting.t < si2.t)) {
                            result += si.bsdf->eval(&si, global_cosi) * lighting.emissiveColor * Dot(global_cosi, si.n) / (cos(theta) / M_PI);
                        }
                    }
                }

            }
            if (samplingtype == 2) {
                for (int i = 0; i < spp; i++) {
                    Ray cameraRay = this->scene.camera.generateRay(x, y);
                    Interaction si = this->scene.rayIntersect(cameraRay);
                    Interaction light_interaction = this->scene.rayEmitterIntersect(cameraRay);
                    if (light_interaction.didIntersect && light_interaction.t < si.t) {
                        result += light_interaction.emissiveColor;
                    }


                    if (si.didIntersect) {
                        Vector3f radiance;
                        LightSample ls;
                        int random_Light_Idx = rand() % this->scene.lights.size();
                        Light& randomLight = this->scene.lights[random_Light_Idx];
                        std::tie(radiance, ls) = randomLight.sample(&si);
                        Ray shadowRay(si.p + 1e-3f * si.n, ls.wo);
                        Interaction shadow = this->scene.rayIntersect(shadowRay);

                        if (!shadow.didIntersect || shadow.t > ls.d) {
                            result += si.bsdf->eval(&si, si.toLocal(ls.wo)) * radiance * AbsDot(si.n, ls.wo) * AbsDot(-ls.wo, ls.n);

                        }
                    }

                }
            }

            if(samplingtype == 3){
                for (int i = 0; i < spp; i++) {

                    Ray cameraRay = this->scene.camera.generateRay(x, y);
                    Interaction si = this->scene.rayIntersect(cameraRay);
                    

                    Interaction light_interaction = this->scene.rayEmitterIntersect(cameraRay);
                    if (light_interaction.didIntersect && light_interaction.t < si.t) {
                        result += light_interaction.emissiveColor;
                        continue;
                    }

                    
                }

            }

            result = result / spp;

            this->outputImage.writePixelColor(result, x, y);
        }
    }
    auto finishTime = std::chrono::high_resolution_clock::now();

    return std::chrono::duration_cast<std::chrono::microseconds>(finishTime - startTime).count();
}

int main(int argc, char **argv)
{
    if (argc != 5) {
        std::cerr << "Usage: ./render <scene_config> <out_path> <num_samples> <sampling_strategy>";
        return 1;
    }
    Scene scene(argv[1]);

    Integrator rayTracer(scene);
    int spp = atoi(argv[3]);
    int sampling_type = atoi(argv[4]);
    // for testing purpose 0 - None , 1- uniform , 2- cosine , 3- cosine 
    // have to change it to 0- uniform , 1 - cosine , 2 - importance
    auto renderTime = rayTracer.render(spp,sampling_type);
    
    std::cout << "Render Time: " << std::to_string(renderTime / 1000.f) << " ms" << std::endl;
    rayTracer.outputImage.save(argv[2]);

    return 0;
}
