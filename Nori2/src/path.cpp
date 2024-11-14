#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

/*
Implementa la iluminación directa mediante muestreo de emisores
Muestrear directamente los emisores (como las luces de área) y calcular la radiancia directa hacia los puntos visibles de la escena.
*/

class PathTracing : public Integrator {
public:
    PathTracing(const PropertyList &props) {
        /* No parameters this time */
    }

    /// Compute the radiance along a ray
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &initialRay) const {
        Color3f Lo(0.0f);            // Final accumulated radiance
        Color3f throughput(1.0f);     // Keeps track of accumulated color along the path
        Ray3f ray = initialRay;       // Start with the initial ray from the camera
        int depth = 0;                // Depth of bounces
        const int maxDepth = 10;      // Set a maximum depth for safety (optional)

        while (true) {
            Intersection its;
            if (!scene->rayIntersect(ray, its)) {
                // If ray doesn't hit anything, add background color contribution
                Lo += throughput * scene->getBackground(ray);
                break;
            }

            // If the hit point is on an emitter, add the emitter's radiance
            if (its.mesh->isEmitter()) {
                
                EmitterQueryRecord eRec(ray.o); 
                eRec.p = its.p;
                eRec.wi = ray.d;
                eRec.n = its.shFrame.n;
                if (depth == 0 ) {
                    its.mesh->getEmitter()->eval(eRec);
                    Color3f emitter_color = its.mesh->getEmitter()->eval(eRec);
                    std::cout << emitter_color << "\n";
                    return emitter_color;
                }
                Lo += throughput * its.mesh->getEmitter()->eval(eRec);
            }

            // Sample a random emitter in the scene for direct lighting
            float pdfEmitter;
            const Emitter *emitter = scene->sampleEmitter(sampler->next1D(), pdfEmitter);
            if (emitter && pdfEmitter > 0.0f) {
                EmitterQueryRecord lRec;
                lRec.ref = its.p;
                Color3f Le = emitter->sample(lRec, sampler->next2D(), 0.0f);

                // Check if the path to the emitter is unobstructed
                Ray3f shadowRay(its.p, lRec.wi, Epsilon, lRec.dist - Epsilon);
                if (!scene->rayIntersect(shadowRay)) {
                    const BSDF *bsdf = its.mesh->getBSDF();
                    BSDFQueryRecord bsdfRec(its.toLocal(-ray.d), its.toLocal(lRec.wi), its.uv, ESolidAngle);
                    Color3f bsdfVal = bsdf->eval(bsdfRec);

                    float cosTheta = std::max(0.0f, its.shFrame.n.dot(lRec.wi));
                    float pdfLight = emitter->pdf(lRec);
                    
                    if (pdfLight > 0.0f) {
                        Color3f contribution = (Le * bsdfVal * cosTheta) / (pdfEmitter * pdfLight);
                        Lo += throughput * contribution;
                    }
                }
            }

            // Sample the BSDF to get a new direction for the next bounce
            const BSDF *bsdf = its.mesh->getBSDF();
            BSDFQueryRecord bsdfRec(its.toLocal(-ray.d));
            Color3f bsdfSample = bsdf->sample(bsdfRec, sampler->next2D());

            // If the BSDF sample returns zero, stop the path
            if (bsdfSample.isZero()) break;

            // Update throughput with the BSDF sample and cosine factor
            throughput *= bsdfSample * std::max(0.0f, its.shFrame.n.dot(its.toWorld(bsdfRec.wo)));

            // Generate the next ray to continue the path
            ray = Ray3f(its.p, its.toWorld(bsdfRec.wo));

            // Russian roulette termination
            if (depth >= maxDepth) break;  // Safety limit to prevent excessive depth
            float p = std::min(throughput.maxCoeff(), 0.95f);  // Probability of continuing
            if (sampler->next1D() > p) break;  // Terminate the path based on Russian roulette
            throughput /= p;  // Adjust throughput to account for Russian roulette

            depth++;  // Increment bounce depth
        }

        return Lo;
    }

    std::string toString() const {
        return "DirectEmsIntegrator []";
    }

};

NORI_REGISTER_CLASS(PathTracing, "path");
NORI_NAMESPACE_END

