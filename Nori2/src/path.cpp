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

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray, Color3f &throughput) const {
        Color3f Lo(0.0f); 
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return scene->getBackground(ray) * throughput;
        }


        if (its.mesh->isEmitter()) {
            EmitterQueryRecord eRec(its.p);
            eRec.ref = ray.o;                   
            eRec.wi = ray.d;
            eRec.n = its.shFrame.n;           
            return its.mesh->getEmitter()->eval(eRec) * throughput;
        }


        Point2f sample = sampler->next2D();
        BSDFQueryRecord bsdfRec(its.toLocal(-ray.d), sample);

        const BSDF *bsdf = its.mesh->getBSDF();
        Color3f bsdfSample = bsdf->sample(bsdfRec, sample);

        if (bsdfSample.isZero() || bsdfSample.hasNaN()) {
            return Lo; // No contribution from this path
        }

        Vector3f woWorld = its.toWorld(bsdfRec.wo);
        float cosTheta = std::max(0.0f, its.shFrame.n.dot(woWorld));
        throughput *= bsdfSample; //* cosTheta;

        float rrProb = std::min(throughput.maxCoeff(), 0.95f); // Survival probability based on throughput
        if (sampler->next1D() > rrProb) {
            return Lo;
        }
        throughput /= rrProb;

        // Trace the new ray
        Ray3f newRay(its.p, woWorld);
        Lo += Li(scene, sampler, newRay, throughput);
        return Lo;
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Color3f Lo(0.0f); // Accumulated radiance along the ray
        Color3f throughput(1.0f);
        return Li(scene, sampler, ray, throughput);
    }

    /// Compute the radiance along a ray
    Color3f Li2(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Color3f Lo(0.0f); // Accumulated radiance along the ray
        Color3f throughput(1.0f);
        
        // Find the surface that is visible in the requested direction
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return scene->getBackground(ray); // Return environment map if no intersection
        }


        // Direct lighting from emitters
        if (its.mesh->isEmitter()) {
            EmitterQueryRecord eRec(its.p);
            eRec.ref = ray.o;                   
            eRec.wi = ray.d;
            eRec.n = its.shFrame.n;           
            Lo += its.mesh->getEmitter()->eval(eRec);
        }


        // Sample a new direction using the BSDF
        Point2f sample = sampler->next2D();
        BSDFQueryRecord bsdfRec(its.toLocal(-ray.d), sample);

        const BSDF *bsdf = its.mesh->getBSDF();
        Color3f bsdfSample = bsdf->sample(bsdfRec, sample);

        if (bsdfSample.isZero() || bsdfSample.hasNaN()) {
            return Lo; // No contribution from this path
        }

        

        Vector3f woWorld = its.toWorld(bsdfRec.wo); // Transform local direction to world
        float cosTheta = std::max(0.0f, its.shFrame.n.dot(woWorld));

        // Trace the new ray
        Ray3f newRay(its.p, woWorld);
        Color3f indirectLighting = Li(scene, sampler, newRay);
                // Russian Roulette termination probability

        float rrProb = std::min(bsdfSample.maxCoeff(), 0.95f);
        if (sampler->next1D() > rrProb) {
            return Lo; // Terminate path if random number exceeds probability
        }

        // Accumulate indirect lighting contribution
        Lo += bsdfSample * indirectLighting * cosTheta / rrProb;

        return Lo; // Return the accumulated radiance
    }


    std::string toString() const {
        return "PathTracing []";
    }

};

NORI_REGISTER_CLASS(PathTracing, "path");
NORI_NAMESPACE_END

