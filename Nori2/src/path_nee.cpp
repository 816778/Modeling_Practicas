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

class PathTracingNEE : public Integrator {
public:
    PathTracingNEE(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray, Color3f &throughput) const {
        Color3f Lo(0.0f); 
        Intersection its;
        /**
         * Ray intersection
         */
        if (!scene->rayIntersect(ray, its)) {
            return scene->getBackground(ray) * throughput;
        }

        /**
         * Emitter sampling
         */
        if (its.mesh->isEmitter()) {
            EmitterQueryRecord eRec(its.p);
            eRec.ref = ray.o;                   
            eRec.wi = ray.d;
            eRec.n = its.shFrame.n;    
            eRec.uv = its.uv;       
            return its.mesh->getEmitter()->eval(eRec) * throughput;
        }
        

        /**
         * Direct lighting
         */
        Point2f sample = sampler->next2D();
        BSDFQueryRecord bsdfRec(its.toLocal(-ray.d), sample);
        const BSDF *bsdf = its.mesh->getBSDF();
        Color3f bsdfSample = bsdf->sample(bsdfRec, sample);

        if (bsdfSample.isZero() || bsdfSample.hasNaN()) {
            return Lo; 
        }

        /**
         * accumulate radiance from light sources if the BSDF is perfectly smooth
         */
        if (bsdfRec.measure == EDiscrete) {
            //std::cout << "Discrete BSDF" << std::endl;
            float pdfEmitter;
            const Emitter *emitter = scene->sampleEmitter(sampler->next1D(), pdfEmitter);
            if (emitter && pdfEmitter > 0.0f) {
                EmitterQueryRecord lRec(its.p);
                Color3f Le = emitter->sample(lRec, sampler->next2D(), 0.0f);

                Ray3f shadowRay(its.p, lRec.wi, Epsilon, lRec.dist - Epsilon);
                Intersection lightIts;
                bool inShadow = scene->rayIntersect(shadowRay, lightIts);

                // Verificar que no hay intersección o que el emisor está más cerca
                if (!inShadow || lightIts.t >= (lRec.dist - Epsilon)) {
                    BSDFQueryRecord lightBsdfRec(its.toLocal(-ray.d), its.toLocal(lRec.wi), its.uv, ESolidAngle);
                    Color3f bsdfVal = bsdf->eval(lightBsdfRec);
                    float cosTheta = std::max(0.0f, its.shFrame.n.dot(lRec.wi));
                    float pdfLight = emitter->pdf(lRec);

                    if (pdfLight > 0.0f) {
                        Lo += throughput * (Le * bsdfVal /* cosTheta*/) / (pdfLight * pdfEmitter);
                    }
                }
            }
        }

        Vector3f woWorld = its.toWorld(bsdfRec.wo);
        float cosTheta = std::max(0.0f, its.shFrame.n.dot(woWorld));
        throughput *= bsdfSample;// * cosTheta;


        /**
         * Russian Roulette
         */
        float rrProb = std::min(throughput.maxCoeff(), 0.95f); // Survival probability based on throughput
        if (sampler->next1D() > rrProb) {
            return Lo;
        }
        throughput /= rrProb;

        /**
         * Trace new ray
         */
        Ray3f newRay(its.p, woWorld);
        Lo += Li(scene, sampler, newRay, throughput);
        return Lo;
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Color3f throughput(1.0f);
        return Li(scene, sampler, ray, throughput);
    }

    std::string toString() const {
        return "PathTracing []";
    }

};

NORI_REGISTER_CLASS(PathTracingNEE, "path_nee");
NORI_NAMESPACE_END

