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

class PathTracingMIS : public Integrator {
public:
    PathTracingMIS(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray, Color3f &throughput) const {
        Color3f Lo(0.0f); 
        Intersection its;

        // 1. Si no hay intersección, devolver el mapa de entorno
        if (!scene->rayIntersect(ray, its)) {
            return scene->getBackground(ray) * throughput;
        }

        // 2. Si golpea un emisor, acumular su radiancia (muestreo BSDF)
        if (its.mesh->isEmitter()) {
            EmitterQueryRecord eRec(its.p);
            eRec.ref = ray.o;                   
            eRec.wi = ray.d;
            eRec.n = its.shFrame.n;    
            eRec.uv = its.uv;       
            Lo += throughput * its.mesh->getEmitter()->eval(eRec);
        }

        // 3. Muestreo de emisores (MIS)
        float pdfEmitter;
        const Emitter *emitter = scene->sampleEmitter(sampler->next1D(), pdfEmitter);
        if (emitter && pdfEmitter > 0.0f) {
            EmitterQueryRecord lRec(its.p);
            Color3f Le = emitter->sample(lRec, sampler->next2D(), 0.0f);

            Ray3f shadowRay(its.p, lRec.wi, Epsilon, lRec.dist - Epsilon);
            Intersection lightIts;
            bool inShadow = scene->rayIntersect(shadowRay, lightIts);

            // Verificar si el rayo no está en sombra
            if (!inShadow || lightIts.t >= (lRec.dist - Epsilon)) {
                BSDFQueryRecord lightBsdfRec(its.toLocal(-ray.d), its.toLocal(lRec.wi), its.uv, ESolidAngle);
                Color3f bsdfVal = its.mesh->getBSDF()->eval(lightBsdfRec);
                float cosTheta = std::max(0.0f, its.shFrame.n.dot(lRec.wi));
                float pdfLight = emitter->pdf(lRec);
                float pdfBsdf = its.mesh->getBSDF()->pdf(lightBsdfRec);

                if (pdfLight > 0.0f) {
                    // Calcular el peso MIS para emisores
                    float w_ems = pdfLight / (pdfLight + pdfBsdf);
                    Lo += throughput * (Le * bsdfVal * cosTheta * w_ems) / (pdfLight * pdfEmitter);
                }
            }
        }

        // 4. Muestreo de BSDF (MIS)
        Point2f sample = sampler->next2D();
        BSDFQueryRecord bsdfRec(its.toLocal(-ray.d), sample);
        const BSDF *bsdf = its.mesh->getBSDF();
        Color3f bsdfSample = bsdf->sample(bsdfRec, sample);

        if (bsdfSample.isZero() || bsdfSample.hasNaN()) {
            return Lo; 
        }

        Vector3f woWorld = its.toWorld(bsdfRec.wo);
        float cosTheta = std::max(0.0f, its.shFrame.n.dot(woWorld));
        throughput *= bsdfSample;

        // Si BSDF no es discreto, acumular radiancia del emisor golpeado
        if (bsdfRec.measure == EDiscrete) {
            Ray3f bsdfRay(its.p, woWorld);
            Intersection bsdfIts;

            if (scene->rayIntersect(bsdfRay, bsdfIts) && bsdfIts.mesh->isEmitter()) {
                EmitterQueryRecord eRec(bsdfIts.p);
                eRec.ref = its.p;                   
                eRec.wi = woWorld;
                eRec.n = bsdfIts.shFrame.n;

                const Emitter* emitter = bsdfIts.mesh->getEmitter(); // Obtén el emisor
                float pdfLight = emitter->pdf(eRec); // Usa el emisor directamente
                float pdfBsdf = bsdf->pdf(bsdfRec);

                if (pdfBsdf > 0.0f) {
                    float w_bsdf = pdfBsdf / (pdfBsdf + pdfLight);
                    Color3f Le = emitter->eval(eRec);
                    Lo += throughput * Le * w_bsdf;
                }
            }
        }

        // 5. Russian Roulette
        float rrProb = std::min(throughput.maxCoeff(), 0.95f);
        if (sampler->next1D() > rrProb) {
            return Lo;
        }
        throughput /= rrProb;

        // 6. Traza el siguiente rebote
        Ray3f newRay(its.p, woWorld);
        Lo += Li(scene, sampler, newRay, throughput);
        return Lo;
    }


    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Color3f Lo(0.0f); // Accumulated radiance along the ray
        Color3f throughput(1.0f);
        return Li(scene, sampler, ray, throughput);
    }

    std::string toString() const {
        return "PathTracing []";
    }

};

NORI_REGISTER_CLASS(PathTracingMIS, "path_mis");
NORI_NAMESPACE_END

