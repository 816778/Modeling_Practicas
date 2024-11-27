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

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray, Color3f throughput, bool wasSmooth, bool first) const {
        Intersection its;
        Color3f Lo(0.0f);
        bool doit = wasSmooth || first;

        // Check for intersection
        if (!scene->rayIntersect(ray, its)) {
           return (doit ? throughput * scene->getBackground(ray) : Lo);
        }

        
        float w_mat = 0.0f, w_em = 0.0f, p_em = 0.0f, p_mat = 0.0f;

        // Add emitted radiance if hitting an emitter directly
        if (its.mesh->isEmitter() && doit) {
            EmitterQueryRecord eRec(its.p);
            eRec.ref = ray.o;
            eRec.wi = ray.d;
            eRec.n = its.shFrame.n;
            eRec.uv = its.uv;
            Lo += throughput * its.mesh->getEmitter()->eval(eRec);
        }

        // MIS: Direct illumination from BSDF sampling
        if (its.mesh->isEmitter()) {
            const Emitter *em_mat = its.mesh->getEmitter();
            EmitterQueryRecord eRec(em_mat, its.p, its.p, its.shFrame.n, its.uv);
            eRec.ref = ray.o;
            eRec.wi = ray.d;
            eRec.n = its.shFrame.n;
            eRec.dist = its.t;

            Color3f Le = em_mat->eval(eRec);
            BSDFQueryRecord bsdfQR(its.toLocal(-ray.d));
            p_mat = its.mesh->getBSDF()->pdf(bsdfQR);
            p_em = em_mat->pdf(eRec);

            Color3f Lmat = Le * throughput;

            if (wasSmooth) {
                w_mat = 1.0f;
            } else {
                if (p_em + p_mat > Epsilon) {
                    w_mat = p_mat / (p_em + p_mat);
                }
            }

            Lo += Lmat * w_mat;
        }

        // MIS: Direct illumination from emitter sampling
        if (!wasSmooth) {
            float pdf;
            const Emitter *emitter = scene->sampleEmitter(sampler->next1D(), pdf);

            if (emitter && pdf > 0.0f) {
                EmitterQueryRecord eRec(its.p);
                Color3f Le = emitter->sample(eRec, sampler->next2D(), 0.0f);

                // Shadow ray check
                Intersection lightIts;
                Ray3f shadowRay(its.p, eRec.wi);
                bool inShadow = scene->rayIntersect(shadowRay, lightIts);

                if (!inShadow || lightIts.t >= eRec.dist - Epsilon) {
                    BSDFQueryRecord bsdfQR(its.toLocal(-ray.d), its.toLocal(eRec.wi), its.uv, ESolidAngle);
                    Color3f bsdfVal = its.mesh->getBSDF()->eval(bsdfQR);

                    float cosTheta = std::max(0.0f, its.shFrame.n.dot(eRec.wi));
                    float emitterPdf = eRec.pdf * pdf;

                    if (emitterPdf > Epsilon) {
                        Color3f Lem = Le * cosTheta * bsdfVal / emitterPdf;

                        p_mat = its.mesh->getBSDF()->pdf(bsdfQR);
                        p_em = emitterPdf;

                        if (p_em + p_mat > Epsilon) {
                            w_em = p_em / (p_em + p_mat);
                            Lo += throughput * Lem * w_em;
                        }
                    }
                }
            }
        }
        Point2f sample = sampler->next2D();
        BSDFQueryRecord bsdfRec(its.toLocal(-ray.d));
        Color3f brdfVal = its.mesh->getBSDF()->sample(bsdfRec, sample);

        if (brdfVal.isZero() || brdfVal.hasNaN()) {
            return Lo;
        }
        throughput *= brdfVal;

        // Russian Roulette termination
        float rrProb = std::min(throughput.maxCoeff(), 0.95f);
        if (sampler->next1D() > rrProb) {
            return Lo; 
        }

        Vector3f woWorld = its.toWorld(bsdfRec.wo);
        throughput /= rrProb;

        // Trace the next ray recursively
        Ray3f nextRay(its.p, woWorld);
        Lo += Li(scene, sampler, nextRay, throughput, bsdfRec.measure == EDiscrete, false);

        return Lo;
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Color3f Lo(0.0f); // Accumulated radiance along the ray
        Color3f throughput(1.0f);
        return Li(scene, sampler, ray, throughput, false, true);
    }

    std::string toString() const {
        return "PathTracing []";
    }

};

NORI_REGISTER_CLASS(PathTracingMIS, "path_mis");
NORI_NAMESPACE_END

