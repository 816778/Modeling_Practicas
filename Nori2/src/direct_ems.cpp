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

class DirectEmitterSampling : public Integrator {
public:
    DirectEmitterSampling(const PropertyList &props) {
        /* No parameters this time */
    }

    /// Compute the radiance along a ray
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Color3f Lo(0.0f);
       
        // Find the surface that is visible in the requested direction
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return scene->getBackground(ray);
        }

        // Get the BSDF at the hit point
        const BSDF *bsdf = its.mesh->getBSDF();

        if (its.mesh->isEmitter()) {
            // Crear un EmitterQueryRecord y establecer sus campos manualmente
            EmitterQueryRecord eRec;
            eRec.ref = its.p;                      // Punto de referencia (intersección)
            eRec.wi = -ray.d;                      // Dirección hacia el observador (invertida)
            eRec.emitter = its.mesh->getEmitter(); // Puntero al emisor actual
            eRec.n = its.shFrame.n; 
            // Agregar la radiancia directa emitida al total
            Lo += eRec.emitter->eval(eRec);
        }


        float pdfEmitter;
        const Emitter *emitter = scene->sampleEmitter(sampler->next1D(), pdfEmitter);
        if (pdfEmitter == 0.0f || !emitter) {
            return Lo;  // Si no hay PDF o el emisor es inválido, retornar 0
        }

        EmitterQueryRecord lRec;
        lRec.ref = its.p;
        Color3f Le = emitter->sample(lRec, sampler->next2D(), 0.);
        if (lRec.pdf <= 0.0f) {
            return Lo;  // Si la PDF es inválida, no añadimos contribución de esta muestra
        }

        // float pdfComplete = pdfEmitter * lRec.pdf;

        Ray3f shadowRay(its.p, lRec.wi, Epsilon, lRec.dist - Epsilon);
        if (!scene->rayIntersect(shadowRay)) {
            // Si no hay obstrucción, calcula el BRDF
            BSDFQueryRecord bsdfRec(its.toLocal(-ray.d), its.toLocal(lRec.wi), its.uv, ESolidAngle);
            Color3f bsdfVal = bsdf->eval(bsdfRec);
            float cosTheta = std::max(0.0f, its.shFrame.n.dot(lRec.wi));
            float pdfLight = emitter->pdf(lRec);

            if (pdfLight > 0.0f) {
                // Agregar contribución del emisor a la iluminación directa
                Lo += (Le * bsdfVal * cosTheta) / (pdfLight * pdfEmitter);
            }
        }


        return Lo;
    }

    std::string toString() const {
        return "DirectEmsIntegrator []";
    }

};

NORI_REGISTER_CLASS(DirectEmitterSampling, "direct_ems");
NORI_NAMESPACE_END
