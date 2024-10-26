#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

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
        int N = 1; // Número de muestras, puedes ajustar este valor

        // Find the surface that is visible in the requested direction
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return scene->getBackground(ray);
        }

        // Get the BSDF at the hit point
        const BSDF *bsdf = its.mesh->getBSDF();

        // Sumatoria sobre N muestras
        for (int k = 0; k < N; ++k) {
            // Muestrear un emisor aleatorio y obtener su PDF
            float pdfEmitter;
            const Emitter *emitter = scene->sampleEmitter(sampler->next1D(), pdfEmitter);

            if (std::abs(pdfEmitter) < Epsilon || !emitter) {
                continue;  // Si no hay PDF o el emisor es inválido, saltar esta muestra
            }

            EmitterQueryRecord lRec(its.p);
            // cout << lRec.toString();
            Color3f Le = emitter->sample(lRec, sampler->next2D(), 0.);

            // Rayo de sombra
            Ray3f shadowRay(its.p, lRec.wi, Epsilon, lRec.dist);
            if (!scene->rayIntersect(shadowRay)) {
                // Evaluar la BSDF
                BSDFQueryRecord bsdfRec(its.toLocal(-ray.d), its.toLocal(lRec.wi), its.uv, ESolidAngle);
                Color3f bsdfVal = bsdf->eval(bsdfRec);
                
                // Factor coseno
                float cosTheta = std::max(0.0f, its.shFrame.n.dot(lRec.wi));
                float pdfLight = emitter->pdf(lRec);
                
                // Si el pdf es válido, acumular el valor en Lo
                if (pdfLight > 0.0f) {
                    Lo += (Le * bsdfVal * cosTheta) / (pdfLight * pdfEmitter);
                }
            }
        }

        // Promediar las muestras
        Lo /= N;

        return Lo;
    }

    std::string toString() const {
        return "DirectEmsIntegrator []";
    }
};

NORI_REGISTER_CLASS(DirectEmitterSampling, "direct_ems");
NORI_NAMESPACE_END
