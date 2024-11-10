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

        /**
         * Selección un emisor aleatoriamente proporcional a su radiancia
         */
        float pdfEmitter;
        const Emitter *emitter = scene->sampleEmitter(sampler->next1D(), pdfEmitter);
        if (pdfEmitter == 0.0f || !emitter) {
            //cout << "DEVUELVO 0\n";
            return Lo;  // Si no hay PDF o el emisor es inválido, retornar 0
        }
        // cout << "PdfEmiter: " << pdfEmitter << "\n";


        /**
         * Dentro del emisor, elegir un punto.
         * Adicionalmente lRec.ref = punto de la superficie que estamos iluminando.
         */
        EmitterQueryRecord lRec;
        lRec.ref = its.p; //
        Color3f Le = emitter->sample(lRec, sampler->next2D(), 0.);
        // float pdfComplete = pdfEmitter * lRec.pdf;
        // cout << Le.toString();

        /**
         * Si es el emisor tener en cuenta: Le(x,wo)
         */
        if (its.mesh->isEmitter()) {
            // Crear un EmitterQueryRecord y establecer sus campos manualmente
            EmitterQueryRecord eRec(its.p);
            eRec.ref = ray.o;                      // Punto de referencia (intersección)
            eRec.wi = ray.d;                      // Dirección hacia el observador (invertida)
            eRec.n = its.shFrame.n; 
            return its.mesh->getEmitter()->eval(eRec); 
        }

        /**
         * Verificar que el camino hacia la luz esté libre
         */
        Ray3f shadowRay(its.p, lRec.wi, Epsilon, lRec.dist - Epsilon);
        if (!scene->rayIntersect(shadowRay)) {
            /**
             * Calcular BRDF, odescribe como la luz refleja una superficie opaca
             */
            const BSDF *bsdf = its.mesh->getBSDF();
            BSDFQueryRecord bsdfRec(its.toLocal(-ray.d), its.toLocal(lRec.wi), its.uv, ESolidAngle);
            Color3f bsdfVal = bsdf->eval(bsdfRec);
            // cout << bsdfVal.toString();
            /**
             * Ángulo entre la normal de la fuente de luz y la dirección al punto
             */
            float cosTheta = std::max(0.0f, its.shFrame.n.dot(lRec.wi));
            float pdfLight = emitter->pdf(lRec);
            // cout << cosTheta << "\n";
            // cout << "pdfLight: " <<  pdfLight << "\n";
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
