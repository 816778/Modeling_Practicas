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

class DirectMIS : public Integrator {
public:
    DirectMIS(const PropertyList &props) {
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
         * Si es el emisor tener en cuenta: Le(x,wo)
         */
        if (its.mesh->isEmitter()) {
            // Crear un EmitterQueryRecord y establecer sus campos manualmente
            EmitterQueryRecord eRec(ray.o);
            eRec.p = its.p;                          
            eRec.wi = ray.d.normalized();                      
            eRec.n = its.shFrame.n; 
            Lo += its.mesh->getEmitter()->eval(eRec);
            return Lo;
        }

        // W em: Emitter based sampling

        Color3f Le_em(0.0f);  // Contribution from emitter sampling
        float pdf_em = 0.0f;
        const Emitter *emitter = scene->sampleEmitter(sampler->next1D(), pdf_em);

        if (emitter && pdf_em > 0.0f) {
            EmitterQueryRecord lRec(its.p);
            Color3f Le = emitter->sample(lRec, sampler->next2D(), 0.0f);

            if (Le.isZero() || lRec.pdf == 0.0f) {
                return Le_em; 
            }

            Ray3f shadowRay(its.p, lRec.wi);
            Intersection its_light;
            if (!scene->rayIntersect(shadowRay, its_light) || its_light.t >= (lRec.dist - Epsilon)) {
                // Compute the BSDF value
                const BSDF *bsdf = its.mesh->getBSDF();
                BSDFQueryRecord bsdfRec(its.toLocal(-ray.d), its.toLocal(lRec.wi), its.uv, ESolidAngle);
                Color3f f = bsdf->eval(bsdfRec);

                lRec.dist = its.t;

                float cosTheta = std::max(0.0f, its.shFrame.n.dot(lRec.wi));

                float pdf_mat = bsdf->pdf(bsdfRec); 

                float denom = pdf_em * lRec.pdf + pdf_mat;
                float w_em = (pdf_em * lRec.pdf)  / denom;

                Le_em = w_em * (Le * f * cosTheta) / (pdf_em * lRec.pdf);
            }
        }

        // Step 4: Direct illumination using Material (BRDF) sampling
        Color3f Le_mat(0.0f);  // Contribution from material sampling
        BSDFQueryRecord bsdfRec(its.toLocal(-ray.d), its.uv);
        const BSDF *bsdf = its.mesh->getBSDF();
        Color3f bsdfSample = bsdf->sample(bsdfRec, sampler->next2D());
        float pdf_mat = bsdf->pdf(bsdfRec);

        if (!bsdfSample.isZero() && pdf_mat > 0.0f) {
            Vector3f woWorld = its.toWorld(bsdfRec.wo);
            float cosTheta = std::max(0.0f, its.shFrame.n.dot(woWorld));

            // Cast a shadow ray in the direction of the material sample
            Ray3f shadowRay(its.p, woWorld);
            Intersection lightIts;
            if (scene->rayIntersect(shadowRay, lightIts) && lightIts.mesh->isEmitter()) {
                const Emitter *lightEmitter = lightIts.mesh->getEmitter();
                EmitterQueryRecord lRec(lightEmitter, its.p, lightIts.p, its.shFrame.n, lightIts.uv);
                Color3f Le = lightEmitter->eval(lRec);

                float pdf_em = lightEmitter->pdf(lRec);  // PDF using emitter sampling
                float pdfEmitter = emitter->pdf(lRec);
                
                // MIS weight for material sampling
                float w_mat = pdf_mat / (pdf_mat + pdf_em);
                Le_mat = w_mat * (Le * bsdfSample * cosTheta) / pdf_mat;
            }
            else if (!scene->rayIntersect(shadowRay, lightIts) ){
                Le_mat = bsdfSample * scene->getBackground(shadowRay) ;
            }
        }

        // Combine the contributions from both sampling methods
        Lo += Le_em + Le_mat;
        return Lo;
    }

    std::string toString() const {
        return "DirectEmsIntegrator []";
    }

};

NORI_REGISTER_CLASS(DirectMIS, "direct_mis");
NORI_NAMESPACE_END
