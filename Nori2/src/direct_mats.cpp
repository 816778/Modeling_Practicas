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

class DirectMaterialSampling : public Integrator {
public:
    DirectMaterialSampling(const PropertyList &props) {
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
         * its.p = punto de intersección
         * its.shFrame.n = normal en el punto de intersección
         * its.mesh = malla en la que se encuentra el punto de intersección
         * its.uv = coordenadas UV del punto de intersección en la superficie
         * its.t = distancia desde el origen del rayo hasta el punto de intersección
         */
        /**
         * ray.o = origen del rayo
         * ray.d = dirección del rayo apunta al punto de intersección
         */

        /**
         * Si es el emisor tener en cuenta: Le(x,wo)
         */
        if (its.mesh->isEmitter()) {
            // Crear un EmitterQueryRecord y establecer sus campos manualmente
            EmitterQueryRecord eRec(its.p);
            /**
             * eRec.ref = punto donde queremos calcular efecto de luz
             * eRec.p = Este punto se encuentra en la superficie del emisor
             * eRec.n = Normal en el punto de intersección (luz)
             * eRec.uv = Coordenadas UV del punto de intersección en la superficie
             * eRec.pdf = Densidad de probabilidad de muestreo del emisor
             * eRec.wi = Dirección desde ref hacia p
             */
            //eRec.ref = ray.o;                               
            eRec.wi = ray.d.normalized();                      
            eRec.n = its.shFrame.n; 
            Lo += its.mesh->getEmitter()->eval(eRec);
        }


        /**
         * Muestra una nueva dirección usando la BRDF del material en el punto de intersección
         */
        const BSDF *bsdf = its.mesh->getBSDF();
        BSDFQueryRecord bsdfRec(its.toLocal(-ray.d));
        bsdfRec.uv = its.uv;
        bsdfRec.measure = ESolidAngle;

        Color3f bsdfSample = bsdf->sample(bsdfRec, sampler->next2D());
        //std::cout << "BSDF Sample: " << bsdfSample.toString() << std::endl;
        if (bsdfSample.isZero()) {
            return Lo;  // Si no hay contribución de la BRDF, retorna solo la radiancia del emisor
        }

        Vector3f woWorld = its.toWorld(bsdfRec.wo);
        float cosTheta = std::max(0.0f, its.shFrame.n.dot(woWorld));

        Ray3f shadowRay(its.p, woWorld);
        Intersection lightIts;
        if (scene->rayIntersect(shadowRay, lightIts) && lightIts.mesh->isEmitter()) {
            const Emitter *emitter = lightIts.mesh->getEmitter();
            EmitterQueryRecord lRec(its.p);
            lRec.p = lightIts.p;
            lRec.n = lightIts.shFrame.n;
            const Emitter* lightEmitter = lightIts.mesh->getEmitter();
            Color3f Le = lightEmitter->eval(lRec);

            float pdf = bsdf->pdf(bsdfRec);

            if (pdf > 0.0f) {
                Lo += (Le * bsdfSample * cosTheta) / pdf;
            }
        }

        return Lo;
    }

    std::string toString() const {
        return "DirectEmsIntegrator []";
    }

};

NORI_REGISTER_CLASS(DirectMaterialSampling, "direct_mats");
NORI_NAMESPACE_END
