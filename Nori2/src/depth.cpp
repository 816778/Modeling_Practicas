#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class DepthIntegrator : public Integrator {
public:
    DepthIntegrator(const PropertyList &props) {
        /* No se requiere ningún parámetro en este caso */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const override {
        Intersection its;
        
        // Verifica si el rayo intersecta con algún objeto en la escena
        if (!scene->rayIntersect(ray, its)) {
            // Si no hay intersección, retorna negro (sin profundidad)
            return Color3f(0.0f);
        }

        // Calcula la distancia desde el origen de la cámara hasta el punto de intersección
        float depth = its.t;

        // Para visualización, usaremos 1/d para que los objetos más cercanos sean más brillantes
        return Color3f(1.0f / depth);
    }

    std::string toString() const override {
        return "DepthIntegrator[]";
    }
};

NORI_REGISTER_CLASS(DepthIntegrator, "depth")
NORI_NAMESPACE_END
