#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/texture.h>

NORI_NAMESPACE_BEGIN

class Normalmap : public BSDF {
public:
    Normalmap(const PropertyList &propList) {}

    void activate() {
        if (!m_texture)
            throw NoriException("Normalmap BSDF: no normalmap provided!");
        if (!m_baseBSDF)
            throw NoriException("Normalmap BSDF: no base BSDF provided!");
    }

    /// Perturba el vector normal usando la textura
    Vector3f perturbNormal(const Vector2f& uv, const Vector3f& baseNormal,
                           const Vector3f& dpdu) const {
        Color3f c = m_texture->eval(uv);
        Vector3f normalTex(2.0f * c.x() - 1.0f, 2.0f * c.y() - 1.0f, 2.0f * c.z() - 1.0f);
        
        // Marco local original
        Vector3f n = baseNormal;
        Vector3f s = (dpdu - n * n.dot(dpdu)).normalized();
        Vector3f t = n.cross(s);

        // Transformar la normal de la textura al espacio mundial
        Vector3f perturbedNormal = (normalTex.x() * s + normalTex.y() * t + normalTex.z() * n).normalized();
        return perturbedNormal;
    }

    Color3f eval(const BSDFQueryRecord &bRec) const override {
        Vector3f baseNormal = Vector3f(0.0f, 0.0f, 1.0f); // plano por defecto
        Vector3f dpdu = Vector3f(1.0f, 0.0f, 0.0f);       // derivada en u

        Vector3f perturbedN = perturbNormal(bRec.uv, baseNormal, dpdu);
        Frame perturbedFrame(perturbedN);

        BSDFQueryRecord perturbedBRec(
            perturbedFrame.toLocal(Frame(baseNormal).toWorld(bRec.wi)),
            perturbedFrame.toLocal(Frame(baseNormal).toWorld(bRec.wo)),
            bRec.uv,
            bRec.measure
        );

        if (Frame::cosTheta(bRec.wo) * Frame::cosTheta(perturbedBRec.wo) <= 0)
            return Color3f(0.0f);

        return m_baseBSDF->eval(perturbedBRec) / Frame::cosTheta(bRec.wo) * Frame::cosTheta(perturbedBRec.wo);
    }

    float pdf(const BSDFQueryRecord &bRec) const override {
        Vector3f baseNormal = Vector3f(0.0f, 0.0f, 1.0f);
        Vector3f dpdu = Vector3f(1.0f, 0.0f, 0.0f);

        Vector3f perturbedN = perturbNormal(bRec.uv, baseNormal, dpdu);
        Frame perturbedFrame(perturbedN);

        BSDFQueryRecord perturbedBRec(
            perturbedFrame.toLocal(Frame(baseNormal).toWorld(bRec.wi)),
            perturbedFrame.toLocal(Frame(baseNormal).toWorld(bRec.wo)),
            bRec.uv,
            bRec.measure
        );

        if (Frame::cosTheta(bRec.wo) * Frame::cosTheta(perturbedBRec.wo) <= 0)
            return 0.0f;

        return m_baseBSDF->pdf(perturbedBRec);
    }

    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const override {
        Vector3f baseNormal = Vector3f(0.0f, 0.0f, 1.0f);
        Vector3f dpdu = Vector3f(1.0f, 0.0f, 0.0f);

        Vector3f perturbedN = perturbNormal(bRec.uv, baseNormal, dpdu);
        Frame perturbedFrame(perturbedN);

        BSDFQueryRecord perturbedBRec(
            perturbedFrame.toLocal(Frame(baseNormal).toWorld(bRec.wi)),
            bRec.uv
        );

        Color3f result = m_baseBSDF->sample(perturbedBRec, sample);

        if (!result.isZero()) {
            bRec.wo = Frame(baseNormal).toLocal(perturbedFrame.toWorld(perturbedBRec.wo));
            bRec.eta = perturbedBRec.eta;
            bRec.measure = perturbedBRec.measure;

            if (Frame::cosTheta(bRec.wo) * Frame::cosTheta(perturbedBRec.wo) <= 0)
                return Color3f(0.0f);
        }

        return result;
    }

    bool isDiffuse() const override {
        return m_baseBSDF->isDiffuse();
    }

    std::string toString() const override {
        return tfm::format(
            "Normalmap[\n"
            "  normalmap = %s\n"
            "  baseBSDF = %s\n"
            "]",
            m_texture ? m_texture->toString() : "null",
            m_baseBSDF ? m_baseBSDF->toString() : "null");
    }

    void addChild(NoriObject *obj)  {
        switch (obj->getClassType()) {
            case ETexture:
                if (m_texture)
                    throw NoriException("Normalmap: múltiples texturas no permitidas.");
                m_texture = static_cast<Texture *>(obj);
                break;
            case EBSDF:
                if (m_baseBSDF)
                    throw NoriException("Normalmap: múltiples BSDF base no permitidas.");
                m_baseBSDF = static_cast<BSDF *>(obj);
                break;
            default:
                throw NoriException("Normalmap::addChild(<%s>) no soportado!",
                                    classTypeName(obj->getClassType()));
        }
    }

private:
    const Texture *m_texture = nullptr;
    BSDF *m_baseBSDF = nullptr;
};

NORI_REGISTER_CLASS(Normalmap, "normalmap");

NORI_NAMESPACE_END
