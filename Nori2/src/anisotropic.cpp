#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/reflectance.h>
#include <nori/warp.h>


NORI_NAMESPACE_BEGIN

template <typename T>
T clamp(T val, T minVal, T maxVal) {
    return (val < minVal) ? minVal : ((val > maxVal) ? maxVal : val);
}

template <typename T>
T Sqr(T v) { return v * v; }


float LengthSquared(const Vector3f& v) { return Sqr(v.x()) + Sqr(v.y()) + Sqr(v.z()); }

float AbsDot(const Vector3f& v1,const Vector3f& v2) { return std::abs(v1.dot(v2)); }

class AnisotropicMicrofacet  : public BSDF {
public:
    AnisotropicMicrofacet (const PropertyList &propList) {
        m_alphaU = propList.getFloat("alphaU", 0.2f);
        m_alphaV = propList.getFloat("alphaV", 0.4f);
        if (m_alphaU < 0.3f) m_alphaU = clamp(2 * m_alphaU, 0.1f, 0.3f);
        if (m_alphaV < 0.3f) m_alphaV = clamp(2 * m_alphaV, 0.1f, 0.3f);
        m_reflectance = propList.getColor("reflectance", Color3f(1.0f));
        m_eta = propList.getColor("eta", Color3f(0.17, 0.35, 1.5)); // ejemplo: oro ~ (0.17, 0.35, 1.5)
        m_k   = propList.getColor("k",   Color3f(3.1, 2.7, 1.9)); // ejemplo: oro ~ (3.1, 2.7, 1.9)
    }

    Color3f eval(const BSDFQueryRecord &bRec) const override {
        if (bRec.measure != ESolidAngle || Frame::cosTheta(bRec.wi) <= 0 || Frame::cosTheta(bRec.wo) <= 0)
        { return Color3f(0.0f);}

        // half vector
        Vector3f wm = (bRec.wi + bRec.wo).normalized();
        float Dval = D(wm);

        // Fresnel de Schlick
        float cosTheta = bRec.wo.dot(wm);
        Color3f F = fresnelConductor(cosTheta, m_eta, m_k);
        // Color3f F = m_reflectance + (Color3f(1.0f) - m_reflectance) * std::pow(1.0f - cosTheta, 5.0f);
        
        // Geometría: masking-shadowing
        float Gval = G(bRec.wi, bRec.wo); 

        float denom = 4.0f * Frame::cosTheta(bRec.wi) * Frame::cosTheta(bRec.wo);
        return (Dval * F * Gval) / denom;

    }

    float pdf(const BSDFQueryRecord &bRec) const override{
        // if (Frame::cosTheta(bRec.wi) <= 0.0f || Frame::cosTheta(bRec.wo) <= 0.0f){ return 0.0f;}

        Vector3f wm = (bRec.wi + bRec.wo).normalized();

        if (LengthSquared(wm) <= 0.0f){return 0.0f; }

        if (wm.dot(Vector3f(0, 0, 1)) < 0.0f) 
        {
            wm = -wm; 
        }

        float pdf = D_2(bRec.wo, wm);
        return pdf / (4.0f * AbsDot(bRec.wo, wm));

    }

    
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const override {
        if (Frame::cosTheta(bRec.wi) <= 0.0f)
            return Color3f(0.0f);

        // Transform w to hemispherical configuration
        Vector3f wi = bRec.wi;
        Vector3f wh = Vector3f(m_alphaU * wi.x(), m_alphaV * wi.y(), wi.z());
        wh = wh.normalized();
        if (wh.z() < 0.0f) {wh = -wh;}

        // Find orthonormal basis for visible normal sampling
        Vector3f T1 = (std::abs(wh.z()) < 0.999f) ? wh.cross(Vector3f(0, 0, 1)).normalized() : Vector3f(1, 0, 0);
        Vector3f T2 = wh.cross(T1);

        // Generate uniformly distributed points on the unit disk
        Point2f p = Warp::squareToUniformDisk(sample);

        // Warp hemispherical projection for visible normal sampling
        float h = std::sqrt(1 - Sqr(p.x()));
        p.y() = lerp((1 + wh.z()) / 2.0f, h, p.y()); 

        // Reproject to hemisphere and transform normal to ellipsoid configuration
        float pz = std::sqrt(std::max(0.0f, 1.0f - (Sqr(p.x()) + Sqr(p.y()))));
        Vector3f nh = p.x() * T1 + p.y() * T2 + pz * wh;
        Vector3f wm = Vector3f(m_alphaU * nh.x(), m_alphaV * nh.y(), std::max(1e-6f, nh.z())).normalized();

        // Reflect the incoming direction around the half vector
        Vector3f wo = reflect(wi, wm);

        if (Frame::cosTheta(wo) <= 0.0f){ return Color3f(0.0f);}

        bRec.wo = wo;
        bRec.measure = ESolidAngle;

        // Evaluate BRDF and return importance-weighted value
        float pdfVal = pdf(bRec);
        Color3f fr = eval(bRec);
        return (pdfVal > 0.0f) ? fr * Frame::cosTheta(wo) / pdfVal : Color3f(0.0f);
    }

    std::string toString() const {
        return tfm::format(
            "AnisotropicMicrofacet[\n"
            "  alphaU = %f,\n"
            "  alphaV = %f\n"
            "]",
            m_alphaU, m_alphaV);
    }
private:
    float m_alphaU, m_alphaV;
    Color3f m_reflectance;
    Color3f m_eta; // Índice real del material
    Color3f m_k;// Extinción

    Color3f fresnelConductor(float cosThetaI, const Color3f &eta, const Color3f &k) const {
        Color3f cos2ThetaI = Color3f(cosThetaI * cosThetaI);
        Color3f sin2ThetaI = Color3f(1.0f) - cos2ThetaI;

        Color3f eta2 = eta * eta;
        Color3f k2 = k * k;

        Color3f t0 = eta2 - k2 - sin2ThetaI;
        Color3f a2plusb2 = (t0 * t0 + 4.0f * eta2 * k2).sqrt();
        Color3f t1 = a2plusb2 + cos2ThetaI;
        Color3f a = ((a2plusb2 + t0) * 0.5f).sqrt();

        Color3f Rs = (a2plusb2 - 2.0f * a * cosThetaI + cos2ThetaI) / t1;
        Color3f Rp = Rs * ((a2plusb2 * cos2ThetaI - 2.0f * a * cosThetaI + 1.0f) / (a2plusb2 - cos2ThetaI));

        return 0.5f * (Rp + Rs);
    }


    float G(const Vector3f &wi, const Vector3f &wo) const {
        return 1 / (1 + Lambda(wi) + Lambda(wo));
    }

    float Lambda(const Vector3f &v) const {
        if (Frame::cosTheta(v) <= 0.0f)
            return 0.0f;

        float tanTheta = Frame::tanTheta(v);
        if (tanTheta == 0.0f)
            return 0.0f;

        float cosPhi2 = Frame::cosPhi2(v);
        float sinPhi2 = Frame::sinPhi2(v);

        float alpha2 = (cosPhi2 / (m_alphaU * m_alphaU)) + (sinPhi2 / (m_alphaV * m_alphaV));
        float alpha = std::sqrt(1.0f / alpha2);

        float a = 1.0f / (alpha * tanTheta);
        if (a < 1.6f)
            return (1.0f - 1.259f * a + 0.396f * a * a) / (3.535f * a + 2.181f * a * a);
        else
            return 0.0f;
    }

    float G1(const Vector3f &v) const {
        return 1.0f / (1.0f + Lambda(v));
    }

    float D_2(const Vector3f &w, Vector3f &wm) const{
        return G1(w) / std::abs(wm.z()) * D(wm) * AbsDot(w, wm);
    }
    float D(const Vector3f &m) const {
        float cosThetaM = Frame::cosTheta(m);
        float cosThetaM2 = cosThetaM * cosThetaM;
        float cosThetaM4 = cosThetaM2 * cosThetaM2;

        if (cosThetaM <= 0.0f)
            return 0.0f;

        float tanTheta  = Frame::tanTheta(m);
        float tanTheta2 = tanTheta * tanTheta;

        float cosPhi2 = Frame::cosPhi2(m);
        float sinPhi2 = Frame::sinPhi2(m);

        float alphaU2 = m_alphaU * m_alphaU;
        float alphaV2 = m_alphaV * m_alphaV;

        float exponent = (cosPhi2 / alphaU2 + sinPhi2 / alphaV2) * tanTheta2;
        float denom = M_PI * m_alphaU * m_alphaV * cosThetaM4 * std::pow(1.0f + exponent, 2.0f);

        return 1.0f / denom;
    }

    Vector3f reflect(const Vector3f &v, const Vector3f &n) const {
        return -v + 2.0f * v.dot(n) * n;
    }


};

NORI_REGISTER_CLASS(AnisotropicMicrofacet , "anisotropic");
NORI_NAMESPACE_END
