/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
	
	v1 - Dec 01 2020
    v2 - Oct 30 2021
	Copyright (c) 2021 by Adrian Jarabo

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>
#include <nori/reflectance.h>
#include <nori/texture.h>

NORI_NAMESPACE_BEGIN

#define KS_THRES 0.

class RoughConductor : public BSDF {
public:
    RoughConductor(const PropertyList& propList) {
        /* RMS surface roughness */
        m_alpha = new ConstantSpectrumTexture(propList.getFloat("alpha", 0.1f));

        /* Reflectance at direction of normal incidence.
           To be used when defining the Fresnel term using the Schlick's approximation*/
        m_R0 = new ConstantSpectrumTexture(propList.getColor("R0", Color3f(0.5f)));

        alpha = propList.getFloat("alpha", 0.1f);
        /* Reflectance at direction of normal incidence.
           To be used when defining the Fresnel term using the Schlick's approximation*/
        R0 = propList.getColor("R0", Color3f(0.5f));
    }


    /// Evaluate the BRDF for the given pair of directions
    Color3f eval(const BSDFQueryRecord& bRec) const {
        /* This is a smooth BRDF -- return zero if the measure
        is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle || Frame::cosTheta(bRec.wi) <= 0 || Frame::cosTheta(bRec.wo) <= 0)
            return Color3f(0.0f);

        Vector3f wh = (bRec.wi + bRec.wo).normalized();
        // Reflectance::BeckmannNDF
        float D = Reflectance::BeckmannNDF(wh, alpha);
        //float F = Reflectance::fresnel(bRec.wi.dot(wh), /* extIOR */ 1.0f, /* intIOR */ 1.5f);
        
        Color3f F = Reflectance::fresnel(bRec.wi.dot(wh), R0);
        float G = Reflectance::G1(bRec.wi, wh, alpha) * Reflectance::G1(bRec.wo, wh, alpha);

        float cosThetaI = Frame::cosTheta(bRec.wi);
        float cosThetaO = Frame::cosTheta(bRec.wo);
        return (D * F * G) / (4.0f * cosThetaI * cosThetaO);
    }

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord& bRec) const {
        /* This is a smooth BRDF -- return zero if the measure
        is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return 0.0f;
        
        Vector3f wh = (bRec.wi + bRec.wo).normalized();
        float pdf_wh = Warp::squareToBeckmannPdf(wh, alpha);
        //return pdf_wh;

        return pdf_wh / (4.0f * std::abs(bRec.wi.dot(wh)));
    }

    /// Sample the BRDF
    Color3f sample(BSDFQueryRecord& bRec, const Point2f& sample) const {
        // Note: Once you have implemented the part that computes the scattered
        // direction, the last part of this function should simply return the
        // BRDF value divided by the solid angle density and multiplied by the
        // cosine factor from the reflection equation, i.e.
        // return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
        if (Frame::cosTheta(bRec.wi) <= 0)
            return Color3f(0.0f);

        bRec.measure = ESolidAngle;
        Vector3f wh = Warp::squareToBeckmann(sample, alpha); // `alpha` is the roughness parameter
        bRec.wo = 2.0f * wh.dot(bRec.wi) * wh - bRec.wi;

        if (Frame::cosTheta(bRec.wo) <= 0)
        return Color3f(0.0f);

        bRec.measure = ESolidAngle;

        // Calculate the weight
        float pdfVal = pdf(bRec);
        if (pdfVal == 0)
            return Color3f(0.0f);

        return eval(bRec) * Frame::cosTheta(bRec.wo) / pdfVal;
    }

    bool isDiffuse() const {
        /* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
        return true;
    }

    void addChild(NoriObject* obj, const std::string& name = "none") {
        switch (obj->getClassType()) {
        case ETexture:
            if (name == "R0")
            {
                delete m_R0;
                m_R0 = static_cast<Texture*>(obj);
            }
            else if (name == "alpha")
            {
                delete m_alpha;
                m_alpha = static_cast<Texture*>(obj);
            }
            else
                throw NoriException("RoughConductor::addChild(<%s>,%s) is not supported!",
                    classTypeName(obj->getClassType()), name);
            break;
        default:
            throw NoriException("RoughConductor::addChild(<%s>) is not supported!",
                classTypeName(obj->getClassType()));
        }
    }

    std::string toString() const {
        return tfm::format(
            "RoughConductor[\n"
            "  alpha = %f,\n"
            "  R0 = %s,\n"
            "]",
            m_alpha->toString(),
            m_R0->toString()
        );
    }
private:
    Texture* m_alpha;
    Texture* m_R0;
    float alpha;
    Color3f R0;
};


class RoughDielectric : public BSDF {
public:
    RoughDielectric(const PropertyList& propList) {
        /* RMS surface roughness */
        m_alpha = new ConstantSpectrumTexture(propList.getFloat("alpha", 0.1f));

        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);

        /* Tint of the glass, modeling its color */
        m_ka = new ConstantSpectrumTexture(propList.getColor("ka", Color3f(1.f)));

        alpha = propList.getFloat("alpha", 0.1f);
    }


    /// Evaluate the BRDF for the given pair of directions
    Color3f eval(const BSDFQueryRecord& bRec) const {
        /* This is a smooth BSDF -- return zero if the measure is wrong */
        if (bRec.measure != ESolidAngle)
            return Color3f(0.0f);


        throw NoriException("RoughDielectric::eval() is not yet implemented!");
    }

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord& bRec) const {
        /* This is a smooth BSDF -- return zero if the measure is wrong */
        if (bRec.measure != ESolidAngle)
            return 0.0f;

        throw NoriException("RoughDielectric::eval() is not yet implemented!");
    }

    /// Sample the BRDF
    Color3f sample(BSDFQueryRecord& bRec, const Point2f& _sample) const {
        // Note: Once you have implemented the part that computes the scattered
        // direction, the last part of this function should simply return the
        // BRDF value divided by the solid angle density and multiplied by the
        // cosine factor from the reflection equation, i.e.
        // return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
        bRec.measure = ESolidAngle;

        throw NoriException("RoughDielectric::sample() is not yet implemented!");
    }

    bool isDiffuse() const {
        /* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
        return true;
    }

    void addChild(NoriObject* obj, const std::string& name = "none") {
        switch (obj->getClassType()) {
        case ETexture:
            if (name == "m_ka")
            {
                delete m_ka;
                m_ka = static_cast<Texture*>(obj);
            }
            else if (name == "alpha")
            {
                delete m_alpha;
                m_alpha = static_cast<Texture*>(obj);
            }
            else
                throw NoriException("RoughDielectric::addChild(<%s>,%s) is not supported!",
                    classTypeName(obj->getClassType()), name);
            break;
        default:
            throw NoriException("RoughDielectric::addChild(<%s>) is not supported!",
                classTypeName(obj->getClassType()));
        }
    }

    std::string toString() const {
        return tfm::format(
            "RoughDielectric[\n"
            "  alpha = %f,\n"
            "  intIOR = %f,\n"
            "  extIOR = %f,\n"
            "  ka = %s,\n"
            "]",
            m_alpha->toString(),
            m_intIOR,
            m_extIOR,
            m_ka->toString()
        );
    }
private:
    float m_intIOR, m_extIOR;
    Texture* m_alpha;
    Texture* m_ka;
    float alpha;
};



class RoughSubstrate : public BSDF {
public:
    RoughSubstrate(const PropertyList &propList) {
        /* RMS surface roughness */
        m_alpha = new ConstantSpectrumTexture(propList.getFloat("alpha", 0.1f));

        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);

        /* Albedo of the diffuse base material (a.k.a "kd") */
        m_kd = new ConstantSpectrumTexture(propList.getColor("kd", Color3f(0.5f)));

        kd = propList.getColor("kd", Color3f(0.5f));

        alpha = propList.getFloat("alpha", 0.1f);
    }


    /// Evaluate the BRDF for the given pair of directions
    Color3f eval(const BSDFQueryRecord &bRec) const {
        /* This is a smooth BRDF -- return zero if the measure
        is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return Color3f(0.0f);
        Vector3f wh = (bRec.wi + bRec.wo).normalized();

        float cosThetaI = Frame::cosTheta(bRec.wi);
        float cosThetaO = Frame::cosTheta(bRec.wo);

        // Reflectance::BeckmannNDF
        float D = Reflectance::BeckmannNDF(wh, alpha);
        float F = Reflectance::fresnel(cosThetaI, m_extIOR, m_intIOR);
        float G = Reflectance::G1(bRec.wi, wh, alpha) * Reflectance::G1(bRec.wo, wh, alpha);

        Color3f Fmf = (D * F * G) / (4.0f * cosThetaI * cosThetaO);

        Color3f term1 = 28 * kd / (23 * M_PI);
        double eta_ratio = (m_extIOR - m_intIOR) / (m_extIOR + m_intIOR);
        double term2 = 1 - std::pow(eta_ratio, 2);

        double term3 = 1 - std::pow(1 - 0.5 * cosThetaI, 5);
        double term4 = 1 - std::pow(1 - 0.5 * cosThetaO, 5);

        Color3f Fdiff = term1 * term2 * term3 * term4;

        return Fmf + Fdiff;
	}

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord &bRec) const {
        /* This is a smooth BRDF -- return zero if the measure
       is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return 0.0f;

        Vector3f wh = (bRec.wi + bRec.wo).normalized();
        float F = Reflectance::fresnel(Frame::cosTheta(bRec.wi), m_extIOR, m_intIOR);

        // Probability of microfacet lobe (using Beckmann distribution)
        float microfacet_pdf = Warp::squareToBeckmannPdf(wh, alpha) / (4.0f * std::abs(bRec.wi.dot(wh)));

        // Probability of diffuse lobe (cosine-weighted hemisphere sampling)
        float diffuse_pdf = Frame::cosTheta(bRec.wo) / M_PI;

        // Combine based on Fresnel term
        return F * microfacet_pdf + (1 - F) * diffuse_pdf;
    }

    /// Sample the BRDF
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {
        // Note: Once you have implemented the part that computes the scattered
        // direction, the last part of this function should simply return the
        // BRDF value divided by the solid angle density and multiplied by the
        // cosine factor from the reflection equation, i.e.
        // return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
        if (Frame::cosTheta(bRec.wi) <= 0)
            return Color3f(0.0f);

        float F = Reflectance::fresnel(Frame::cosTheta(bRec.wi), m_extIOR, m_intIOR);

        if (_sample.x() < F) {
            // Sample microfacet lobe
            Point2f newSample(_sample.x() / F, _sample.y());
            Vector3f wh = Warp::squareToBeckmann(newSample, alpha);
            bRec.wo = 2.0f * bRec.wi.dot(wh) * wh - bRec.wi;
        } else {
            // Sample diffuse lobe (cosine-weighted hemisphere)
            Point2f newSample((_sample.x() - F) / (1 - F), _sample.y());
            bRec.wo = Warp::squareToCosineHemisphere(newSample);
        }

        if (Frame::cosTheta(bRec.wo) <= 0)
            return Color3f(0.0f);

        bRec.measure = ESolidAngle;

        // Compute PDF and return weighted BRDF value
        float pdfVal = pdf(bRec);
        if (pdfVal == 0)
            return Color3f(0.0f);

        return eval(bRec) * Frame::cosTheta(bRec.wo) / pdfVal;
	}

    bool isDiffuse() const {
        /* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
        return true;
    }

    void addChild(NoriObject* obj, const std::string& name = "none") {
        switch (obj->getClassType()) {
        case ETexture:
            if (name == "kd")
            {
                delete m_kd;
                m_kd = static_cast<Texture*>(obj);
            }
            else if (name == "alpha")
            {
                delete m_alpha;
                m_alpha = static_cast<Texture*>(obj);
            }
            else 
                throw NoriException("RoughSubstrate::addChild(<%s>,%s) is not supported!",
                    classTypeName(obj->getClassType()), name);
            break;
        default:
            throw NoriException("RoughSubstrate::addChild(<%s>) is not supported!",
                classTypeName(obj->getClassType()));
        }
    }

    std::string toString() const {
        return tfm::format(
            "RoughSubstrate[\n"
            "  alpha = %f,\n"
            "  intIOR = %f,\n"
            "  extIOR = %f,\n"
            "  kd = %s,\n"
            "]",
            m_alpha->toString(),
            m_intIOR,
            m_extIOR,
            m_kd->toString()
        );
    }
private:
    float m_intIOR, m_extIOR, alpha;
    Texture* m_alpha;
    Texture* m_kd;
    Color3f kd;
};

NORI_REGISTER_CLASS(RoughConductor, "roughconductor");
NORI_REGISTER_CLASS(RoughDielectric, "roughdielectric");
NORI_REGISTER_CLASS(RoughSubstrate, "roughsubstrate");

NORI_NAMESPACE_END
