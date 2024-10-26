/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
	
	v1 - Dec 01 2020
	Copyright (c) 2020 by Adrian Jarabo

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

#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/mesh.h>
#include <nori/texture.h>

NORI_NAMESPACE_BEGIN

class AreaEmitter : public Emitter {
public:
	AreaEmitter(const PropertyList &props) {
		m_type = EmitterType::EMITTER_AREA;
		m_radiance = new ConstantSpectrumTexture(props.getColor("radiance", Color3f(1.f)));
		m_scale = props.getFloat("scale", 1.);
	}

	virtual std::string toString() const {
		return tfm::format(
			"AreaLight[\n"
			"  radiance = %s,\n"
			"  scale = %f,\n"
			"]",
			m_radiance->toString(), m_scale);
	}

	// We don't assume anything about the visibility of points specified in 'ref' and 'p' in the EmitterQueryRecord.
	virtual Color3f eval(const EmitterQueryRecord & lRec) const {
		if (!m_mesh)
			throw NoriException("There is no shape attached to this Area light!");

		if (!m_radiance) {
			throw NoriException("AreaEmitter: m_radiance is null. It has not been initialized!");
		}
		// This function call can be done by bsdf sampling routines.
		// Hence the ray was already traced for us - i.e a visibility test was already performed.
		// Hence just check if the associated normal in emitter query record and incoming direction are not backfacing
		float cosTheta = lRec.n.dot(lRec.wi);  // lRec.n es la normal de la superficie
		if (cosTheta <= 0) {
			return Color3f(0.0f);  // No hay radiancia si estamos viendo el lado atrás de la luz
		}
		if (m_radiance) {
			return m_radiance->eval(lRec.uv) * m_scale;  // Escalamos la radiancia si es necesario
		}
		// Si no hay textura, devolvemos la radiancia constante
    	return m_radiance->eval(lRec.uv) * m_scale;
	}

	virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample, float optional_u) const {
		if (!m_mesh)
			throw NoriException("There is no shape attached to this Area light!");

		Point3f p;
		Normal3f n;
		Point2f uv;
		m_mesh->samplePosition(sample, p, n, uv);

		// Calcula la dirección hacia el punto de referencia
		lRec.p = p;              // Punto en el emisor
		lRec.n = n;              // Normal en el emisor
		lRec.uv = uv;            // Coordenadas UV
		lRec.wi = (lRec.ref - lRec.p).normalized();  // Dirección hacia el punto de referencia
		lRec.dist = (lRec.ref - lRec.p).norm();      // Distancia entre ref y p
		
		// está emitiendo hacia la dirección correcta
		float cosTheta = lRec.n.dot(lRec.wi);
		if (cosTheta <= 0) {
			return Color3f(0.0f);  // No hay radiancia si estamos viendo el lado trasero del emisor
		}

		// Evalúa la radiancia en la dirección de la muestra
		Color3f radiance = m_radiance->eval(lRec.uv) * m_scale;

		// Calcula la PDF en el ángulo sólido
		float pdfPos = m_mesh->pdf(lRec.p);  // PDF de la posición
		lRec.pdf = pdfPos * (lRec.dist * lRec.dist) / std::abs(cosTheta);  // Convertir a PDF en ángulo sólido
		// std::cout << radiance.toString() << endl;
		// Devuelve la radiancia emitida
		return radiance;
	}

	// Returns probability with respect to solid angle given by all the information inside the emitterqueryrecord.
	// Assumes all information about the intersection point is already provided inside.
	// WARNING: Use with care. Malformed EmitterQueryRecords can result in undefined behavior. 
	//			Plus no visibility is considered.
	virtual float pdf(const EmitterQueryRecord &lRec) const {
		if (!m_mesh)
			throw NoriException("There is no shape attached to this Area light!");

		float pdfSurface = m_mesh->pdf(lRec.p);
		float dist2 = (lRec.ref - lRec.p).squaredNorm();
		float cosTheta = lRec.n.dot(lRec.wi);
		// float cosTheta = std::fabs(lRec.n.dot(-lRec.wi));
		/*std::cout << "[DEBUG] pdfSurface: " << pdfSurface << std::endl;
		std::cout << "[DEBUG] dist2: " << dist2 << std::endl;
		std::cout << "[DEBUG] cosTheta: " << cosTheta << std::endl;
		std::cout << "[DEBUG] lRec.n: " << lRec.n.transpose() << std::endl;
    	std::cout << "[DEBUG] -lRec.wi: " << (-lRec.wi).transpose() << std::endl;
		*/
		 if (cosTheta <= 0.0f) {
			return 0.0f;  // Esto significa que la luz no es visible desde el punto de vista de la cámara
		}
		return pdfSurface * dist2 / cosTheta;
	}


	// Get the parent mesh
	void setParent(NoriObject *parent)
	{
		auto type = parent->getClassType();
		if (type == EMesh)
			m_mesh = static_cast<Mesh*>(parent);
	}

	// Set children
	void addChild(NoriObject* obj, const std::string& name = "none") {
		switch (obj->getClassType()) {
		case ETexture:
			if (name == "radiance")
			{
				delete m_radiance;
				m_radiance = static_cast<Texture*>(obj);
			}
			else
				throw NoriException("AreaEmitter::addChild(<%s>,%s) is not supported!",
					classTypeName(obj->getClassType()), name);
			break;

		default:
			throw NoriException("AreaEmitter::addChild(<%s>) is not supported!",
				classTypeName(obj->getClassType()));
		}
	}
protected:
	Texture* m_radiance;
	float m_scale;
};

NORI_REGISTER_CLASS(AreaEmitter, "area")
NORI_NAMESPACE_END
