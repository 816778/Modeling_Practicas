/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    v1 - Dec 2020
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


#include <nori/mesh.h>
#include <nori/bbox.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

Mesh::Mesh() { }

Mesh::~Mesh() {
    m_pdf.clear();
    delete m_bsdf;
    delete m_emitter;
}

void Mesh::activate() {
    if (!m_bsdf) {
        /* If no material was assigned, instantiate a diffuse BRDF */
        m_bsdf = static_cast<BSDF *>(
            NoriObjectFactory::createInstance("diffuse", PropertyList()));
    }

    m_pdf.reserve(m_F.cols());

    for (uint32_t i = 0; i < m_F.cols(); ++i) { // num triángulos en la malla
        float area = surfaceArea(i);
        m_pdf.append(area);
    }

    m_pdf.normalize();
}

float Mesh::surfaceArea(n_UINT index) const {
    n_UINT i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);

    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

    return 0.5f * Vector3f((p1 - p0).cross(p2 - p0)).norm();
}

bool Mesh::rayIntersect(n_UINT index, const Ray3f &ray, float &u, float &v, float &t) const {
    n_UINT i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);
    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

    /* Find vectors for two edges sharing v[0] */
    Vector3f edge1 = p1 - p0, edge2 = p2 - p0;

    /* Begin calculating determinant - also used to calculate U parameter */
    Vector3f pvec = ray.d.cross(edge2);

    /* If determinant is near zero, ray lies in plane of triangle */
    float det = edge1.dot(pvec);

    if (det > -1e-8f && det < 1e-8f)
        return false;
    float inv_det = 1.0f / det;

    /* Calculate distance from v[0] to ray origin */
    Vector3f tvec = ray.o - p0;

    /* Calculate U parameter and test bounds */
    u = tvec.dot(pvec) * inv_det;
    if (u < 0.0 || u > 1.0)
        return false;

    /* Prepare to test V parameter */
    Vector3f qvec = tvec.cross(edge1);

    /* Calculate V parameter and test bounds */
    v = ray.d.dot(qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0)
        return false;

    /* Ray intersects triangle -> compute t */
    t = edge2.dot(qvec) * inv_det;

    return t >= ray.mint && t <= ray.maxt;
}

BoundingBox3f Mesh::getBoundingBox(n_UINT index) const {
    BoundingBox3f result(m_V.col(m_F(0, index)));
    result.expandBy(m_V.col(m_F(1, index)));
    result.expandBy(m_V.col(m_F(2, index)));
    return result;
}

Point3f Mesh::getCentroid(n_UINT index) const {
    return (1.0f / 3.0f) *
        (m_V.col(m_F(0, index)) +
         m_V.col(m_F(1, index)) +
         m_V.col(m_F(2, index)));
}

/**
 * \brief Uniformly sample a position on the mesh with
 * respect to surface area. Returns both position and normal
 */
void Mesh::samplePosition(const Point2f &sample, Point3f &p, Normal3f &n, Point2f &uv) const
{
    float pdfTriangle;
    float sampleValue = sample.x();
    size_t triangleIdx = m_pdf.sampleReuse(sampleValue, pdfTriangle); //índice del triángulo de la malla seleccionado

    //Vértices del triángulo
	n_UINT i0 = m_F(0, triangleIdx), i1 = m_F(1, triangleIdx), i2 = m_F(2, triangleIdx);
    const Point3f &v0 = m_V.col(i0);
    const Point3f &v1 = m_V.col(i1);
    const Point3f &v2 = m_V.col(i2);

    // elegir un punto aleatorio dentro del triángulo
    Point2f barycentric = Warp::squareToUniformTriangle(sample);

    // Transforma usando las coordenadas baricéntricas
    p = (1.0f - barycentric.x() - barycentric.y()) * v0 + barycentric.x() * v1 + barycentric.y() * v2;
    // 5. Interpola la normal 'n' (si las normales están disponibles)
    if (m_N.size() > 0) {
        const Normal3f &n0 = m_N.col(i0);
        const Normal3f &n1 = m_N.col(i1);
        const Normal3f &n2 = m_N.col(i2);
        n = (1.0f - barycentric.x() - barycentric.y()) * n0 + barycentric.x() * n1 + barycentric.y() * n2;
    } else {
        // Si no hay normales, usa la normal geométrica del triángulo
        n = (v1 - v0).cross(v2 - v0).normalized();
    }
    n.normalize();

    // 6. Interpola las coordenadas UV (si están presentes)
    if (m_UV.size() > 0) {
        const Point2f &uv0 = m_UV.col(i0);
        const Point2f &uv1 = m_UV.col(i1);
        const Point2f &uv2 = m_UV.col(i2);
        uv = (1.0f - barycentric.x() - barycentric.y()) * uv0 + barycentric.x() * uv1 + barycentric.y() * uv2;
    } else {
        // Si no hay UVs, devuelve (0, 0)
        uv = Point2f(0.0f, 0.0f);
    }
}

/// Return the surface area of the given triangle
float Mesh::pdf(const Point3f &p) const
{
	for (uint32_t i = 0; i < m_F.cols(); ++i) {
        // bounding box del triángulo
        BoundingBox3f bbox = getBoundingBox(i);


        if (bbox.contains(p)) {
            float areaTotal = m_pdf.getNormalization();
                
                // cout << this->toString();
                // La PDF es el área del triángulo dividido por el área total
            return 1.0f / areaTotal;

            // Si el punto está dentro de la bounding box del triángulo, verificarlo:
            n_UINT i0 = m_F(0, i), i1 = m_F(1, i), i2 = m_F(2, i);
            const Point3f &v0 = m_V.col(i0);
            const Point3f &v1 = m_V.col(i1);
            const Point3f &v2 = m_V.col(i2);

            Vector3f edge0 = v1 - v0;
            Vector3f edge1 = v2 - v0;
            Vector3f vp = p - v0;

            float d00 = edge0.dot(edge0);
            float d01 = edge0.dot(edge1);
            float d11 = edge1.dot(edge1);
            float d20 = vp.dot(edge0);
            float d21 = vp.dot(edge1);
            float denom = d00 * d11 - d01 * d01;

            float v = (d11 * d20 - d01 * d21) / denom;
            float w = (d00 * d21 - d01 * d20) / denom;
            float u = 1.0f - v - w;

            if (u >= 0 && v >= 0 && w >= 0) {
                // float areaTriangle = surfaceArea(i);

                // Obtener el área total de la malla desde el DiscretePDF
                float areaTotal = m_pdf.getNormalization();
                
                // cout << this->toString();
                // La PDF es el área del triángulo dividido por el área total
                return 1.0f / areaTotal;
            }
        }
    }
	
	return 0.;
}


void Mesh::addChild(NoriObject *obj, const std::string& name) {
    switch (obj->getClassType()) {
        case EBSDF:
            if (m_bsdf)
                throw NoriException(
                    "Mesh: tried to register multiple BSDF instances!");
            m_bsdf = static_cast<BSDF *>(obj);
            break;

        case EEmitter: {
                Emitter *emitter = static_cast<Emitter *>(obj);
                if (m_emitter)
                    throw NoriException(
                        "Mesh: tried to register multiple Emitter instances!");
                m_emitter = emitter;
            }
            break;

        default:
            throw NoriException("Mesh::addChild(<%s>) is not supported!",
                                classTypeName(obj->getClassType()));
    }
}

std::string Mesh::toString() const {
    return tfm::format(
        "Mesh[\n"
        "  name = \"%s\",\n"
        "  vertexCount = %i,\n"
        "  triangleCount = %i,\n"
        "  bsdf = %s,\n"
        "  emitter = %s\n"
        "]\n",
        m_name,
        m_V.cols(),
        m_F.cols(),
        m_bsdf ? indent(m_bsdf->toString()) : std::string("null"),
        m_emitter ? indent(m_emitter->toString()) : std::string("null")
    );
}

std::string Intersection::toString() const {
    if (!mesh)
        return "Intersection[invalid]";

    return tfm::format(
        "Intersection[\n"
        "  p = %s,\n"
        "  t = %f,\n"
        "  uv = %s,\n"
        "  shFrame = %s,\n"
        "  geoFrame = %s,\n"
        "  mesh = %s\n"
        "]",
        p.toString(),
        t,
        uv.toString(),
        indent(shFrame.toString()),
        indent(geoFrame.toString()),
        mesh ? mesh->toString() : std::string("null")
    );
}

NORI_NAMESPACE_END
