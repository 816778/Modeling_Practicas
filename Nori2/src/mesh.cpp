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
    float meshArea = 0.0f;
    
    for (uint32_t i = 0; i < m_F.cols(); ++i) { // num triángulos en la malla
        float area = surfaceArea(i);
        //cout << "Area:" <<  area << endl;
        m_pdf.append(area);
    }

    m_pdf.normalize();
    //cout << "M_PDF:" +  m_pdf.toString();
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
    Point2f barycentric = Warp::squareToUniformTriangle(Point2f(sampleValue, sample.y()));


    const MatrixXu &F = getIndices();
    n_UINT idx0 = F(0, triangleIdx), idx1 = F(1, triangleIdx), idx2 = F(2, triangleIdx);
	const MatrixXf &V = getVertexPositions();
    const MatrixXf &N = getVertexNormals();
    const MatrixXf &UV = getVertexTexCoords();

    Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2); //punto dentro del triángulo
    // Transforma usando las coordenadas baricéntricas
    float bary0 = 1 - barycentric.x() - barycentric.y();
    float bary1 = barycentric.x();
    float bary2 = barycentric.y();
    // Compute the sampled position using barycentric coordinates
    p = bary0 * p0 + bary1 * p1 + bary2 * p2;

    if (N.size() > 0) {
        Normal3f n0 = N.col(idx0), n1 = N.col(idx1), n2 = N.col(idx2);
        n = (bary0 * n0 + bary1 * n1 + bary2 * n2).normalized();
    } else {
        n = (p1 - p0).cross(p2 - p0).normalized();
    }
    n.normalize();

    if (UV.size() > 0) {
        Point2f uv0 = UV.col(idx0), uv1 = UV.col(idx1), uv2 = UV.col(idx2);
        uv = bary0 * uv0 + bary1 * uv1 + bary2 * uv2;
    } else {
        uv = Point2f(0.0f, 0.0f);  // Default value if no UV coordinates are present
    }
}

/// Return the surface area of the given triangle
float Mesh::pdf(const Point3f &p) const
{
	return m_pdf.getNormalization();
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
