/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

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

/*
Convierten muestras uniformes del cuadrado a diferentes formas y distribuciones para simular 
correctamente los efectos de iluminación y las propiedades de la superficie en una escena 3D.
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN
/*
Transforma puntos generados de manera uniforme en un cuadrado unitario de [0,1]×[0,1] a sí mismo
 */
Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}
/*
Calcula la función de densidad de probabilidad (PDF) para un punto dentro de un cuadrado unitario [0,1]×[0,1]
*/
float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}
/*
Transformar puntos uniformemente distribuidos en el cuadrado unitario [0,1]×[0,1] a una distribución en forma de "tent" en el mismo rango
[-,1]×[-,1]
Transforma las coordenadas de entrada para que sigan la forma de una pirámide centrada en el origen. 
La densidad de muestras es mayor en el centro y decrece hacia los bordes.
*/
Point2f Warp::squareToTent(const Point2f &sample) {
    auto inverseTent1D = [](float xi) -> float {
        if (xi < 0.5f)
            return -1.0f + std::sqrt(2.0f * xi);      
        else
            return 1.0f - std::sqrt(2.0f * (1.0f - xi));
    };
    float tentX = inverseTent1D(sample.x());
    float tentY = inverseTent1D(sample.y());

    return Point2f(tentX, tentY);
}
/*
Calcula la función de densidad de probabilidad (PDF) para un punto p dentro de la distribución tent en 2D.
evalúa qué tan probable es que un punto específico ocurra en la distribución tent. Devuelve un valor que indica 
la densidad en ese punto; los valores son mayores en el centro y disminuyen hacia los bordes.
*/
float Warp::squareToTentPdf(const Point2f &p) {
    auto tent1DPdf = [](float t) -> float {
        if (t >= -1.0f && t <= 1.0f)
            return 1.0f - std::abs(t); 
        else
            return 0.0f;  // Fuera de los límites -1 <= t <= 1
    };
    return tent1DPdf(p.x()) * tent1DPdf(p.y());
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float r = std::sqrt(sample.x());
    float theta = 2.0f * M_PI * sample.y();
    return Point2f(r * std::cos(theta), r * std::sin(theta));
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    if (p.x() * p.x() + p.y() * p.y() <= 1.0f) {
        return 1.0f / M_PI; 
    }
    return 0.0f;
}

Point2f Warp::squareToUniformTriangle(const Point2f& sample) {
    float s = sample.x();
    float t = sample.y();
    
    if (s + t > 1.0f) {
        s = 1.0f - s;
        t = 1.0f - t;
    }
    
    return Point2f(s, t);
}

float Warp::squareToUniformTrianglePdf(const Point2f& p) {
    if (p.x() >= 0.0f && p.y() >= 0.0f && (p.x() + p.y() <= 1.0f)) {
        return 1.0f; 
    }
    return 0.0f;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    float z = 1.0f - 2.0f * sample.x(); // Map to [-1, 1]
    float r = std::sqrt(std::max(0.0f, 1.0f - z * z)); // Radius of the circle at z
    float phi = 2.0f * M_PI * sample.y();
    return Vector3f(r * std::cos(phi), r * std::sin(phi), z);   
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    if (std::abs(v.norm() - 1.0f) < 1e-6) { // Check if point lies on the sphere
        return 1.0f / (4.0f * M_PI); // Area of unit sphere is 4π
    }
    return 0.0f;}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    float z = 1.0f - 2.0f * sample.x(); // Map to [-1, 1]
    float r = std::sqrt(std::max(0.0f, 1.0f - z * z)); // Radius of the circle at z
    float phi = M_PI * sample.y();
    return Vector3f(r * std::cos(phi), r * std::sin(phi), z);   }

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    throw NoriException("Warp::squareToUniformHemispherePdf() is not yet implemented!");
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    throw NoriException("Warp::squareToCosineHemisphere() is not yet implemented!");
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    throw NoriException("Warp::squareToCosineHemispherePdf() is not yet implemented!");
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    throw NoriException("Warp::squareToBeckmann() is not yet implemented!");
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    throw NoriException("Warp::squareToBeckmannPdf() is not yet implemented!");
}

NORI_NAMESPACE_END
