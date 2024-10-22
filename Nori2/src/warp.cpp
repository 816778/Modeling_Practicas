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

/*
transformar puntos que están uniformemente distribuidos en un cuadrado unitario [0,1]×[0,1] a puntos distribuidos uniformemente 
en un disco de radio 1 centrado en el origen (0,0)
*/
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

/*
Transforma puntos uniformemente distribuidos en un cuadrado unitario [0,1]×[0,1] a puntos distribuidos uniformemente en un triángulo.
*/
Point2f Warp::squareToUniformTriangle(const Point2f& sample) {
    float s = sample.x();
    float t = sample.y();
    
    if (s + t > 1.0f) {
        s = 1.0f - s;
        t = 1.0f - t;
    }
    
    return Point2f(s, t);
}

/*
calcula la función de densidad de probabilidad (PDF) para un punto p en un triángulo.
*/
float Warp::squareToUniformTrianglePdf(const Point2f& p) {
    Point2f A(0.0f, 0.0f);  // Vértice A
    Point2f B(1.0f, 0.0f);  // Vértice B
    Point2f C(0.0f, 1.0f);

    float area = 0.5f * std::abs(A.x() * (B.y() - C.y()) +
                                 B.x() * (C.y() - A.y()) +
                                 C.x() * (A.y() - B.y()));

    if (p.x() < 0.0f || p.y() < 0.0f || (p.x() + p.y()) > 1.0f) {
        return 0.0f;  // El punto está fuera del triángulo, PDF es 0
    }

    return 1.0f/area;
}

/*
transformar puntos que están uniformemente distribuidos en un cuadrado unitario  [0,1]×[0,1] 
en puntos uniformemente distribuidos en la superficie de una esfera en 3D
*/
Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    float z = 1.0f - 2.0f * sample.x(); 
    float r = std::sqrt(std::max(0.0f, 1.0f - z * z)); // Radius of the circle at z
    float phi = 2.0f * M_PI * sample.y();
    return Vector3f(r * std::cos(phi), r * std::sin(phi), z);   
}

/*
Calcula la función de densidad de probabilidad (PDF) para un punto v en la superficie de una esfera unitaria. 
*/
float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    if (std::abs(v.norm() - 1.0f) < 1e-6) { // Check if point lies on the sphere
        return 1.0f / (4.0f * M_PI); // Area of unit sphere is 4π
    }
    return 0.0f;
}

/*
transforma un punto uniformemente distribuido en un cuadrado unitario [0,1]×[0,1] en un punto 
uniformemente distribuido en la superficie de una hemisferio unitaria en 3D.
*/
Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    float z = 1.0f - 2.0f * sample.x(); // Map to [-1, 1]
    float r = std::sqrt(std::max(0.0f, 1.0f - z * z)); // Radius of the circle at z
    float phi = M_PI * sample.y();
    return Vector3f(r * std::cos(phi), r * std::sin(phi), z);   
}

/*
FIXME: calcular la función de densidad de probabilidad (PDF) para un punto v que se encuentra en la superficie de una hemisferio unitaria
*/
float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    if (v.z() < 0.0f || v.norm() != 1.0f) {
        return 0.0f;  
    }
    
    return 1.0f / (2.0f * M_PI);
}

/*
FIXME: A diferencia de la distribución uniforme, en una distribución cosenoidal, los puntos están más concentrados alrededor del eje z
*/
Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    float cosTheta = std::sqrt(sample.x());  // cos(θ) = sqrt(ξ_1)
    float sinTheta = std::sqrt(1.0f - cosTheta * cosTheta);  // sin(θ) = sqrt(1 - cos^2(θ))
    float phi = 2.0f * M_PI * sample.y();

    return Vector3f(sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta);
}


/*
Calcula la función de densidad de probabilidad (PDF) para un punto v en la superficie de una hemisferio unitaria, pero esta vez bajo una distribución cosenoidal.
*/
float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    if (v.z() < 0.0f || std::abs(v.norm() - 1.0f) > 1e-6) {
        return 0.0f;  
    }
    
    return v.z() / M_PI;    
}

/*
FIXME: mapea puntos uniformemente distribuidos en un cuadrado unitario [0,1]×[0,1] a una distribución de 
Beckmann, que es comúnmente utilizada en gráficos por computadora para describir la distribución de normales en superficies rugosas o microfacetadas
*/
Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    float phi = 2.0f * M_PI * sample.y();
    float theta = std::atan(alpha * std::sqrt(-std::log(1.0f - sample.x())));

    return Vector3f(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));
}

/*
calcula la función de densidad de probabilidad (PDF) para un vector normal m (de una microfaceta) generado según la distribución de Beckmann. 
*/
float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    if (m.z() <= 0.0f) {
        return 0.0f;  // El vector está fuera de la hemisferio superior
    }

    // Calcular tan^2(θ) = (1 - cos^2(θ)) / cos^2(θ)
    float cosTheta = m.z();
    float tanTheta2 = (1.0f - cosTheta * cosTheta) / (cosTheta * cosTheta);

    // Calcular la PDF de Beckmann
    float exponent = -tanTheta2 / (alpha * alpha);
    float beckmannPdf = std::exp(exponent) / (M_PI * alpha * alpha * std::pow(cosTheta, 3));

    return beckmannPdf;
}

NORI_NAMESPACE_END
