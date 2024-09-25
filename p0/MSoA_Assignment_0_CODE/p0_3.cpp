#include <iostream>
#include <math.h>

float triangleAreaFromVertices(float v[]){
	// Calculate and return the triangle area using the vertex coordinates in v[]
	float area = abs((v[0] * (v[3] - v[5]) + 
                      v[2] * (v[5] - v[1]) + 
                      v[4] * (v[1] - v[3])) / 2.0);
    return area;
}

int main() {
	// Now you can give the triangle the values on the code itself
	float v[] = {1.0f, 0.0f, 3.0f, 0.0f, 2.0f, 2.0f};
	
	
	std::cout << "The area of the triangle is " << triangleAreaFromVertices(v) << std::endl;
		
	/* Once you tested the example above, create a new array of nine floats, 
	and try to assign the six coordinates of the triangle vertices, the triangle area, 
	the number of vertices of the triangle, and a string with the triangle name (e.g., “Triangle”) */ 
	float extendedTriangle[9];
    
    // Asignar las coordenadas de los vértices (x1, y1), (x2, y2), (x3, y3)
    for (int i = 0; i < 6; ++i) {
        extendedTriangle[i] = v[i];
    }

    // Calcular el área del triángulo y asignarlo
    extendedTriangle[6] = triangleAreaFromVertices(v);
    // Asignar el número de vértices (3)
    extendedTriangle[7] = 3;
	
    return 0;
}