#!/bin/bash

make ~/Descargas/Master/Modeling_and_Simulation_of_Appearance/Practicas/Nori2/build

./nori ../scenes/assignment-1/bunny-normals.xml


Duplicar el repositorio en local en linux

```bash
git clone https://github.com/816778/Modeling_Practicas.git
```

Entrar a la carpeta del repositorio build: 
```bash
cd Modeling_Practicas/Practicas/Nori2/build
```

Compilar el proyecto:
```bash
make
```

Ejecutar el proyecto:
```bash
./nori [ruta del archivo xml] e.g. ./nori ../scenes/assignment-1/bunny-normals.xml
```