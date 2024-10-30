# Monte Carlo Direct Illumination

**Authors:**

- Eryka Liced Rimacuna Castillo     816778
- Javier Franco Ramírez             799577


**Extras**

- Implementation of `Warp::squareToBeckmann` and `Warp::squareToBeckmannPdf`

- Implementation of `Warp::squareToUniformDisk` and `Warp::squareToUniformDiskPdf`

- Implementaton of Importance Sampling


**Importance Sampling**

Naïve Sampling: In this approach, each light source has an equal probability of being sampled, regardless of its brightness. This can lead to inefficient rendering because dim lights are sampled as frequently as bright lights, even though they contribute less to the final image. This often results in higher noise and slower convergence as we can see in the imagen `figures/2_/without_improvment/serapis_ems_1`

Radiance-Based Importance Sampling: In this approach, each light source is sampled proportionally to its radiance. Brighter lights have a higher probability of being chosen, which improves convergence by focusing sampling effort on lights that significantly impact the image. In comparation with the previous imagen, thios one shows less noise:
`figures/2_/important_sampling/serapis_ems_1`


**References**

- https://mathworld.wolfram.com/BarycentricCoordinates.html

- https://pbr-book.org/3ed-2018/contents



