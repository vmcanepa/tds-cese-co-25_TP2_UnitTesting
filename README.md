# Trabajo Práctico Número 2: Test Unitario

**NOTA: soy alumno externo y no cuento con trabajo final de especialización.**

Este proyecto implementa un integrador cinemático de cuaterniones para estimar la orientación de un cuerpo rígido (satélite) a partir de mediciones de velocidad angular provenientes de giróscopos.

Un cuaternión `q = [q1, q2, q3, q0]` es una forma muy habitual para representar la orientación sin singularidades como las de los ángulos de Euler. El cuaternión cumple la restricción de norma unitaria: |q| = 1.

La ecuación diferencial de la cinemática de actitud, en función de la velocidad angular `w = [wx, wy, wz]`, es:

q_dot = 0.5 * E(q) * w

donde `E(q)` es la matriz construida a partir del cuaternión actual.

El integrador implementa el método de Euler para propagar la orientación:

q_{k+1} := q_k + 0.5 * E(q_k) * w_k * dt

y posteriormente normaliza el resultado para mantener `|q| = 1`.

**Palabras clave:** cuaternión, actitud, integración numérica, giroscopio, navegación inercial.

## Uso del repositorio

Este repositorio utiliza [pre-commit](https://pre-commit.com) para validaciones de formato. Para trabajar con el mismo usted debería tener instalado:

1. pre-commit (https://pre-commit.com/#install)

Después de clonar el repositorio usted debería ejecutar el siguiente comando:

```
pre-commit install
```

Para generar la documentación del proyecto se utiliza el siguiente comando:

```
make doc

```
Para `compilar` el proyecto se utiliza el siguiente comando:

```
make all
```

## License

This work is distributed under the terms of the [MIT](https://spdx.org/licenses/MIT.html) license.
