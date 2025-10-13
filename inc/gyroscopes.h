#ifndef GYROSCOPES_H
#define GYROSCOPES_H

/** @brief Índice de giróscopo en eje X: */
#define ID_GYRO_X 1

/** @brief Índice de giróscopo en eje Y: */
#define ID_GYRO_Y 2

/** @brief Índice de giróscopo en eje Z: */
#define ID_GYRO_Z 3

/** @brief Función que lee los registros de los sensores de giróscopos y parsea
 * a valores double con unidades en rad/s (unidad de lectura de velocidad
 * angular).
 *
 * @param gyros Puntero al driver de giróscopos.
 * @param w1: Dirección a variable con datos de velocidad angular de gyro X.
 * @param w2: Dirección a variable con datos de velocidad angular de gyro Y.
 * @param w3: Dirección a variable con datos de velocidad angular de gyro Z.
 *
 * @return Devuelve 0 si lee correctamente, sino 1.
 */
int leer_gyros(void * gyros, double * w1, double * w2, double * w3);

#endif
