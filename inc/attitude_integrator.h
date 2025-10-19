#ifndef ATTITUDE_INTEGRATOR_H
#define ATTITUDE_INTEGRATOR_H

#include "stdint.h"
#include "math.h"

/** @brief Componente de cuaternion con valor erroneo.
 *
 * @details Las componentes del cuaternion unitario no pueden superar en norma
 * el valor 1. El número asignado para un valor erroneo debe superar en norma
 * a 1.
 */
#define QUAT_VALUE_ERROR (-2)

/** @brief Maximo paso de integracion permitido en milisegundos: 1 dia */
#define MAX_TIME_STEP (24 * 60 * 60 * 1000)

/** @brief Maximo valor tolerado de entrada de dato de giroscopo [rad/s]. */
#define W_MAXRANGE (3.1415 * 0.5)

/**
 * @brief Estructura de un cuaternión unitario.
 *
 * @details Un cuaternión es un objeto matemático de dimensión 4.
 * El cuaternión unitario, es decir de norma 1 (norm(q) == 1),
 * es una representación muy utilizada para definir la actitud
 * (orientación) de un cuerpo en el espacio. Otras representaciones de
 * orientación, más conocidas, son los ángulos de Euler y la matriz de
 * cosenos directores, sin embargo para ciertos tipos de aplicaciones es
 * conveniente el uso de cuaterniones.
 *
 * @see
 * https://en-wikipedia-org.translate.goog/wiki/Quaternions_and_spatial_rotation?_x_tr_sl=en&_x_tr_tl=es&_x_tr_hl=es&_x_tr_pto=tc
 */
typedef struct
{
    double q1; /**< 1er componente vectorial del cuaternión. */
    double q2; /**< 2da componente vectorial del cuaternión. */
    double q3; /**< 3er componente vectorial del cuaternión. */
    double q0; /**< componente escalar del cuaternión. */
} quaternion_t;

/**
 * @brief Orientacion (actitud) inicial en cuaternion.
 *
 * @param q Referencia a cuaternion para darle los valores de orientación
 * inicial.
 */
void attitude_init(quaternion_t * q);

/** @brief Actualizacion de la orientacion (actitud) usando datos de giróscopos
 * y paso de integracion.
 *
 * @param attitude Cuaternion de actitud (orientacion actual).
 * @param gyr_sensors Direccion a driver de giróscopos.
 * @param time_step_ms Paso de integracion, en milisegundos.
 *
 * @return Retorna 0 si no hay error, sino -1.
 */
int attitude_step_kinematic(quaternion_t * attitude, void * gyr_sensors,
                            uint32_t time_step_ms);

/** @brief Leer componente i del cuaternion.
 *
 * @param q cuaternion.
 * @param i componente: 0, 1, 2 o 3.
 * @return Componente que lee del cuaternion recibido.
 */
static inline double attitude_get_component(const quaternion_t * q, uint8_t i)
{
    double qi;

    switch(i)
    {
        case 0:
            qi = q->q0;
            break;
        case 1:
            qi = q->q1;
            break;
        case 2:
            qi = q->q2;
            break;
        case 3:
            qi = q->q3;
            break;
        default:
            qi = QUAT_VALUE_ERROR;
            break;
    }
    return qi;
}

#endif
