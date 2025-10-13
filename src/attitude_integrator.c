#include <math.h>
#include "attitude_integrator.h"
#include "gyroscopes.h"

void attitude_init(quaternion_t * q)
{
    q->q1 = 0.0;
    q->q2 = 0.0;
    q->q3 = 0.0;
    q->q0 = 1.0;
}

void attitude_step_kinematic(quaternion_t * attitude, void * gyr_sensors,
                             uint16_t time_step_ms)
{
    double w1, w2, w3; /* coordenadas de velocidad angular medida [rad/s] */
    double t;

    leer_gyros(gyr_sensors, &w1, &w2, &w3);

    t = (double) (time_step_ms / 1000);

    attitude->q1 += 0.5 * w1 * t;
    attitude->q2 += 0.5 * w2 * t;
    attitude->q3 += 0.5 * w3 * t;
    attitude->q0 = 1.0;
}
