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

int attitude_step_kinematic(quaternion_t * attitude, void * gyr_sensors,
                            uint32_t time_step_ms)
{
    double w1, w2, w3; /* coordenadas de velocidad angular medida [rad/s] */
    double t, q[4];
    double norm_q;

    if(time_step_ms <= 0 || time_step_ms >= MAX_TIME_STEP)
    {
        /* no es posible usar el paso de integracion establecido. */
        return -1;
    }

    leer_gyros(gyr_sensors, &w1, &w2, &w3);

    t    = ((double) time_step_ms) / 1000.0;
    q[1] = attitude->q1;
    q[2] = attitude->q2;
    q[3] = attitude->q3;
    q[0] = attitude->q0;

    /* propagacion de actitud */
    attitude->q1 += 0.5 * (q[0] * w1 - q[3] * w2 + q[2] * w3) * t;
    attitude->q2 += 0.5 * (q[3] * w1 + q[0] * w2 - q[1] * w3) * t;
    attitude->q3 += 0.5 * (-q[2] * w1 + q[1] * w2 + q[0] * w3) * t;
    attitude->q0 += -0.5 * (q[1] * w1 + q[2] * w2 + q[3] * w3) * t;

    /* cuaternion propagado bajo la restriccion de unitario*/
    q[1]   = attitude->q1;
    q[2]   = attitude->q2;
    q[3]   = attitude->q3;
    q[0]   = attitude->q0;
    norm_q = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

    attitude->q1 /= norm_q;
    attitude->q2 /= norm_q;
    attitude->q3 /= norm_q;
    attitude->q0 /= norm_q;

    return 0;
}
