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
    double t;

    if(time_step_ms <= 0 || time_step_ms >= MAX_TIME_STEP)
    {
        /* no es posible usar el paso de integracion establecido. */
        return -1;
    }

    leer_gyros(gyr_sensors, &w1, &w2, &w3);

    t = (double) (time_step_ms / 1000);

    attitude->q1 += 0.5 * w1 * t;
    attitude->q2 += 0.5 * w2 * t;
    attitude->q3 += 0.5 * w3 * t;
    attitude->q0 = 1.0;

    return 0;
}
