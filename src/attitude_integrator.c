#include "attitude_integrator.h"
#include <math.h>

void attitude_init(quaternion_t * q)
{
    q->q1 = 0.0;
    q->q2 = 0.0;
    q->q3 = 0.0;
    q->q0 = 1.0;
}
