/* Testing unitario: integrador numérico de orientación de un CubeSat en función
 *                   de los datos de giróscopos.
 *
 * Test cases:
 * 0) La orientación inicial en el espacio se asigna con el cuaternión [0 0 0 1]
 *    (vector de dimensión 4).
 * 1) Si la lectura de la terna de giróscopos es nula, la orientación calculada
 *    no cambia respecto a la inicial [orientación q(t) == q(t_0), con t>t_0].
 * 2) El paso de tiempo dt para el integrador numérico debe ser mayor estricto
 *    que 0.
 * 3) Para un paso chico de tiempo (dt << 1 s), una orientación inicial conocida
 *    (q_0 := [0 0 0 1]) y una velocidad angular medida conocida con los datos
 *    de giróscopos (w1,w2,w3), la orientación integrada se aproxima a:
 *    q(dt+t0) == [0.5*w1*dt, 0.5*w2*dt, 0.5*w3*dt, 1].
 * 4) Usando dt := 1 seg y sensando la terna de gyros (w1,w2,w3):=(9,0,0) deg/s,
 *    luego de 10 pasos (10 segundos) la orientación iniciada q(t0):=[0 0 0 1]
 *    resulta q(10 s):=[sqrt(2)/2 0 0 sqrt(2)/2] (rotación de 90 deg en eje X).
 * 5) Repetir (4) pero ahora usando 5 pasos (dt := 2seg). El resultado debe ser
 *    aproximadamente cuando el tiempo final es 10 segundos.
 * 6) Repetir (4 o 5) pero luego integrar con datos opuestos de giróscopos
 *    [-w1,-w2,-w3] hasta llegar a los 20 segundos. El resultado final debe
 *    acercarse a la orientación inicial.
 * 7) Si la orientación inicial es [0 0 0 1], luego de algunos pasos hasta que
 *    la orientación final se aleja de la inicial, se debe conservar la
 *    condición norm(q) == 1. La orientación en el espacio se calcula dentro de
 *    la esfera de 4 dimensiones y norma 1.
 * 8) Si los datos de algún sensor están fuera de rango, el integrador retorna
 *    error.
 */

#include "unity.h"
#include "attitude_integrator.h"

static quaternion_t attitude;

void setUp(void)
{
    attitude_init(&attitude);
}

void tearDown(void)
{
}

void test_0_orientacion_inicial_por_defecto(void)
{
    quaternion_t attitude;

    attitude_init(&attitude);

    TEST_ASSERT_EQUAL_DOUBLE(0.0, attitude_get_component(&attitude, 1));
    TEST_ASSERT_EQUAL_DOUBLE(0.0, attitude_get_component(&attitude, 2));
    TEST_ASSERT_EQUAL_DOUBLE(0.0, attitude_get_component(&attitude, 3));
    TEST_ASSERT_EQUAL_DOUBLE(1.0, attitude_get_component(&attitude, 0));
}

void test_0b_obtener_componente_desde_0_hasta_3_del_cuaternion(void)
{
    int idx_fuera_de_rango = 4;

    TEST_ASSERT_EQUAL_DOUBLE(
        QUAT_VALUE_ERROR,
        attitude_get_component(&attitude, idx_fuera_de_rango));
}

/* TEST_ASSERT_DOUBLE_WITHIN(1.0E-1, 0.0, attitude_get_component(&attitude, 0));
 */
