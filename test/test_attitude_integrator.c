/* Testing unitario: integrador numérico de orientación de un CubeSat en función
 *                   de los datos de giróscopos.
 */

#include "unity.h"
#include "mock_gyroscopes.h"
#include "attitude_integrator.h"

static quaternion_t attitude;

void setUp(void)
{
    attitude_init(&attitude);
}

void tearDown(void)
{
}

/**
 * @brief La orientación inicial en el espacio se asigna con el cuaternión
 * [0 0 0 1] (vector de dimensión 4).
 */
void test_0_orientacion_inicial_por_defecto(void)
{
    quaternion_t attitude;

    attitude_init(&attitude);

    TEST_ASSERT_EQUAL_DOUBLE(0.0, attitude_get_component(&attitude, 1));
    TEST_ASSERT_EQUAL_DOUBLE(0.0, attitude_get_component(&attitude, 2));
    TEST_ASSERT_EQUAL_DOUBLE(0.0, attitude_get_component(&attitude, 3));
    TEST_ASSERT_EQUAL_DOUBLE(1.0, attitude_get_component(&attitude, 0));
}

/**
 * @brief Solo es posible leer índices 0 a 3 del cuaternion. Cualquier otro
 * índice debe interpretarse como error.
 */
void test_0b_obtener_componente_desde_0_hasta_3_del_cuaternion(void)
{
    int idx_fuera_de_rango = 4;

    TEST_ASSERT_EQUAL_DOUBLE(
        QUAT_VALUE_ERROR,
        attitude_get_component(&attitude, idx_fuera_de_rango));
}

/**
 *  @brief Si la lectura de la terna de giróscopos es nula, la orientación
 * calculada no cambia respecto a la inicial [orientación q(t) == q(t_0), con
 * t>t_0].
 */
void test_1_cuando_se_leen_gyros_con_valores_nulos_se_mantiene_constante_la_orientacion(
    void)
{
    double   x_velang = 0.0, y_velang = 0.0, z_velang = 0.0;
    double   q[4], qref[4];       /* cuaterniones */
    uint32_t time_step_ms = 1000; /* paso de 1 segundo */
    void *   gyr_sensors;         /* direccion a driver de gyros. */

    qref[0] = 0.0; /* referencia para comparacion en test */
    qref[1] = 0.0; /* referencia para comparacion en test */
    qref[2] = 0.0; /* referencia para comparacion en test */
    qref[3] = 1.0; /* referencia para comparacion en test */

    /* cmock: cuando se llame a leer_gyros() devolver OK */
    leer_gyros_ExpectAnyArgsAndReturn(0);
    /* cmock: se debe cargar 0 en la direccion de cada puntero: */
    leer_gyros_ReturnThruPtr_w1(&x_velang);
    leer_gyros_ReturnThruPtr_w2(&y_velang);
    leer_gyros_ReturnThruPtr_w3(&z_velang);

    /* paso de integracion: */
    attitude_step_kinematic(&attitude, &gyr_sensors, time_step_ms);

    /* guardar componentes de cuaternion propagado: */
    q[0] = attitude_get_component(&attitude, 1);
    q[1] = attitude_get_component(&attitude, 2);
    q[2] = attitude_get_component(&attitude, 3);
    q[3] = attitude_get_component(&attitude, 0);

    /* comparar: */
    TEST_ASSERT_DOUBLE_WITHIN(1.0E-3, qref[0], q[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1.0E-3, qref[1], q[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1.0E-3, qref[2], q[2]);
    TEST_ASSERT_DOUBLE_WITHIN(1.0E-3, qref[3], q[3]);
}

/**
 *  @brief El paso de tiempo dt para el integrador numérico debe ser mayor
 * estricto que 0.
 */
void test_2_argumento_dt_debe_ser_positivo(void)
{
    int      state;
    void *   gyr_sensors; /* direccion a driver de gyros. */
    uint32_t dt_ms;       /* paso en milisegundos */

    /* cmock: cuando se llame a leer_gyros() devolver OK */
    leer_gyros_ExpectAnyArgsAndReturn(0);
    dt_ms = 1; /* paso de 1 milisegundo */
    state = attitude_step_kinematic(&attitude, &gyr_sensors, dt_ms);
    TEST_ASSERT_EQUAL_INT(0, state);

    /* cmock: cuando se llame a leer_gyros() devolver OK */
    dt_ms = 0; /* paso NULO de 0 milisegundo (debe dar error) */
    state = attitude_step_kinematic(&attitude, &gyr_sensors, dt_ms);
    TEST_ASSERT_EQUAL_INT(-1, state);

    /* cmock: cuando se llame a leer_gyros() devolver OK */
    dt_ms = -1; /* paso NEGATIVO de -1 milisegundo (debe dar error) */
    state = attitude_step_kinematic(&attitude, &gyr_sensors, dt_ms);
    TEST_ASSERT_EQUAL_INT(-1, state);
}

/**
 *  @brief Para un paso chico de tiempo (dt << 1 s), una orientación inicial
 * conocida (q_0 := [0 0 0 1]) y una velocidad angular medida conocida con los
 * datos de giróscopos (w1,w2,w3), la orientación integrada se aproxima a:
 * q(dt+t0) == [0.5*w1*dt, 0.5*w2*dt, 0.5*w3*dt, 1].
 */
void test_3_paso_chico_de_integracion(void)
{
    uint32_t time_step_ms = 1000; /* paso de 1 segundo */
    double   t, x_velang, y_velang, z_velang;
    double   q[4], qref[4]; /* cuaterniones */
    void *   gyr_sensors;   /* direccion a driver de gyros. */

    /* cmock: cuando se llame a leer_gyros() devolver OK */
    leer_gyros_ExpectAnyArgsAndReturn(0);
    /* cmock: se debe cargar el valor preestablecido en la direccion de cada
     * puntero: */
    x_velang = 10.0 * 3.1415 / 180.0; /* rad/s */
    y_velang = 0.0 * 3.1415 / 180.0;  /* rad/s */
    z_velang = 0.0 * 3.1415 / 180.0;  /* rad/s */
    leer_gyros_ReturnThruPtr_w1(&x_velang);
    leer_gyros_ReturnThruPtr_w2(&y_velang);
    leer_gyros_ReturnThruPtr_w3(&z_velang);

    /* paso de integracion: */
    attitude_step_kinematic(&attitude, &gyr_sensors, time_step_ms);

    /* guardar componentes de cuaternion propagado: */
    q[0] = attitude_get_component(&attitude, 1);
    q[1] = attitude_get_component(&attitude, 2);
    q[2] = attitude_get_component(&attitude, 3);
    q[3] = attitude_get_component(&attitude, 0);

    /* comparar: */
    t = time_step_ms / 1000;
    TEST_ASSERT_DOUBLE_WITHIN(1.0E-3, 0.5 * x_velang * t, q[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1.0E-3, 0.5 * y_velang * t, q[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1.0E-3, 0.5 * z_velang * t, q[2]);
    TEST_ASSERT_DOUBLE_WITHIN(1.0E-3, 1.0, q[3]);
}

/* Test cases restantes:
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
