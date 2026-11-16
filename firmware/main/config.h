#ifndef CONFIG_H
#define CONFIG_H

// ======================================
// CONFIGURACAO DOS EIXOS
// ======================================

#define TOTAL_EIXOS 6

#define EIXO_BASE       0
#define EIXO_OMBRO      1
#define EIXO_COTOVELO   2
#define EIXO_PUNHO_ROT  3
#define EIXO_PUNHO_FLEX 4
#define EIXO_GARRA      5

// ======================================
// MAPA DE PINOS (doc/diagramas.md, esquematico v4)
// ======================================
// Pinagem do esquematico. A ligacao fisica ainda nao foi validada com este firmware:
// o teste anterior do Start usou o GPIO15 antigo (hoje base); so o LED onboard e o mesmo.

// Servos (PWM). A base (MG996R) e tratada como giro continuo: o comando de
// servo de bancada <GPIO> <GRAUS> a recusa (ver gpio_policy).
#define SERVO_BASE_GPIO        15
#define SERVO_OMBRO_GPIO       16
#define SERVO_COTOVELO_GPIO    17
#define SERVO_PUNHO_ROT_GPIO   18
#define SERVO_PUNHO_FLEX_GPIO   8
#define SERVO_GARRA_GPIO        9

// ======================================
// BOTOES E ENTRADAS (ativos em nivel baixo, com pull-up)
// ======================================

#define BTN_START_GPIO  4
// Emergencia: borda de descida com interrupcao. O firmware a habilita por
// CONFIG_MAGNIBOT_ESTOP_GPIO (Kconfig, padrao = BTN_EMERG_GPIO); gpio_policy a
// reserva para a emergencia.
#define BTN_EMERG_GPIO  6
#define BTN_FIM_GPIO    7     // fim de curso (referencia do grau 0); sem driver ainda

#define POT_GPIO        5     // potenciometro (ADC); sem driver ainda
#define ENC_SW_GPIO    10     // encoder KY-040; sem driver ainda
#define ENC_CLK_GPIO   13
#define ENC_DT_GPIO    14

// ======================================
// SENSOR I2C (VL53L0X) - sem driver ainda
// ======================================

#define I2C_SDA_GPIO  11
#define I2C_SCL_GPIO  12

// ======================================
// LED RGB
// ======================================

#define WS2812_GPIO 48

// ======================================
// COMUNICACAO
// ======================================

#define UART_BAUDRATE 115200

#endif
