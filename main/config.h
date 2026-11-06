
#ifndef CONFIG_H
#define CONFIG_H

// ======================================
// CONFIGURACAO DOS EIXOS
// ======================================

#define TOTAL_EIXOS 6

#define EIXO_BASE       0
#define EIXO_OMBRO      1
#define EIXO_COTOVELO   2
#define EIXO_PULSO_1    3
#define EIXO_PULSO_2    4
#define EIXO_GARRA      5

// GPIOs provisórios dos servos
#define SERVO_BASE_GPIO       4
#define SERVO_OMBRO_GPIO      5
#define SERVO_COTOVELO_GPIO   6
#define SERVO_PULSO1_GPIO     7
#define SERVO_PULSO2_GPIO    10
#define SERVO_GARRA_GPIO     11

// ======================================
// BOTOES
// ======================================

#define BTN_START_GPIO  15
#define BTN_AUX_GPIO    16
#define BTN_EMERG_GPIO  17

// ======================================
// SENSOR I2C
// ======================================

#define I2C_SDA_GPIO  8
#define I2C_SCL_GPIO  9

// ======================================
// LED RGB
// ======================================

#define WS2812_GPIO 48

// ======================================
// COMUNICACAO
// ======================================

#define UART_BAUDRATE 115200

#endif
