#ifndef CONFIG_H
#define CONFIG_H

/* Only pins that are verified and in use belong here. Servo, I2C and aux-button pins
 * are added together with their drivers, after the wiring is validated. */
#define BTN_START_GPIO  15
#define WS2812_GPIO     48

#endif
