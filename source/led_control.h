/*
 * led_control.h
 *
 *  Created on: Dec 2, 2021
 *      Author: gulziibayar
 */

#ifndef LED_CONTROL_H_
#define LED_CONTROL_H_

enum color{BLACK, BLUE, GREEN, CYAN, RED, MAGENTA, YELLOW, WHITE};
void set_led(enum color led_color);
#endif /* LED_CONTROL_H_ */
