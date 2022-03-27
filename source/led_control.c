/*
 * led_control.c
 *
 *  Created on: Dec 2, 2021
 *      Author: gulziibayar
 */
#include "board.h"
#include "led_control.h"

static void rgb_led(uint8_t r, uint8_t g, uint8_t b)
{
	GPIO_PinWrite(BOARD_USER_LED_RED_GPIO, BOARD_USER_LED_RED_GPIO_PIN, r);
	GPIO_PinWrite(BOARD_USER_LED_GREEN_GPIO, BOARD_USER_LED_GREEN_GPIO_PIN, g);
	GPIO_PinWrite(BOARD_USER_LED_BLUE_GPIO, BOARD_USER_LED_BLUE_GPIO_PIN, b);

}

void set_led(enum color led_color)
{
	switch(led_color)
	{
		case BLACK:
			rgb_led(0,0,0);
			break;
		case BLUE:
			rgb_led(0,0,1);
			break;
		case GREEN:
			rgb_led(0,1,0);
			break;
		case CYAN:
			rgb_led(0,1,1);
			break;
		case RED:
			rgb_led(1,0,0);
			break;
		case MAGENTA:
			rgb_led(1,0,1);
			break;
		case YELLOW:
			rgb_led(1,1,0);
			break;
		case WHITE:
			rgb_led(1,1,1);
			break;
		default:
			rgb_led(0,0,0);
			break;
	}
}
