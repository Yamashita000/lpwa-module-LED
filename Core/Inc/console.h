/*
 * console.h
 *
 *  Created on: 2021/06/04
 *      Author: kazuaki
 */

#ifndef SRC_CONSOLE_H_
#define SRC_CONSOLE_H_

// LEDs on board
enum LEDs {
	LED_BLUE,
	LED_RED,
	LED_GREEN,
	LED_RED2,
};

void SerialOutput(char *argv, ...);
void LedOutput(enum LEDs led, int value);

#endif /* SRC_CONSOLE_H_ */
