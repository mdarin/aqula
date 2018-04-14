/*****
 * name: WaterChip 
 * Контроллер наполения бака и управления нагревом воды в нём
 * Дата: 16.08.2014
 * Версия: 0.1b
 * 
 *
 ***/
#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/version.h>
#if __AVR_LIBC_VERSION__ < 10606UL
#error "please update to avrlibc 1.6.6 or newer, not tested with older versions"
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>

// KEYS

#define NO_KEYS 0
#define MENU_KEY 1
#define AUTO_KEY 2
#define INC_KEY 3
#define DEC_KEY 4

// typedef key_t ...

void init_keys(void);
uint8_t get_key(void);

#endif //KEYBOARD_H
