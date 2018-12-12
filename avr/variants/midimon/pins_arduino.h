/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

// 0 - SS            PB2
// 1 - MOSI          PB3
// 2 - MISO          PB4
// 3 - SCK           PB5
// 4 - LCD_DC        PD7
// 5 - LCD_RESET     PD6
// 6 - BTN_A         PC0 PCINT8
// 7 - BTN_B         PC1 PCINT9
// 8 - BTN_UP        PC5 PCINT13
// 9 - BTN_DOWN      PC3 PCINT11
//10 - BTN_LEFT      PC4 PCINT12
//11 - BTN_RIGHT     PC2 PCINT10

#define NUM_DIGITAL_PINS            12
#define NUM_ANALOG_INPUTS           0
//#define analogInputToDigitalPin(p)  ((p < 6) ? (p) + 14 : -1)

#define PIN_SPI_SS        (0)
#define PIN_SPI_MOSI      (1)
#define PIN_SPI_MISO      (2)
#define PIN_SPI_SCK       (3)
#define PIN_LCD_DC        (4)
#define PIN_LCD_RESET     (5)
#define PIN_BTN_A         (6)
#define PIN_BTN_B         (7)
#define PIN_BTN_UP        (8)
#define PIN_BTN_DOWN      (9)
#define PIN_BTN_LEFT      (10)
#define PIN_BTN_RIGHT     (11)

static const uint8_t SS            = PIN_SPI_SS;
static const uint8_t MOSI          = PIN_SPI_MOSI;
static const uint8_t MISO          = PIN_SPI_MISO;
static const uint8_t SCK           = PIN_SPI_SCK;
static const uint8_t LCD_DC        = PIN_LCD_DC;
static const uint8_t LCD_RESET     = PIN_LCD_RESET;
static const uint8_t BTN_A         = PIN_BTN_A;
static const uint8_t BTN_B         = PIN_BTN_B;
static const uint8_t BTN_UP        = PIN_BTN_UP;
static const uint8_t BTN_DOWN      = PIN_BTN_DOWN;
static const uint8_t BTN_LEFT      = PIN_BTN_LEFT;
static const uint8_t BTN_RIGHT     = PIN_BTN_RIGHT;

#define SDA 0
#define SCL 0

#define digitalPinToPCICR(p)      (((p) >= PIN_BTN_A && (p) <= PIN_BTN_RIGHT) ? (&PCICR) : (uint8_t*)0)
#define digitalPinToPCICRbit(p)   (((p) >= PIN_BTN_A && (p) <= PIN_BTN_RIGHT) ? 1 : 0)
#define digitalPinToPCMSK(p)      (((p) >= PIN_BTN_A && (p) <= PIN_BTN_RIGHT) ? (&PCMSK1) : ((uint8_t*)0))
#define digitalPinToPCMSKbit(p)   (((p) == PIN_BTN_A) ? 0 : (((p) == PIN_BTN_DOWN) ? 3 : (((p) == PIN_BTN_UP) ? 5 : (((p) == PIN_BTN_B) ? 1 : (((p) == PIN_BTN_LEFT) ? 4 : (((p) == PIN_BTN_RIGHT) ? 2 : 0))))))

#define digitalPinToInterrupt NOT_AN_INTERRUPT

#ifdef ARDUINO_MAIN

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// ATMEL ATMEGA8 & 168 / ARDUINO
//
//                  +-\/-+
//            PC6  1|    |28  PC5 (AI 5)
//      (D 0) PD0  2|    |27  PC4 (AI 4)
//      (D 1) PD1  3|    |26  PC3 (AI 3)
//      (D 2) PD2  4|    |25  PC2 (AI 2)
// PWM+ (D 3) PD3  5|    |24  PC1 (AI 1)
//      (D 4) PD4  6|    |23  PC0 (AI 0)
//            VCC  7|    |22  GND
//            GND  8|    |21  AREF
//            PB6  9|    |20  AVCC
//            PB7 10|    |19  PB5 (D 13)
// PWM+ (D 5) PD5 11|    |18  PB4 (D 12)
// PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM
//      (D 7) PD7 13|    |16  PB2 (D 10) PWM
//      (D 8) PB0 14|    |15  PB1 (D 9) PWM
//                  +----+
//
// (PWM+ indicates the additional PWM pins on the ATmega168.)

// ATMEL ATMEGA1280 / ARDUINO
//
// 0-7 PE0-PE7   works
// 8-13 PB0-PB5  works
// 14-21 PA0-PA7 works 
// 22-29 PH0-PH7 works
// 30-35 PG5-PG0 works
// 36-43 PC7-PC0 works
// 44-51 PJ7-PJ0 works
// 52-59 PL7-PL0 works
// 60-67 PD7-PD0 works
// A0-A7 PF0-PF7
// A8-A15 PK0-PK7


// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[NUM_DIGITAL_PINS] = {
	PB,
	PB,
	PB,
	PB,
	PD,
	PD,
	PC,
	PC,
	PC,
	PC,
	PC,
	PC,
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[NUM_DIGITAL_PINS] = {
	_BV(2), // SS
	_BV(3), // MOSI
	_BV(4), // MISO
	_BV(5), // SCK
	_BV(7), // LCD_DC
	_BV(6), // LCD_RESET
	_BV(1), // A
	_BV(0), // B
	_BV(5), // UP
	_BV(3), // DOWN
	_BV(4), // LEFT
	_BV(2), // RIGHT
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[NUM_DIGITAL_PINS] = {
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
};

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR   Serial
#define SERIAL_PORT_HARDWARE  Serial

#endif
