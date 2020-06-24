/**
   Social Distancing Badge Firmware - lcd.h
   
   Copyright (C) 2020 SensorDots.org
   
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LCD_H_
#define LCD_H_

#include <stdint.h>

#define letterD 12
#define letterS 5
#define dash 11
#define space 10
#define letterU 13
#define letterM 14
#define letterL 15
#define letterH 16
#define letterF 17
#define letterE 18
#define letterC 19
#define letterN 20

uint8_t send(uint8_t addr, uint8_t reg, uint8_t *s, uint8_t num);

uint8_t PCF8562_init(void);

uint8_t PCF8562_displayString(uint8_t *buf, uint8_t start_addr, uint8_t num);

void lcdPrint(uint8_t * buf);

uint32_t lcdDigitConvert(uint8_t position, uint8_t value);

void lcdPrintNumber(uint16_t v);

void lcdClear();

void lcdWrite();

void lcdPrintDP(uint8_t position);

void lcdBegin();

void lcdTest();

void lcdPrintColon();





#endif /* LCD_H_ */
