/**
   Social Distancing Badge Firmware - lcd.c
   
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

#include "lcd.h"
#include <stdint.h>
#include <string.h>
#include <Arduino.h>
#include <Wire.h>

//Two I2C-bus slave addresses (0111 000 and 0111 001)
#define PCF8562_ADDR    0x38 //Arduino libraries add the bit
//#define PCF8562_ADDR    0x70      //8562 (the i2c libraries modify the LSB for R/W)

#define	D1A	5
#define	D1B	4
#define	D1C	21
#define	D1D	22
#define	D1E	23
#define	D1F	6
#define	D1G	7
#define	D2A	1
#define	D2B	0
#define	D2C	17
#define	D2D	18
#define	D2E	19
#define	D2F	2
#define	D2G	3
#define	D3A	12
#define	D3B	11
#define	D3C	27
#define	D3D	26
#define	D3E	25
#define	D3F	13
#define	D3G	14
#define	D4A	8
#define	D4B	16
#define	D4C	31
#define	D4D	30
#define	D4E	29
#define	D4F	9
#define	D4G	10
#define	COL	15
#define	DP1	20
#define	DP2	24
#define	DP3	28

const uint8_t lcdMap[4][8] = {
  { D1A, D1B, D1C, D1D, D1E, D1F, D1G, DP1 },
  { D2A, D2B, D2C, D2D, D2E, D2F, D2G, DP2 },
  { D3A, D3B, D3C, D3D, D3E, D3F, D3G, DP3 },
  { D4A, D4B, D4C, D4D, D4E, D4F, D4G, COL }
};

/*
   SSegments are in the form GFEDCBA

       __A__
      |     |
     F|     |B
      |__G__|
      |     |
     E|     |C
      |__D__|

*/



uint8_t reverse(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

const uint8_t segmentMap[] = {
  0b00111111, // 0/O
  0b00000110, // 1/I
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5/S
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111, // 9
  0b00000000, // Space
  0b01000000, // -
  0b01011110, // d
  0b00111110, // U
  0b00101010, // m
  0b00111000, // L
  0b01110100, // h
  0b01110001, // F
  0b01111001, // E
  0b00111001, // C
  0b01010100, // n

};

#define NUMBER_LENGTH 4

uint32_t lcdSegments = 0;

uint32_t lcdDigitConvert(uint8_t position, uint8_t value)
{
  uint32_t digitSegments = 0;

  if (value <= sizeof(segmentMap) / sizeof(segmentMap[0])) { //Allows for all chars in segmentMap

    //We are only processing the digit segments here (not the LCD decimal points)
    for (uint8_t i = 0; i < 7; i++)
    {
      digitSegments |= (((uint32_t)((segmentMap[value] >> i) & 0x01)) << lcdMap[position][i]);
    }
  }

  return digitSegments;

}

void lcdTest()
{
  uint8_t lcdBuffer[4] = {0x00, 0x00, 0x00, 0x00};
  uint32_t segments;

  for (uint8_t i = 0; i < 8; i++)
  {
    segments = 0;
    for (uint8_t j = 0; j < 4; j++) {
      segments |= ((uint32_t)1) << lcdMap[j][i];
    }
    for (uint8_t i = 0; i < 4; i++)
    {
      lcdBuffer[i] = (segments >> (i * 8)) & 0xff;
    }
    send(PCF8562_ADDR, 0x00, lcdBuffer, 4);
    delay(300);
  }
}

void lcdPrintDP(uint8_t position)
{
  if (position > 0 && position <= 3) {
    lcdSegments |= ((uint32_t)1) << lcdMap[position - 1][7];
  }
}

void lcdPrintColon() {
  lcdSegments |= ((uint32_t)1) << lcdMap[3][7];
}

void lcdClear()
{
  lcdSegments = 0;
}


void lcdPrint(uint8_t * buf)
{
  for (uint8_t i = 0; i < NUMBER_LENGTH; i++)
  {
    lcdSegments |= lcdDigitConvert(i, buf[i]);
  }
}

void lcdWrite()
{
  uint8_t lcdBuffer[NUMBER_LENGTH];

  //Need this to be separate from the above loop because the segments are not in any specific order
  for (uint8_t i = 0; i < NUMBER_LENGTH; i++)
  {
    lcdBuffer[i] = (lcdSegments >> (i * 8)) & 0xff;
  }

  PCF8562_displayString(lcdBuffer, 0x00, NUMBER_LENGTH);
}

void lcdPrintNumber(uint16_t value) {

  uint32_t r;
  uint8_t buf[NUMBER_LENGTH + 2];

  memset(buf, 0, sizeof buf);

  // Will it fit
  if (value > 9999) {
    return;
  }

  // Convert number individual digits
  for (uint8_t x = NUMBER_LENGTH; x-- > 0; ) {
    r = value % 10;
    value /= 10;
    buf[x] = r;
  }

  // Display
  lcdPrint(buf);
}

void lcdBegin()
{
  PCF8562_init();
}

uint8_t send(uint8_t addr, uint8_t reg, uint8_t *s, uint8_t num)
{

  Wire.beginTransmission(addr);
  Wire.write(reg);
  for (uint8_t i = 0; i < num; i++)
  {
    Wire.write(*s);
    s++;
  }
  Wire.endTransmission();

  return (1);
}


uint8_t PCF8562_init(void)
{
  //0 1 0 0 1 1  0  1
  //C 1 0 x E B M1 M0

  /*
    0x4C|0x80,  // Mode set (1/3 bias, enabled, static display), more commands
    0x60|0x80,  // Select device 0, more commands
    0x70|0x80,  // No blinking, more commands
    0x00,       // Reset data pointer, last command
    0xFF,       // 8 characters follows, 2 bytes each
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    } ;*/

  uint8_t initCode[3] = {0x60 | 0x80, 0x70 | 0x80, 0x00};
  if ( send(PCF8562_ADDR, 0x4D | 0x80, initCode, 3) )
    return (1);
  else
    return (0);
}


uint8_t  PCF8562_displayString(uint8_t *buf, uint8_t start_addr, uint8_t num  )
{
  start_addr = start_addr & 0x3f; // C 0 P5 P4 P3 P2 P1 P0
  if ( send(PCF8562_ADDR, start_addr, buf, num) )
    return (1);
  else
    return (0);
}
