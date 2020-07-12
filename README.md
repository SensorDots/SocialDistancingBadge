# SensorDots Social Distancing Badge Firmware

Firmware sources for the Social Distancing Badge - https://www.kickstarter.com/projects/sensordots/the-social-distancing-badge?ref=user_menu

## Menu

Powering on/off - When out of the menu, hold the button to turn off (the display will go blank). When off, hold button to turn on.

To operate the menu double click to enter menu. Single click to rotate through menu options. Double click to enter sub-menu. Single click to change value. Double click to exit sub-menu. Menu will timeout after 5 seconds.

The menu has the following hierarchy:

 - S - Speed
   - 20 - 20Hz
   - 5  - 5Hz
   - 1  - 1Hz
 
 - U - Units
   - FE - Feet
   - In - Inches
   - FI - Feet - Inches (display feet as left value and the remaining inches on the right)
   - SI - Metric Units
 
 - d - Decimal Point Location (used when in metric, when in imperial the decimal point location will change automatically)
   - mm - millimeters
   - cm - centimeters
   - m  - meters
 
 - L - Loudness
   - OF - Sound off
   - LO - Sound half volume
   - hI - Sound full volume
 
 - F - Field of View (angle)
   - 27 - 27 degrees
   - 15 - 15 degrees
   - 21 - 21 degrees
 
 - E - Effect (sound)
    - 0 - Geiger counter
    - 1 - Varying frequency clicks
    - 2 - Alien
 
 - h - Effect Threshold (distance at which the threshold will activate; value will be meters or feet depending on whether in metric or imperial modes)
  
## Hidden Debug Menu

To access the hidden debug menu:

- Double click the button to activate the menu and you will see 5 on the left of the screen.
- Hold down the button until you see "d8 7" (this means deBug Test).
- Single click to cycle through the debug menu options.

The debug settings are as follows (To activate the current menu option, double click the button when the setting is shown):

- "d8 7" - deBug Test. This will cycle through an LED, LCD and Speaker test.
- "d8 C" - deBug Calibrate. This will offset and crosstalk (if using a window cover) calibrate the sensor to a white target that is 200mm away. The LED will turn off when calibrating and turn back on when done. Click again to return to normal operation.
- "d8 d" - deBug delete calibration. This will clear any previous calibration settings.



## GPIO
There are three GPIO pins broken out to the ABCDEF pins on the right hand side of the board. They have the following pinout:
- A - VCC (Battery Voltage - 3.1-3.6V)
- B - GND
- C - GND
- D - PD7 - Arduino Pin 7
- E - PB0 - Arduino Pin 8
- F - PB1 - Arduino Pin 9

The ICSP programming pins also broken out to the top of the board. These can be used for programming via SPI or also extra GPIO pins. They have the following pinout:

- 1 - PB4 - Arduino Pin 12 (MISO)
- 2 - VCC (Battery Voltage - 3.1-3.6V)
- 3 - PB5 - Arduino Pin 13 (SCK)
- 4 - PB3 - Arduino Pin 11 (MOSI)
- 5 - RST
- 6 - GND

They all have defines in the .ino file, so you can use them there:

```
#define GPIO_D 7
#define GPIO_E 8
#define GPIO_F 9
#define GPIO_1 12
#define GPIO_3 13
#define GPIO_4 11
```

## Installation

For setting up the Arduino environment for development or firmware update, please see https://sensordots.org/portmuxr_arduino (the instructions currently reference the Port MuxR, but they are the same for the Social Distancing Badge).

To load the latest precompiled hex file without installing the Arduino IDE you can use avrdude with the following command (substitute /dev/ttyUSBx with comX under Windows). You may need to replace the avrdude.conf file from here - https://github.com/SensorDots/SocialDistancingBadge/blob/master/doc/avrdude.conf

avrdude -v -p atmega328pb -c arduino -P /dev/ttyUSB0 -b 57600 -D -U flash:w:F1.03.hex:i

You should get the following output:

    avrdude: Version 6.3, compiled on Feb 17 2016 at 09:25:53
         Copyright (c) 2000-2005 Brian Dean, http://www.bdmicro.com/
         Copyright (c) 2007-2014 Joerg Wunsch

         System wide configuration file is "avrdude.conf"

         Using Port                    : com17
         Using Programmer              : arduino
         Overriding Baud Rate          : 57600
         AVR Part                      : ATmega328PB
         Chip Erase delay              : 9000 us
         PAGEL                         : PD7
         BS2                           : PC2
         RESET disposition             : dedicated
         RETRY pulse                   : SCK
         serial program mode           : yes
         parallel program mode         : yes
         Timeout                       : 200
         StabDelay                     : 100
         CmdexeDelay                   : 25
         SyncLoops                     : 32
         ByteDelay                     : 0
         PollIndex                     : 3
         PollValue                     : 0x53
         Memory Detail                 :

                                  Block Poll               Page                       Polled
           Memory Type Mode Delay Size  Indx Paged  Size   Size #Pages MinW  MaxW   ReadBack
           ----------- ---- ----- ----- ---- ------ ------ ---- ------ ----- ----- ---------
           eeprom        65    20     4    0 no       1024    4      0  3600  3600 0xff 0xff
           flash         65     6   128    0 yes     32768  128    256  4500  4500 0xff 0xff
           lfuse          0     0     0    0 no          1    0      0  4500  4500 0x00 0x00
           hfuse          0     0     0    0 no          1    0      0  4500  4500 0x00 0x00
           lock           0     0     0    0 no          1    0      0  4500  4500 0x00 0x00
           calibration    0     0     0    0 no          1    0      0     0     0 0x00 0x00
           signature      0     0     0    0 no          3    0      0     0     0 0x00 0x00
           efuse          0     0     0    0 no          1    0      0  4500  4500 0x00 0x00

         Programmer Type : Arduino
         Description     : Arduino
         Hardware Version: 3
         Firmware Version: 4.4
         Vtarget         : 0.3 V
         Varef           : 0.3 V
         Oscillator      : 28.800 kHz
         SCK period      : 3.3 us
    
    avrdude: AVR device initialized and ready to accept instructions
    
    Reading | ################################################## | 100% 0.02s
    
    avrdude: Device signature = 0x1e9516 (probably m328pb)
    avrdude: safemode: hfuse reads as 0
    avrdude: safemode: efuse reads as 0
    avrdude: reading input file "portmuxr_v1_00.hex"
    avrdude: writing flash (12480 bytes):
    
    Writing | ################################################## | 100% 4.14s
    
    avrdude: 12480 bytes of flash written
    avrdude: verifying flash memory against portmuxr_v1_00.hex:
    avrdude: load data flash data from input file portmuxr_v1_00.hex:
    avrdude: input file portmuxr_v1_00.hex contains 12480 bytes
    avrdude: reading on-chip flash data:
    
    Reading | ################################################## | 100% 3.82s
    
    avrdude: verifying ...
    avrdude: 12480 bytes of flash verified
    
    avrdude: safemode: hfuse reads as 0
    avrdude: safemode: efuse reads as 0
    avrdude: safemode: Fuses OK (E:00, H:00, L:00)
    
    avrdude done.  Thank you.


## Firmware Versions
The Social Distancing Badge can be queried for its firmware version on startup (re-insert the battery):

  - [F1.04](https://raw.githubusercontent.com/SensorDots/SocialDistancingBadge/master/releases/F1.04.hex) - Fix for Feet/Inch Display - [commit](https://github.com/SensorDots/SocialDistancingBadge/tree/dd6b8fbf8eaf43e7e10bf8af431262f737108c4c)
  - [F1.03](https://raw.githubusercontent.com/SensorDots/SocialDistancingBadge/master/releases/F1.03.hex) - Kickstarter Release Version - [commit](https://github.com/SensorDots/SocialDistancingBadge/tree/de15916056ef4eb462cb7ae99f545dcebe0092ae)
