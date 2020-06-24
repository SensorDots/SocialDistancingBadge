# SensorDots Social Distancing Badge Firmware

Firmware sources for the Social Distancing Badge - https://www.kickstarter.com/projects/sensordots/the-social-distancing-badge?ref=user_menu

## Menu

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
 
 When out of the menu hold button to turn off. When off, hold button to turn on.


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

   - [F1.03](https://raw.githubusercontent.com/SensorDots/SocialDistancingBadge/master/releases/F1.03.hex) - Kickstarter Release Version - [commit](https://github.com/SensorDots/SocialDistancingBadge/tree/de15916056ef4eb462cb7ae99f545dcebe0092ae)
