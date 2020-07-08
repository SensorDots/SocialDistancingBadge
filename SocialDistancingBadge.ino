/**
   Social Distancing Badge Firmware
   
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

#include "ComponentObject.h"
#include "RangeSensor.h"
#include "vl53l1x_class.h"
#include "vl53l1_error_codes.h"
#include "lcd.h"
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <Wire.h>
#include "VL53L1X.h"
#include "OneButton.h"
#include "MenuBackend.h"
#include "bwlpf.h"
#include "history_buffer.h"
#include "ToneAC2.h"
#include <EEPROM.h>

#define VERSION 1.03

/* Averaging history size */
#define HISTORY_SIZE 10
/* Low pass filtering frequency cutoff */
#define FILTER_FREQUENCY              6  //Hz

//#define PROTOTYPE
//#define DEBUG

/* Pin Definitions */
#define SHUTDOWN_PIN   16
#define INTERRUPT_PIN  17
#define REGULATOR_2_8  2
#define LED            21
#ifdef PROTOTYPE
#define LCD_POWER      4
#else
#define LCD_POWER      22
#define SPK_NEG        4
#define SPK_POS        3
#define SPK_DC         5
#endif
#define BUTTON         20

#define GPIO_D 7
#define GPIO_E 8
#define GPIO_F 9
#define GPIO_1 12
#define GPIO_3 13
#define GPIO_4 11

#define RX 0
#define TX 1

#define PL(x) Serial.println(F(x));
#define PR(x) Serial.print(F(x));

/* Menu Definitions */
#define TWENTY_HZ 0
#define FIVE_HZ 1
#define ONE_HZ 2

#define DP_MM 0
#define DP_CM 3
#define DP_M 1

#define VOL_OFF 0
#define VOL_HIGH 2
#define VOL_LOW 1

#define FOV15 0
#define FOV21 1
#define FOV27 2

#define METRIC 0
#define FEET 1
#define INCH 2
#define FEETINCH 3

#define TOTAL_SOUND_EFFECTS 3

#define EFFECT_DELAY 10

#define GEIGER 0
#define FREQ_INCR 1
#define ALIEN 2

/* Variables */
SFEVL53L1X distanceSensor(Wire);            //VL53L1 Object

int16_t offset;                             //Offset calibration value
uint16_t crosstalk;                         //Crosstalk calibration value

volatile uint8_t volume = VOL_OFF;          //Volume setting
volatile uint8_t fov = FOV27;               //Field of view setting

OneButton button(BUTTON, true, false);      //Button object

volatile bool measurementInterrupt = false; //Measurement interrupt flag (gets triggered on VL53L1 Interrupt)

unsigned long previousButtonEventMillis;    //Previous button event time
volatile bool buttonEvent = false;          //Did button event happen?

volatile bool powerState = true;            //Current power state
volatile bool powerStateChange = false;     //Did we have a state change?

volatile uint8_t soundEffect = 0;           //Current sound effect setting

volatile uint8_t measurementMode = 0;       //Current measurement mode setting
volatile bool measurementModeChange = false;//Did we have a measurement mode change?

volatile bool interrupt_timeout_interrupt_fired = false; //Did we not get a VL53L1 interrupt after some time

volatile uint8_t decimalPointPosition = DP_MM; // Decimal point position

volatile uint8_t units = METRIC;            //Current units display setting

volatile bool runTest = false;              //Are we running tests?

volatile bool calibrating = false;          //Are we calibrating?

volatile bool clearCalib = false;           //Are we clearing calibration data?

unsigned long lastSensorReadMillis = 0;     //Last time the sensor was read

int8_t effectStartDelay = EFFECT_DELAY;     //Small delay before sound effect begins

/* Low pass filter and history buffers */
filter_state low_pass_filter_state;
uint16_t filtered_distance         = 0;
uint16_t real_distance             = 0;
uint16_t prev_real_distance        = 0;
circular_history_buffer history_buffer;
circular_history_buffer long_history_buffer;
circular_history_buffer signal_rate_history_buffer;


#ifdef DEBUG
unsigned long previousMeasurementMillis = 0; //Used for statistics during debug
#endif

bool noSignal = false;                       //Are we in a no signal state?
uint8_t noSignalCounter = 0;                 //How many no signal responses have we got?
uint8_t noSignalDisplayPosition = 0;         //Display moving decimal points to indicate no signal

bool speakerHalfVolToggle = false;           //Speaker volume

uint8_t LCDBuffer[4] = { 0 };                //LCD character display buffer

unsigned long lastSoundEffectMillis = 0;     //When did the sound effect run last for certain effects

volatile uint16_t effectThreshold = 1500;    //At which point in mm do we enabled sound effects?

void runTests();

uint16_t measurement_budget           = 50; //Current measurement budget of VL53L1 Sensor

volatile bool menuActive = false;           //Is the menu currently active?

uint16_t zeroCounter = 0;                   //How many times did we get a zero value for auto turn off

volatile bool menuDisplayChanged = false;   //Did we have a menu change event?

void menuUseEvent(MenuUseEvent used) {}

void menuChangeEvent(MenuChangeEvent changed) {}

/* Menu items instantiation */
MenuBackend menu = MenuBackend(menuUseEvent, menuChangeEvent);

/* List of menu items */
MenuItem MSpeed = MenuItem("S");    //Speed
MenuItem MSpeed20 = MenuItem("20"); //20Hz
MenuItem MSpeed5 = MenuItem("5");   //5Hz
MenuItem MSpeed1 = MenuItem("1");   //1Hz

MenuItem MFOV = MenuItem("Fo");     //Field of View
MenuItem MFOV15 = MenuItem("15");   //15 degrees
MenuItem MFOV21 = MenuItem("21");   //21 degrees
MenuItem MFOV27 = MenuItem("27");   //27 degrees

MenuItem MUnits = MenuItem("U");     //Units
MenuItem MUnitsI = MenuItem("I");    //Inches
MenuItem MUnitsM = MenuItem("M");    //Metric
MenuItem MUnitsF = MenuItem("F");    //Feet
MenuItem MUnitsFI = MenuItem("FI");  //Feet:Inches

MenuItem MVol = MenuItem("L");       //Volume
MenuItem MVolHi = MenuItem("Hi");    //High
MenuItem MVolLo = MenuItem("Lo");    //Low
MenuItem MVolOff = MenuItem("Of");   //Off

MenuItem MDP = MenuItem("d");        //Metric Decimal Point
MenuItem MDP_MM = MenuItem("mm");    //mm
MenuItem MDP_CM = MenuItem("cm");    //cm
MenuItem MDP_M = MenuItem("m");      //m

MenuItem MEffect = MenuItem("E");    //Sound Effect
MenuItem MEffect0 = MenuItem("E0");

MenuItem MThresh = MenuItem("H");    //Effect Threshold
MenuItem MThresh0 = MenuItem("H0");

MenuItem MDebug = MenuItem("DM");       //Debug Menu
MenuItem MDebugTest = MenuItem("DT");   //Test mode
MenuItem MDebugCalib = MenuItem("DC");  //Calibrate mode
MenuItem MDebugDelete = MenuItem("DD"); //Delete calibration


/* Set up the menu subsystem */
void menuSetup()
{
  menu.getRoot().add(MSpeed);

  MSpeed.addAfter(MUnits);
  MUnits.addAfter(MDP);
  MDP.addAfter(MVol);
  MVol.addAfter(MFOV);
  MFOV.addAfter(MEffect);
  MEffect.addAfter(MThresh);
  MThresh.addAfter(MSpeed);

  MDebug.addRight(MDebugTest);
  MDebugTest.addAfter(MDebugCalib);
  MDebugCalib.addAfter(MDebugDelete);
  MDebugDelete.addAfter(MDebugTest);

  MDebugTest.addLeft(MDebug);
  MDebugCalib.addLeft(MDebug);
  MDebugDelete.addLeft(MDebug);

  MSpeed.addRight(MSpeed20);
  MSpeed20.addAfter(MSpeed5);
  MSpeed5.addAfter(MSpeed1);
  MSpeed1.addAfter(MSpeed20);

  MSpeed1.addLeft(MSpeed);
  MSpeed5.addLeft(MSpeed);
  MSpeed20.addLeft(MSpeed);


  MUnits.addRight(MUnitsM);
  MUnitsM.addAfter(MUnitsF);
  MUnitsF.addAfter(MUnitsI);
  MUnitsI.addAfter(MUnitsFI);
  MUnitsFI.addAfter(MUnitsM);

  MUnitsM.addLeft(MUnits);
  MUnitsI.addLeft(MUnits);
  MUnitsF.addLeft(MUnits);
  MUnitsFI.addLeft(MUnits);

  MDP.addRight(MDP_MM);
  MDP_MM.addAfter(MDP_CM);
  MDP_CM.addAfter(MDP_M);
  MDP_M.addAfter(MDP_MM);

  MDP_MM.addLeft(MDP);
  MDP_CM.addLeft(MDP);
  MDP_M.addLeft(MDP);

  MVol.addRight(MVolLo);
  MVolLo.addAfter(MVolHi);
  MVolHi.addAfter(MVolOff);
  MVolOff.addAfter(MVolLo);

  MVolLo.addLeft(MVol);
  MVolHi.addLeft(MVol);
  MVolOff.addLeft(MVol);

  MFOV.addRight(MFOV15);
  MFOV15.addAfter(MFOV21);
  MFOV21.addAfter(MFOV27);
  MFOV27.addAfter(MFOV15);

  MFOV15.addLeft(MFOV);
  MFOV21.addLeft(MFOV);
  MFOV27.addLeft(MFOV);

  MEffect.addRight(MEffect0);
  MEffect0.addAfter(MEffect0);

  MEffect0.addLeft(MEffect);

  MThresh.addRight(MThresh0);
  MThresh0.addAfter(MThresh0);

  MThresh0.addLeft(MThresh);
}

/* Menu double click handler */
void doubleClick() {

  //Only change values if we are switched on (to prevent misclicks in sleep changing settings)
  if (menuActive && powerState) {
    
    MenuItem currentMenu = menu.getCurrent();

    if (currentMenu == MSpeed || currentMenu == MDP || currentMenu == MUnits || currentMenu == MVol || currentMenu == MFOV || currentMenu == MEffect || currentMenu == MThresh) {
      menu.moveRight(); //Move to next by default

      //Set menu to current setting if configured
      if (currentMenu == MSpeed)
      {
        if (measurementMode == TWENTY_HZ) menu.setCurrent(&MSpeed20);
        if (measurementMode == FIVE_HZ) menu.setCurrent(&MSpeed5);
        if (measurementMode == ONE_HZ) menu.setCurrent(&MSpeed1);
      }
      else if (currentMenu == MDP)
      {
        if (decimalPointPosition == DP_CM) menu.setCurrent(&MDP_CM);
        if (decimalPointPosition == DP_MM) menu.setCurrent(&MDP_MM);
        if (decimalPointPosition == DP_M) menu.setCurrent(&MDP_M);
      }
      else if (currentMenu == MUnits)
      {
        if (units == METRIC) menu.setCurrent(&MUnitsM);
        if (units == FEET)   menu.setCurrent(&MUnitsF);
        if (units == INCH)   menu.setCurrent(&MUnitsI);
        if (units == FEETINCH)   menu.setCurrent(&MUnitsFI);
      }
      else if (currentMenu == MVol)
      {
        if (volume == VOL_HIGH) menu.setCurrent(&MVolHi);
        if (volume == VOL_LOW) menu.setCurrent(&MVolLo);
        if (volume == VOL_OFF) menu.setCurrent(&MVolOff);
      }
      else if (currentMenu == MFOV)
      {
        if (fov == FOV15) menu.setCurrent(&MFOV15);
        if (fov == FOV21) menu.setCurrent(&MFOV21);
        if (fov == FOV27) menu.setCurrent(&MFOV27);
      }
      else if (currentMenu == MEffect)
      {
        menu.setCurrent(&MEffect0);
      }
      else if (currentMenu == MThresh)
      {
        menu.setCurrent(&MThresh0);
      }
    } else if (currentMenu == MDebugTest || currentMenu == MDebugCalib || currentMenu == MDebugDelete) {
      if (currentMenu == MDebugTest) {
        runTest = true;
      } else if (currentMenu == MDebugCalib) {
        calibrating = true;
      }
      else if (currentMenu == MDebugDelete) {
        clearCalib = true;
      }
    } else {
      menu.moveLeft();
    }

    menuDisplayChanged = true;

  }
  if (powerState && !menuActive)
  {
    menuActive = true;
    menu.toRoot();
    menu.moveDown();
    menuDisplayChanged = true;
  }
}

/* Menu single click handler */
void singleClick()
{
  if (powerState)
  {
    if (menuActive)
    {

      menu.moveDown();

      MenuItem currentMenu = menu.getCurrent();

      //if (currentMenu == MUnits && !metric) menu.moveDown(); //Skip DP placement if in imperial. (TODO fix this)

      if (currentMenu == MSpeed20) {
        measurementMode = TWENTY_HZ;
        measurementModeChange = true;
      }
      else if (currentMenu == MSpeed5) {
        measurementMode = FIVE_HZ;
        measurementModeChange = true;
      }
      else if (currentMenu == MSpeed1) {
        measurementMode = ONE_HZ;
        measurementModeChange = true;
      }
      else if (currentMenu == MFOV15) {
        fov = FOV15;
        measurementModeChange = true;
      }
      else if (currentMenu == MFOV21) {
        fov = FOV21;
        measurementModeChange = true;
      }
      else if (currentMenu == MFOV27) {
        fov = FOV27;
        measurementModeChange = true;
      }
      else if (currentMenu == MVolLo) {
        volume = VOL_LOW;
      }
      else if (currentMenu == MVolHi) {
        volume = VOL_HIGH;
      }
      else if (currentMenu == MVolOff) {
        volume = VOL_OFF;
      }
      else if (currentMenu == MDP_MM) {
        decimalPointPosition = DP_MM;
      }
      else if (currentMenu == MDP_CM) {
        decimalPointPosition = DP_CM;
      }
      else if (currentMenu == MDP_M) {
        decimalPointPosition = DP_M;
      }
      else if (currentMenu == MUnitsM) {
        units = METRIC;
      }
      else if (currentMenu == MUnitsI) {
        units = INCH;
      }
      else if (currentMenu == MUnitsF) {
        units = FEET;
      }
      else if (currentMenu == MUnitsFI) {
        units = FEETINCH;
      }
      else if (currentMenu == MEffect0) {
        soundEffect++;
        if (soundEffect > TOTAL_SOUND_EFFECTS - 1) soundEffect = 0;
      }
      else if (currentMenu == MThresh0) {
        effectThreshold = effectThreshold + 100;
        if (effectThreshold > 2000) effectThreshold = 100;
      }
    }
    menuDisplayChanged = true;
  }
}

/* Calibration routines */
void calibrate()
{
  //Turn off LED
#ifdef PROTOTYPE
  digitalWrite(LED, HIGH);
#else
  digitalWrite(LED, LOW);
#endif

  distanceSensor.calibrateOffset(200);
  distanceSensor.calibrateXTalk(200);
  offset = distanceSensor.getOffset();
  crosstalk = distanceSensor.getXTalk();

  EEPROM.update(0x00, 0xAA); //Set pattern for blank EEPROM

  EEPROM.update(0x10, offset & 0xff);
  EEPROM.update(0x11, offset >> 8 & 0xff);
  EEPROM.update(0x20, crosstalk);
  EEPROM.update(0x21, crosstalk >> 8 & 0xff);

#ifdef DEBUG
  PL("Calibrated");
  PR("Offset: ");
  Serial.println(offset);
  PR("Crosstalk: ");
  Serial.println(crosstalk);
#endif

  //Turn on LED
#ifdef PROTOTYPE
  digitalWrite(LED, LOW);
#else
  digitalWrite(LED, HIGH);
#endif

  delay(500);


  //Turn off LED
#ifdef PROTOTYPE
  digitalWrite(LED, HIGH);
#else
  digitalWrite(LED, LOW);
#endif

  calibrating = false;
}

/* Device test routines */
void runTests()
{
  //Test LCD
  lcdTest();

  delay(500);

  //Test Speaker
  toneAC2(SPK_NEG, SPK_POS, 600, 100);
  toneAC2(SPK_NEG, SPK_POS, 1500, 100);
  toneAC2(SPK_NEG, SPK_POS, 1000, 100);

  delay(500);

  //Test LED

  //Turn off
#ifdef PROTOTYPE
  digitalWrite(LED, HIGH);
#else
  digitalWrite(LED, LOW);
#endif

  //Turn on

  delay(500);

#ifdef PROTOTYPE
  digitalWrite(LED, LOW);
#else
  digitalWrite(LED, HIGH);
#endif

  delay(500);


  //Turn off
#ifdef PROTOTYPE
  digitalWrite(LED, HIGH);
#else
  digitalWrite(LED, LOW);
#endif

  //Return to menu
  menuDisplayChanged = true;
}

/* Menu long press handler */
void longPress() {

  if (menuActive)
  {
    menu.setCurrent(&MDebugTest);
    menuDisplayChanged = true;
  } else {
    powerState = !powerState;
    powerStateChange = true;
  }
}


/* The Interrupt Service Routine for Pin Change Interrupt on PE2 */
ISR(PCINT3_vect) {
  button.tick(); // call tick() to check the state of the button
  buttonEvent = true;
}

/* Interrupt for interrupt line from VL53L1 */
ISR (PCINT1_vect)
{
  if (digitalRead(INTERRUPT_PIN) == 0) //TODO We need to note the polarity of the VL53L1 library
  {
    measurementInterrupt = true;
  }
}

/* No interrupt overflow (this will fire every ~2000ms if no interrupt from the VL53L1 has arrived) */
ISR(TIMER4_OVF_vect)
{
  interrupt_timeout_interrupt_fired = true;
}

void setup() {

  cli();

  /* disable ADC */
  ADCSRA = 0x00;
  ADCSRB = 0x00;
  DIDR0 = (1 << ADC5D) | (1 << ADC4D) | (1 << ADC3D) | (1 << ADC2D) | (1 << ADC1D) | (1 << ADC0D); // | (1 << ADC0D); //Disable digital input buffer on ADC pins
  DIDR0 |= 0xC0 ; //ADC7D and ADC6D are undefined in header file so set bits this way
  DIDR1 |= (1 << AIN1D) | (1 << AIN0D); //Disable digital input buffer on Analog comparator pins
  ACSR |= (1 << ACD); /* Disable Analog Comparator */

  /* Reset watchdog timer */
  wdt_reset();
  MCUSR &= ~(1 << WDRF); /* Clear WDRF in MCUSR */
  WDTCSR = 0x00; /* Turn off WDT */

  sei();

  pinMode(REGULATOR_2_8, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(LCD_POWER, OUTPUT);
#ifndef PROTOTYPE
  pinMode(SPK_NEG, OUTPUT);
  pinMode(SPK_POS, OUTPUT);
  pinMode(SPK_DC, OUTPUT);
#endif
  pinMode(BUTTON, INPUT);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(SHUTDOWN_PIN, OUTPUT);

  pinMode(GPIO_F, OUTPUT);
  pinMode(GPIO_E, OUTPUT);

  /* Turn off LED*/
#ifdef PROTOTYPE
  digitalWrite(LED, HIGH);
#else
  digitalWrite(LED, LOW);
#endif

  /* Turn 2.8V Regulator for VL53L1 */
  digitalWrite(REGULATOR_2_8, HIGH);
  
  /* Vl53L1 startup delay */
  delay(2); 
  
  /* Disable VL53L1 Shutdown Pin */
  digitalWrite(SHUTDOWN_PIN, HIGH);

  /* Turn on LCD */
  digitalWrite(LCD_POWER, HIGH);

  Wire.begin();
  Wire.setClock(100000);

#ifdef DEBUG
  Serial.begin(57600);
#endif

  delay(500);

  /* Dual lcdBegin call to account for slow startup times of different battery states */
  lcdBegin();
  lcdBegin();

  /* Print Version number on LCD */
#ifdef DEBUG
  LCDBuffer[0] = letterD;
#else
  LCDBuffer[0] = letterF;
#endif
  LCDBuffer[1] = (int)(VERSION) % 10;
  LCDBuffer[2] = (int)(VERSION * 10) % 10;
  LCDBuffer[3] = (int)(VERSION * 100) % 10;
  lcdPrint(LCDBuffer);
  lcdPrintDP(2);
  lcdWrite();

  delay(1000);

  /* Wait for sensor to respond */
  while (distanceSensor.checkBootState() == false)
  {
    delay(2);
  }

  if (distanceSensor.begin() == 0) //Begin returns 0 on a good init
  {
#ifdef DEBUG
    PL("Sensor OK!");
#endif
  } else {
	  /* Flash LED if sensor has failed (indicates problem with sensor) */
    while (1)
    {
#ifdef PROTOTYPE
      digitalWrite(LED, HIGH);
#else
      digitalWrite(LED, LOW);
#endif
      delay(500);
#ifdef PROTOTYPE
      digitalWrite(LED, LOW);
#else
      digitalWrite(LED, HIGH);
#endif
      delay(500);

    }
  }

  /* enable button menu */
  button.attachDoubleClick(doubleClick);
  button.attachClick(singleClick);
  button.attachLongPressStart(longPress);
  button.setPressTicks(2000);
  button.setClickTicks(300);
  button.setDebounceTicks(50);

  menuSetup();

  cli();

  /* Setup button interrupt */
  /* PE2 - PCINT26 (PCMSK3) */
  PCIFR |= (1 << PCIE3);
  PCICR |= (1 << PCIE3); // set PCIE3 to enable PCMSK3 scan
  PCMSK3 |= (1 << PCINT26); // set PCINT26 to trigger an interrupt on state change

  /* Setup measurement interrupt */
  /* PC3 - PCINT11 (PCMSK1) */
  PCIFR |= (1 << PCIE1);
  PCICR |= (1 << PCIE1);    // set PCIE1 to enable PCMSK1 scan
  PCMSK1 |= (1 << PCINT11);  // set PCINT11 to trigger an interrupt on state change

  /* Setup no interrupt (from VL53L1) interrupt */
  /* Enable TC4 */
  PRR1 &= ~(1 << PRTIM4);

  TCCR4A = (0 << COM4A1) | (0 << COM4A0) /* Normal port operation, OCA disconnected */
           | (0 << COM4B1) | (0 << COM4B0) /* Normal port operation, OCB disconnected */
           | (0 << WGM41) | (0 << WGM40); /* TC16 Mode 0 Normal */

  TCCR4B = (0 << WGM43) | (0 << WGM42)                /* TC16 Mode 0 Normal */
           | 0 << ICNC4                               /* Input Capture Noise Canceler: disabled */
           | 0 << ICES4                               /* Input Capture Edge Select: disabled */
           //| (0 << CS42) | (1 << CS41) | (1 << CS40); /* IO clock divided by 64 */
           | (1 << CS42) | (0 << CS41) | (0 << CS40); /* IO clock divided by 256 */

  ICR4 = 0; /* Input capture value, used as top counter value in some modes: 0 */

  OCR4A = 0; /* Output compare A: 0 */

  OCR4B = 0; /* Output compare B: 0 */

  TIMSK4 = 0 << OCIE4B   /* Output Compare B Match Interrupt Enable: disabled */
           | 0 << OCIE4A /* Output Compare A Match Interrupt Enable: disabled */
           | 0 << ICIE4  /* Input Capture Interrupt Enable: disabled */
           | 1 << TOIE4; /* Overflow Interrupt Enable: enabled */

  sei();

  /* Setup sensor */
  distanceSensor.startTemperatureUpdate();
  distanceSensor.setInterruptPolarityLow();

  /* Needs to be set before timing budget */
  distanceSensor.setDistanceModeLong(); 

  /*
  VL53L1 Lite libraries allow 15,20,33,50,100,200,500 ms timing budget
  The inter-measurement period (IMP) is the time between two consecutive measurements. The IMP must be
  greater than or equal to the TB otherwise the actual IMP is double the expected value.

  The lite libraries apparently support Dynamic SPAD Selection
  This causes dynamic speeds on occasion  
  */

  distanceSensor.setTimingBudgetInMs(measurement_budget);
  distanceSensor.setIntermeasurementPeriod(measurement_budget);

  /* Check for blank EEPROM */
  if ( EEPROM.read(0x00) == 0xAA) 
  {
	/* Get calibration variables */
    offset = (int16_t)EEPROM.read(0x10) | ((int16_t)EEPROM.read(0x11)) << 8;
    crosstalk = (uint16_t)EEPROM.read(0x20) | ((uint16_t)EEPROM.read(0x21)) << 8;
    distanceSensor.setOffset(offset);
    distanceSensor.setXTalk(crosstalk);
  }

#ifdef DEBUG
  PR("Offset: ");
  Serial.println(offset);
  PR("Crosstalk: ");
  Serial.println(crosstalk);
#endif

  /* Start measurement */
  distanceSensor.startRanging();

  /* Initialise Filter */
  bwlpf_init(&low_pass_filter_state, 1000 / measurement_budget, FILTER_FREQUENCY);

  /* Initialise history buffers */
  hb_init(&history_buffer, HISTORY_SIZE, sizeof(uint16_t));
  hb_init(&long_history_buffer, HISTORY_SIZE * 2, sizeof(uint16_t));
  hb_init(&signal_rate_history_buffer, HISTORY_SIZE, sizeof(int32_t));

  /* Init button ticks */
  previousButtonEventMillis = millis();
  button.tick();

  lastSensorReadMillis = millis();

  /* Play startup tone */
#ifndef DEBUG
  toneAC2(SPK_NEG, SPK_POS, 600, 100);
  toneAC2(SPK_NEG, SPK_POS, 1500, 100);
  toneAC2(SPK_NEG, SPK_POS, 1000, 100);
#endif

  /* Reset no interrupt (from VL53L1) timer interrupt;
  reset the counter, clear the timer's count register (TCNTx). */
  TCNT4 = 0;
  interrupt_timeout_interrupt_fired = false;


}


void loop() {

  unsigned long currentMillis = millis();

  if (buttonEvent)
  {
    buttonEvent = false;
    previousButtonEventMillis = currentMillis;
  }

  if (interrupt_timeout_interrupt_fired)
  {
    interrupt_timeout_interrupt_fired = false;

    /* Reset pullup on interrupt pin. */
    pinMode(INTERRUPT_PIN, INPUT_PULLUP);

#ifdef DEBUG
    PL("Measurement Interrupt Not Fired!");
#endif

    /* Reset VL53L1 interrupt if we have a communication timeout */
    distanceSensor.clearInterrupt();
    measurementModeChange = true;
  }

  if (measurementInterrupt)
  {
    /* Reset VL53L1 no interrupt timer interrupt */
    TCNT4 = 0;

    distanceSensor.clearInterrupt();

    lastSensorReadMillis = currentMillis;

    if (!menuActive) {

      prev_real_distance = real_distance;

      real_distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
      int32_t signalRate = distanceSensor.getSignalRate();
      int32_t avgSignalRate = 0;
      byte rangeStatus = distanceSensor.getRangeStatus();

      /* Check if we got a "zero" distance */
      if (real_distance <= 30)
      {
        zeroCounter++;
      } else {
        zeroCounter = 0;
      }

      /* Turn off badge if sensor displays the same false readout after 2 minutes 
	  (in case it's in a pocket or the like). */
      if (zeroCounter > (1000 / measurement_budget * 60 * 2))
      {
        powerStateChange = true;
        powerState = false;
      }
	  
	  /* Only filter on high update rates (because of the lower measurement time). */
      if (measurementMode == TWENTY_HZ) 
      {
        filtered_distance = bwlpf(real_distance, &low_pass_filter_state);
        /* Push current measurement into ring buffer for averaging */
        hb_push_back(&history_buffer, &filtered_distance);
        hb_push_back(&long_history_buffer, &filtered_distance);
        hb_push_back(&signal_rate_history_buffer, &signalRate);

        avgSignalRate = avg32((int32_t*)signal_rate_history_buffer.buffer, HISTORY_SIZE);

        if ((avgSignalRate > 5000) | (measurementMode == ONE_HZ))
        {
          filtered_distance = avg((uint16_t*)history_buffer.buffer, HISTORY_SIZE);
        } else {
          filtered_distance = avg((uint16_t*)long_history_buffer.buffer, HISTORY_SIZE * 2);
        }

        /* if we get an eroneous average reading, display "searching" on the display */
        if (avgSignalRate < 1500 && filtered_distance < 1000 && rangeStatus >= 2) 
        {
          noSignal = true;
        } else {
          noSignal = false;
        }

      } else {
        filtered_distance = real_distance;
		
		/* if we get an eroneous average reading, display "searching" on the display */
        if (signalRate < 600 && rangeStatus >= 2)
        {
          noSignal = true;
        } else {
          noSignal = false;
        }
      }

      /* Fix "negative" uint flashing issues with LCD writes */
      if (filtered_distance > 5000) filtered_distance = 5000;

      measurementInterrupt = false;
#ifdef DEBUG
      unsigned long testMillis = currentMillis;
#endif

      /*Note imperial measurements are all operated on with integer maths (rather than floats to avoid rounding errors)
      All these values are stored multiplied by 1000 and should be treated as such*/
	  
      uint32_t distanceInches = filtered_distance * 39.3701;
      uint32_t distanceFeet = distanceInches / 12;

#ifdef DEBUG
      PR("Distance(mm): ");
      Serial.print(filtered_distance);

      PR("\tDistance(ft): ");
      Serial.print((float)distanceFeet / 1000, 3);

      PR("\tInches(in): ");
      Serial.print((float)distanceInches / 1000, 3);

      PR("\tUpdate (hz): ");
      Serial.print((float)1000 / (millis() - previousMeasurementMillis), 1);

      PR("\tSignal rate: ");
      Serial.print(avgSignalRate);


      PR("\tStatus: ");

      //TODO change to short range when signal rate is really low (sensor works better with this)

      switch (rangeStatus)
      {
        case 0:
          PR("Good");
          break;
        case 1:
          PR("Sigma fail");
          break;
        case 2:
          PR("Signal fail");
          break;
        case 7:
          PR("Wrapped target fail");
          break;
        default:
          PR("Unknown: ");
          Serial.print(rangeStatus);
          break;
      }

      previousMeasurementMillis = currentMillis;

#endif

      lcdClear();
      if (noSignal)
      {

        if (measurementMode == TWENTY_HZ)
        {
          noSignalCounter = noSignalCounter + 2;

        } else if (measurementMode == FIVE_HZ)
        {
          noSignalCounter = noSignalCounter + 4;
        } else if (measurementMode == ONE_HZ)
        {
          noSignalCounter = noSignalCounter + 20;
        }

        if (noSignalCounter >= 20)
        {
          noSignalCounter = 0;
          noSignalDisplayPosition++;
        }

        if (noSignalDisplayPosition > 3) noSignalDisplayPosition = 0;
        lcdPrintDP(noSignalDisplayPosition);
		
      } else {
        if (units == METRIC) {
		  /* round to centimeters at long range */
          if (filtered_distance > 2000) 
          {
            filtered_distance = ((uint16_t)(filtered_distance / 10)) * 10;
          }
		  /* round to 10 cm at even longer range to stop jitter */
          if (filtered_distance > 3000) 
          {
            filtered_distance = ((uint16_t)(filtered_distance / 100)) * 100;
          }
          lcdPrintNumber(filtered_distance);
          lcdPrintDP(decimalPointPosition);
        } else if (units == FEET) {
          /* Note integer math here (values are stored x1000) */
		  
		  /* round to hundreths at long range */
          if (distanceFeet > 6000) 
          {
            distanceFeet = (distanceFeet / 10) * 10;
          }
		  /* round to tenths at longer range */
          if (distanceFeet > 9000) 
          {
            distanceFeet = (distanceFeet / 100) * 100;
          }
		  /* If digits don't overflow display, display all */
          if (distanceFeet < 10000) 
          {
            lcdPrintNumber(distanceFeet);
            lcdPrintDP(1);
          } else {
            lcdPrintNumber((int)(distanceFeet / 10));
            lcdPrintDP(2);
          }
        } else if ( units == INCH)
        {
          /* Note integer math here (values are stored x1000) */
		  
		  /* round to tenths at long range */
          if (distanceInches > 72000) 
          {
            distanceInches = (distanceInches / 100) * 100;
          }
		  /* round to inches at longer range */
          if (distanceInches > 108000) 
          {
            distanceInches = (distanceInches / 1000) * 1000;
          }
          if (distanceInches < 10000)
          {
            lcdPrintNumber(distanceInches);
            lcdPrintDP(1);
          } else if (distanceInches / 10 < 10000)
          {
            lcdPrintNumber((int)(distanceInches / 10));
            lcdPrintDP(2);
          }
          else if (distanceInches / 100 < 10000)
          {
            lcdPrintNumber((int)(distanceInches / 100));
            lcdPrintDP(3);
          }
        }
        else if ( units == FEETINCH)
        {
          int distanceInch = distanceFeet - (distanceFeet / 1000) * 1000;
          distanceFeet = (distanceFeet - distanceInch) / 1000;
          distanceInch = 12 * distanceInch / 1000;

          uint8_t LCDBuffer[4] = { 0 };
          LCDBuffer[0] = (int)(distanceFeet / 10) % 10;
          LCDBuffer[1] = (int)(distanceFeet) % 10;
          LCDBuffer[2] = (int)(distanceInch / 10) % 10;
          LCDBuffer[3] = (int)(distanceInch) % 10;

          lcdPrintColon();
          lcdPrint(LCDBuffer);
        }
      }
#ifndef PROTOTYPE
      if (!noSignal && volume > 0)
      {
        if (effectStartDelay > 0)
        {
          effectStartDelay--;
        } else {

          if (filtered_distance < effectThreshold)
          {
            pinMode(GPIO_F, OUTPUT);
            pinMode(GPIO_E, OUTPUT);

            if (soundEffect == GEIGER) {
              double clickProbability = ((double)random(0, 100)) / 100;

              if (clickProbability + 0.1 < (((double)effectThreshold - (double)filtered_distance) / (double)effectThreshold))
              {
                if (volume == 1)
                {

                  digitalWrite(SPK_POS, HIGH);

                  toneAC2(SPK_NEG, SPK_DC, 600, 3, true);

                } else {
                  toneAC2(SPK_NEG, SPK_POS, 600, 3, true);
                }

              }
            } else if (soundEffect == FREQ_INCR)
            {
              if (volume == 1)
              {
 
                digitalWrite(SPK_POS, HIGH);

                toneAC2(SPK_NEG, SPK_DC, ((double)(effectThreshold - filtered_distance)) / effectThreshold * 4000 + 100, 15, true);

              } else {
                toneAC2(SPK_NEG, SPK_POS, ((double)(effectThreshold - filtered_distance)) / effectThreshold * 4000 + 100, 15, true);
              }
            } else if (soundEffect == ALIEN)
            {
              if (currentMillis - lastSoundEffectMillis >= 500 + (((double)filtered_distance) / effectThreshold * 500))
              {
                lastSoundEffectMillis = currentMillis;
                if (volume == 1)
                {
                  digitalWrite(SPK_POS, HIGH);

                  toneAC2(SPK_NEG, SPK_DC, ((double)(effectThreshold - filtered_distance)) / effectThreshold * 1000 + 1000, 150, true);

                } else {
                  toneAC2(SPK_NEG, SPK_POS, ((double)(effectThreshold - filtered_distance)) / effectThreshold * 1000 + 1000, 150, true);
                }

              }

            }
          } else {

            digitalWrite(SPK_NEG, LOW);
            digitalWrite(SPK_POS, LOW);
          }
        }
      }
#endif
      lcdWrite();

#ifdef DEBUG
      //Serial.print(distanceSensor.getInterruptPolarity()); //Default high
      PR("\tProcess (ms): ");
      Serial.print(millis() - testMillis);
      PL("");
#endif

    }
  }
  /* check sensor and display low battery when cannot communicate. 
     (2.6V is the sensor min, but things will work down to much lower). */

  if (currentMillis - lastSensorReadMillis > 5000 && powerState) //if the sensor is not read, the battery is too low
  {
    lcdClear();
    LCDBuffer[0] = space;
    LCDBuffer[1] = letterL;
    LCDBuffer[2] = 0; //Letter O
    LCDBuffer[3] = space;
    lcdPrint(LCDBuffer);

    lcdWrite();
    delay(10);
  }

  if (menuActive && powerState)
  {
    if (runTest)
    {
      runTests();
      runTest = false;
    }
    if (calibrating)
    {
      calibrate();
      calibrating = false;
    }
    
	/* Clear calibration data */
    if (clearCalib)
    {
      offset = 0;
      crosstalk = 0;

      /* Turn off LED */
#ifdef PROTOTYPE
      digitalWrite(LED, HIGH);
#else
      digitalWrite(LED, LOW);
#endif
      /* Set pattern to blank EEPROM */
      EEPROM.update(0x00, 0x00); 

      EEPROM.update(0x10, 0x00);
      EEPROM.update(0x11, 0x00);
      EEPROM.update(0x20, 0x00);
      EEPROM.update(0x21, 0x00);
      clearCalib = false;

      distanceSensor.setOffset(0);
      distanceSensor.setXTalk(0);
      delay(800);
      /* Turn on LED */
#ifdef PROTOTYPE
      digitalWrite(LED, LOW);
#else
      digitalWrite(LED, HIGH);
#endif

      measurementModeChange = true;
    }
	
    if (menuDisplayChanged) {
      MenuItem currentMenu = menu.getCurrent();
      lcdClear();

      LCDBuffer[0] = space;
      LCDBuffer[1] = space;
      LCDBuffer[2] = space;
      LCDBuffer[3] = space;

      if (currentMenu == MSpeed20) {
        LCDBuffer[0] = letterS;
        LCDBuffer[1] = dash;
        LCDBuffer[2] = 2;
        LCDBuffer[3] = 0;
      }
      else if (currentMenu == MSpeed5) {
        LCDBuffer[0] = letterS;
        LCDBuffer[1] = dash;
        LCDBuffer[3] = 5;
      }
      else if (currentMenu == MSpeed1) {
        LCDBuffer[0] = letterS;
        LCDBuffer[1] = dash;
        LCDBuffer[3] = 1;
      }
      else if (currentMenu == MDP_MM) { //MM
        LCDBuffer[0] = letterD;
        LCDBuffer[1] = dash;
        LCDBuffer[2] = letterM;
        LCDBuffer[3] = letterM;
        lcdPrintDP(decimalPointPosition);
      }
      else if (currentMenu == MDP_CM) {//CM
        LCDBuffer[0] = letterD;
        LCDBuffer[1] = dash;
        LCDBuffer[2] = letterC;
        LCDBuffer[3] = letterM;
        lcdPrintDP(decimalPointPosition);
      }
      else if (currentMenu == MDP_M) { //M
        LCDBuffer[0] = letterD;
        LCDBuffer[1] = dash;
        LCDBuffer[3] = letterM;
        lcdPrintDP(decimalPointPosition);
      }
      else if (currentMenu == MUnitsF) {
        LCDBuffer[0] = letterU;
        LCDBuffer[1] = dash;
        LCDBuffer[2] = letterF;
        LCDBuffer[3] = letterE;
      }
      else if (currentMenu == MUnitsI) {
        LCDBuffer[0] = letterU;
        LCDBuffer[1] = dash;
        LCDBuffer[2] = 1; //Letter I
        LCDBuffer[3] = letterN;
      }
      else if (currentMenu == MUnitsFI) {
        LCDBuffer[0] = letterU;
        LCDBuffer[1] = dash;
        LCDBuffer[2] = letterF;
        LCDBuffer[3] = 1; //Letter I
      }
      else if (currentMenu == MUnitsM) {
        LCDBuffer[0] = letterU;
        LCDBuffer[1] = dash;
        LCDBuffer[2] = 5; //Letter S
        LCDBuffer[3] = 1; //Letter I
      }
      else if (currentMenu == MFOV15) {
        LCDBuffer[0] = letterF;
        LCDBuffer[1] = dash;
        LCDBuffer[2] = 1;
        LCDBuffer[3] = 5;
      }
      else if (currentMenu == MFOV21) {
        LCDBuffer[0] = letterF;
        LCDBuffer[1] = dash;
        LCDBuffer[2] = 2;
        LCDBuffer[3] = 1;
      }
      else if (currentMenu == MFOV27) {
        LCDBuffer[0] = letterF;
        LCDBuffer[1] = dash;
        LCDBuffer[2] = 2;
        LCDBuffer[3] = 7;
      }
      else if (currentMenu == MVolOff) {
        LCDBuffer[0] = letterL;
        LCDBuffer[1] = dash;
        LCDBuffer[2] = 0; //Letter O
        LCDBuffer[3] = letterF;
      }
      else if (currentMenu == MVolLo) {
        LCDBuffer[0] = letterL;
        LCDBuffer[1] = dash;
        LCDBuffer[2] = letterL;
        LCDBuffer[3] = 0; //Letter O
      }
      else if (currentMenu == MVolHi) {
        LCDBuffer[0] = letterL;
        LCDBuffer[1] = dash;
        LCDBuffer[2] = letterH;
        LCDBuffer[3] = 1; //Letter I
      }
      else if (currentMenu == MDebugTest) {
        LCDBuffer[0] = letterD;
        LCDBuffer[1] = 8; //letterB
        LCDBuffer[2] = space;
        LCDBuffer[3] = 7; //letterT
      }
      else if (currentMenu == MDebugCalib) {
        LCDBuffer[0] = letterD;
        LCDBuffer[1] = 8; //letterB
        LCDBuffer[2] = space;
        LCDBuffer[3] = letterC;
      }
      else if (currentMenu == MDebugDelete) {
        LCDBuffer[0] = letterD;
        LCDBuffer[1] = 8; //letterB
        LCDBuffer[2] = space;
        LCDBuffer[3] = letterD;
      }
      else if (currentMenu == MEffect0) {
        LCDBuffer[0] = letterE;
        LCDBuffer[1] = dash;
        LCDBuffer[2] = space;
        LCDBuffer[3] = soundEffect;
      }
      else if (currentMenu == MThresh0) {
        LCDBuffer[0] = letterH;
        LCDBuffer[1] = dash;

        uint32_t distanceInches = effectThreshold * 39.3701;
        uint32_t distanceFeet = distanceInches / 12;

        if (units == METRIC) {
          LCDBuffer[2] = effectThreshold / 1000;
          LCDBuffer[3] = (effectThreshold / 100) % 10;
          lcdPrintDP(3);
        } else if (units == FEET || units == FEETINCH)
        {
          //Threshold never goes above 9 feet
          LCDBuffer[2] = distanceFeet / 1000;
          LCDBuffer[3] = (distanceFeet / 100) % 10;
          lcdPrintDP(3);
        }
        else if (units == INCH)
        {
          //Threshold never goes above 9 feet
          LCDBuffer[2] = distanceInches / 1000;
          LCDBuffer[3] = (distanceInches / 100) % 10;
        }
      }
      else if (currentMenu == MSpeed) {
        LCDBuffer[0] = letterS;
      }
      else if (currentMenu == MDP) {
        LCDBuffer[0] = letterD;
      }
      else if (currentMenu == MUnits) {
        LCDBuffer[0] = letterU;
      }
      else if (currentMenu == MVol) {
        LCDBuffer[0] = letterL; //Loudness
      }
      else if (currentMenu == MFOV) {
        LCDBuffer[0] = letterF;
      }
      else if (currentMenu == MEffect) {
        LCDBuffer[0] = letterE;
      }
      else if (currentMenu == MThresh) {
        LCDBuffer[0] = letterH;
      }

      lcdPrint(LCDBuffer);

      lcdWrite();
      menuDisplayChanged = false;

#ifdef DEBUG
      Serial.println(menu.getCurrent().getName());
#endif
    }

    /* Turn on LED */
#ifdef PROTOTYPE
    digitalWrite(LED, LOW);
#else
    digitalWrite(LED, HIGH);
#endif
  } else {
    /* Turn off LED */
#ifdef PROTOTYPE
    digitalWrite(LED, HIGH);
#else
    digitalWrite(LED, LOW);
#endif
  }

  if (currentMillis - previousButtonEventMillis > 5000) //Button millis event timer times out after 5 seconds
  {
#ifndef DEBUG
    /* We're down to about 158uA in sleep */
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); //BOD, WDT and PCINT/INT still active
#endif
    /* BOD uses about 20uA @3V *
    /* MCU in sleep uses 1-1.5uA @3V */
    /* Charger uses 2uA when not charging and VCC floating */
    /* 2.8V regulator en pin uses 1uA and 1uA when in shutdown (2uA). Regulator quiescent is 29uA when operating. */

    /* Reset menu if active. */
    if (menuActive) 
    {
      menuActive = false;
      effectStartDelay = EFFECT_DELAY;
    }
  } 
  
  /* We don't sleep when button mode active (clicked) */

  if (digitalRead(BUTTON) == LOW) /* Needed for long press (interrupt only fires on state change). */
  {
    button.tick();
    delay(1);
  } else {

    if (currentMillis - previousButtonEventMillis < 5000)
    {
      /* When button interraction is recent, process all ticks */
      button.tick(); 

      set_sleep_mode(SLEEP_MODE_IDLE);

    } else {
      if (!toneRunning()) {
        if (soundEffect == 2 && volume > 0) //Alien needs millis
        {
          set_sleep_mode(SLEEP_MODE_IDLE);
        } else {
          set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        }

      } else {
        //delay(1);
        set_sleep_mode(SLEEP_MODE_IDLE);
      }
    }
    noInterrupts();
    sleep_enable();
	
	/* disable BOD during sleep to save more power; will reactivate when waken */
    /* BOD disable must be done just before sleep */
    /* turn off brown-out enable in software; this only works in power down, power save and standby modes */
    MCUCR = bit (BODS) | bit (BODSE);
    MCUCR = bit (BODS);
    interrupts ();             // guarantees next instruction executed
    sleep_cpu ();              // sleep within 3 clock cycles of above
    sleep_disable();
  }

  if (measurementModeChange && !menuActive)
  {
    measurementModeChange = false;
    distanceSensor.stopRanging();

    distanceSensor.setDistanceModeLong();

    switch (measurementMode)
    {
      case TWENTY_HZ: //20Hz
        {
          measurement_budget = 50;
          distanceSensor.setTimingBudgetInMs(measurement_budget);
          distanceSensor.setIntermeasurementPeriod(measurement_budget);
          break;
        }
      case FIVE_HZ: //5 Hz
        {
          measurement_budget = 100;
          distanceSensor.setTimingBudgetInMs(measurement_budget);
          distanceSensor.setIntermeasurementPeriod(200);
          break;
        }
      case ONE_HZ: //1 Hz
        {
          measurement_budget = 200;
          distanceSensor.setTimingBudgetInMs(measurement_budget);
          distanceSensor.setIntermeasurementPeriod(1000);
          break;
        }
    }
	
	/* Optical center 191
    void setROI(uint8_t x, uint8_t y, uint8_t opticalCenter); 
	Set the height and width of the ROI(region of interest) in SPADs, 
	lowest possible option is 4. Set optical center based on above table */

    if (fov == FOV15)
    {
      distanceSensor.setROI(4, 4, 191);
    } else if (fov == FOV21)
    {
      distanceSensor.setROI(8, 8, 191);
    } else {
		
      /* set to 27 degrees */
      distanceSensor.setROI(16, 16, 191);
    }

    if ( EEPROM.read(0x00) == 0xAA) //Check pattern for blank EEPROM
    {
      offset = (int16_t)EEPROM.read(0x10) | ((int16_t)EEPROM.read(0x11)) << 8;
      crosstalk = (uint16_t)EEPROM.read(0x20) | ((uint16_t)EEPROM.read(0x21)) << 8;
      distanceSensor.setOffset(offset);
      distanceSensor.setXTalk(crosstalk);
    }

#ifdef DEBUG
    PR("Mode: ");
    Serial.print(measurementMode);
    PR(" Inter: ");
    Serial.print(distanceSensor.getIntermeasurementPeriod());
    PR(" Budget: ");
    Serial.print(distanceSensor.getTimingBudgetInMs());
    PR(" ROI: x-");
    Serial.print(distanceSensor.getROIX());
    PR(" y-");
    Serial.println(distanceSensor.getROIY());
#endif

    distanceSensor.startRanging();
  }

  //TODO de-boilerplate this
  if (powerStateChange)
  {
    powerStateChange = false;
	
	/* Turn on badge */
    if (powerState)
    {
      /* Turn on LED */
#ifdef PROTOTYPE
      //digitalWrite(LED, LOW);
#else
      //digitalWrite(LED, HIGH);
#endif

      /* Turn 2.8V Regulator */
      digitalWrite(REGULATOR_2_8, HIGH); //Need to be set first
      delay(2);
      digitalWrite(SHUTDOWN_PIN, HIGH);

      /* Turn on LCD */
      digitalWrite(LCD_POWER, HIGH);

      /* Turn on Pullup */
      pinMode(INTERRUPT_PIN, INPUT_PULLUP);

      delay(500);

      lcdBegin();
      lcdBegin();

      while (distanceSensor.checkBootState() == false)
      {
        delay(2);
      }

      if (distanceSensor.begin() == 0) //Begin returns 0 on a good init
      {
#ifdef DEBUG
        PL("Sensor OK!");
#endif
      }
      if (volume > 0) {
        toneAC2(SPK_NEG, SPK_POS, 600, 100);
        toneAC2(SPK_NEG, SPK_POS, 1500, 100);
        toneAC2(SPK_NEG, SPK_POS, 1000, 100);
      }

      distanceSensor.startTemperatureUpdate();

      distanceSensor.setInterruptPolarityLow();

      distanceSensor.setDistanceModeLong();

      switch (measurementMode)
      {
        case TWENTY_HZ: //20Hz
          {
            distanceSensor.setTimingBudgetInMs(50);
            distanceSensor.setIntermeasurementPeriod(50);
            break;
          }
        case FIVE_HZ: //5 Hz
          {
            distanceSensor.setTimingBudgetInMs(100);
            distanceSensor.setIntermeasurementPeriod(200);
            break;
          }
        case ONE_HZ: //1 Hz
          {
            distanceSensor.setTimingBudgetInMs(100);
            distanceSensor.setIntermeasurementPeriod(1000);
            break;
          }
      }
      if (fov == FOV15)
      {
        distanceSensor.setROI(4, 4, 191);
      } else if (fov == FOV21)
      {
        distanceSensor.setROI(8, 8, 191);
      } else {
        /* set to 27 degrees */
        distanceSensor.setROI(16, 16, 191);
      }

      if ( EEPROM.read(0x00) == 0xAA) //Check pattern for blank EEPROM
      {
        offset = (int16_t)EEPROM.read(0x10) | ((int16_t)EEPROM.read(0x11)) << 8;
        crosstalk = (uint16_t)EEPROM.read(0x20) | ((uint16_t)EEPROM.read(0x21)) << 8;
        distanceSensor.setOffset(offset);
        distanceSensor.setXTalk(crosstalk);
      }

      /* Turn on VL53L1 no interrupt timer */
      TCCR4B = (0 << WGM43) | (0 << WGM42)                /* TC16 Mode 0 Normal */
               | 0 << ICNC4                               /* Input Capture Noise Canceler: disabled */
               | 0 << ICES4                               /* Input Capture Edge Select: disabled */
               //| (0 << CS42) | (1 << CS41) | (1 << CS40); /* IO clock divided by 64 */
               | (1 << CS42) | (0 << CS41) | (0 << CS40); /* IO clock divided by 256 */

      distanceSensor.startRanging();

    } else {
      distanceSensor.stopRanging();
      lcdClear();
      lcdWrite();
      delay(200);

      if (volume > 0) {
        toneAC2(SPK_NEG, SPK_POS, 200, 140);
      }

      /* Turn off LED */
#ifdef PROTOTYPE
      digitalWrite(LED, HIGH);
#else
      digitalWrite(LED, LOW);
#endif

      /* Disable no VL53L1 interrupt timer */
      TCCR4B = (0 << WGM43) | (1 << WGM42) /* TC16 Mode 0 Normal */
               | 0 << ICNC4 /* Input Capture Noise Canceler: disabled */
               | 0 << ICES4 /* Input Capture Edge Select: disabled */
               | (0 << CS42) | (0 << CS41) | (0 << CS40); /* No clock source (Timer/Counter stopped) */

      menuActive = false;

      /* Turn off 2.8V Regulator */
      digitalWrite(SHUTDOWN_PIN, LOW); //Needs to be set first on shutdown
      delay(2);
      digitalWrite(REGULATOR_2_8, LOW);
      //pinMode(REGULATOR_2_8, INPUT);

      /* Turn off LCD */
      digitalWrite(LCD_POWER, LOW);
      //pinMode(LCD_POWER, INPUT);


      /* Turn off pullup */
      pinMode(INTERRUPT_PIN, INPUT);

      measurementInterrupt = false;

      /* disable any button events to go to sleep straight away */
      previousButtonEventMillis = previousButtonEventMillis - 10000; 

    }
  }

}
