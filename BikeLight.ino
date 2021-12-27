/*
  Simple Poc for TinyZero that use the RTC, accelerometer and led array to demonstrate a light function
  
 */

#include <RTCZero.h>  // for the RTC support - same as Arduino Zero
#include <Wire.h>         // For I2C communication with sensor
#include <Adafruit_NeoPixel.h> // For the neopixel integration
#include "BMA250.h"   // and the accelerometer

#if defined (ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#include <SoftwareSerial.h>
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif

#define PIN 6
#define NUMPIXELS 7

RTCZero rtc; // create an rtc object
// needs ‘w’ for the white element
//Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_RGBW + NEO_KHZ800);

// sequeneces
#define TYPE_BREATH  0
#define TYPE_FAST 1
#define TYPE_MEDIUM 2
#define TYPE_SLOW 3
#define LIGHTTYPES 4

int lightType = TYPE_FAST; // the default
const int buttonPin = 2;

int previousState = HIGH;
//unsigned int previousPress = 0; // initailised as 0
int buttonState; // not used
int buttonFlag = 0;
int buttonDebounce = 200; // found to be 120-150 for the simple buttons


/* Change these values to set the current initial time */
const byte hours = 13;
const byte minutes = 0;
const byte seconds = 0;

/* Change these values to set the current initial date */
const byte day = 1;
const byte month = 12;
const byte year = 21;

const byte secDelay = 30;
int timeDelay = 0;
int sensitivity = 5;
byte Debug = true; // this could be a #def

bool lightRunning = false;
bool motionInt = false; /* we assume motion to start with */
bool prevMotionInt = false;
bool motionInterrupt = false; // used to flag motion
//int orgPos = 0;

BMA250 accel_sensor;
int x, y, z;
double temp;

void setup() {
  int leds = 0; // num to light to indicate battery status
  
  // put your setup code here, to run once:
  if (Debug == true) SerialMonitorInterface.begin(115200);
  rtc.begin(); // initialize RTC
  Wire.begin(); // enable access to the BMA250

  // Set the time
  rtc.setHours(hours);
  rtc.setMinutes(minutes);
  rtc.setSeconds(seconds);

  // Set the date
  rtc.setDay(day);
  rtc.setMonth(month);
  rtc.setYear(year);

  // note this is the time and not the delay
  // if you set this to 30 it will trigger every minute when the seconds are 30!
  timeDelay = secDelay;
  // this ensures the port is connnected and timeouts if not
  // this means it works if usb is connected or not
  if (Debug == true) while (!SerialMonitorInterface  && (millis() < 2500));

  rtc.setAlarmSeconds (secDelay);
  rtc.enableAlarm(rtc.MATCH_SS);

  rtc.attachInterrupt(alarmMatch); /* call back routine */
  lightRunning = false;

  // bma250 bit
  if (Debug == true) SerialMonitorInterface.println("Initialize BMA250 ...");
  accel_sensor.begin(BMA250_range_2g, BMA250_update_time_64ms); 

  /* wait for serial to be available and fl ag start */

  // initialise and turn leds off?
  pixels.begin();
  pixels.show(); // all pixels to 'off'
  
  //if (Debug == true) SerialMonitorInterface.println("Starting ...");
  //LedOn(0); // turnoff leds

  // setup interrupt for button
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonInterrupt, CHANGE);

  // for bma250 interrupts
  //pinMode(1, INPUT_PULLDOWN);
  pinMode(1, INPUT_PULLDOWN); //BMA raises pin high on interrupt
  attachInterrupt(digitalPinToInterrupt(1), BMAInterrupt, CHANGE);

}

// put your main code here, to run repeatedly:
void loop(){
  static int orgPos = 0;
  int pos = 0;

  // check for button press
  //if((millis() - previousPress) > buttonDebounce && buttonFlag)
  //{
  if (buttonFlag) {
    lightType = (lightType + 1) % LIGHTTYPES;
    buttonFlag = 0;
  }

  if (lightRunning == true) {

    switch (lightType) {
      case TYPE_BREATH:
        pixelsBreath(15);
        break;
      case TYPE_FAST:
        pixelsFlash(25);
        break;
      case TYPE_MEDIUM:
        pixelsFlash(50);
        break;
      case TYPE_SLOW:
        pixelsFlash(100);
        break;
      default:
        pixelsBreath(15);
    }

  } else { 
    pixels.clear(); // ensure all off
    pixels.show();
    delay (100); // greater than 64msec
  }


  
  /* here we check the bma250 to see if there has been a change */
  /* we just set the motion flag if there appears to have been motion to simulate an interupt */

  accel_sensor.read();
  x = accel_sensor.X;
  y = accel_sensor.Y;
  z = accel_sensor.Z;
  
  pos = sqrt(sq(x) + sq(y) + sq(z)); // 3dim 'position'
  if (orgPos == 0) orgPos = pos; // sets initial 'pos'

  //if (Debug == true) SerialMonitorInterface.print(pos - orgPos); 
  //if (pos >= orgPos + sensitivity || pos <= orgPos - sensitivity) { // have we moved?
  if (motionInterrupt == true) { // relies on interrupt routine
    if (Debug == true) SerialMonitorInterface.println("Motion detected");
    motionInt = true;
    // need to add a 500msec gap
    if (prevMotionInt == false) { // guard to stop battery display if continual motion
      displayBatt();
      prevMotionInt = true; // only reset on inactivety
    }
    
    lightRunning = true; // we also set this in case it was stopped
    

    if (Debug == true) {
      SerialMonitorInterface.print("Pos "); 
      SerialMonitorInterface.print(pos);
      SerialMonitorInterface.print(" ,");
      SerialMonitorInterface.println(orgPos);
    }

    //accel_sensor.readInter();
    SerialMonitorInterface.print("interruptStatus: ");
    SerialMonitorInterface.println(accel_sensor.interruptStatus);
    motionInterrupt = false;
  }
  orgPos = pos;

}

void BMAInterrupt()
{
  accel_sensor.readInter(); // read the type of interrupt 
  if (Debug == true) SerialMonitorInterface.println("BMA interrupt");
  motionInterrupt = true;
}

void buttonInterrupt()
{
  static unsigned int previousPress = 0;  // retain the last press

  /*
  if (Debug == true) {
    SerialMonitorInterface.print("Button pressed: ");
    //SerialMonitorInterface.print(millis());
    //SerialMonitorInterface.print(", ");
    SerialMonitorInterface.println(millis() - previousPress);
  }
  */

  // the buttonDebounce should be suffixcient but buttonFlag added as an extra guard
  if((millis() - previousPress) > buttonDebounce && buttonFlag == 0) {
    //if (Debug == true) SerialMonitorInterface.println("Press debounced");
        
    previousPress = millis();
    buttonFlag = 1;
  }
  // ensure lights are on?
  //lightRunning = true;
}

// flash the leds
void pixelsFlash(int lightSpeed) {

    int cnt = 200/lightSpeed; // slow the speed reduces the loops

    for (int i = 1; i < cnt; i++) { // do this 10 times or around 1 second

      for (int i = 0; i < NUMPIXELS; i++) { 
        pixels.setPixelColor(i, pixels.Color(255, 255, 255, 255));
      }

      pixels.show();
      delay(lightSpeed * 10); // on
      pixels.clear();
      pixels.show();
      delay(lightSpeed * 5); // off
    } 
} // end of flash

void pixelsBreath(int lightDelay) { 
    int step = 4; // adjust the speed
      
    pixels.clear();

    // try setStrip?
    for (int i = 0; i < NUMPIXELS; i++) { 
      pixels.setPixelColor(i, pixels.Color(255, 255, 255, 255));
    }

    for (int i = 30; i < 255; i = i + step) { pixels.setBrightness(i); pixels.show(); delay(lightDelay); }
    for (int i = 255; i > 30; i = i - step) { pixels.setBrightness(i); pixels.show(); delay(lightDelay); }

} // end of breath


void alarmMatch() {

    /* if we've seen motion then continue to light leds */
    if (motionInt == true) { // we had motion during the interval
      lightRunning = true;
    } else {
      lightRunning = false;
      prevMotionInt = false;
      //LedOn(0); // turnoff leds
    }
    //prevMotionInt = false;
    motionInt = false; // and reset the flag

    // this assumes delay less than a minute
    timeDelay = timeDelay + secDelay;
    if (timeDelay >= 60) timeDelay = timeDelay % 60;

    rtc.setAlarmSeconds(timeDelay); // we just change the seconds for the check
    //rtc.enableAlarm(rtc.MATCH_SS);
    //rtc.attachInterrupt(alarmMatch);

    if (Debug == true){
      SerialMonitorInterface.print("Alarm reset: ");
      print2digits(rtc.getHours());
      SerialMonitorInterface.print(":");
      print2digits(rtc.getMinutes());
      SerialMonitorInterface.print(":");
      print2digits(rtc.getSeconds());
      SerialMonitorInterface.println();
    }
}

// routine lights a number of leds to indicate battery charge based on measured voltage
void displayBatt() {
    int leds = 0;
    
    float battVoltageReading = getBattVoltage();
    if (Debug == true) SerialMonitorInterface.println(battVoltageReading);

    // discharged is roughly 3.3 
    leds = int( (battVoltageReading - 3.3) * 7);

    for (int i = 0; i < leds; i++) {
      // this is GRBW
      pixels.setPixelColor(i, pixels.Color(255, 0, 0, 0));
      
      pixels.show();
      delay(300);
    }
    delay (800);
    
  pixels.clear();
  pixels.show(); // all pixels to 'off'
}

// This function gets the battery VCC internally, you can checkout this link 
// if you want to know more about how: 
// http://atmel.force.com/support/articles/en_US/FAQ/ADC-example
float getVCC() {
  SYSCTRL->VREF.reg |= SYSCTRL_VREF_BGOUTEN;
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->SAMPCTRL.bit.SAMPLEN = 0x1;
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->INPUTCTRL.bit.MUXPOS = 0x19;         // Internal bandgap input
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->SWTRIG.bit.START = 1;  // Start conversion
  ADC->INTFLAG.bit.RESRDY = 1;  // Clear the Data Ready flag
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->SWTRIG.bit.START = 1;  // Start the conversion again to throw out first value
  while ( ADC->INTFLAG.bit.RESRDY == 0 );   // Waiting for conversion to complete
  uint32_t valueRead = ADC->RESULT.reg;
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  SYSCTRL->VREF.reg &= ~SYSCTRL_VREF_BGOUTEN;
  float vcc = (1.1 * 1023.0) / valueRead;
  return vcc;
}

// Calculate the battery voltage
float getBattVoltage(void) {
  const int VBATTpin = A4;
  float VCC = getVCC();

  // Use resistor division and math to get the voltage
  float resistorDiv = 0.5;
  float ADCres = 1023.0;
  float battVoltageReading = analogRead(VBATTpin);
  battVoltageReading = analogRead(VBATTpin); // Throw out first value
  float battVoltage = VCC * battVoltageReading / ADCres / resistorDiv;

  return battVoltage;
}
/* turns a led on and off */
void LedOn(int ledNum)
{
  for(int i = 5; i < 10; i++) {
    pinMode(i, INPUT);
    digitalWrite(i, LOW);
  };

  if(ledNum < 1 || ledNum > 16) return;

  char highpin[16] = {5,6,5,7,6,7,6,8,5,8,8,7,9,7,9,8};
  char lowpin[16] = {6,5,7,5,7,6,8,6,8,5,7,8,7,9,8,9};
  ledNum--;

  digitalWrite(highpin[ledNum], HIGH);
  digitalWrite(lowpin[ledNum], LOW);
  pinMode(highpin[ledNum], OUTPUT);
  pinMode(lowpin[ledNum], OUTPUT);
}

/* routine for printing hours/mintues/seconds */
void print2digits(int number) {
  if (number < 10) {
    SerialMonitorInterface.print("0"); // print a 0 before if the number is < than 10
  }
  SerialMonitorInterface.print(number);
}
