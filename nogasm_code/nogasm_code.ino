// Protogasm Code, forked from Nogasm Code Rev. 3
/* Drives a vibrator and uses changes in pressure of an inflatable buttplug
 * to estimate a user's closeness to orgasm, and turn off the vibrator
 * before that point.
 * A state machine updating at 60Hz creates different modes and option menus
 * that can be identified by the color of the LEDs, especially the RGB LED
 * in the central button/encoder knob.
 * 
 * [Red]    Manual Vibrator Control
 * [Blue]   Automatic vibrator edging, knob adjusts orgasm detection sensitivity
 * [Green]  Setting menu for maximum vibrator speed in automatic mode
 * [White]  Debubbing menu to show data from the pressure sensor ADC
 * [Off]    While still plugged in, holding the button down for >3 seconds turns
 *          the whole device off, until the button is pressed again.
 * 
 * Settings like edging sensitivity, or maximum motor speed are stored in EEPROM,
 * so they are saved through power-cycling.
 * 
 * In the automatic edging mode, the vibrator speed will linearly ramp up to full
 * speed (set in the green menu) over 30 seconds. If a near-orgasm is detected,
 * the vibrator abruptly turns off for 15 seconds, then begins ramping up again.
 * 
 * The motor will beep during power on/off, and if the plug pressure rises above
 * the maximum the board can read - this condition could lead to a missed orgasm 
 * if unchecked. The analog gain for the sensor is adjustable via a trimpot to
 * accomidate different types of plugs that have higher/lower resting pressures.
 * 
 * Motor speed, current pressure, and average pressure are reported via USB serial
 * at 115200 baud. Timestamps can also be enabled, from the main loop.
 * 
 * There is some framework for more features like an adjustable "cool off" time 
 * other than the default 15 seconds, and options for LED brightness and enabling/
 * disabling beeps.
 * 
 * Note - Do not set all the LEDs to white at full brightness at once
 * (RGB 255,255,255) It may overheat the voltage regulator and cause the board 
 * to reset.
 */
//=======Libraries===============================
#include <Encoder.h>
#include <EEPROM.h>
#include "FastLED.h"
#include "RunningAverage.h"

//=======Hardware Setup===============================
//LEDs
#define NUM_LEDS 24
#define LED_PIN 10
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define BRIGHTNESS 50 //Subject to change, limits current that the LEDs draw

//Encoder
#define ENC_SW   5 //Pushbutton on the encoder
Encoder myEnc(3, 2); //Quadrature inputs
#define ENC_SW_UP   HIGH
#define ENC_SW_DOWN LOW

//Motor
#define MOTPIN 9

//Pressure Sensor Analog In
#define BUTTPIN A0
// Sampling 4x and not dividing keeps the samples from the Arduino Uno's 10 bit
// ADC in a similar range to the Teensy LC's 12-bit ADC.  This helps ensure the
// feedback algorithm will behave similar to the original.
#define OVERSAMPLE 4
#define ADC_MAX 1023

//=======Software/Timing options=====================
#define FREQUENCY 60 //Update frequency in Hz
#define LONG_PRESS_MS 600 //ms requirements for a long press, to move to option menus
#define V_LONG_PRESS_MS 2500 //ms for a very long press, which turns the device off

//Update/render period
#define period (1000/FREQUENCY)
#define longBtnCount (LONG_PRESS_MS / period)

//Running pressure average array length and update frequency
#define RA_HIST_SECONDS 25
#define RA_FREQUENCY 6
#define RA_TICK_PERIOD (FREQUENCY / RA_FREQUENCY)
RunningAverage raPressure(RA_FREQUENCY*RA_HIST_SECONDS);
int sensitivity = 0; //orgasm detection sensitivity, persists through different states

//=======State Machine Modes=========================
#define MANUAL      1
#define AUTO        2
#define OPT_SPEED   3
#define OPT_RAMPSPD 4
#define OPT_BEEP    5
#define OPT_PRES    6


//Button states - no press, short press, long press
#define BTN_NONE   0
#define BTN_SHORT  1
#define BTN_LONG   2
#define BTN_V_LONG 3


uint8_t state = MANUAL;
//=======Global Settings=============================
#define MOT_MAX 255 // Motor PWM maximum
#define MOT_MIN 20  // Motor PWM minimum.  It needs a little more than this to start.

CRGB leds[NUM_LEDS];

int pressure = 0;
int avgPressure = 0; //Running 25 second average pressure
//int bri =100; //Brightness setting
int rampTimeS = 30; //Ramp-up time, in seconds
#define DEFAULT_PLIMIT 600
int pLimit = DEFAULT_PLIMIT; //Limit in change of pressure before the vibrator turns off
int maxSpeed = 255; //maximum speed the motor will ramp up to in automatic mode
float motSpeed = 0; //Motor speed, 0-255 (float to maintain smooth ramping to low speeds)

//=======EEPROM Addresses============================
//128b available on teensy LC
#define BEEP_ADDR         1
#define MAX_SPEED_ADDR    2
#define SENSITIVITY_ADDR  3
//#define RAMPSPEED_ADDR    4 //For now, ramp speed adjustments aren't implemented

//=======Setup=======================================
//Beep out tones over the motor by frequency (1047,1396,2093) may work well
void beep_motor(int f1, int f2, int f3){
  analogWrite(MOTPIN, 0);
  tone(MOTPIN, f1);
  delay(250);
  tone(MOTPIN, f2);
  delay(250);
  tone(MOTPIN, f3);
  delay(250);
  noTone(MOTPIN);
  analogWrite(MOTPIN,motSpeed);
}

void setup() {
  pinMode(ENC_SW,   INPUT); //Pin to read when encoder is pressed
  digitalWrite(ENC_SW, HIGH); // Encoder switch pullup

  analogReference(EXTERNAL);

  // Classic AVR based Arduinos have a PWM frequency of about 490Hz which
  // causes the motor to whine.  Change the prescaler to achieve 31372Hz.
  //sbi(TCCR1B, CS10);
  //cbi(TCCR1B, CS11);
  //cbi(TCCR1B, CS12);
  TCCR1B |= (1<<CS10);
  TCCR1B |= (1<<CS11);
  TCCR1B |= (1<<CS12);

  pinMode(MOTPIN,OUTPUT); //Enable "analog" out (PWM)
  
  pinMode(BUTTPIN,INPUT); //default is 10 bit resolution (1024), 0-3.3
  
  raPressure.clear(); //Initialize a running pressure average

  digitalWrite(MOTPIN, LOW);//Make sure the motor is off

  delay(3000); // 3 second delay for recovery

  Serial.begin(115200);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  // limit power draw to .6A at 5v... Didn't seem to work in my FastLED version though
  //FastLED.setMaxPowerInVoltsAndMilliamps(5,DEFAULT_PLIMIT);
  FastLED.setBrightness(BRIGHTNESS);

  //Recall saved settings from memory
  sensitivity = EEPROM.read(SENSITIVITY_ADDR);
  maxSpeed = min(EEPROM.read(MAX_SPEED_ADDR),MOT_MAX); //Obey the MOT_MAX the first power cycle after changing it.
  beep_motor(1047,1396,2093); //Power on beep
}

//=======LED Drawing Functions=================

//Draw a "cursor", one pixel representing either a pressure or encoder position value
//C1,C2,C3 are colors for each of 3 revolutions over the 13 LEDs (39 values)
void draw_cursor_3(int pos,CRGB C1, CRGB C2, CRGB C3){
  pos = constrain(pos,0,NUM_LEDS*3-1);
  int colorNum = pos/NUM_LEDS; //revolution number
  int cursorPos = pos % NUM_LEDS; //place on circle, from 0-12
  switch(colorNum){
    case 0:
      leds[cursorPos] = C1;
      break;
    case 1:
      leds[cursorPos] = C2;
      break;
    case 2:
      leds[cursorPos] = C3;
      break;
  }
}

//Draw a "cursor", one pixel representing either a pressure or encoder position value
void draw_cursor(int pos,CRGB C1){
  pos = constrain(pos,0,NUM_LEDS-1);
  leds[pos] = C1;
}

//Draw 3 revolutions of bars around the LEDs. From 0-39, 3 colors
void draw_bars_3(int pos,CRGB C1, CRGB C2, CRGB C3){
  pos = constrain(pos,0,NUM_LEDS*3-1);
  int colorNum = pos/NUM_LEDS; //revolution number
  int barPos = pos % NUM_LEDS; //place on circle, from 0-12
  switch(colorNum){
    case 0:
      fill_gradient_RGB(leds,0,C1,barPos,C1);
      //leds[barPos] = C1;
      break;
    case 1:
      fill_gradient_RGB(leds,0,C1,barPos,C2);
      break;
    case 2:
      fill_gradient_RGB(leds,0,C2,barPos,C3);
      break;
  }
}

//Provide a limited encoder reading corresponting to tacticle clicks on the knob.
//Each click passes through 4 encoder pulses. This reduces it to 1 pulse per click
int encLimitRead(int minVal, int maxVal){
  if(myEnc.read()>maxVal*4)myEnc.write(maxVal*4);
  else if(myEnc.read()<minVal*4) myEnc.write(minVal*4);
  return constrain(myEnc.read()/4,minVal,maxVal);
}

//=======Program Modes/States==================

// Manual vibrator control mode (red), still shows orgasm closeness in background
void run_manual() {
  //In manual mode, only allow for 13 cursor positions, for adjusting motor speed.
  int knob = encLimitRead(0,NUM_LEDS-1);
  motSpeed = map(knob, 0, NUM_LEDS-1, 0., (float)MOT_MAX);
  analogWrite(MOTPIN, motSpeed);

  //gyrGraphDraw(avgPressure, 0, 4 * 3 * NUM_LEDS);
  int presDraw = map(constrain(pressure - avgPressure, 0, pLimit),0,pLimit,0,NUM_LEDS*3);
  draw_bars_3(presDraw, CRGB::Green,CRGB::Yellow,CRGB::Red);
  draw_cursor(knob, CRGB::Red);
}

// Automatic edging mode, knob adjust sensitivity.
void run_auto() {
  static float motIncrement = 0.0;
  motIncrement = ((float)maxSpeed / ((float)FREQUENCY * (float)rampTimeS));

  int knob = encLimitRead(0,(3*NUM_LEDS)-1);
  sensitivity = knob*4; //Save the setting if we leave and return to this state
  //Reverse "Knob" to map it onto a pressure limit, so that it effectively adjusts sensitivity
  pLimit = map(knob, 0, 3 * (NUM_LEDS - 1), 600, 1); //set the limit of delta pressure before the vibrator turns off
  //When someone clenches harder than the pressure limit
  if (pressure - avgPressure > pLimit) {
    motSpeed = -.5*(float)rampTimeS*((float)FREQUENCY*motIncrement);//Stay off for a while (half the ramp up time)
  }
  else if (motSpeed < (float)maxSpeed) {
    motSpeed += motIncrement;
  }
  if (motSpeed > MOT_MIN) {
    analogWrite(MOTPIN, (int) motSpeed);
  } else {
    analogWrite(MOTPIN, 0);
  }

  int presDraw = map(constrain(pressure - avgPressure, 0, pLimit),0,pLimit,0,NUM_LEDS*3);
  draw_bars_3(presDraw, CRGB::Green,CRGB::Yellow,CRGB::Red);
  draw_cursor_3(knob, CRGB(50,50,200),CRGB::Blue,CRGB::Purple);

}

//Setting menu for adjusting the maximum vibrator speed automatic mode will ramp up to
void run_opt_speed() {
  Serial.println("speed settings");
  int knob = encLimitRead(0,NUM_LEDS-1);
  motSpeed = map(knob, 0, NUM_LEDS-1, 0., (float)MOT_MAX);
  analogWrite(MOTPIN, motSpeed);
  maxSpeed = motSpeed; //Set the maximum ramp-up speed in automatic mode
  //Little animation to show ramping up on the LEDs
  static int visRamp = 0;
  if(visRamp <= FREQUENCY*NUM_LEDS-1) visRamp += 16;
  else visRamp = 0;
  draw_bars_3(map(visRamp,0,(NUM_LEDS-1)*FREQUENCY,0,knob),CRGB::Green,CRGB::Green,CRGB::Green);
}

//Not yet added, but adjusts how quickly the vibrator turns back on after being triggered off
void run_opt_rampspd() {
  Serial.println("rampSpeed");
}

//Also not completed, option for enabling/disabling beeps
void run_opt_beep() {
  Serial.println("Brightness Settings");
}

//Simply display the pressure analog voltage. Useful for debugging sensitivity issues.
void run_opt_pres() {
  int p = map(analogRead(BUTTPIN),0,ADC_MAX,0,NUM_LEDS-1);
  draw_cursor(p,CRGB::White);
}

//Poll the knob click button, and check for long/very long presses as well
uint8_t check_button(){
  static bool lastBtn = ENC_SW_DOWN;
  static unsigned long keyDownTime = 0;
  uint8_t btnState = BTN_NONE;
  bool thisBtn = digitalRead(ENC_SW);

  //Detect single presses, no repeating, on keyup
  if(thisBtn == ENC_SW_DOWN && lastBtn == ENC_SW_UP){
    keyDownTime = millis();
  }
  
  if (thisBtn == ENC_SW_UP && lastBtn == ENC_SW_DOWN) { //there was a keyup
    if((millis()-keyDownTime) >= V_LONG_PRESS_MS){
      btnState = BTN_V_LONG;
    }
    else if((millis()-keyDownTime) >= LONG_PRESS_MS){
      btnState = BTN_LONG;
      }
    else{
      btnState = BTN_SHORT;
      }
    }

  lastBtn = thisBtn;
  return btnState;
}

//run the important/unique parts of each state. Also, set button LED color.
void run_state_machine(uint8_t state){
  switch (state) {
      case MANUAL:
        run_manual();
        break;
      case AUTO:
        run_auto();
        break;
      case OPT_SPEED:
        run_opt_speed();
        break;
      case OPT_RAMPSPD:
        run_opt_rampspd();
        break;
      case OPT_BEEP:
        run_opt_beep();
        break;
      case OPT_PRES:
        run_opt_pres();
        break;
      default:
        run_manual();
        break;
    }
}

//Switch between state machine states, and reset the encoder position as necessary
//Returns the next state to run. Very long presses will turn the system off (sort of)
uint8_t set_state(uint8_t btnState, uint8_t state){
  if(btnState == BTN_NONE){
    return state;
  }
  if(btnState == BTN_V_LONG){
    //Turn the device off until woken up by the button
    Serial.println("power off");
    fill_gradient_RGB(leds,0,CRGB::Black,NUM_LEDS-1,CRGB::Black);//Turn off LEDS
    FastLED.show();
    analogWrite(MOTPIN, 0);
    beep_motor(2093,1396,1047);
    analogWrite(MOTPIN, 0); //Turn Motor off
    while(!digitalRead(ENC_SW))delay(1);
    beep_motor(1047,1396,2093);
    return MANUAL ;
  }
  else if(btnState == BTN_SHORT){
    switch(state){
      case MANUAL:
        myEnc.write(sensitivity);//Whenever going into auto mode, keep the last sensitivity
        motSpeed = 0; //Also reset the motor speed to 0
        return AUTO;
      case AUTO:
        myEnc.write(0);//Whenever going into manual mode, set the speed to 0.
        motSpeed = 0;
        EEPROM.update(SENSITIVITY_ADDR, sensitivity);
        return MANUAL;
      case OPT_SPEED:
        myEnc.write(0);
        EEPROM.update(MAX_SPEED_ADDR, maxSpeed);
        //return OPT_RAMPSPD;
        //return OPT_BEEP;
        motSpeed = 0;
        analogWrite(MOTPIN, motSpeed); //Turn the motor off for the white pressure monitoring mode
        return OPT_PRES; //Skip beep and rampspeed settings for now
      case OPT_RAMPSPD: //Not yet implimented
        //motSpeed = 0;
        //myEnc.write(0);
        return OPT_BEEP;
      case OPT_BEEP:
        myEnc.write(0);
        return OPT_PRES;
      case OPT_PRES:
        myEnc.write(map(maxSpeed,0,255,0,4*(NUM_LEDS)));//start at saved value
        return OPT_SPEED;
    }
  }
  else if(btnState == BTN_LONG){
    switch (state) {
          case MANUAL:
            myEnc.write(map(maxSpeed,0,255,0,4*(NUM_LEDS)));//start at saved value
            return OPT_SPEED;
          case AUTO:
            myEnc.write(map(maxSpeed,0,255,0,4*(NUM_LEDS)));//start at saved value
            return OPT_SPEED;
          case OPT_SPEED:
            myEnc.write(0);
            return MANUAL;
          case OPT_RAMPSPD:
            return MANUAL;
          case OPT_BEEP:
            return MANUAL;
          case OPT_PRES:
            myEnc.write(0);
            return MANUAL;
        }
  }
  else return MANUAL;
}

//=======Main Loop=============================
void loop() {
  static uint8_t state = MANUAL;
  static int sampleTick = 0;
  //Run this section at the update frequency (default 60 Hz)
  if (millis() % period == 0) {
    delay(1);
    
    sampleTick++; //Add pressure samples to the running average slower than 60Hz
    if (sampleTick % RA_TICK_PERIOD == 0) {
      raPressure.addValue(pressure);
      avgPressure = raPressure.getAverage();
    }
    
    pressure=0;
    for(uint8_t i=OVERSAMPLE; i; --i) {
      pressure += analogRead(BUTTPIN);
      if(i) {      // Don't delay after the last sample
        delay(1);  // Spread samples out a little
      }
    }
    fadeToBlackBy(leds,NUM_LEDS,20); //Create a fading light effect. LED buffer is not otherwise cleared
    uint8_t btnState = check_button();
    state = set_state(btnState,state); //Set the next state based on this state and button presses
    run_state_machine(state);
    FastLED.show(); //Update the physical LEDs to match the buffer in software

    //Alert that the Pressure voltage amplifier is railing, and the trim pot needs to be adjusted
    if(pressure > 4030)beep_motor(2093,2093,2093); //Three high beeps

    //Report pressure and motor data over USB for analysis / other uses. timestamps disabled by default
    //Serial.print(millis()); //Timestamp (ms)
    //Serial.print(",");
    Serial.print(motSpeed); //Motor speed (0-255)
    Serial.print(",");
    Serial.print(pressure); //(Original ADC value - 12 bits, 0-4095)
    Serial.print(",");
    Serial.println(avgPressure); //Running average of (default last 25 seconds) pressure

  }
}
