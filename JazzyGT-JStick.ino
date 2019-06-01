/* Jazzy Select GT Mobility Scooter power base
 * controlling it via direct signaling to the Joystick pigtail on the base controller unit( GC 2 by Pride Mobility )
 * joystick data packet and checksum sent ~10ms
 * immediately reponded to by the base controller with a returned checksum packet
 *
 * jstk pulses low INH 32uS, stick responds with preable 0x52, _79uS_ 0xad
 * base responds with 0x72, 0x10, 0x00, 0x7D
 * NOTE: if this base-response is triggered, the jstik will work without further INH pulses...
 * jstk pulses low INH 32us, stick responds with jstk data
 *
 * v0.1
 * working prototype using the Gravis Joystick and 4 buttons. Starts in PowerOff mode, requires zero'ing of Jstick
 * Zero'ing is done using Base button. Then the Palm button can be used to go into PowerOn mode and trigger the
 * initialization sequence.
 *
 * v0.2
 * adding Wii Nunchuck joystick type and cleaning up buttons for 2 button operation
 * Button 1/C for Power On/Off
 * Button 2/Z for Zero'ing the joystick.
 * TODO: Calibration/Zero'ing will be a long press of a button( > 5 seconds )
 * TODO: Throttle position changes???? X axis tilt/roll? >0 adds, <0 subtracts
 */
/******************************************************************************
* Author: Doug LaRue doug dot larue at gmail dot com
* License: FreeBSD
******************************************************************************/
/******************************************************************************
Copyright (c) 2015, Doug LaRue
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the FreeBSD Project.

*******************************************************************************/


//#define DEBUG     // enable FTDI port output for debugging, disable serial write of JSIG data
// JOYSTICK DEFINITIONS
//#define __GRAVIS__
#define __NUNCHUCK__

#ifdef __GRAVIS__
// Analog Joystick and Throttle pin definitions
#define POTYPIN 0   // Stick Y axis POT - Gravis GRAY wire
#define POTXPIN 1   // Stick X axis POT - Gravis BLUE wire
#define POTZPIN 2   // Throttle POT - Gravis ORAGNGE wire
// Button Digial I/O pin assignments
#define THMBTN  3   // using the Thumb Buton - Gravis YELLOW wire
#define PLMBTN  4   // using the Palm Button - Gravis PINK wire
#define TRGBTN  10  // using trigger button - Gravis GREEN wire
#define ZEROBTN 12  // using base button - Gravis RED wire
#endif

#define INHPIN  11
#define JSIGPIN 1   // TX pin
#define LEDPIN  13

#include<Wire.h>
#include <Wiichuck.h>
Wiichuck chuck;

unsigned char data[6];
unsigned long pTrainTimer = 0; // timer val used to JSIG pulse train period
int joyXoffset = 0;
int joyYoffset = 0;
bool inhibitState = true; // start in inhibit enabled mode. NOTE: this is a flag of the inhibit state, not the inhibit signal level

void setup() {
#ifdef DEBUG
  Serial.begin(19200); // previous 19200);
  Serial.println("Start DEBUG");
#else
  Serial.begin(38400, SERIAL_8E1);
#endif

#ifdef __GRAVIS__
  pinMode(ZEROBTN, INPUT_PULLUP);
  pinMode(TRGBTN, INPUT_PULLUP);
  pinMode(THMBTN, INPUT_PULLUP);
  pinMode(PLMBTN, INPUT_PULLUP);
  pinMode(LEDPIN, OUTPUT);
  pinMode(INHPIN, OUTPUT);
#endif
#ifdef __NUNCHUCK__
  //chuck.init(); // default using analong pins 2 and 3 for VCC/GND
  //chuck.init(12, 13); // specify using analong pins 12 and 13 for VCC/GND
  chuck.init(0, 0); // specify not to use analong pins for VCC/GND, provided some other way

  // set zero position
  chuck.poll(); // get nunchuck data
  chuck.poll(); // get data again as required to get a good calibration
  chuck.calibrate(); // set to zero values of last values read from the device
#ifdef DEBUG
  Serial.println("Nunchuck-calibrated");
#endif
#endif


  //digitalWrite(INHPIN, HIGH);
  powerOff(); // start in Stop mode with INHPIN and JSIGPIN Low
  digitalWrite(LEDPIN, HIGH);
  //delay(5); //HACk; delay x Sec HACK
  pTrainTimer = millis() + 100; // set timer for start of Jstick stream in 100mS
}

int throttleRAW = 5; //initialize
void loop()
{
  int drive;
  int turn;
  int throttle;
  //***********************************************************
  //  Generate and send drive data packet
  //***********************************************************
  if ( millis() >= pTrainTimer ) // only send data stream on 10mS and inhibit false
  {
    pTrainTimer += 10;
    // drive F/B
    // turn R/L
#ifdef __GRAVIS__
    int driveRAW = -(analogRead(POTYPIN) - 512) / 5; //-Read joystick pot - range +/-100
    int turnRAW = (analogRead(POTXPIN) - 512) / 5; //-Read joystick pot - range +/-100
    int throttleRAW = abs(analogRead(POTZPIN) / 100); // /68); //-Read joystick throttle pot - JSelect->range 0-10, JSelectGT->range 0-15(/68)
#endif
#ifdef __NUNCHUCK__
    int driveRAW = chuck.joyY();  //-Read joystick - range +/-100
    int turnRAW = chuck.joyX();   //-Read joystick - range +/-100

    // set/adj throttle via X axis roll //-Read joystick throttle - JSelect->range 0-10, JSelectGT->range 0-15(/68)
    // chuck.accelX() is 0 to -200 rolled left and 0 to +200 rolled right.
    static int prevThrottleRoll = 5; // initialize
    //if( chuck.accelX()/20 > 4 && chuck.accelX()/20 > prevThrottleRoll )
    if ( chuck.accelX() / 20 > 4 && chuck.buttonZ() )
      if ( throttleRAW < 10 ) {
        throttleRAW++;
        prevThrottleRoll = chuck.accelX() / 20;
      }
    //if( chuck.accelX()/20 < -4 && chuck.accelX()/20 < prevThrottleRoll )
    if ( chuck.accelX() / 20 < -4 && chuck.buttonZ() )
      if ( throttleRAW > 0 ) {
        //Serial.print("accelX="); Serial.println(chuck.accelX());
        throttleRAW--;
        prevThrottleRoll = chuck.accelX() / 20;
      }
#endif

    turn = turnRAW + joyXoffset;
    drive = driveRAW + joyYoffset;
    throttle = throttleRAW; //10;  // 0-15( 0x00-0x0e )

    // drive and turn max at 100%
    if (drive > 100)
      drive = 100;
    if (drive < -100)
      drive = -100;
    // let's give a buffer area around center position to keep zero's.
    // +/- 2nd parameter
    keepzero(drive, 10);

    // turn left/right max at 100%
    if (turn > 100)
      turn = 100;
    if (turn < -100)
      turn = -100;
    keepzero(turn, 10);
#ifdef DEBUG
    Serial.print("****drive=("); Serial.print(drive);
    Serial.print(")****turn=("); Serial.print(turn);
    Serial.print(")****throttle=("); Serial.print(throttle); Serial.println(")");
    delay(500);
#endif
    // charging: b1=0x4a, 0x01, 0x04, 0x02, 0x01, 0xAD
    // running : b1=0x4a, 0x01, 0x04, 0x02, 0x01, 0xAD
    if ( !inhibitState )
    {
      //build serial packet
      data[0] = 0x4A;                                            //-Datagram always start with 0x4A NOT 0x6A
      data[1] = 01;                                              //-unchanging TBD The Horn signal might show up here
      data[2] = throttle;                                        //-Drive mode: 0-15 JazzyGT, 0-10 Jazzy ( 0x00=lowest, 0x01=slow, 0x0c=fast, 0x0e=fastest )
      data[3] = drive;                                           //-Drive +/-100
      data[4] = turn;                                            //-Turn +/-100
      data[5] = 0xff - (data[0] + data[1] + data[2] + data[3] + data[4]); //-Checksum

      digitalWrite(INHPIN, LOW); // there is a 10mS !INH at start of pre-amble bytes
      for (unsigned char i = 0; i < 6; i++)
      {
#ifndef DEBUG
        Serial.write(data[i]);
#endif
        if ( i == 3 ) // trying to get a 32uS pulse
          digitalWrite(INHPIN, HIGH);
      }
    } // if( !inhibitState )

    // Now that we've sent the data to keep the timing tight, let's do other things
#ifdef __GRAVIS__
    bool bZeroBtn = !digitalRead(ZEROBTN);
#endif
#ifdef __NUNCHUCK__
    chuck.poll();
    bool bZeroBtn = chuck.buttonZ(); //TODO: this should be set with a long hold
#endif
    if ( bZeroBtn )
    {
#ifdef DEBUG
      Serial.println("Zero Offsets");
#endif
      joyXoffset = -turnRAW;
      joyYoffset = -driveRAW;
    }
    // Test for Trigger button press, TOGGLE Inhibit line
    //  stop Jstick data stream sending on Inhibit true;
  #ifdef __GRAVIS__
  if( !digitalRead(TRGBTN) )
  {
    #ifdef DEBUG
    Serial.println("Trigger Button Toggle");
    #endif
    if( inhibitState ){
      //When initialized and running, toggling INH seems to tell base the system is charging
      digitalWrite(INHPIN, HIGH);
      inhibitState=false;
      // do initialize handshake here
      //jazzyInit(); //initializeComms();
    }
    else // inhibitState is false, toggle to inhibit enabled/true
    {
      digitalWrite(INHPIN, LOW);
      // TODO: do we need to pull the TX line low too?
      inhibitState=true;
      //powerOff(); // send Off pulse
    }
  } // if !TRGBTN
  #ifdef DEBUG  // until we have a use for the Thumb button(horn?) skip it
  if( !digitalRead(THMBTN) )
  {
    Serial.println("THUMB Button Toggle");
  }
  #endif
  #endif //__GRAVIS__ trigger button / inhibit toggle

    // Power On/Off button - Gravis=Palm Button, Nunchuck=C button used as the POWER ON/OFF toggle button
#ifdef __GRAVIS__
    bool bPowerBtn = !digitalRead(PLMBTN);
#endif
#ifdef __NUNCHUCK__
    bool bPowerBtn = chuck.buttonC();
#endif

    if ( bPowerBtn )
    {
#ifdef DEBUG
      Serial.println("Power Button Toggle");
#endif
      if ( inhibitState ) { // in OFF/STOP/INHIBIT mode, lets toggle into ON mode
        jazzyPowerOn();
      }
      else {// lets toggle into OFF mode
        powerOff();
      }
    }

    //
  } // if pTrainTimer

  // LED (pin 13) indicator
  if ( !inhibitState ) { //running
#ifdef DEBUG
    Serial.println("LED Indicator - !inhibit");
#endif
    // give indicator of zero'ed stick
    if ( drive == 0 && turn == 0 )
      digitalWrite(LEDPIN, HIGH);
    else
      digitalWrite(LEDPIN, LOW);
  }
  else {  // stopped
    blinkIndicator();
  }


} // loop()

// this is where all the initialization should happen since we know the joystick stream works, even without INHPIN toggling
// after a proper handshaking has occured.
// see captured data file "capture-joystickAlone-Startup-20KHz.olp"
// 1)after long low on INH + JSIG the JSIG goes high for 400uS and 150uS after rising edge INH goes high for 250uS. They go low together.
// 2a)both go High after 213mS. JSIG remains High for 110mS after which ?standard JSIG messaging begins.
// 2b)INH goes high for 13.5mS, then Low for 7.6mS, then goes High but with a quick 50uS Low pulse after 150uS, then High for 6.45mS and then
// 2c)a quick 50uS Low pulse then a 50uS High pulse before going Low 11.6mS before a string of 6 High pulses TBD...for now
//
void jazzyInit() {
#ifdef DEBUG
  Serial.println("jazzyInit");//Serial.println(bLedOn);
#endif
  //Start initialization.
  // take control of TX pin
  Serial.end();
  pinMode(JSIGPIN, OUTPUT);
  //Step 1
  digitalWrite(JSIGPIN, HIGH);
  delayMicroseconds(150);
  digitalWrite(INHPIN, HIGH);
  delayMicroseconds(250);
  //should be simultaneous LOW
  if (true) {
    digitalWrite(JSIGPIN, LOW);
    digitalWrite(INHPIN, LOW);
  }
  //Step 2
  delay(213);
  //should be simultaneous HIGH
  if (true) {
    digitalWrite(JSIGPIN, HIGH);
    digitalWrite(INHPIN, HIGH);
  }
  delay(13);  // should be 13.5mS
  digitalWrite(INHPIN, LOW);
  delay(7); // should be 7.6mS
  digitalWrite(INHPIN, HIGH);
  delayMicroseconds(150);
  digitalWrite(INHPIN, LOW);
  delayMicroseconds(50);
  digitalWrite(INHPIN, HIGH);
  delay(6); // should be 6.45mS
  digitalWrite(INHPIN, LOW);
  delayMicroseconds(50);
  digitalWrite(INHPIN, HIGH);
  delayMicroseconds(50);
  digitalWrite(INHPIN, LOW);
  delay(11); // should be 11.6mS
  // unkown HIGH pulse train(6 pulses) in 3.5mS
  delay(4); // should be 3.5mS
  delay(67); // end of initialization stream
  //return to main loop;
  //jazzyPowerOn();
#ifdef DEBUG
  Serial.begin(19200); // previous 19200);
#else
  Serial.begin(38400, SERIAL_8E1);
#endif
}

void powerOff() {
#ifdef DEBUG
  Serial.println("Power Off");//Serial.println(bLedOn);
#endif
  inhibitState = true;
  Serial.end();  
  // From power off capture - ~350uS High pulse of both INH + JSIG. note: JSIG is 450uS wide so it's getting off last
  digitalWrite(INHPIN, HIGH);
  digitalWrite(JSIGPIN, HIGH);
  delayMicroseconds(350);

  // take control of TX pin
  //pinMode(INHPIN, OUTPUT);
  // set both INH + JSIG low for extended period ?Seconds
  digitalWrite(INHPIN, LOW);
  digitalWrite(JSIGPIN, LOW);
#ifdef DEBUG
  Serial.begin(19200); // previous 19200);
#else
  Serial.begin(38400, SERIAL_8E1);
#endif

}

void jazzyPowerOn() {
#ifdef DEBUG
  Serial.println("Power On");//Serial.println(bLedOn);
#endif
  inhibitState = false;
  digitalWrite(INHPIN, HIGH);
  // set zero position
  chuck.poll(); // get nunchuck data
  chuck.poll(); // get data again as required to get a good calibration
  chuck.calibrate(); // set to zero values of last values read from the device
  jazzyInit();
}


void initializeComms() {
  // start bytes, just 2 then ~10mS before Jstick stream start
  data[0] = 0x52;
  data[1] = 0xad;
  digitalWrite(INHPIN, LOW); // there is a 32mS !INH pulse at start of pre-amble bytes
  Serial.write(data[0]);
  //delay(5); //HACK delay 5mS
  digitalWrite(INHPIN, LOW);
  Serial.write(data[1]);
  digitalWrite(INHPIN, LOW); // try to get 32uS delay before going high
  digitalWrite(INHPIN, HIGH);
  delay(1);
  // base sends in response... what I saw sniffed....
  //data[0]=0x72;
  //data[1]=0x10;
  //data[2]=0x00;
  //data[3]=0x7D;
  //for(unsigned char i=0;i<4;i++)
  //   ;//Serial.write(data[i]);


} // initializeComms()

void keepzero(int &iVal, int swing) {
  if ( abs(iVal) <= swing )
    iVal = 0;
}

void blinkIndicator() {
  static int mSecBlink = 0;
  static bool bLedOn = true;
  //int mSecBlinkDelay = 1000; // 1 sec on/off blink

  int cur_mSec = millis();
  if ( cur_mSec > mSecBlink ) {
#ifdef DEBUG
    Serial.print("Blink LED Indicator:"); Serial.println(bLedOn);
#endif
    if ( bLedOn ) {
      digitalWrite(LEDPIN, LOW);
      bLedOn = false;
    }
    else {
      digitalWrite(LEDPIN, HIGH);
      bLedOn = true;
    }
    mSecBlink = cur_mSec + 1000;
  }
}
