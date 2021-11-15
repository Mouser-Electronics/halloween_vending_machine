#include <Stepper.h>

// gesture sensor
#include "DFRobot_Gesture_Touch.h"
#include "HardwareSerial.h"
HardwareSerial    mySerial(0);


// change this to fit the number of steps per revolution for your motor
#define STEPS 2038


// initialize the stepper library on pins :
Stepper choco(STEPS, 2, 13, 4, 5);

Stepper arm(STEPS, 15, 23, 33, 19);

DFRobot_Gesture_Touch   DFGT(&mySerial);    // init sensor object, request write and read function


void setup() {
  // initialize the serial port:
  
  Serial.begin(115200);
  mySerial.begin(9600);
  delay(500);
  while(mySerial.available())  // flush serial buffer
    mySerial.read();
  Serial.println("gesture&touch sensor test");


  DFGT.setGestureDistance(20);             // suggest default value
  DFGT.enableFunction(DFGT_FUN_ALL);       // enable all functions
  DFGT.disableFunction(DFGT_FUN_RIGHT | DFGT_FUN_LEFT);    // disable function test
}

void loop()
{
  int8_t    rslt = DFGT.getAnEvent();  // get an event that data saved in serial buffer
  if(rslt != DF_ERR) {
    // DFGT.setSleep(DFGT_SLEEP_DISABLE);  // disable auto sleep
    switch(rslt) {
      case DFGT_EVT_BACK: {
        Serial.println("get event back");
      } break;
      case DFGT_EVT_FORWARD: {
        Serial.println("get event forward");
        arm.setSpeed(10);
  
        arm.step(-500);
        
        choco.setSpeed(15);
        choco.step(-2000);
        delay(1000);
        choco.step(2000);
        delay(5000);
  
        arm.step(500);
        delay(2000);
        
      } break;
      case DFGT_EVT_RIGHT: {
        Serial.println("get event right");
      } break;
      case DFGT_EVT_LEFT: {
        Serial.println("get event left");
      } break;
      case DFGT_EVT_PULLUP: {
        Serial.println("get event pull up");
      } break;
      case DFGT_EVT_PULLDOWN: {
        Serial.println("get event pull down");
      } break;
      case DFGT_EVT_PULLREMOVE: {
        Serial.println("get event pull and remove");
      } break;
      case DFGT_EVT_TOUCH1: {
        Serial.println("get event touch1");
      } break;
      case DFGT_EVT_TOUCH2: {
        Serial.println("get event touch2");
      } break;
      case DFGT_EVT_TOUCH3: {
        Serial.println("get event touch3");
      } break;
      case DFGT_EVT_TOUCH4: {
        Serial.println("get event touch4");
      } break;
      case DFGT_EVT_TOUCH5: {
        Serial.println("get event touch5");
      } break;
      default: {
        //turn off the GPIOs of the motors
        digitalWrite(2,LOW);
        digitalWrite(4,LOW);
        digitalWrite(5,LOW);
        digitalWrite(13,LOW);
        digitalWrite(15,LOW);
        digitalWrite(19,LOW);
        digitalWrite(23,LOW);
        digitalWrite(33,LOW); 
        } break;
    }
  }
}
