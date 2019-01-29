/*This is the main code for the core functions that will need
  to be run on/through the Arduino.
  Author: Garrett Barton
  Date Started: 10Dec18
  Liscence: MIT Garrett Barton (c) (2018)
*/
#include <PID_v1.h> //Arduino PID Library - V1.2.1 by Brett Beauregard (c) (2017)
#include <SevSeg.h>//Arduino Seven Segment Display Library - V3.3.0 - Apache License (version 2.0) by Dean Reading (2017)

SevSeg sevseg; //Instantiate a seven segment object

//double on Arduino is the same as float else where
double SetPoint; //vaule desired
double Input; //value recieved from encoder
double Output; //value sent to motor driver
//Intialization of PID Parameters
double Kp=0, Ki=0, Kd=0; //coefficients as defined by the PID equation in standard form

//create PID instances
PID myPID)(&Input,&Output,&SetPoint,Kp,Ki,Kd,Direct);

void setup()
  {
  Serial.begin(9600); //setting baud rate
  }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Display
void setup()
  {
    byte numDigits = 4;
    byte digitPins[] = {}; //sets the ground pins for each digit (need 4)
    byte segmentPins[] = {6, 5, 2, 3, 4, 7, 8, 9}; //sets (A,B,C,D,E,F,G,DP) in order
    bool resistorsOnSegments = true; //is there a current limiting resistor? (need for multidigit displays)
    bool updateWithDelaysIn = false; // Default 'false' is Recommended
    bool leadingZeros = false; // Use 'true' if you'd like to keep the leading zeros
    bool disableDecPoint = false; // Use 'true' if your decimal point doesn't exist or isn't connected. Then, you only need to specify 7 segmentPins[]

 
    byte hardwareConfig = COMMON_CATHODE; //sets what is common (i.e. cathode or anode)
    sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments,updateWithDelays, leadingZeros, disableDecPoint);
    sevseg.setBrightness(90); //range from 0 to 100
  }

void loop()
  {
        sevseg.setNumber(4); //prints what is inputed (i.e. "4" prints 4)
                            //to print decimal, (4999, 3) is 4.999 (i.e DP is "3" from the right
        sevseg.refreshDisplay(); //required at end of loop to continue displaying the number
  }

void loop() //used as an example to display numbers
  {
    for(int i = 0; i < 100; i++)
      {
        sevseg.setNumber(i, 1);
        delay(1000);
        sevseg.refreshDisplay(); 
      }
  }
  http://www.circuitbasics.com/arduino-7-segment-display-tutorial/
  https://github.com/DeanIsMe/SevSeg
  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Buttons ("run" and "down")
const int LEDDownPin = 2; //sets the controlling pin of the LED to pin 2
const int LEDRunPin = 4;
const int DownButtonPin = 3; //sets the pin for the down button to 3
const int RunButtonPin = 5;

void setup()
  {
    pinMode(LEDPin,OUTPUT); //initializes LED control pin as output
    pinMode(DownButtonPin,INPUT); //initializes down button pin as input
    pinMode(RunButtonPin,INPUT);
  }
  
  void loop()
    {
      DownButtonState = digitalRead(DownButtonPin); //Reads pin for state
      RunButtonState = digitalRead(RunButtonPin);
      if(DownButtonState == HIGH)
        {
          digitalWrite(DownButtonPin,HIGH);
          //run that code
        }
      else
        {
          digitalWrite(DownButtonPin, LOW);
          //just loop
        }
      if(RunButtonState == HIGH)
        {
          digitalWrite(RunButtonPin,HIGH);
          //run that code
        }
      else
        {
          digitalWrite(RunButtonPin, LOW);
          //just loop
        }
    }
