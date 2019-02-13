/*This is the main code for the core functions that will need
  to be run on/through the Arduino for the NASA device.
  Author: Garrett Barton
  Date Started: 10Dec18
  Liscence: MIT Garrett Barton (c) (2018)
*/
////////////////////////////////////////////////////////////
//Libraries
#include <PID_v1.h> //Arduino PID Library - V1.2.1 by Brett Beauregard (c) (2017)
#include <SevSeg.h>//Arduino Seven Segment Display Library - V3.3.0 - Apache License (version 2.0) by Dean Reading (2017)
#include <SD.h> //Arduino SD library - by Limor Fried (modified by Tom Igoe)
#include <math.h> //allows mathematical operations
#include <HX711.h> //Library to work with the HX711 amplifier - MIT License by Bogdan Bogdan Necula (2018)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Global variable definitions
    ////////////////////////////////////////////////////////////
    //Display
    
    ////////////////////////////////////////////////////////////
    //SD breakout
    float LoadCellReadout_MAX = 0; //intialize the maximum force to zero
    ////////////////////////////////////////////////////////////
    //Buttons
    const int LEDDownPin = 2; //sets the controlling pin of the LED to pin 2
    const int LEDRunPin = 4;
    
    const int DownButtonPin = 3; //sets the pin for the down button to 3
    const int RunButtonPin = 5;
    
    const int DownButtonLED_Pin = 23; //sets the pin for the down button LED to 23
    const int RunButtonLED_Pin = 22;


    ////////////////////////////////////////////////////////////
    //Amplifier breakout
    const float calibration_factor = -7050.0; //Value found using the SparkFun_HX711_Calibration sketch
    const int DOUT = 3;
    const int CLK = 2;
    float LoadCellReadout; // floats have 7 digits of percision and can keep decimal points in calculations
    ////////////////////////////////////////////////////////////
/*
  Later more user control variables will be defined here
    int LinearSpeed = 3; //(inches per minute)
    int MaxForceAllowed = 250; // (pounds) in order to stop the test before its overloaded
    
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
  {
    Serial.begin(9600); //setting baud rate for all processes (9600 is standard)
    
    ////////////////////////////////////////////////////////////
    //Display
    byte numDigits = 4;
    byte digitPins[] = {}; //sets the ground pins for each digit (need 4)
    byte segmentPins[] = {6, 5, 2, 3, 4, 7, 8, 9}; //sets (A,B,C,D,E,F,G,DP) in order
    bool resistorsOnSegments = true; //is there a current limiting resistor? (need for multidigit displays)
    bool updateWithDelaysIn = false; // Default 'false' is Recommended
    bool leadingZeros = false; // Use 'true' if you'd like to keep the leading zeros
    bool disableDecPoint = false; // Use 'true' if your decimal point doesn't exist or isn't connected. Then, you only need to specify 7 segmentPins[]

    byte hardwareConfig = COMMON_CATHODE; //sets what is common (i.e. cathode or anode)
    sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments,updateWithDelays, leadingZeros, disableDecPoint);
    sevseg.setBrightness(90); //range is from 0 to 100
    
    ////////////////////////////////////////////////////////////
    //SD breakout
    pinMode(10, OUTPUT); // make sure that the default chip select pin is set to output, even if you don't use it
  
    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect))
      {
        Serial.println("Card failed, or not present");
        return;
      }
    
    ////////////////////////////////////////////////////////////
    //Buttons
    pinMode(LEDPin,OUTPUT); //initializes LED control pin as output
    pinMode(DownButtonPin,INPUT); //initializes down button pin as input
    pinMode(RunButtonPin,INPUT);
    
    ////////////////////////////////////////////////////////////
    //Amplifier breakout
    scale.set_scale(calibration_factor);
    scale.tare(); //with NO weight on the load cell, resets the read out to 0
    
    ////////////////////////////////////////////////////////////
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Display
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //SD breakout
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Buttons ("run" and "down")
    int DownButtonState = digitalRead(DownButtonPin); //Reads pin for state
    int RunButtonState = digitalRead(RunButtonPin);
    
    ////////////////////////////////////////////////////////////
    //logic for buttons is flipped due to using pullup resistors
    if(DownButtonState == HIGH) {DownButtonPressed = 0;} 
    else {DownButtonPressed = 1}
   
    if(DownButtonState == HIGH) {RunButtonPressed = 0;}
    else {RunButtonPressed = 1} 
    ////////////////////////////////////////////////////////////
    
    if(DownButtonPressed == 1) //down button is pressed
      {
        digitalWrite(DownButtonPin,HIGH);
        //run that code
        digitalWrite(DownButtonLED_Pin, HIGH); //to acknowledge that this point was reached
      }
    else
      {
        //just loop
        digitalWrite(DownButtonLED_Pin, LOW); //to turn LED off when button is released
      }
    ////////////////////////////////////////////////////////////
    if(RunButtonPressed == 1) //run button is pressed
      {
        //run that code
        digitalWrite(RunButtonLED_Pin, HIGH); //to acknowledge that this point was reached
      }
    else
      {
        //just loop
        digitalWrite(RunButtonLED_Pin, LOW); //to turn LED off when button is released
      }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Amplifier breakout
    // Using Bogdan's library (go read the library definition for more on function descriptions)
    // See the calibration sketch for getting the calibration offset.
    
    HX711 scale(DOUT, CLK);
    
    LoadCellReadout = scale.get_units(); //returns a float of the current measurement, 
                                         //is actually the average of a set number (10) of readings
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DownHook(bool DownButtonState)
  {
      // actuator down
      // stop when button released
  }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Display (see example from Arduino)
void Display()
  {
    sevseg.setNumber(4); //prints what is inputed (i.e. "4" prints 4)
                        //to print decimal, (4999, 3) is 4.999 (i.e DP is "3" from the right
    sevseg.refreshDisplay(); //required at end of loop to continue displaying the number
  }

/*
void loop() //used as an example to display numbers
  {
    for(int i = 0; i < 100; i++)
      {
        sevseg.setNumber(i, 1);
        delay(1000);
        sevseg.refreshDisplay(); 
      }
  }
 // http://www.circuitbasics.com/arduino-7-segment-display-tutorial/
 // https://github.com/DeanIsMe/SevSeg
 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SD Breakout (see example from Arduino)
//CS pin = 53 on Mega
//https://www.arduino.cc/en/Serial/Flush need?
////////////////////////////////////////////////////////////
void LogData()
  {
    String dataString = ""; //intialized an empty string for assembling the data to log
  
    while(RunTest == true) //collects all sensor data for a test
      {
        int sensor = analogRead(analogPin); //reads an integer from the analog pin (aka the sensor)
        dataString += String(sensor); //concatenates the integer read to the string to be stored
        dataString += ",\n\r";  //adds comma, new line, and return carriage between each sensor's readings (comma delineation)
      }
  
    File dataFile = SD.open("datalog.txt", FILE_WRITE); //opens text file and assigns a variable name to text file
  
    if (dataFile) //if the file is available, write to it...Only opens file once per test
      {
        dataFile.println(dataString); //writes the string of stored readings
        dataFile.close(); //closes the text file...ONLY ONE FILE CAN BE OPEN AT A TIME
      }
    else  // if the file isn't open, pop up an error:
      {
        //put error handling here!
      } 
  }
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
// re-open the file for reading: functions: https://www.arduino.cc/en/Reference/FunctionDeclaration
int MaxForce()
  {
    dataFile = SD.open("test.txt");
    if (dataFile) 
      {
        // read from the file until there's nothing else in it:
        while (myFile.available())
         {
             SDReadout_Data = myFile.read(); //reads data from SD card
             //make it usable to pull max force
             if(LoadCellReadout_MAX < SDReadout_Data)
               {
                   LoadCellReadout_MAX = SDReadout_Data;
               }
          }
         //now have the max force stored in LoadCellReadout_MAX
      }
      myFile.close(); //close file when done.
      return LoadCellReadout_MAX;
  }
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
//in setup() {char FileName[] = "01234567.CSV"}
//in main loop() {MakeFileName(FileName);}

void MakeFileName(char *FileName)
  {
    int zero = random(0,9); //a random number of 7 digits
    int one = random(0,9);  //these aren't staying, but I need something to work with in the mean time
    int two = random(0,9);
    int three = random(0,9);
    int four = random(0,9);
    int five = random(0,9);
    int six = random(0,9);
    int seven = random(0,9);
    
    FileName[0] = zero;
    FileName[1] = one;
    FileName[2] = two;
    FileName[3] = three;
    FileName[4] = four;
    FileName[5] = five;
    FileName[6] = six;
    FileName[7] = seven;
    FileName[8] = '.';
    FileName[9] = 'C'; //Comma Seperated Values
    FileName[10] = 'S';
    FileName[11] = 'V';
    return;
  }

