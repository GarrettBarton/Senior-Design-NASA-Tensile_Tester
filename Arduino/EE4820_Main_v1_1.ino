/*This is the main code for the core functions that will need
  to be run on/through the Arduino for the NASA device.
  Author: Garrett Barton
  Date Started: 10Dec18
  Date Installed: 
  Version: 1.1
  Liscence: MIT Garrett Barton (c) (2019)
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////    Library Definitions    ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Libraries
#include <PID_v1.h> //Arduino PID Library - V1.2.1 by Brett Beauregard (c) (2017)
#include <SevSeg.h>//Arduino Seven Segment Display Library - V3.3.0 - Apache License (version 2.0) by Dean Reading (2017)
#include <SD.h> //Arduino SD library - by Limor Fried (modified by Tom Igoe) (C) Copyright 2010 SparkFun Electronics
#include <SPI.h> //Arduino SPI library - 
#include <HX711.h> //Library to work with the HX711 amplifier - MIT License by Bogdan Bogdan Necula (2018)
#include <DualMC33926MotorShield.h> //motor controller - V4.0.0 - by Pololu Corporation Copyright (c) 2012

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////    Global Variable Definitions    /////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Display
    SevSeg sevseg;  //instantiate a seven segment controller object
    int DisplayTime = 3000; //time that the display will show the error and/or the max force (in milliseconds)    
    
    //Error Codes
    char Error_MemoryFull[] = "E00";
    char Error_NotConnectedToLoadCell[] = "E01";
    char Error_MaxWeight[] = "E02";
    char Error_TimeOut[] = "E03";
 
    //Warning Codes
    char Warning_SDError[] = "3 00";
    char Warning_InvalidTest[] = "3 01";
    char Warning_FreeMemoryLow[] = "3 02";
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //SD breakout
//    Sd2Card card;  //sets up objects for controlling with SD library
//    SdVolume volume; //these are the .cpp files in the "utility" folder in SD
//    SdFile root;
//    File fatty;
    
    float LoadCellReadOut_MAX = 0.0; //intialize the maximum force to zero
    //these are for if i need to make seperate files for each test run
    int hundreds = 0; //increaments the file name
    int tens = 0;  
    int ones = 1;
    
    char FileName[12] = "0000000.txt";

    int chipSelect = 53;

    File dataFile; //Instantiate a text file controller object
    String dataString = "Load Cell Reading, Encoder Reading\n\r"; //initialized an empty string for assembling the data to log
    int TestNumber = 0; //uesed to identify the different tests in a "testing session" (i.e. the time the device is ON)  

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Buttons 
    const int DownButtonPin = 34; //sets the pin for the down button to 3
    const int RunButtonPin = 35;
    
    const int DownButtonLED_Pin = 36; //sets the pin for the down button LED to 23
    const int RunButtonLED_Pin = 37;
    
    int DownButtonState; //Reads pin for state
    int RunButtonState;
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Amplifier breakout
    const float calibration_factor = -7050.0; //Value found using the SparkFun_HX711_Calibration sketch
    const int DOUT = 46;
    const int CLK = 44;
    float LoadCellReadOut; // floats have 7 digits of percision and can keep decimal points in calculations
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Encoder
    int EncoderDataPin = 48;
    double EncoderReadOut_Diff = 0.0;
    int MeasuredTime = 100;
    unsigned long RunStartTime = millis();
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Motor Driver
    double InputMotorSpeed_UP = 0; //This will be the value that changes the motor controller "speed"
    double InputMotorSpeed_DOWN = 0; //Makes the motor go down 
    
    DualMC33926MotorShield md; //Instantiate a motor sheild controller object for moving up
    //default pinout - M1PWM = 9, M1DIR = 7, M1FB = N/A, SF = 12 
    
    int StopFactor; //used to stop test while running
   
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Control Variables and Settings
    double Kp = 1;  //found with the use of the PID tuning code from same creator of PID controller
    double Ki = 1;
    double Kd = 1;
    
    double LinearSpeed_SP_UP = 3.0; //Set point for linear speed (in inches per minute)
    double LinearSpeed_SP_DOWN = -3.0; //Set point for linear speed (in inches per minute)
    int MaxForceAllowed_SP = 250; //Set point for maximum force (in pounds) in order to stop the test before its overloaded
    int MaxTimeAllowed_SP = 1500; //Set point for maximum allowed time for the force to be read at zero before the test is ended in ms

    double RotationalSpeed_SP_UP = ((LinearSpeed_SP_UP * 16) * (30.681) * ((1/60)*(1/1000)) * (MeasuredTime)); 
    double RotationalSpeed_SP_DOWN = ((LinearSpeed_SP_DOWN * 16) * (30.681) * ((1/60)*(1/1000)) * (MeasuredTime));
    //((inches to 1/16ths of inches)*(Lead screw rotations to get that many inches))*(conversion to rotations per millisecond)*(the delay value)
/*
    gear ratio = 30.681 and
    1 rotation of lead screw = 1/16 inch of linear motion;
    therefore, the motor needs to be 1472.73 rotations/min to get 3 in/min. 
    The PPR of the encoder is set to 512
    EncoderReadOut_Diff should be equal to 2.455
*/
    //int RotationalSpeed_SP_DOWN = (int) LinearSpeed_SP_DOWN...convert to the range [-400,0]
    
    //Specify the links and initial tuning parameters
    //Instantiate a PD controller object for moving up and down
    //P_ON_E is Proportional On Error - the Kp will with respect to the error
    //DIRECT is the type of responce the PID will output when faced with an error (REVERSE would be used with a cooling system)
    
    PID myPID_UP(&EncoderReadOut_Diff, &InputMotorSpeed_UP, &RotationalSpeed_SP_UP, Kp, Ki, Kd, P_ON_E, DIRECT);
    PID myPID_DOWN(&EncoderReadOut_Diff, &InputMotorSpeed_DOWN, &RotationalSpeed_SP_DOWN, Kp, Ki, Kd, P_ON_E, DIRECT);
    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////    Setup Loop    /////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
  {
    Serial.begin(9600); //setting baud rate for all processes (9600 is standard)
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Display
    byte numDigits = 4;
    byte digitPins[] = {22, 23, 24, 25}; //Digits: 1,2,3,4 //sets the ground pins for each digit (need 4)
    byte segmentPins[] = {26, 27, 28, 29, 30, 31, 32, 33}; //sets (A,B,C,D,E,F,G,DP) in order
    bool resistorsOnSegments = false; //is there a current limiting resistor? (need for multidigit displays)
    bool updateWithDelays = false; // Default 'false' is Recommended
    bool leadingZeros = false; // Use 'true' if you'd like to keep the leading zeros
//    bool disableDecPoint = false; // Use 'true' if your decimal point doesn't exist or isn't connected. Then, you only need to specify 7 segmentPins[]

    byte hardwareConfig = COMMON_CATHODE; //sets what is common (i.e. cathode or anode)
    sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments,updateWithDelays, leadingZeros);
    sevseg.setBrightness(10); //range is from 0 to 100
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //SD breakout
    char FileName[] = "000.txt";
    pinMode(53, OUTPUT); // make sure that the default chip select pin is set to output, even if you don't use it

   
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Buttons
    pinMode(RunButtonLED_Pin,OUTPUT); //initializes LED control pin as output
    pinMode(DownButtonLED_Pin,OUTPUT); //initializes LED control pin as output
    pinMode(DownButtonPin,INPUT); //initializes down button pin as input
    pinMode(RunButtonPin,INPUT);
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Amplifier breakout
    HX711 scale(DOUT, CLK); //Instantiate an amplifier controller object for moving up
    scale.set_scale(calibration_factor);
    scale.tare(); //with NO weight on the load cell, resets the read out to 0
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Motor Controller
    md.init();
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  }
  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////    Main Loop    /////////////////////////////////////////////////////////
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
//    if(freeKB < 1000) {DisplayString(Error_SDError);}
    MakeFileName(FileName); //creates a file for the test
    dataFile = SD.open("DataLog.txt", FILE_WRITE); //opens text file "DataLog"

    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Button logic conversion
    //Buttons ("run" and "down")
      int DownButtonState = digitalRead(DownButtonPin); //Reads pin for state
      int RunButtonState = digitalRead(RunButtonPin);
        
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Down
      if(DownButtonState == 1) //down button is pressed
        {
          digitalWrite(DownButtonLED_Pin, HIGH); //to acknowledge that this point was reached
          int StopFactor = 0; //don't need to stop test
          
          unsigned long RunStartTime = millis();  //put here to get the start of the test
          DownHook();
        }
      //just loop, but turn off the LED in the button
      else {digitalWrite(DownButtonLED_Pin, LOW);} //to turn LED off when button is released
           
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Run test
      if(RunButtonState == 1) //run button is pressed
        {
          digitalWrite(RunButtonLED_Pin, HIGH); //to acknowledge that this point was reached
         
          dataString += "\n\rTest " + String(TestNumber) + "\n\r"; //identifies  test number
          TestNumber++; //increments TestNumber such that the test runs can be identified in a session
          
          unsigned long RunStartTime = millis();  //put here to get the start of the test 
          RunTest();
        }
        
      //just loop, but turn off the LED in the button
      else {digitalWrite(RunButtonLED_Pin, LOW);} //to turn LED off when button is released
      
      dataFile.close(); //closes the text file...ONLY ONE FILE CAN BE OPEN AT A TIME            
      DisplayNumber(LoadCellReadOut_MAX); //displays the maximum force recorded
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////    Function Definitions    ////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DownHook()
  {
      // actuator down at some constant speed while the "down" button is pressed
      // stop when button released
      while(DownButtonState == 1) //down button is pressed
      {
        int DownButtonState = digitalRead(DownButtonPin); //see functionality in loop(), these are here to keep undated readings
        
        MotorDown(); //motor controller code
      }
  }
  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Display (see example from Arduino)
void DisplayNumber(float DisplayValue)
  {
    //String thisString = String(13) //gives "13"
    sevseg.setNumber(DisplayValue); //prints what is inputed (i.e. "4" prints 4)
                        //to print decimal, (4999, 3) is 4.999 (i.e DP is "3" from the right
    delay(DisplayTime);
    sevseg.refreshDisplay(); //required at end of loop to continue displaying the number
  }

void DisplayString(char DisplayValue)
  {  
    sevseg.setChars(DisplayValue); //prints what is inputed (i.e. "4" prints 4)
                        //to print decimal, (4999, 3) is 4.999 (i.e DP is "3" from the right
    delay(DisplayTime);
    sevseg.refreshDisplay(); //required at end of loop to continue displaying the number
  }
  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SD Breakout (see example from Arduino)
//CS pin = 53 on Mega
//https://www.arduino.cc/en/Serial/Flush need?
//////////////////////////////////////////////////////////
void HandleData() //Intializes string for data storage, reads data, finds maxe force, prevents over load, stores data to SD
  {
    HX711 scale(DOUT, CLK); //Instantiate an amplifier controller object for moving up, agian due to error otherwise
       
    float LoadCellReadOut = scale.get_units(); //returns a float of the current measurement, 
                                     //is actually the average of a set number (10) of readings
    //float EncoderReadOut = analogRead(EncoderDataPin); //reads a float from the encoder to get position data
    // don't need this because it has been found in GetRotationalSpeed()
    
    if(LoadCellReadOut_MAX < LoadCellReadOut) //Bubble sort to find maximum force
      {LoadCellReadOut_MAX = LoadCellReadOut;}
    
    dataString += String(LoadCellReadOut) + ", "; //concatenates the data read to the string to be stored
    dataString += String(EncoderReadOut_Diff); //concatenates the pulses read to the string to be stored
    dataString += "\n\r";  //adds new line and return carriage between readings (comma delimited)
    
    if (dataFile) //if the file is available, write to it...Only opens file once per test
      {
//        dataFile = SD.open("DataLog.txt", FILE_WRITE); //opens text file "DataLog"
        dataFile.println(dataString); //writes the string of stored readings
//        dataFile.close(); //closes the text file...ONLY ONE FILE CAN BE OPEN AT A TIME
      }
    else  // if the file isn't open, pop up an error:
      {
        DisplayString(Warning_SDError); //doesn't stop the test, just notifies user
        //this is error is not critical to the validitity of the test
      }
    

  }
 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RunTest()
    {
      /*
        Flow:
          Read encoder for intial position
          Read load cell for intial reading
          Log data gathered from encoder, load cell, and elapsed time
          Run motor up (find suitable delay/loop timeline) 
      */
      HandleData();
      MotorUp();
      IsDone(); //is the test done? i.e did the force go to zero
    }
    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int IsDone() //conditions that the test (reading of new data) should be ended
  { 
    unsigned long RunCurrentTime = millis();
    unsigned long RunElapsedTime = RunCurrentTime - RunStartTime; //gets the new elapsed time of test being run

    if(LoadCellReadOut_MAX > MaxForceAllowed_SP) //prevents test from continuing if the max force bound is crassed
      {
        md.setM1Speed(0); //stops motor
        md.setM1Speed(-100); //pushes the hook down in order to release force
        DisplayString(Error_MaxWeight); //displays the error code for this error UPDATE ERROR CODE!!
        StopFactor = 1;
      }
          
    else if(RunElapsedTime >= MaxTimeAllowed_SP)
      {
        DisplayString(Error_TimeOut); //displays the error code for this error UPDATE ERROR CODE!!
        StopFactor = 1; 
      }
      
     else if(md.getFault()) //checks for fault with/in the motor controller
      {
        StopFactor = 1;
      }
      
     return StopFactor;
   }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// re-open the file for reading: functions: https://www.arduino.cc/en/Reference/FunctionDeclaration
char MakeFileName(char *FileName)
  {
    //the variables are defined in the Global Variable section so that the function does not overwrite the varibles each time its called
    FileName[5] = hundreds;
    FileName[6] = tens;
    FileName[7] = ones;
    FileName[8] = '.';
    FileName[9] = 't'; //just using text file, but can be changed to whatever...its no more difficult to import into Excel
    FileName[10] = 'x';
    FileName[11] = 't';
    ones++;

    if(ones == 10)    //these make sure the count up is correct
    {tens++; ones=0;}
    if(tens == 10)
    {hundreds++; tens=0;}

    return FileName;
  }
  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MotorDown()
  { 
    GetRotationalSpeed();
    myPID_DOWN.Compute();
    md.setM1Speed(InputMotorSpeed_DOWN); //sets motor input current (speed) (negative number)
       IsDone(); //stops if error detected with motor controller
    md.setM1Speed(0); //turns motor "off"
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MotorUp()
  {
    GetRotationalSpeed();
    myPID_UP.Compute();
    md.setM1Speed(InputMotorSpeed_UP); //sets motor input current (speed) (positive number)
    IsDone(); //stops if error detected with motor controller
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double GetRotationalSpeed()
  {
    int PulseCount = 0; //counts number of pulses (implicit connection to agular speed)
    unsigned long ElapsedTime = 0;
    
    unsigned long StartTime = millis();  //put here to get the most accurate elapsed time
    while(ElapsedTime <= MeasuredTime) //runs for the set time (in milliseconds)
      {
        //EncoderReadOut = digitalRead(EncoderDataPin); //reads for pulse...don't need this stand alone statement
        if((digitalRead(EncoderDataPin)) == HIGH) {PulseCount++;} //reads for pulse and counts up the pluses read
        
        unsigned long CurrentTime = millis();
        ElapsedTime = CurrentTime - StartTime; //gets the new elapsed time to check with the while loop
      }
    
    double EncoderReadOut_Diff = (double) PulseCount; //position difference..."(double)" is a cast operator to change variables to double type
    
    return EncoderReadOut_Diff;
    //the number of pulses read can be used later to find an average speed and consequently total displacement
  }
