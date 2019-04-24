/*This is the main code for the core functions that will need to be run on/through the Arduino for the NASA device.
  Author: Garrett Barton
  GitHub Repository: https://github.com/GarrettBarton/Senior-Design-NASA-Tensile_Tester
  Date Started: 10Dec18
  Date Installed (version):  
  Version: 1.2
  Liscence: MIT Garrett Barton (c) (2019)
*/
int debug = 1; //make it 1 to have the path print out to the serial monitor
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////    Library Definitions    ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <PID_v1.h> //Arduino PID Library - V1.2.1 by Brett Beauregard (c) (2017)
#include <SevSeg.h>//Arduino Seven Segment Display Library - V3.3.0 - Apache License (version 2.0) by Dean Reading (2017)
#include <SD.h> //Arduino SD library - by Limor Fried (modified by Tom Igoe) (c) Copyright 2010 SparkFun Electronics
#include <SPI.h> //Arduino Serial Peripheral Interface library - by Cristian Maglie (c) Copyright 2010
#include <HX711.h> //For the HX711 amplifier - by Bogdan Necula MIT License (2018)
#include <DualMC33926MotorShield.h> //For motor driver - V4.0.0 - by Pololu Corporation Copyright (c) 2012

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////    Global Variable Definitions    /////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * Global variables are stored in SRAM, which means that they are "on call" for use. However, this means that there is less 
 * room for "on call" data, which can be a problem when doing lots of processing (or storing a lot of data into a string).
 * The Arduino Mega 2560 has a 8 kB SRAM these glodal variables take up about 1364 B (16%) of that. If you need more SRAM 
 * capacity you will need to move variables out of global positions.
 */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Display
      SevSeg sevseg;  //instantiate a seven segment controller object
      int DisplayTime = 3000; //time that the display will show the error and/or the max force (in milliseconds)    
      
    //Error Codes
      char Error_NotConnectedToLoadCell[] = "E00";
      char Error_MaxWeight[] = "E01";
      char Error_TimeOut[] = "E02";
      char Test = 'E';
 
    //Warning Codes
      char Warning_SDError[] = "3 00";
      char Warning_InvalidTest[] = "3 01";
      char Warning_GoingDown[] = "DOUN";
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //SD breakout
      //there is no need to instantiate an object for the SD control because it is preset as "SD"
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
      HX711 scale; //Instantiate an amplifier controller object
      const float calibration_factor = -3000; //Value found using the calibration sketch (experimental)
      const int DOUT = 46;
      const int CLK = 44;
      const int gain = 64; //can use 128, but will return 2x the weight
      float LoadCellReadOut; // floats have 7 digits of percision and can keep decimal points in calculations
      float LoadCellReadOut_MAX = 0.0; //intialize the maximum force to zero
      int ForceDifferential_SP_MAX = -5; //minimum drop in force to be considered done (pounds)
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Encoder
      int EncoderPPR = 512; //this is selectable, but this what is set currently
      int EncoderData_PinA = 48;
      int EncoderData_PinB = 47;
      double EncoderReadOut_Diff = 0.0;  //this is a double because the PID reqires an input of a double
      int MeasuredTime = 10; //this will make the SP about 125 (really its 125.69...)
      unsigned long RunStartTime = millis();
      int EncoderReadOut_PinA_Current = LOW; //intializing for reading from the encoder
      int EncoderReadOut_PinA_Last = LOW;
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Motor Driver
      DualMC33926MotorShield md; //Instantiate a motor sheild controller object for moving up
      //default pinout - M1PWM = 9, M1DIR = 7, M1FB = N/A, SF = 12 
      
      double InputMotorSpeed_UP = 0; //This will be the value that changes the motor controller "speed"
      double InputMotorSpeed_DOWN = -160; //Makes the motor go down 
      int InputMotorSpeed_Offset = 160; //gets motor to the about the SP initially 
                                       //doing this to shorten time needed to get to SP and oscillations, 
                                      //since this jump will occur every time the same way    
      int StopFactor; //used to stop test while running
   
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //PID
      double Kp = 2;  //experimentally found by force feeding inputs
      double Ki = 0.5;
      double Kd = 0.2;
  
      double LinearSpeed_SP_UP = 3.0; //Set point for linear speed (in inches per minute)

      int MaxForceAllowed_SP = 250; //Set point for maximum force (in pounds) in order to stop the test before its overloaded
      int MaxTimeAllowed_SP = 15000; //Set point for maximum allowed time for the force to be read at zero before the test is ended in ms
  
      double RotationalSpeed_SP_UP = ((LinearSpeed_SP_UP * 16) * (30.681) * ((0.01667)*(0.001)) * (EncoderPPR) * (MeasuredTime)); 
      //((inches to 1/16ths of inches)*(Lead screw rotations to get that many inches))*(conversion to rotations per millisecond)*(EncoderPPR)*(the delay value)
  /*
      gear ratio = 30.681 and
      1 rotation of lead screw = 1/16 inch of linear motion;
      therefore, the motor needs to be 1472.73 rotations/min to get 3 in/min. 
      The PPR of the encoder is set to 512, which means that there will be about 12.57 pulses read per ms
      If the MeasuredTime is set to 10 ms, then the set point should be equal to 125.69
      EncoderReadOut_Diff should be equal to ~125
  */
      
      PID myPID_UP(&EncoderReadOut_Diff, &InputMotorSpeed_UP, &RotationalSpeed_SP_UP, Kp, Ki, Kd, P_ON_E, DIRECT);
      //Specify the links and initial tuning parameters
      //Instantiate a PID controller object for moving up and down
      //P_ON_E is Proportional On Error - the Kp will with respect to the error
      //DIRECT is the type of responce the PID will output when faced with an error (REVERSE would be used with a cooling system)
    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////   Setup Function   ////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
  {
    Serial.begin(9600); //setting baud rate (9600 is standard and pretty common)
    if(debug==1){Serial.println("Inside of setup()");}
    
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
      char FileName[] = "000.txt"; //creates the "space" for the file name to be created
//      SD.begin(53);
      if (!SD.begin(53)) {
    Serial.println("initialization failed!");
    while (1);
  }
//      pinMode(53, OUTPUT); //make sure that the default chip select (CS) pin is set to output, even if you don't use it
         
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Buttons
      pinMode(RunButtonLED_Pin,OUTPUT); //initializes LED control pin as output
      pinMode(DownButtonLED_Pin,OUTPUT); 
      pinMode(DownButtonPin,INPUT); //initializes button pin as input
      pinMode(RunButtonPin,INPUT);
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Amplifier breakout
      scale.begin(DOUT, CLK, gain); //initializes amplifier with pinout and set gian
      delay(100); //found previously that some time is needed as buffer after such statements
      scale.set_scale(calibration_factor);
      scale.tare(10); //with NO weight on the load cell, resets the read out to 0

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Encoder
      pinMode (EncoderData_PinA, INPUT);
      pinMode (EncoderData_PinB, INPUT);
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Motor Controller
      md.init(); //initializes motor default pinout
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //PID
      myPID_UP.SetMode(AUTOMATIC); //makes the PID run automatically
      double PIDOutput_Max = 400; //this is the input range of the motor driver
      double PIDOutput_Min = -400;
      myPID_UP.SetOutputLimits(PIDOutput_Min, PIDOutput_Max); //enusres that the PID won't output beyond the range set
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  }
  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////   Main Function   ////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
  {  
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Read Buttons
    //Buttons ("run" and "down")
      DownButtonState = digitalRead(DownButtonPin); //Reads pin for state
      RunButtonState = digitalRead(RunButtonPin);
        
    /////////////////////////////////////////////////////////////////////////////////////////////////////// Down Button Pressed
      if(DownButtonState == 1) //down button is pressed
        {
          if(debug==1){Serial.println("Down button pressed");}
          md.setM1Speed(InputMotorSpeed_DOWN); //turns motor to intial 3 in/min
          digitalWrite(DownButtonLED_Pin, HIGH); //to acknowledge that the command was taken
          int StopFactor = 0; //don't need to stop test
          unsigned long RunStartTime = millis();  //put here to get the start of the test
          DisplayString(Warning_GoingDown);
        }
        
      else if (DownButtonState == 0) //to turn LED off and turn off motor when button is released
        {
          digitalWrite(DownButtonLED_Pin, LOW); 
          md.setM1Speed(0); 
          sevseg.blank(); //turns display off
        }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////// Run Test Button Pressed
      if(RunButtonState == 1) //run button is pressed
        {   
          if(debug==1){Serial.println("Run button pressed");}     
          digitalWrite(RunButtonLED_Pin, HIGH); //to acknowledge that the command was taken
          int StopFactor = 0; //don't need to stop test
          
          MakeFileName(FileName); //creates a file name for the test (suppose to count up)
          dataFile = SD.open("DataLog.txt", FILE_WRITE); //opens text file "DataLog"
          
          dataString += "\n\rTest " + String(TestNumber) + "\n\r"; //identifies  test number
          TestNumber++; //increments TestNumber such that the test runs can be identified in a session

          if(debug==1){Serial.println("Start RunStartTime");}
          unsigned long RunStartTime = millis();  //put here to get the start of the test 
          md.setM1Speed(InputMotorSpeed_Offset); // sets motor to the intial 3 in/min rate (assuming no to little load)
          RunTest();
          dataFile.close(); //closes the text file...ONLY ONE FILE CAN BE OPEN AT A TIME
          sevseg.blank(); //turns display off
        }

      else if (RunButtonState == 0) //to turn LED off and turn off motor when button is released
        { 
          digitalWrite(RunButtonLED_Pin, LOW);
          md.setM1Speed(0); 
          sevseg.blank(); //turns display off
        }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////    Function Definitions    ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Display
//I lumped these together because they are basically the same thing, just so slightly different purposes
void DisplayNumber(float DisplayValue)
  {
    unsigned long DisplayElapsedTime = 0; //initialize the time elapsed for reading encoder
    unsigned long DisplayStartTime = millis();  //starting time of reading...put here to get the most accurate elapsed time
    
    if(debug==1){Serial.println("Inside DisplayNumber()");}
    while(DisplayElapsedTime <= DisplayTime)
      {
        sevseg.setNumber(DisplayValue); //prints what is inputed (i.e. "4" prints 4)
                                       //to print decimal, (4999, 3) is 4.999 (i.e DP is "3" from the right
        
        sevseg.refreshDisplay(); //required at end of loop to continue displaying the number
        unsigned long DisplayCurrentTime = millis();
        DisplayElapsedTime = DisplayCurrentTime - DisplayStartTime; //gets the new elapsed time to check in the while loop conditional
      }
    sevseg.blank(); //turns display off
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
void DisplayString(char DisplayValue[])
  {  
    unsigned long DisplayElapsedTime = 0; //initialize the time elapsed for reading encoder
    unsigned long DisplayStartTime = millis();  //starting time of reading...put here to get the most accurate elapsed time
    
    if(debug==1){Serial.println("Inside DisplayString()");}
    while(DisplayElapsedTime <= DisplayTime)
      {
        sevseg.setChars(DisplayValue); //prints what is inputed (i.e. "4" prints 4)
                                      //to print decimal, (4999, 3) is 4.999 (i.e DP is "3" from the right
        sevseg.refreshDisplay(); //required at end of loop to continue displaying the number
    
        unsigned long DisplayCurrentTime = millis();
        DisplayElapsedTime = DisplayCurrentTime - DisplayStartTime; //gets the new elapsed time to check in the while loop conditional
      }
    sevseg.blank(); //turns display off
  }
  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// HandleData()
void HandleData() //Intializes string for data storage, reads data, finds maxe force, prevents over load, stores data to SD
  {     
    if(debug==1){Serial.println("Inside HandleData()");}
    /////////////////////////////////////////////////////////////////////////////////////////////////////// Get readouts
    LoadCellReadOut = scale.get_units(10); //returns a float of the current measurement, 
                                          //is actually the average of a set number (10) of readings
    EncoderReadOut_Diff = GetRotationalSpeed(); //must use a proxy variable or GetRotationalSpeed() will dump the returned value
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    
    if(LoadCellReadOut_MAX < LoadCellReadOut) //Bubble sort method to find maximum force
      {LoadCellReadOut_MAX = LoadCellReadOut;}
      
    /////////////////////////////////////////////////////////////////////////////////////////////////////// Package for storage 
    dataString += String(LoadCellReadOut) + ", "; //concatenates the data read to the string to be stored
    dataString += String(EncoderReadOut_Diff); //concatenates the pulses read to the string to be stored
    dataString += "\n\r";  //adds new line and return carriage between readings (comma delimited)
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
  }
 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// RunTest()
void RunTest()
    {
      /*
        Flow:
          Set the motor driver to get the 3 in/min from the get-go
          Read encoder for intial position
          Read load cell for intial reading
          Log data gathered from encoder, load cell, and elapsed time
          Use PID to adjust the motor driver
          Close the file when done (prevents erros)
          Print the maximum force read to the display
      */
      if(debug==1){Serial.println("Inside of RunTest()");}
      StopFactor = IsDone(); //proxy variable for StopFactor
      
      while(StopFactor == 0)
        {
          if(debug==1){Serial.println("Inside of RunTest() while loop");}
          /////////////////////////////////////////////////////////////////////////////////// Main Flow
          md.setM1Speed(InputMotorSpeed_Offset); //gets to 3 in/min and then PID takes over
          HandleData(); //gets readings and stores them
          MotorUp(); 
          ///////////////////////////////////////////////////////////////////////////////////
                                
          StopFactor = IsDone(); //gets the new StopFactor to check in the while loop conditional
        }

      /////////////////////////////////////////////////////////////////////////////////////////////////////// Store it if ya got it
      if (dataFile) //if the file is available, write to it...Only opens file once per test
        {dataFile.println(dataString);if(debug==1){Serial.println("dataFile open...");}} //writes the string of stored readings
        
      else  // if the file isn't open, pop up an error:
        {DisplayString(Warning_SDError);if(debug==1){Serial.println("dataFile not open!)");}} //doesn't stop the test, just notify user because its not critical
      ///////////////////////////////////////////////////////////////////////////////////////////////////////
    
      /////////////////////////////////////////////////////////////////////////////////// Wrap-up Procedures        
      dataFile.close(); //closes the text file...ONLY ONE FILE CAN BE OPEN AT A TIME            
      DisplayNumber(LoadCellReadOut_MAX); //displays the maximum force recorded
      ///////////////////////////////////////////////////////////////////////////////////
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// IsDone()
int IsDone() //conditions that the test (reading of new data) should be ended
  { 
    if(debug==1){Serial.println("Inside of IsDone()");}
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////// Zeros
    unsigned long RunCurrentTime = millis();
    unsigned long RunElapsedTime = RunCurrentTime - RunStartTime; //gets the new elapsed time of test being run
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////// Force Differential
    //negative when force it decreasing
    float LoadCellReadOut_Current = scale.get_units();
    float LoadCellReadOut_Diff = LoadCellReadOut_Current - LoadCellReadOut; 
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////// Max Force
    if(LoadCellReadOut_MAX > MaxForceAllowed_SP) //prevents test from continuing if the max force bound is crassed
      {
        /////////////////////////////////////////////////////////////////////////////////// Immediate Remedial Action 
        md.setM1Speed(0); //stops motor
        delay(50); //just like a vehicle, don't just slam it in reverse when you're on the highway
        md.setM1Speed(InputMotorSpeed_DOWN); //pushes the hook down in order to release force
        ///////////////////////////////////////////////////////////////////////////////////
        DisplayString(Error_MaxWeight); //displays the error code for this error UPDATE ERROR CODE!!
        StopFactor = 1;
      }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////// Timeout
    else if(RunElapsedTime >= MaxTimeAllowed_SP)
      {
//        DisplayString(Error_TimeOut); //displays the error code for this error UPDATE ERROR CODE!!
        StopFactor = 1; 
      }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////// Motor Driver Fault
    else if(md.getFault()) //checks for fault with/in the motor controller
      {StopFactor = 1;}
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////// Run Button Released
    if(digitalRead(RunButtonPin) == 0) //checks it run button has been released
      {StopFactor = 1;}
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////// Force Went To Zero
    else if(LoadCellReadOut_Diff <= ForceDifferential_SP_MAX ) //checks for fault with/in the motor controller
      {StopFactor = 1;}
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////// Not Done
    else //if none of the conditions above are met keep rolling
      {StopFactor = 0;}
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     
    return StopFactor;
   }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// MakeFileName()
//// re-open the file for reading: functions: https://www.arduino.cc/en/Reference/FunctionDeclaration
char MakeFileName(char *FileName)
  {
    if(debug==1){Serial.println("Inside of MakeFileName()");}
    /////////////////////////////////////////////////////////////////////////////////// Break Down
    //the variables are defined in the Global Variable section so that the function 
    //  does not overwrite the varibles each time its called
    FileName[5] = hundreds;
    FileName[6] = tens;
    FileName[7] = ones;
    FileName[8] = '.';
    FileName[9] = 't'; //just using text file, but can be changed to whatever...its no more difficult to import into Excel
    FileName[10] = 'x';
    FileName[11] = 't';
    ones++;
    ///////////////////////////////////////////////////////////////////////////////////
    
    /////////////////////////////////////////////////////////////////////////////////// Count Up
    if(ones == 10)    //these make sure the count up is correct
    {tens++; ones=0;}
    if(tens == 10)
    {hundreds++; tens=0;}
    ///////////////////////////////////////////////////////////////////////////////////
    
    return FileName;
  }
  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// MotorUp()
void MotorUp()
  {
    if(debug==1){Serial.println("Inside of MotorUp()");}
    EncoderReadOut_Diff = GetRotationalSpeed();
    myPID_UP.Compute();
    delay(50); //experimentally found to be needed (50 ms was the lowest delay to get real outputs)
    md.setM1Speed(InputMotorSpeed_UP); //sets motor input current (speed) (positive number)
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// GetRotationalSpeed()
double GetRotationalSpeed()
  {
    if(debug==1){Serial.println("Inside of GetRotationalSpeed()");}
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Zeros
    int PulseCount = 0; //counts number of pulses (implicit connection to agular speed and linear speed) for each function call
    unsigned long EncoderElapsedTime = 0; //initialize the time elapsed for reading encoder
    unsigned long EncoderStartTime = millis();  //starting time of reading...put here to get the most accurate elapsed time
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Reading Logic
    while(EncoderElapsedTime <= MeasuredTime) //runs for the set time (in milliseconds)
      {     
            EncoderReadOut_PinA_Current = digitalRead(EncoderData_PinA); //reads current level of pin A on the encoder
            
            if ((EncoderReadOut_PinA_Last == LOW) && (EncoderReadOut_PinA_Current == HIGH)) //did pin A hit positive edge?
            {
              if (digitalRead(EncoderData_PinB) == LOW) //did pin B have a negative edge?
                {PulseCount++;} //if yes, hook is going UP 
              else 
                {PulseCount--;} //if not, hook is going DOWN    
            }
            
        EncoderReadOut_PinA_Last = EncoderReadOut_PinA_Current; //saves that last read level on pin A for next reading period

        unsigned long EncoderCurrentTime = millis();
        EncoderElapsedTime = EncoderCurrentTime - EncoderStartTime; //gets the new elapsed time to check in the while loop conditional
      }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    double EncoderReadOut_Diff = (double) PulseCount; //position difference (magnitude)..."(double)" is a cast operator to change variables to double type
    
    return EncoderReadOut_Diff; //this returns the value, but does not store it anywhere (i.e. need proxy variable to store to)
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////     That's all folks!      ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
