/*This is the main code for the core functions that will need to be run on/through the Arduino for the NASA device.
  Author: Garrett Barton
  GitHub Repository: https://github.com/GarrettBarton/Senior-Design-NASA-Tensile_Tester
  Date Started: 10Dec18
  Date Installed (version):  
  Version: 2.0
  Liscence: MIT Liscence Garrett Barton (c) (2019)
*/
int debug_path = 0; //make it 1 to have the path print out to the serial monitor
int debug_data = 0; //make it 1 to have the data print out to the serial monitor
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////    Library Definitions    ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
      int chipSelect = 53;
      File dataFile; //Instantiate a text file controller object
      
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Buttons 
      const int DownButtonPin = 34; //sets the pin for the down button to 3
      const int RunButtonPin = 35;
      
      const int DownButtonLED_Pin = 36; //sets the pin for the down button LED to 23
      const int RunButtonLED_Pin = 37;
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Amplifier breakout
      HX711 scale; //Instantiate an amplifier controller object
      const float calibration_factor = -5000; //Value found using the calibration sketch (experimental)
      const int DOUT = 46;
      const int CLK = 44;
      const int gain = 64; //can use 128, but will return 2x the weight
      float LoadCellReadOut; // floats have 7 digits of percision and can keep decimal points in calculations
      float LoadCellReadOut_MAX = 0.0; //intialize the maximum force to zero
      int ForceDifferential_SP_MAX = 5; //minimum drop in force to be considered done (pounds)
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Motor Driver
      DualMC33926MotorShield md; //Instantiate a motor sheild controller object
      //default pinout - M1PWM = 9, M1DIR = 7, M1FB = N/A, SF = 12 
      
      double InputMotorSpeed_UP = 0; //This will be the value that changes the motor controller "speed"
      double InputMotorSpeed_DOWN = -160; //Makes the motor go down 
      int InputMotorSpeed_Offset = 150; //gets motor to the about the SP initially 
                                       //doing this to shorten time needed to get to SP and oscillations, 
                                      //since this jump will occur every time the same way 
      int MaxForceAllowed_SP = 250; //Set point for maximum force (in pounds) in order to stop the test before its overloaded
      int StopFactor; //used to stop test while running
    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////   Setup Function   ////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
  {
    Serial.begin(9600); //setting baud rate (9600 is standard and pretty common)
    if(debug_path==1){Serial.println("Inside of setup()");}
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Display
      byte numDigits = 4;
      byte digitPins[] = {22, 23, 24, 25}; //Digits: 1,2,3,4 //sets the ground pins for each digit (need 4)
      byte segmentPins[] = {26, 27, 28, 29, 30, 31, 32, 33}; //sets (A,B,C,D,E,F,G,DP) in order
      bool resistorsOnSegments = false; //is there a current limiting resistor? (need for multidigit displays)
      bool updateWithDelays = false; // Default 'false' is Recommended
      bool leadingZeros = false; // Use 'true' if you'd like to keep the leading zeros
      byte hardwareConfig = COMMON_CATHODE; //sets what is common (i.e. cathode or anode)
      sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments,updateWithDelays, leadingZeros);
      sevseg.setBrightness(90); //range is from 0 to 100
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //SD breakout
      if (!SD.begin(53)) 
        {
          if(debug_path==1){Serial.println("initialization failed!"); while (1);}
        }
      dataFile = SD.open("TestData.txt", FILE_WRITE); //opens text file "TestData"...limited to 8 characters!
      dataFile.println(",///////////////"); //Older test data above \ Newer test data below
     
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Buttons
      pinMode(RunButtonLED_Pin,OUTPUT); //initializes LED control pin as output
      pinMode(DownButtonLED_Pin,OUTPUT);
      pinMode(DownButtonPin,INPUT); //initializes button pin as input
      pinMode(RunButtonPin,INPUT);
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Amplifier breakout
      scale.begin(DOUT, CLK, gain); //initializes amplifier with pinout and set gian
if(debug_path==1){Serial.println("Load cell readout:");Serial.println(-1*scale.get_units(10));}
      delay(100); //found previously that some time is needed as buffer after such statements
      scale.set_scale(calibration_factor);
      scale.tare(10); //with NO weight (load other than the harware being used) on the load cell, resets the read out to 0
if(debug_path==1){Serial.println("Load cell readout:");Serial.println(-1*scale.get_units(10));}
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Motor Controller
      md.init(); //initializes motor default pinout
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    DisplayString("DONE");
  }
  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////   Main Function   ////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
  {  
    if(debug_data==1){Serial.println(-1*scale.get_units(10));}
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Read Buttons
    //Buttons ("run" and "down")
      int DownButtonState = digitalRead(DownButtonPin); //Reads pin for state
      int RunButtonState = digitalRead(RunButtonPin);
        
    /////////////////////////////////////////////////////////////////////////////////////////////////////// Down Button Pressed
      if(DownButtonState == 1) //down button is pressed
        {
          if(debug_path==1){Serial.println("Down button pressed");}
          
          while(digitalRead(DownButtonPin) == 1)
          { 
            md.setM1Speed(InputMotorSpeed_DOWN); //turns motor to intial 3 in/min
            digitalWrite(DownButtonLED_Pin, HIGH); //to acknowledge that the command was taken
//            DisplayString(Warning_GoingDown);
          }
        }
        
      else if (DownButtonState == 0) //to turn LED off and turn off motor when button is released
        {
          digitalWrite(DownButtonLED_Pin, LOW); 
          md.setM1Speed(0); 
          sevseg.blank(); //turns display off
        }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////// Run Test Button Pressed
      if(RunButtonState == 1) //run button is pressed
        {   
          if(debug_path==1){Serial.println("Run button pressed");}     
          digitalWrite(RunButtonLED_Pin, HIGH); //to acknowledge that the command was taken
          int StopFactor = 0; //don't need to stop test

          dataFile.println("\n\r,Load Cell Reading, Run Time (ms)");
          
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
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////    Function Definitions    ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// IsDone()
int IsDone() //conditions that the test (reading of new data) should be ended
  { 
    if(debug_path==1){Serial.println("Inside of IsDone()");}
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////// Max Force
    if((-1 * scale.get_units(10)) > MaxForceAllowed_SP) //prevents test from continuing if the max force bound is crassed
      {
        /////////////////////////////////////////////////////////////////////////////////// Immediate Remedial Action 
        md.setM1Speed(0); //stops motor
        delay(50); //just like a vehicle, don't just slam it in reverse when you're on the highway
        md.setM1Speed(InputMotorSpeed_DOWN); //pushes the hook down in order to release force
        ///////////////////////////////////////////////////////////////////////////////////
        DisplayString(Error_MaxWeight); //displays the error code for this error UPDATE ERROR CODE!!
        StopFactor = 1;
      }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////// Motor Driver Fault
    else if(md.getFault()) //checks for fault with/in the motor controller
      {StopFactor = 1;}
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////// Run Button Released
    if(digitalRead(RunButtonPin) == 0) //checks it run button has been released
      {StopFactor = 1;}
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////// Not Done
    else //if none of the conditions above are met keep rolling
      {StopFactor = 0;}
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     
    return StopFactor;
   }
   
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// RunTest()
void RunTest()
    {
      /*
        Flow:
          Set the motor driver to get the 3 in/min from the get-go
          Read load cell for intial reading
          Log data gathered from  load cell and elapsed time
          Use else if statements to adjust the motor driver
          Close the file when done (prevents errors)
          Print the maximum force read to the display
      */
      //the addition of the else if statements marks an acceptance of the failure to get the PID working properly...it seriously saddens me
        /*
         * for the numbers being added the range is [-400, 400], but negative is down.
         * the approxiamtions are as follows:   
         *    + 12.6 Vdc means 3 Vdc per 100 means: 
         *      ~ 8 = 0.25 Vdc
         *      ~ 17 = 0.5 Vdc
         *      ~ 25 = 0.75 Vdc
         *      ~ 33 = 1 Vdc
         * there is a greater focus on resolution on the lower end because that is the end the device will typically works in
         */
      
      if(debug_path==1){Serial.println("Inside of RunTest()");}
      if(debug_path==1){Serial.println("Start RunStartTime");}
      LoadCellReadOut_MAX = 0; //reset the maximum saved value
      unsigned long RunStartTime = millis();  //put here to get the start of the test 

      StopFactor = IsDone(); //proxy variable for StopFactor

      while(StopFactor == 0)
        {
          if(debug_path==1){Serial.println("Inside of RunTest() while loop");}

          ///////////////////////////////////////////////////////////////////////////////////////
          LoadCellReadOut = -1 * scale.get_units(10); //returns a float of the current measurement, 
                                                     //is actually the average of a set number (10) of readings
          ///////////////////////////////////////////////////////////////////////////////////////////////////////////
          if(debug_data==1){Serial.print("LoadCellReadOut: "); Serial.println(LoadCellReadOut);}
          
          /////////////////////////////////////////////////////////////////////////////////////////////////////// Bubble sorting for max
          if(LoadCellReadOut_MAX < LoadCellReadOut) //Bubble sort method to find maximum force
            {LoadCellReadOut_MAX = LoadCellReadOut;}
          ///////////////////////////////////////////////////////////////////////////////////////////////////////////
          StopFactor = IsDone(); //gets the new StopFactor to check in the while loop conditional
          
          /////////////////////////////////////////////////////////////////////////////////////////////////////// Package and store
          dataFile.print(", ");
          dataFile.print(LoadCellReadOut);
          dataFile.print(", ");
          dataFile.println(millis()); //gives a time stamp
          ///////////////////////////////////////////////////////////////////////////////////////////////////////////
          
          /////////////////////////////////////////////////////////////////////////////////////////////////////// Motor control 
          if(StopFactor == 0)
            { 
              if(LoadCellReadOut <= 22) {InputMotorSpeed_UP = (InputMotorSpeed_Offset - 8);} //3.75 Vdc
              else if(LoadCellReadOut <= 45) {InputMotorSpeed_UP = (InputMotorSpeed_Offset + 8);} //4.25 Vdc
              else if(LoadCellReadOut <= 68) {InputMotorSpeed_UP = (InputMotorSpeed_Offset + 17);} //4.5 Vdc 
              else if(LoadCellReadOut <= 110) {InputMotorSpeed_UP = (InputMotorSpeed_Offset + 25);} //4.75 Vdc
              else if(LoadCellReadOut <= 110) {InputMotorSpeed_UP = (InputMotorSpeed_Offset + 33);} //5 Vdc
              else if(LoadCellReadOut <= 120) {InputMotorSpeed_UP = (InputMotorSpeed_Offset + 50);} //5.5 Vdc
              else if(LoadCellReadOut <= 250) {InputMotorSpeed_UP = (InputMotorSpeed_Offset + 66);} //6 Vdc
              else  {InputMotorSpeed_UP = InputMotorSpeed_Offset;}

              md.setM1Speed(InputMotorSpeed_UP);
            } 
          ///////////////////////////////////////////////////////////////////////////////////////////////////////////    
                 
          if(debug_data==1){Serial.print("LoadCellReadOut_MAX in while loop: "); Serial.println(LoadCellReadOut_MAX);} 
        }
        
      unsigned long RunEndTime = millis();  //put here to get the start of the test
      md.setM1Speed(0); //stops motor
      

    
      /////////////////////////////////////////////////////////////////////////////////// Wrap-up Procedures        
      dataFile.print(",Total test time: ,");
      dataFile.println(RunEndTime);
      dataFile.close(); //closes the text file...ONLY ONE FILE CAN BE OPEN AT A TIME            
//      DisplayNumber((LoadCellReadOut_MAX/0.785398)); //displays the maximum tensile stress recorded ... force / (pi x r^2)
      DisplayNumber((LoadCellReadOut_MAX)); //displays the maximum force recorded
      ///////////////////////////////////////////////////////////////////////////////////////
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Display
//I lumped these together because they are basically the same thing, just so slightly different purposes
void DisplayNumber(float DisplayValue)
  {
    int DisplayDecimalPlace;
    unsigned long DisplayElapsedTime = 0; //initialize the time elapsed for reading encoder
    unsigned long DisplayStartTime = millis();  //starting time of reading...put here to get the most accurate elapsed time
    
    if(debug_path==1){Serial.println("Inside DisplayNumber()");}

    if(DisplayValue < 10){DisplayDecimalPlace = 1;}
    else if(DisplayValue < 100){DisplayDecimalPlace = 1;}

    while(DisplayElapsedTime <= DisplayTime)
      {
        sevseg.setNumber(DisplayValue, DisplayDecimalPlace); //prints what is inputed (i.e. "4" prints 4)
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
    
    if(debug_path==1){Serial.println("Inside DisplayString()");}
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
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////     That's all folks!      ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
