/*
Measuring AC Current Using ACS712
Code by Rusty B.
*/
#include <Wire.h>               // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h"        // legacy: #include "SSD1306.h"

#include <Ticker.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

SSD1306Wire display(0x3c, SDA, SCL);   // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h

#define LED 2
#define RELAY 1
#define BUTTON 3

#define MINCURRENT 0.4 
#define MAXRUNTIME 30

const int sensorIn = A0;  //use ADC 0
int mVperAmp = 100; // use 185 for 5A Module, 100 for 20A Module, and 66 for 30A Module

double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;
double current = 0;

int readValue;             //value read from the sensor
int maxValue = 0;          // store max value here
int minValue = 1024;       // store min value here
float vpp;                 // peak to peak voltage 

bool fcomputeValues = false; //flag to indicate when the measurement is complete (measurement is done in interrupt)
bool ftimeOut = false;
bool fcurrentDetected = false;
int msTimer = 0;   //millisec timer 
int secTimer = 0;
int minuteTimer = 0;
Ticker OneMsTimer; // Ticker class - used for a millisecond interrupt

//*****************************************************************************
// One millisecond timer interrupt - see below for the action in this interrupt
//*****************************************************************************
void OneMsTimerInterrupt()
{
  msTimer++; //increment ms counter

  //*********************************************************************************************************************
  // ADC Section - read and find the min and max values during a one second period - 1000 times 
  //*********************************************************************************************************************
  readValue = analogRead(sensorIn);  //get the value from the adc happens every millisecond - look for min and max values
  // see if you have a new maxValue
  if (readValue > maxValue) 
  {
    /*record the maximum sensor value*/
    maxValue = readValue;
  }
  //see if you have a new min value
  if (readValue < minValue) 
  {
     /*record the minimum sensor value*/
     minValue = readValue;
  }

  //***********************************************************************************************************
  // One Second section of interrupt   
  //  ----compute the vpp based on 1000 min max accumulations
  //  ----keep track of seconds and minutes
  //  ----toggle LED for a hearbeat indication
  //  ----Set a flag to indicat that a new vpp is ready so the main loop can compute the current current... lol
  //***********************************************************************************************************
  if(msTimer >= 1000)                            //one second has passed - 1000 milliseconds
  {
     digitalWrite(LED, !(digitalRead(LED)));     //toggle the LED to give us a one sec heartbeat
         // Subtract min from max
    vpp = ((maxValue - minValue) * 5.0)/1024.0;  //compute peak to peak voltage

    fcomputeValues = true;                       // let the main loop know we ae ready to compute current
    msTimer = 0;                                 // reset the msTimer 
    maxValue = 0;                                // reset max value
    minValue = 1024;                             // reset min value

    if(fcurrentDetected == true)                         // we only need to update timers if there is current detected
    {
      secTimer++;                                  // increment second timer
      if(secTimer >= 60)                           // check if 60s has passed if so update minute timer and reset second timer
      {
        minuteTimer++;                             // increment minute timer
        secTimer = 0;                              // reset second timer
        if(minuteTimer>= MAXRUNTIME)
        {
          ftimeOut = true;                         //set a flag to indicate timeout has occured 
          digitalWrite(RELAY, 1);                  // energize the power relay to shutoff device 
        }
      }
    }
  }//**************************************** End one second section *********************************************************
}// end interrupt

void setup()
{ 
  //setup the hardware 
  pinMode(LED,OUTPUT);
  pinMode(RELAY,OUTPUT);
  pinMode(BUTTON,INPUT_PULLUP);

  //make sure that the power really is OFF! - its wired to the normally closed side so that power will still work when board is dead
  digitalWrite(RELAY, 0); 

  //Start the ms timer interrupt
  OneMsTimer.attach_ms(1.0, OneMsTimerInterrupt); //Use attach_ms if you need time in ms

  // Initialising the UI will init the display too.
  display.init();
  //display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
    // clear the display
  display.clear();

  // for debugging 
  Serial.begin(9600);
}

//********************************************************************************************************
// Main loop
//********************************************************************************************************
void loop()
{

  //check the flag from the interrupt to see if its time to make the bacon
  if(fcomputeValues == true)
  { //yep time to cook 
    current = getVPP();
    // reset the flag to let intrrupt know we are done cooking
    fcomputeValues = false;

  
    //************************************************************************************
    //  Check for a device drawing current. If it is then start a timer based on a define  
    //  that will power down the device after a set amount of time. If device times out and powers down 
    //  go into idle state waiting for button press - this will be a state machine
    //************************************************************************************
    if(current > MINCURRENT && ftimeOut == false)   //State 0 - a device is connected and powered on must draw more than 0.4 mAmps AC
    {// there is something drawing current so its ON
      fcurrentDetected = true;                            //Set a flag to indicate that there is something on
      display.clear();                                   //Display to the OLED
      display.setFont(ArialMT_Plain_16);
      display.drawString(3, 5, "Amps: ");
      display.drawString(53, 5, String(current));
      display.setFont(ArialMT_Plain_24);
      display.drawString(44, 23, "ON");
      display.setFont(ArialMT_Plain_16);
      display.drawString(3, 48, "TIME-");      
      display.drawString(43 + 10, 48, String(minuteTimer));      
      display.drawString(54 + 10, 48, " : ");      
      display.drawString(65 + 10, 48, String(secTimer));      
      display.display();

    }
    else if( current < MINCURRENT && ftimeOut == false)   //State 1 - nothing connected or turned on - no current          
    {// no current its off
      display.clear();
      display.setFont(ArialMT_Plain_24);
      display.drawString(44, 43, "OFF");
      display.display();
      fcurrentDetected = false;
      //reset the timers
      secTimer = 0;
      minuteTimer = 0;

    }
    else if (ftimeOut == true)      //State 2
    {// A timeout has occured so turn off power relay and wait for button press to reset everything
      display.clear();
      display.setFont(ArialMT_Plain_24);
      //display.setTextAlignment(TEXT_ALIGN_RIGHT);
      display.drawString(44, 23, "TIMEOUT");
      display.setFont(ArialMT_Plain_16);
      display.drawString(3, 48, "PRESS BUTTON TO RESET");
      // write the buffer to the display
      display.display();
      //wait for button press to reset everything
      if(!digitalRead(BUTTON))                     //check for button press
      {
        ftimeOut = false;                          //clear the timeout flag to goto normal operations
        secTimer = 0;
        minuteTimer = 0;
      }

    }
    
  }
}

float getVPP()
{
  
   VRMS = (vpp/2.0) *0.707;  //root 2 is 0.707
   AmpsRMS = (VRMS * 1000)/mVperAmp;
   //Debug Serial Port
   //Serial.print(AmpsRMS);
   //Serial.println(" Amps RMS");
      
   return AmpsRMS;
 }

