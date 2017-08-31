// A very big thank you to the following people!
// Alejandro Nuñez Bernete
// Jackie Swinehart
// John Rudolf
// Mark Gumtang

/*


 --- ALL CURRENT BUGS OR BAD CODE ARE SEARCHABLE USING '*****' --- 

 ________      ________      ___      ___  ________     
|\   __  \    |\   __  \    |\  \    /  /||\_____  \    
\ \  \|\  \   \ \  \|\ /_   \ \  \  /  / /\|____|\ /_   
 \ \  \\\  \   \ \   __  \   \ \  \/  / /       \|\  \  
  \ \  \\\  \   \ \  \|\  \   \ \    / /       __\_\  \ 
   \ \_______\   \ \_______\   \ \__/ /       |\_______\
    \|_______|    \|_______|    \|__|/        \|_______|                                                        
                                                        

 OpenBarbell V3.0 - the sucessor to the succesor of the world's first open source velocity measuring device
 
 This code utilizes an RFduino and an HLC2705 quatrature encoder chip to
 read the position of a retractable string attached to a barbell. 
 
 You can see squatsandscience.com/product/openbarbell-v3/ for more information.
 
 Copyright (c) 2017 squatsandscience.com.  All right reserved.
 This code is free software; you can redistribute it and/or
 modify it under the terms of the Creative Commons 
 Attribution-NonCommercial-ShareAlike 4.0 International Public License.
 This code is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the Creative Commons Attribution-NonCommercial-ShareAlike 
 4.0 International Public License for more details.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  
 Created 2017
 Jordan Berke
 Jonathan Lenoff
 Elliot Noma
 Nick "The Hammer" Johnson
 Squats & Science Labs, LLC
 */
 
// http://www.patorjk.com/software/taag/#p=display&h=0&v=0&f=ANSI%20Shadow&t=*%20W%20A%20R%20N%20I%20N%20G%20*
 
//Fix rest time to be last rest time
//Fix go back at delete prompt

#include <SPI.h>
#include <Wire.h>
#include <Arduino.h>
#include <MAX1704.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306ms.h>
#include <RFduinoBLE.h>
#include <Filters.h>
#include "FastLED.h"
#include <avr/pgmspace.h>


////////////////////////
//#define DEBUG     1
////////////////////////


////-------------------------------------------------------------------------
//////////            0. Preprocessor Definitions            ////////////
////-------------------------------------------------------------------------

//OLED Definitions
  #define OLED_RESET 9
  Adafruit_SSD1306 display(OLED_RESET);

//Filter Definitions
  #define max_tick_time_allowable_init 130000
  #define precisionCounter_start 5
  #define XPOS 0
  #define YPOS 1
  #define DELTAY 2
  #define LOGO16_GLCD_HEIGHT 16 
  #define LOGO16_GLCD_WIDTH  16 
  #define MAXVEL 10

//LED Definitions
  #define NUM_LEDS 1
  #define DATA_PIN 2
  #define LOW_POWER 1
  #define BRIGHTNESS 20
  enum color {RED, GREEN, BLK, WHT};
                                                       

/***********START DEVICE SPECIFIC INFO ***************/                                 
  const char *device_name = "OB 6666";
  const long ticLength = 2718;	
  const int unit_number = 6666;
  color COLOR = RED;
/***********END DEVICE SPECIFIC INFO ***************/

//Version Info
  float CODE_VERSION = 3.01;

//TestBed Section - Do Not Modify
  const bool testbed_readouts = 0;


//-------------------------------------------------------------------------
//////////                  1. Variables                     ////////////
//-------------------------------------------------------------------------

//BT Setup
  boolean bluetoothOn = false;
  boolean bluetoothStartNextLoop = false;
  float repPerformance[] = {0.0, 1.0,2.0,3.0,4.0,5.0};
  bool BTRefresh = false;
  bool BTisconnected = 0;

 //LED Setup
  CRGB leds[1];                       
  byte r,g,b,LVL=BRIGHTNESS,userColor=0;

//Pin Definitions
  const int pin_buttonRight =  0;
  const int pin_buttonLeft = 1;
  //const int pin_led =  2;      // the number of the LED pin
  const int pin_encoder_dir = 3;
  const int pin_encoder_tach = 4;

//State Initializations
  uint16_t charge = 50;
  const unsigned long batteryCheckTime = 10000000;     //The amount of time between battery checks. 10 sec
  volatile int state = LOW;
        /*state flips on startup, so we need to put the temp value at HIGH so we don't run through the 
        code one time at startup before getting an actual tic reading*/
  volatile int currentStateTemp = HIGH;
  volatile bool goingUpward = 0;
  volatile bool isGoingUpwardLast = 0;
  long avgVelocity = 0;
  unsigned long starttime = 0;
  bool overrun = false;
  bool isFlipped = false;
  bool flipPowerOlyScreen = false; //0 = Power screen, 1 = Oly screen
  bool sendData = false;
  bool initialized = false;
  bool flippedDirection = false;
  bool normalDirection = false;
  bool dataCOMPRESSION_enabled = 1;  //THIS MUST BE ENABLED (1) FOR PRODUCTION CODE - ONLY DISABLE (0) FOR TESTING PURPOSES!
  bool full_data_logging_enabled = 0;

//Rep Variable Initializations
  uint16_t rep = 0;
  uint16_t repDone = 0;
  uint16_t repDoneLast = 0;
  uint16_t repDisplay = 2;
  uint16_t repDisplayLast = 0;
  uint16_t peak_vel_at = 0;
  long displacement = 0;
  int minRepThreshold=150000;    //in micrometers - 150000 micrometers = 150 mm = ~5.9 inches

//Time Initializations
  unsigned long tic_time = 0;
  unsigned long tic_timestamp = 0;
  unsigned long tic_timestampLast = 0;
  unsigned long tic_timestampLast2 = 0;
  unsigned long tic_timestamp_last = 0;
  unsigned long minDT = 1000000;
  unsigned long blink_override_threshold = 5000000;
  unsigned long total_time = 0;
  unsigned long displayTime = 0;
  unsigned long batteryTime = 0;
  unsigned long minTimer = 0;
  unsigned long minTimer2 = 0;
  unsigned long twoSecTimer2 = 0;
  unsigned long oneMinute = 60000;
  unsigned long twoSec = 2000;
  unsigned long ticDiff = 0;
  unsigned long ticDiffFiltered = 0;
  unsigned long backlightTime = 10000;
  unsigned long last_avg = 0;
  unsigned long last_press = 0;
  uint16_t restTime = 0;

//Tick Counter Initialization
  const int myDTCounter_size = 1100;                      //1200 Dts will give 1200*~2.68 mm = 3.2 m = 10.5 ft
  uint16_t myDTs[myDTCounter_size] = {0};
  uint16_t myDTCounter = 0;
//  uint16_t FILTER_out[myDTCounter_size] = {0};          
 

//Rep Array Initializations 
  const int repArrayCount=100;
  float repArray[repArrayCount] = {0.0};
  float peakVelocity[repArrayCount] = {0.0};
  uint16_t dispArray[repArrayCount] = {0};
  float timeArray[repArrayCount] = {0.0};
  byte peakVelLocation[repArrayCount] = {0};
  byte rest[repArrayCount]={0};                                      


//Button Action Setup
  bool doubleclick = false;
  uint16_t buttonstateR = 0; 
  uint16_t buttonstateL = 0; 
  int buttonstate = 0;  
  uint16_t buttonstateRtemp = 0;
  uint16_t buttonstateLtemp = 0;
  unsigned long rightHold = 0;
  unsigned long leftHold = 0;
  uint16_t rightHoldActionTime = 1500;
  uint16_t leftHoldActionTime = 1500;
  uint16_t bothHoldActionTime = 3000;
  uint16_t singleHoldActionTime = 3000;
  uint16_t replast = 0;
  boolean backlightFlag = 1;
  boolean RbuttonDepressed = 0;
  boolean LbuttonDepressed = 0;
  bool buttonRightLongPress=0;
  bool buttonLeftLongPress=0;
  bool bothbuttonlong=0;
  const int buttonholdtimer=10;           //delay time
  int counter_buttonRighthold=0;
  int counter_buttonLefthold=0;
  const int threshold_buttonhold=100;     //cycles of buttonholdtimer to cross threshold
  bool accomplishedDoubleHold = false;
bool accomplishedSingleHold = false;

//Accurate Velocity Variables
  float currentInstVel = 0.0;
  float lastInstVel = 0.0;
  float peakVelTemp = 0.0;

//Interrupt Timer
  static unsigned long last_interrupt_time = 0;
  static unsigned long last_interrupt_time2 = 0;
  static unsigned long last_tic_time = 0;

//Start Message
  float startMessage[1] = {-3456.0};

//Moving Average Variables	
	int Filtration_Output = 1;	
	const int moving_average_size = 16;
	int moving_average_offset = 3;
	unsigned long moving_average_holder = 0;
  float peakAccel = 0;
  float peakAccelHolder = 0;
	unsigned long moving_average_vector[moving_average_size] = {15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000,15000};
	float one_over_moving_average_size = 1/(float)moving_average_size;		
	float average_tick_length = 2755.95; //((3.1419+2.37)/2)*1000 for micrometers
	int ticDiffprecision = 10;

//Filter Variables
  float testFrequency = 10;
  FilterOnePole filterOneLowpass( LOWPASS, testFrequency );
  uint16_t precisionCounter = precisionCounter_start;  
  volatile unsigned int counter_simplelengthbytic=0;		//This is a simple counter that is called to count how many times we enter the interrupt
  
  unsigned long micros_holder=0;	//This is a temporary holder used so we don't have to keep calling micros()
  unsigned long  max_tick_time_allowable = max_tick_time_allowable_init;		// max_tick_time_allowable is a variable that is used to determine if the the rep "started" but really it's just pausing - see above for math used to derive number6
  unsigned long time_waiting = 0; // Once we determine that the user is pausing during a rep we start to increment a waiting timer to subtract from the overall time
  
 /*       - "Min" detectable speed = .01 m/s = 10 mm/s
          - "Min" tick length = ~2.6 mm/    
          - "Min" ticks/second = (min detectable speed)/(min tick length) = (10 mm/s)/(2.6 mm/tick) = ~3.8462 ticks/second
          - "Max" time between ticks = (1 tick)/(min ticks/second)=(1)/(~3.8462 ticks/second) = .26 sec = 260000 microseconds = max_tick_time_allowable        */
      

//Fuel Gauge Setup
  MAX1704 fuelGauge;

//-------------------------------------------------------------------------
//////////                     2. Setup                      ////////////
//-------------------------------------------------------------------------
//Initializations of all defined libraries that require function logic //
//-------------------------------------------------------------------------

void setup() {  

//LED Color Configuration
  if (COLOR == RED){ r = 5; g = 0; b = 0;userColor = 200;}
  else if (COLOR == GREEN){ r = 0; g = 5; b = 0;userColor = 50;}
  else if (COLOR == BLK){ r = 5; g = 0; b = 5;userColor = 205;}
  else { r = 5; g = 5; b = 5;userColor = 255;}
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, 1);

//RFDuino Settings
  RFduinoBLE.txPowerLevel = +4;
  RFduinoBLE.deviceName = device_name;
  RFduinoBLE.advertisementData = "OBBT";
  initializeBluetooth();  
 
//I2C + OLED Begin()
  Wire.begin();
  delay(200); //display needs about 100ms to initialize the IC
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x64)  //Nate addition
//Pin Configuration
  pinMode(pin_buttonRight, INPUT_PULLDOWN); 
  pinMode(pin_buttonLeft, INPUT_PULLDOWN); 
  //pinMode(DATA_PIN, INPUT_PULLDOWN); 
  pinMode(pin_encoder_tach, INPUT); 
  pinMode(pin_encoder_dir, INPUT); 

//Interrupt Callback Definition
  RFduino_pinWakeCallback(pin_encoder_tach, HIGH, encoderState);

//Fuel Gauge Configuration
  fuelGauge.reset();
  fuelGauge.quickStart();
  fuelGauge.showConfig();  
  
//Welcome Screen - Logo + Device Information
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(72,9);
  display.println("OBV3");
  display.setTextSize(1);
  display.setCursor(70,36);
  display.print("Unit ");
  display.print(unit_number);
  
  display.setCursor(70,46);
  display.print("Rev ");
  display.print(CODE_VERSION);
  display.display();

  leds[0].setRGB( 0, 0, 0);
  FastLED.show();

//LED Boot Animation
  #ifndef DEBUG
  int R,G,B;
    for(int foo = 0;foo<1800;foo++)
    {
      R=5+.95*cubicwave8(foo);
      G=5+.95*cubicwave8(foo/2);
      B=5+.95*cubicwave8(foo/3);
      leds[0].setRGB( R/8, G/8, B/8);
      FastLED.show();
      delay(1);
    }
   #endif
    leds[0].setRGB( 0, 0, 0);
    FastLED.show();
//Initialize Rest Time
   rest[1] = 0;
   
//Initial Charge Check                                                      //Low brightness addition       *****
  charge = fuelGauge.stateOfCharge();
	if(charge>100){
		charge=100;
	} else if(charge<=20){
    LVL=LOW_POWER;
	}
	else if (charge<=0){	
		charge=1;
	}
 
}


//-------------------------------------------------------------------------
//////////                   3. Main Loop                    ////////////
//-------------------------------------------------------------------------
//    Main logic loop for OpenBarbell V3 - Use CTRL + F to find fcns   //
//-------------------------------------------------------------------------

void loop() {
  directionCalc();                              // 4. Direction Flag   
  calcRep(goingUpward, state);                  // 13. Rep Calculation Algorithm    
  buttonStateCalc();                            // 14. Button Press State Configuration )  
  minuteTimer();                                // 7. Minute Timer  
  displayOffTimer();                            // 9. Display Timeout         
  LEDBlink();                                   // 8. LED Updater
}

//-------------------------------------------------------------------------
//////////                4. Direction Flag                  ////////////
//-------------------------------------------------------------------------
//       Determines the current direction the spindle is spinning      //
//-------------------------------------------------------------------------

void directionCalc() {

	flippedDirection = digitalRead(pin_encoder_dir);                     //define flipped direction (direction when you use your device upside-down)
	normalDirection = (!initialized)?(0):(!flippedDirection);
                                                                       //normal direction is usually directly read from the encoder, but the encoder initializes giving us the 'up'
                                                                       //direction (pin value zero). We need the intial reading to tell us 'down', because that's what it always is
                                                                       //when the string is retracted into the device. So if we haven't read any tics yet, set normalDirection to zero
	goingUpward = (isFlipped)?(flippedDirection):(normalDirection);
 
}

//-------------------------------------------------------------------------
//////////                5. Invert Mode                  ////////////
//-------------------------------------------------------------------------
//               Inverts the screen for top-down use               //
//-------------------------------------------------------------------------

void invertMode(){
  
		isFlipped = !isFlipped;                       
		accomplishedDoubleHold = true;        
		display.ssd1306_command(SSD1306_DISPLAYON); 
		display.clearDisplay();
		display.invertDisplay(isFlipped);
        display.setTextSize(3);
        display.setCursor(0,0);
		if(isFlipped){
			display.print("INVERT");
			display.setCursor(0,32);
			display.setTextSize(2);
			display.print("MODE ON");
		}else {
			display.print("INVERT");
			display.setCursor(0,32);
			display.setTextSize(2);
			display.print("MODE OFF");
		}
        display.display();
		RFduino_ULPDelay(SECONDS(2));
		if(repDone == 0){
			repDisplayLast = 0;
		}
		repDisplay--;	//This is put in here so first rep isn't missed
		//repDisplay = repDone+1;
}

//-------------------------------------------------------------------------
//////////            6. Olympic Mode (Power)            ////////////
//-------------------------------------------------------------------------
//  Switches device operation from velocity to power tracking mode //
//-------------------------------------------------------------------------

void olyPowerMode(){
  
		flipPowerOlyScreen = !flipPowerOlyScreen;		
		display.ssd1306_command(SSD1306_DISPLAYON); 
		display.clearDisplay();
		display.setTextSize(3);
		display.setCursor(0,9);
		if(flipPowerOlyScreen){
			display.print("OLYMPIC");
			display.setCursor(0,32);
			display.print("MODE");
		}else{
			display.print("POWER");
			display.setCursor(0,32);
			display.print("MODE");
		}
		display.display();
		RFduino_ULPDelay(SECONDS(2));
		if(repDone == 0){
			repDisplayLast = 0;
		}
		repDisplay = repDone;
}

//-------------------------------------------------------------------------
//////////                  7. Minute Timer                    ////////////
//-------------------------------------------------------------------------
//        Tracks battery charge and rest time and updates display        //
//-------------------------------------------------------------------------

void minuteTimer(){                             //Update minute timer so that display updates when rest at a minunte    *****
  
  if(((millis()-minTimer)%oneMinute) < 20){     //Every minute (since last rep) with .02s accuracy 
  
  	if((millis()-minTimer2)>30){                //If this function has been called within this minute, don't call it again  
      	
  	  minTimer2 = millis();
  	  restTime++;                               //Rest time accumulates 
  	  rest[repDone%repArrayCount] = restTime;     //Rest time recorded for current rep
  	  charge = fuelGauge.stateOfCharge();       //Check battery status
    	  if(charge>100){
    		charge=100;
    	  } 
    	  else if (charge<=0){
    		charge=1;
    		}
       if(charge<=20){                          //Reduce LED brightness level when battery is under 20 percent
        LVL=LOW_POWER;
       }
       else{
        LVL=BRIGHTNESS;
       }  
     
        if (!goingUpward||(time_waiting > blink_override_threshold)) {
            systemTrayDisplay();
            display.display();
        }
	  }
  }
}

//-------------------------------------------------------------------------
//////////                8. LED Updater                       ////////////
//-------------------------------------------------------------------------
// Function to blink LED every XX seconds, if there's nothing going on   //
//-------------------------------------------------------------------------

void LEDBlink(){    
  
  if(((millis()%twoSec) < 20)&&((!goingUpward)||(time_waiting > blink_override_threshold))){    //If it's been 2n seconds and the encoder's not in use 
	if((millis()-twoSecTimer2)>30){                               //Makes sure the loop trips only once
	  twoSecTimer2 = millis();
	  leds[0].setRGB( LVL*r, LVL*g, LVL*b);
    FastLED.show();
    RFduino_ULPDelay(20);
    leds[0].setRGB( 0, 0, 0);
    FastLED.show();
	}
  }  
}

//-------------------------------------------------------------------------
//////////               9. Display Timeout                  ////////////
//-------------------------------------------------------------------------
//   Periodically turns off LCD backlight to preserve battery life     //
//-------------------------------------------------------------------------

void displayOffTimer(){
  
  // if the displayed rep hasn't changed in a while, we don't need the backlight
  if ((millis() - displayTime) > backlightTime){
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    backlightFlag = 0;
  }
}

//-------------------------------------------------------------------------
//////////              10. Bluetooth Setup                  ////////////
//-------------------------------------------------------------------------
//                Initializes Bluetooth Advertising                    //
//-------------------------------------------------------------------------

void initializeBluetooth(){
  
  bluetoothOn = true;
  RFduinoBLE.begin();
  RFduinoBLE.advertisementInterval = 100; 
  RFduino_ULPDelay(200);
	
//Starting Bluetooth resets timers:
	displayTime = millis();
	minTimer = millis();
	minTimer2 = millis();
	twoSecTimer2 = millis();
}

//-------------------------------------------------------------------------
//////////            11. Bluetooth Management               ////////////
//-------------------------------------------------------------------------
//     Changes LED indication back to default color on disconnect      //
//-------------------------------------------------------------------------

void RFduinoBLE_onDisconnect(){
  BTisconnected = 0;
  if(userColor>0){
    r = userColor/50;                      //r = 255/50 = 5 
    g = (userColor%100)/10;                //g = 255%100/10 = 55/10 = 5
    b = (userColor%10);                    //b = 255%10 = 5
  }
  else{
    if (COLOR == RED){ r = 5; g = 0; b = 0;}
    else if (COLOR == GREEN){ r = 0; g = 5; b = 0;}
    else if (COLOR == BLK){ r = 5; g = 0; b = 5;}
    else { r = 5; g = 5; b = 5;}
  }
}

//-------------------------------------------------------------------------
//     Throws up a splash screen on BT connect and turns LED Blue      //                   Freezes at power on if connected too early
//-------------------------------------------------------------------------

void RFduinoBLE_onConnect(){
  BTisconnected = 1;
  r = 0; g = 0; b = 5;
	BTRefresh = true;
}

//-------------------------------------------------------------------------
//             Changes variables when requested by phone                 //
//-------------------------------------------------------------------------

void RFduinoBLE_onReceive(char *data, int len)
{

	int data_holder = 0;
  
// Check the first byte of the incoming data XX
  if (data[0]){
  	if(bitRead(data[0],0)){                   //01 Change Increment: XX is hex number of displacement from 00 (0 mm) to FF (255000 micrometers = 255 mm = 10.039)  -- Increments of 10.039/255=.039 inches
  		minRepThreshold = (int)(data[1])*1000;
  		}
  	if(bitRead(data[0],1)){                   //02 Change Usage Mode
  		data_holder = (int)(data[1]);  		
  		if(data_holder<10){
  			olyPowerMode();
  		}  		
  		if(data_holder>10){
  			invertMode();
  		}		
	}
	
	if(bitRead(data[0],2)){                     //04 Change Test Frequency
		testFrequency=(int)(data[1]);
		}
   
	if(bitRead(data[0],3)){                     //08 Change Tic Tolerance: XX from device where XX is the precision
		if((int)(data[1])<254){
			precisionCounter = max(1,(int)(data[1]));
		} else if ((int)(data[1])==254){
			ticDiffprecision=10;
		} else if ((int)(data[1])==255){
			ticDiffprecision=1;
		}		
	}
 
	if(bitRead(data[0],4)){                     //10 Change Display Timeout
		backlightTime = data[1]*1000;
		}
		
	if(bitRead(data[0],5)){                     //20 Navigate Displayed Rep Information
		data_holder = (int)(data[1]);
		if(data_holder<10){
			if ((backlightFlag)&&(repDisplay < (repDone + 2))){
				repDisplay += 1;
			  }
		  } else if (data_holder>10 && data_holder< 20){
			if ((backlightFlag)&&(repDisplay > 1)&&(repDisplay < repDone + 2)){
				repDisplay -= 1;
			  }
		  }
		
		if (data_holder==254){                    //Output Raw Data to Display and Remove Filter
      
			Filtration_Output = 0;			
			display.clearDisplay();
			display.setTextSize(2);
			display.setCursor(13,0);
			display.print("RAW VALUE");			
			display.setTextSize(2);	
			display.setCursor(28,22);
			display.print("OUTPUT");		
			display.setCursor(24,42);
			display.print("ENABLED");
			display.display();			
		}		

		if (data_holder==255){                    //Disable Data Compression 
			full_data_logging_enabled = 1;		
			precisionCounter = 1;
			dataCOMPRESSION_enabled =0;
			ticDiffprecision=1;			
			display.clearDisplay();
			display.setTextSize(2);
			display.setCursor(13,0);
			display.print("FULL DATA");			
			display.setTextSize(2);	
			display.setCursor(24,22);
			display.print("LOGGING");		
			display.setCursor(24,42);
			display.print("ENABLED");
			display.display();			
		}
	}
	
	  if(bitRead(data[0],6)){                      //Send Configuration Status to Device
	
  		send_single_float(precisionCounter);
  		send_single_float(ticDiffprecision);
  		send_single_float(max_tick_time_allowable);
  		send_single_float(backlightTime);
  		send_single_float(minRepThreshold);
  		send_single_float(ticLength);		
  		charge = fuelGauge.stateOfCharge();		
  		send_single_float(charge);		
  		send_single_float(unit_number);	
	  }	
  
     if(bitRead(data[0],7)){                     //Get New LED Color   
        data_holder = (int)(data[1]);            //I.E. 255:
        userColor = data_holder;                 //Store color
        r = data_holder/50;                      //r = 255/50 = 5 
        g = (data_holder%100)/10;                //g = 255%100/10 = 55/10 = 5
        b = (data_holder%10);                    //b = 255%10 = 5
        
     } 
     
     if(bitRead(data[0],8)){                     //Send Current LED Color 
        data_holder = r*50 + g*10 + b;
        send_single_float(data_holder);       
     }
  }
}

//-------------------------------------------------------------------------
//              Sends messages over Bluetooth               //
//-------------------------------------------------------------------------

void send_intList_charString(int *intList, int len) {
  
  String intString = "";
  for (int i=0; i < len; i++) {
    if (i > 0) intString += ",";
    intString += String(intList[i]);
    }
    int nbytes = intString.length()+1;
    char bytes[nbytes];
    intString.toCharArray(bytes, nbytes);
    while (!RFduinoBLE.send(bytes, nbytes));
  } 

void send_intList(int *intList, int len) {
  for (int i=0; i < len; i++) {
    while (!RFduinoBLE.sendInt(intList[i]));
  }
} 

void send_floatList(float *floatList, int len) {
  for (int i=0; i < len; i++) {
    while (!RFduinoBLE.sendFloat(floatList[i]));
  }
} 

//-------------------------------------------------------------------------
//   Function to compress and send bulk velocity data over Bluetooth     //
//-------------------------------------------------------------------------

// In order to save on transmission time the uint16s will be combined together (when possible).
// The uint16_t are being combined like so: 
//        i.e: array foo where foo[0] = 16 and foo[1] = 23
//        the numbers would be sent over as foo[0].(foo[1]/10000) or 16.0023.
//        HOWEVER - The floats that are being sent can display up to 8 characters eg. 1234.5678 or 123456.78
//It is therefore necessary to make sure that the values being sent over are not over 10,000 or
//they will be truncated - Thus we check to make sure that the values aren't over 10,000 before combining
//since we are combining every two values, if the list has an odd number of values we need to send the last one

void send_float_from_intList(uint16_t *intList, uint16_t len) { 
			
			display.setTextSize(1);
			display.setCursor(95,22);
			display.print(")");
			
			display.setTextSize(2);
			display.setCursor(102,18);
			display.print(")");
			
			display.setTextSize(3);
			display.setCursor(109,16);
			display.print(")");
			
			display.display();
       
 if (dataCOMPRESSION_enabled){
 
		for(int i=precisionCounter; i<((len-moving_average_size)-precisionCounter); i=i+(2*precisionCounter)){
			if((intList[i]-10000)<0 && (intList[i+precisionCounter]-10000)<0){	//Check to see if there will be a value that may be truncated
				while (!RFduinoBLE.sendFloat((float)intList[i]+(float)intList[i+precisionCounter]/10000));
			} else {	//If there is a potential for truncation then each number will be sent separate
				while (!RFduinoBLE.sendFloat((float)intList[i]));
				while (!RFduinoBLE.sendFloat((float)intList[i+precisionCounter]));
			}
		}
	
		if(len%precisionCounter==1){	//Check if there was an odd number of values
			while(!RFduinoBLE.sendFloat((float)intList[(len-moving_average_size)]));
		}
  
	} else if (dataCOMPRESSION_enabled==0){
      for(int i=precisionCounter; i<((len-moving_average_size)-precisionCounter); i=i+(precisionCounter)){
			  while (!RFduinoBLE.sendFloat((float)intList[i]));                                                           
		  }
    }
  
  display.setTextSize(2);
			display.setTextSize(1);
			display.setCursor(95,22);
			display.print(" ");
			
			display.setTextSize(2);
			display.setCursor(102,18);
			display.print(" ");
			
			display.setTextSize(3);
			display.setCursor(109,16);
			display.print(" ");
			display.display();  
} 

//-------------------------------------------------------------------------
//                  Bluetooth Data Transmit Helpers                    //
//-------------------------------------------------------------------------

void send_single_float(float singleFloat) {
    while (!RFduinoBLE.sendFloat(singleFloat));
    RFduino_ULPDelay(1);
} 

void send_all_data() {
	if (sendData) {       
	  repPerformance[0] = (float) rep;                                  //Rep #
	  repPerformance[1] = (float) repArray[rep%repArrayCount];          //Avg Velocity (m/s)      
	  repPerformance[2] = (float) dispArray[rep%repArrayCount];         //Range of Motion (mm)
	  repPerformance[3] = (float) peakVelocity[rep%repArrayCount];      //Peak Velocity (m/s)
	  repPerformance[4] = (float) peakVelLocation[rep%repArrayCount];   //Peak Velocity Location (%)
	  send_floatList(repPerformance, 5);
    send_single_float(peakAccel);                                     //Peak Acceleration (m/s^2)
	  
	  //Start OBV3 Extra Data
    send_single_float(timeArray[rep%repArrayCount]*1000000);                         //Duration of Rep (microseconds)
    send_single_float(restTime);                                      //Time between Reps (minutes)
    send_single_float(tic_timestamp);                                 //Timesatamp of Rep Completion (microseconds)
    send_single_float(time_waiting);                                  //Timestamp of time "waiting" in rep (microseconds)
    send_single_float(max_tick_time_allowable);                       //Slowest Instantaneous Velocity Allowable (microseconds)
    send_single_float(backlightTime);                                 //Amount of Time the Backlight is allowed to stay on (microseconds)
    send_single_float(minRepThreshold);                               //Minimum allowable Rep Length (micrometers)
    send_single_float(dataCOMPRESSION_enabled);                       //Is data compression enabled for Bulk Data (bool)
    send_single_float(Filtration_Output);                             //Is filtration enabled for bulk data (bool)  

    //Device Info
    send_single_float(CODE_VERSION);                                  //Code Version 
    send_single_float(unit_number);                                   //Unit Number
    send_single_float(userColor);                                     //LED Color (RGB)
    send_single_float(BRIGHTNESS);                                    //Brightness (%)
    send_single_float(LOW_POWER);                                     //Low Power Brightness (%)

    //Start Bulk Data Settings Transmission)
    send_single_float(-9999.0);                                       //Flag Bulk Data Settings
	  send_single_float(precisionCounter);                              //# of Ticks Counted
	  send_single_float(ticDiffprecision);                              //Precision of Compressed Values
	  send_single_float(-9876.0);                                       //Start Bulk Data
	  send_float_from_intList(myDTs, myDTCounter);                      //**BULK DATA** (UNKNOWN LENGTH)
	  send_single_float(-6789.0);                                       //End Bulk Data
	  send_single_float((float)charge);                                 //Battery Charge
	  sendData = false;
	  }
} 

//-------------------------------------------------------------------------
//////////                 12. System Tray                   ////////////
//-------------------------------------------------------------------------
//   Adds rep number, battery and rest timer to the screen buffer      //
//   Will be displayed to the screen outside of this function          //
//-------------------------------------------------------------------------

void systemTrayDisplay(){
  
	display.setTextColor(WHITE,BLACK);
	display.setTextSize(1);
	display.setCursor(0,0);
	display.print("Rep#:");
	if((repDisplay<repDone)||(repDisplay==repDone)){                         //this statement keeps the 'begin set' screen from showing repDisplay, which could be more than 1
    display.print(repDisplay);
  }else display.print("1");
	display.print("  ");
	display.setCursor(55,0);
  if((repDisplay<repDone)||(repDisplay==repDone)){                         //this statement allows the 'begin set' screen to show rest time
    display.print(rest[repDisplay%repArrayCount]);
  }else display.print(restTime);
	display.print(" min");
	display.setCursor(104,0);
	display.print(charge);
	display.print("%");
}

//-------------------------------------------------------------------------
//////////           13. Rep Calculation Algorithm           ////////////
//-------------------------------------------------------------------------
//   Fields incoming tics, records data and updates values accordingly //
//-------------------------------------------------------------------------

void calcRep(bool isGoingUpward, int currentState){

//-------------------------------------------------------------------------
//                            Initial Rep                                //
//-------------------------------------------------------------------------   
 
  if (currentState != currentStateTemp) {                             //Check for change in direction (state changed in <Encoder State Interrupt, Section 15>)
    
	  initialized = 1;                                                  //See <Direction Flag, Section 4>
    tic_time = micros();                                              //Take start time for this rep
    


//-------------------------------------------------------------------------
//                    Going Up (Recording rep data)                      //
//------------------------------------------------------------------------- 
   
    if (isGoingUpward){                                               //Check direction 
         
	    //displacement = counter_simplelengthbytic*ticLength;	          //increment or decrement the distance by one tic length, depending on direction --> 
		  micros_holder = micros();	  	 
    
  		if((micros_holder-tic_timestamp)>max_tick_time_allowable){
  			time_waiting = time_waiting + micros_holder-tic_timestamp;
  		}
        // There was a bug found where it was possible to start going up but then hold a 
        // position without going down...this caused the total_time to
        // continually increase and throw off the average velocity for the rep - to compensate 
        // for this we see how much time someone is waiting and
        // subtract that from the total_time  		 
        	  
  		tic_timestampLast2 = tic_timestampLast;
  		tic_timestampLast = tic_timestamp;
  		tic_timestamp = micros_holder;   
     
      
//-------------------------------------------------------------------------
//                          Starting New Rep                             //
//-------------------------------------------------------------------------

  		if (!isGoingUpwardLast){                                        // If you were just going downward clear your array so you can start a fresh rep
      
  			memset(myDTs,0,sizeof(myDTs));                                //Zero out tic array
  			moving_average_holder = 0;                                    //Reset current average velocity
  			//memset(FILTER_out,0,sizeof(FILTER_out));
  			memset(moving_average_vector,0,sizeof(moving_average_vector));  //Reset velocity array          
  			//memset(instVelTimestamps,0,sizeof(instVelTimestamps));
  			time_waiting=0;                                               //Reset wait time
  			counter_simplelengthbytic=0;                                  //Reset interrupt tic counter
  			starttime = tic_timestamp;                                    //New rep started
  			send_floatList(startMessage, 1);                              //Tell app new rep is starting via BT
  			rep += 1;                                                     //Increase rep count        
        currentInstVel = 0;                                           
  		  lastInstVel = 0;
  			peak_vel_at = 0;
  			minDT = 1000000;
  			myDTCounter = 0;                                              //Zero out position data
        peakAccel = 0;
  		}
  		  
		//keeping instantaneous velocities for our peak velocity reading
		//instVelTimestamps[counter_lengthbyticinfunction] = (unsigned int)(tic_timestamp-tic_timestamp_last);

    
		ticDiff = tic_timestamp - tic_timestamp_last;                     //DT for this rep
		tic_timestamp_last = tic_timestamp;                               //Base time for next rep
    
  		
//-------------------------------------------------------------------------
//                            Moving Average                             //
//-------------------------------------------------------------------------			                                                                                         	
    
			for(int shift_i=0; shift_i < (moving_average_size-1); shift_i ++){                      //Going up, shift the values back (losing the first value) so that we can put our new value on the end  
					moving_average_vector[shift_i]=moving_average_vector[shift_i+1];                    //Improve values by loading (til end) with last recorded value
				}
		
			moving_average_vector[moving_average_size-1] = ticDiff*one_over_moving_average_size;    //push new tic (time between encoder high) onto end of array
			
			if(myDTCounter>=(moving_average_size)){			                                            //If moving average array is full (more than 16 tics) -->
      
				moving_average_holder = 0;	                                                          // Zero out moving average  

				for(int i=0; i <= (moving_average_size - 1); i++){					
					moving_average_holder=moving_average_holder+moving_average_vector[i];               //Add up vector to get the average velocity
				}

        if(last_avg>moving_average_holder){
          peakAccelHolder = (((float)ticLength/(float)moving_average_holder)-((float)ticLength/(float)last_avg))/((float)moving_average_holder/1000000); //Calculate peak acceleration --> dV/dT
           
        }
        else{
          //peakAccelHolder = 0;
        }
        
        last_avg = moving_average_holder;
      
       if(peakAccelHolder>peakAccel){
          peakAccel = peakAccelHolder;
       }

       
			 ticDiffFiltered = moving_average_holder;                                              
				//end moving average
  				
  			if (ticDiffFiltered < minDT){                                                         //if the last tic was faster than the current lowest (minDT) 
  				minDT=ticDiffFiltered;                                                              //new minimum velocity is the current
  				peak_vel_at=myDTCounter;                                                            //save reference to this tic (required)         
  			}	
  		}                                                                                         
      
  		else {                                                                                  
  		
  		/*
  		moving_average_holder = moving_average_holder + ticDiff;
  		
  		if(myDTCounter>1){
  			moving_average_holder=(moving_average_holder/2);
  		}
  		
  		ticDiffFiltered = moving_average_holder;
  		*/	  			
  		ticDiffFiltered = 0;
		} 		 
  			
  		if(myDTCounter >= moving_average_size-1){
  		//precisionCounter = myDTCounter/(highPrecisionMode+1);
  			if(myDTCounter<myDTCounter_size){  				
  					myDTs[myDTCounter-moving_average_size] = (uint16_t)(ticDiffFiltered/ticDiffprecision);  //start filling tic array with average velocities
  				if (Filtration_Output==0){
  					myDTs[myDTCounter] = (uint16_t)(ticDiff/ticDiffprecision);                              //start filling tic array with unfiltered velocities
  				}
  			}
  		}    	  
  		myDTCounter++;                                                                                //End of GoingUp, increase encoder tic total by 1 	  
        
    } else {
      
      // If you're going downward, and you were just going upward, you potentially just finished a rep. 
      // Do your math, check if it fits the rep criteria, and store it in an array.

//-------------------------------------------------------------------------
//                            Rep Calculations                           //
//-------------------------------------------------------------------------    

	  if (isGoingUpwardLast){                                                                             //Modified for overflow reps    
    
	  displacement = counter_simplelengthbytic*ticLength;                                                 //Total distance moved
        if (displacement > minRepThreshold){                                                            //If distance traveled is enough to qualify as one rep
    		  total_time = (tic_timestamp - starttime) - time_waiting;                                      //Calculate time since start
          
    		        peakVelocity[rep%repArrayCount] = float(ticLength)/float(minDT);		 
  		          dispArray[rep%repArrayCount] = displacement/1000;                                                                 //Update displacement array 
                timeArray[rep%repArrayCount] = (float)total_time/1000000;                                                         //Update time array (time per rep)
                peakVelLocation[rep%repArrayCount] = (peak_vel_at*100)/myDTCounter;                                               //Update log of peak velocity locations 
    		        repArray[rep%repArrayCount] = ((float)(counter_simplelengthbytic*ticLength)/(float)(total_time/1000))/1000;       //Get total rep time and store in array
                rest[rep%repArrayCount] = 0;                                                                                      //Reset rest time just in case it's reused
                if(rep>1)    
                    rest[rep%repArrayCount-1] = restTime;                                                                         //Set the rest time of the previous rep
                
                 
                 
    		  repDone = rep;		                                                                            //Sets global rep counter to loop rep count
          minTimer = millis();                                                                          //resets 60 second rest time counter
          minTimer2 = millis();
    		  restTime = 0;
    		  counter_simplelengthbytic=0;
        } 
        else {                                                                                      //Get rid of this rep (it doesn't count)
          rep -= 1;
        }
      }
	  
      displacement -= ticLength;                                                                    //Subtract this amount from total distance traveled (it doesn't qualify as rep distance)
    }
      
    isGoingUpwardLast = isGoingUpward;                                                              //Sets globally that the last state was going upward
    currentStateTemp = currentState;                                                                //Is this necessary?          *****
  }
}

//-------------------------------------------------------------------------
//////////        14. Button Press State Configuration       ////////////
//-------------------------------------------------------------------------
//   Handles Button presses and incorporates most helper functions     //
//-------------------------------------------------------------------------


void buttonStateCalc(){
  
                                                                                    //Read button state once per loop
  buttonstateL = digitalRead(pin_buttonLeft);
  buttonstateR = digitalRead(pin_buttonRight);
  
                                                                                     //Update rep to be displayed to the screen if you recorded a new rep
  if (repDone != repDoneLast){
    repDisplay = repDone;
     
	                                                                                   //RepDisplayLast and repDoneLast are reset below
  	if(!backlightFlag){                                                              //Turn on display if it's off
      display.ssd1306_command(SSD1306_DISPLAYON);
    	systemTrayDisplay();
    	display.display();
  	}
	sendData = true;
  }
  
//-------------------------------------------------------------------------
//                            Right Button Press                         //
//-------------------------------------------------------------------------                                                                                       

  if (buttonstateRtemp && !buttonstateR){                                           //Register a button press on the release of the right button
    
    if ((backlightFlag)&&(repDisplay < (repDone + 2))){                             //If the screen is on and the current displayed rep is not the last
      repDisplay += 1;
    }
    
    else {
      display.ssd1306_command(SSD1306_DISPLAYON); 
  	  backlightFlag = 1;
  	  //systemTrayDisplay();
  	  display.display();
	  }
   
  rightHold = 0;
  RbuttonDepressed = 0;
	                                                                                   //This flag forces the double hold to execute its code for only one loop
	accomplishedDoubleHold = false;
	accomplishedSingleHold = false;
  displayTime = millis();                                                            //Starts display time-out timer
    //bluetoothStartNextLoop = true;    
  } 
//-------------------------------------------------------------------------
//                            Left Button Press                          //
//-------------------------------------------------------------------------  
                                                                                     
  if (buttonstateLtemp && !buttonstateL){                                            //Register a button press on the release of the left button 
        
    if ((backlightFlag)&&(repDisplay > 1)&&(repDisplay < repDone + 2)){              //If the screen is on and the current displayed rep is not the first or last
      if(repDone<=100||(repDisplay-1>repDone%repArrayCount)){     
        repDisplay -= 1;
      }
    } else {
      display.ssd1306_command(SSD1306_DISPLAYON); 	  
      backlightFlag = 1;
	  //systemTrayDisplay();
	  display.display();
	  }
    leftHold = 0;
    LbuttonDepressed = 0;	
  	accomplishedDoubleHold = false;                                                  //This flag forces the double hold to execute its code for only one loop
  	accomplishedSingleHold = false;
    displayTime = millis();                                                          //Starts display time-out timer
    
  }
  
//-------------------------------------------------------------------------
//                              Press and Hold                           //
//------------------------------------------------------------------------- 
 
  if (!buttonstateRtemp && buttonstateR){                                            //Set a flag if you just pressed the right button and start hold timer
    rightHold = millis();
    RbuttonDepressed = 1;
  }

  if (!buttonstateLtemp && buttonstateL){                                            //Set a flag if you just pressed the left button and start hold timer
    leftHold = millis();
    LbuttonDepressed = 1;
  }
  
  if (LbuttonDepressed && RbuttonDepressed){                                         //if both buttons are depressed start invert mode 
    //doubleclick=!doubleclick;                                                      //function to enable peak acceleration in olympic mode
    if (((millis() - rightHold) > bothHoldActionTime)&&((millis() - leftHold) > bothHoldActionTime)){
      if (!accomplishedDoubleHold){
        invertMode();
      }
    }
  } else if (LbuttonDepressed || RbuttonDepressed){                                  //fixes a bug where in certain situations the left button will keep the switching screens from clearing
		if (((millis() - rightHold) > singleHoldActionTime)&&((millis() - leftHold) > singleHoldActionTime)){
			if(!accomplishedSingleHold){
			accomplishedSingleHold = true;                                                 
  			
  			olyPowerMode();                                                              //Start olympic mode
  			if(LbuttonDepressed){
  				repDisplay++;                                                              //Redundant code to increment reps if missed above?          *****
  			}
  			if(RbuttonDepressed){
  				repDisplay--;
  			}
			}
		}
  }
//-------------------------------------------------------------------------
//                              Display Change                           //
//------------------------------------------------------------------------- 

  if ((repDisplay != repDisplayLast)||(repDone != repDoneLast)){                     // if the displayed rep changes, keep the time so we know when to dim the backlight
    
    displayTime = millis();
    // make sure we can see the new rep
    display.ssd1306_command(SSD1306_DISPLAYON);
    backlightFlag = 1;

//-------------------------------------------------------------------------
//                              Past Set Move                            //
//------------------------------------------------------------------------- 

    if (repDisplay == (repDone + 1)){                                                //This 'if' statement keeps you from going from "Begin Set" back to "Delete Past Set?"
		
  		if((repDisplayLast < (repDone + 1))||BTRefresh){
  		  BTRefresh = false;
  		  display.clearDisplay(); 
  		  systemTrayDisplay();        
  		  display.setTextSize(1);
  	    display.setCursor(0,0);
  		  display.print("Rep#:");
  		  display.print(repDisplay - 1);
  		  display.print("  ");
  		  display.setTextSize(2);
  		  display.setTextColor(WHITE);
  		  display.setCursor(0,9);
  		  display.println("Delete");
  		  display.println("Past Set?");
  		  display.setTextSize(1);
  		  display.setCursor(0,40);
  		  display.println("R Button-Delete Set");
  		  display.println("L Button-Go Back");
  		  display.display();        
  		}
    } 
    else if (repDisplay > (repDone + 1)){                                            //This line keeps the repDisplay value from getting too big, and causing a bug to miss the first rep (edit: might not be necessary)  *****
      counter_simplelengthbytic=0; //JDLTEST	  
	    //repDisplay = repDone + 2;  
      rep = (goingUpward)?(1):(0);
      repDone = 0;
      repDoneLast = 0;
	  
	    display.clearDisplay(); 
      display.setTextSize(2);
      display.setTextColor(WHITE,BLACK);
      display.setCursor(0,15);
      display.print("Begin Set!");
	    systemTrayDisplay();
	    display.setTextSize(1);
	    display.setCursor(0,0);
	    display.print("Rep#:1  ");
      display.display(); 
	    RFduino_ULPDelay(1);
	  
      memset(repArray,0,sizeof(repArray));
      memset(myDTs,0,sizeof(myDTs));
      memset(dispArray,0,sizeof(dispArray));
      memset(timeArray,0,sizeof(timeArray));
      memset(peakVelocity,0,sizeof(peakVelocity));
	    //FOR TESTING
	    //memset(FILTER_out,0,sizeof(FILTER_out));
	    memset(moving_average_vector,0,sizeof(moving_average_vector));
		  moving_average_holder = 0;
      //memset(instVelTimestamps,0,sizeof(instVelTimestamps));
      myDTCounter = 0;
    } 
//-------------------------------------------------------------------------
//                    Rep Information Display Logic                      //
//------------------------------------------------------------------------- 
    
    else {                                                                                  //If in standard operating mode ->
  		if(!flipPowerOlyScreen){
  		  display.clearDisplay();
  		  display.setTextSize(1);
  		  display.setTextColor(WHITE,BLACK);
  		  display.setCursor(0,9);
  		  display.print("Avg Vel:");
  		  display.setTextSize(3);
  		  display.setTextColor(WHITE,BLACK);
  		  display.setCursor(0,19);
  		  display.print(repArray[repDisplay%repArrayCount]); 
  		  display.setTextSize(1);
  		  display.print("m/s");
  		  display.setCursor(0,42);
  		  display.print("Peak Vel:");
  		  display.setCursor(0,51);
        
  		  if(peakVelocity[repDisplay%repArrayCount] > MAXVEL){                                                  
  			  display.print("MAX");
  		  }
  		  else {
  			  display.print(peakVelocity[repDisplay%repArrayCount]);
  		    display.print("m/s");
  		  }
         
  		  //display.print(myDTs[10]);
  		  //display.print(myDTCounter/2);  
  		  display.setCursor(82,42);
  		  display.print("ROM:");
  		  display.setCursor(82,51);
  		  display.print(dispArray[repDisplay%repArrayCount]);                                 
  		  display.print("mm");
  
		  	if(testbed_readouts){
  				display.setTextSize(1);
  				display.setCursor(100,12);
  				display.print(peak_vel_at);
  
  				display.setCursor(100,22);
  				display.print(myDTCounter);
			  }
	     } 
	     else {
  		  display.clearDisplay();
  		  display.setTextSize(1);
  		  display.setTextColor(WHITE,BLACK);
  		  display.setCursor(0,9);
       
        if(doubleclick)
  		  display.print("Peak Accel:");
        else
        display.print("Peak Vel:");
        
  		  display.setTextSize(3);
  		  display.setTextColor(WHITE,BLACK);
  		  display.setCursor(0,19);

        if(doubleclick){
          display.print(peakAccel);
          display.setTextSize(1);
          display.print("m/s^2");
        }
        else{   
    		  if(peakVelocity[repDisplay%repArrayCount] > MAXVEL){
    			  display.print("MAX");
    		  }
    		  else {         
    			  display.print(peakVelocity[repDisplay%repArrayCount]);
            display.setTextSize(1);
            display.print("m/s");          
    		  }
        }
    			  
  		  display.setTextSize(1);
  		  display.setCursor(0,42);
  		  display.print("Time:");
  		  display.setCursor(0,51);
  		  display.print(timeArray[repDisplay%repArrayCount]);
  		  display.print("sec");
  		  display.setTextSize(1);
  		  display.setCursor(82,42);
  		  display.print("PeakHt:");
  		  display.setCursor(82,51);
  		  display.print(peakVelLocation[repDisplay%repArrayCount]);
  		  display.print("%");
		    } 
  		systemTrayDisplay();
  		display.display();      
      if(BTisconnected){
  		  send_all_data(); //moved send_all_data here so it doesn't lag the display
      }
    }
    repDoneLast = repDone;
    repDisplayLast = repDisplay; 
  }
  
  buttonstateRtemp = buttonstateR;
  buttonstateLtemp = buttonstateL;
  
}

//-------------------------------------------------------------------------
//////////           15. Encoder State Interrupt             ////////////
//-------------------------------------------------------------------------
//      Increments pulled length when encoder activity is detected     //
//-------------------------------------------------------------------------

int encoderState(uint32_t ulPin)
{
  state = !state;
  
  if(goingUpward){  //This if statement keeps the counter from counting on downward movements
  counter_simplelengthbytic++;  //This is the counter that counts the number of encoder wheel transitions in the interrupt
  }

  return 0;
}



