//CG-scale originally designed and published by Olav Kallhovd; https://github.com/olkal/CG_scale

/*
-------------------------------------------------------
Update 31.08.2017:
 - fixed EEPROM read -problem
 - changed low voltage limit to 6 volts
 - changed smallest calibration step from 0.1 to 0.05
-------------------------------------------------------
*/

/*
Copyright Aaro Malila, Finland, 2017 (aaro.malila@gmail.com)

Only one Arduino board is needed, as LCD-display with i2c-bus is used. 
I used this: http://www.ebay.com/itm/New-Blue-IIC-I2C-TWI-1602-16x2-Serial-LCD-Module-Display-for-Arduino/221439853893?ssPageName=STRK%3AMEBIDX%3AIT&_trksid=p2060353.m2749.l2649
As the i2c-interface installed behind the LCD is quite thick, a new lcd box cover (bottom) was done by modifying Olav's design.

Board type used is Arduino Nano (328 16 MHz). A Arduino Pro Mini can also be used,
but there are no free pins for voltage measurement as HX711's need 4 and i2c-bus 2 analog pins.

Led is removed, as status (power on/off) is easily shown from backlit LCD.
Low voltage alarm is added.

Display is connected via i2c-bus, description is here: https://forum.arduino.cc/index.php?topic=128635.0
A modified lcd-library (LiquidCrystal_I2C.h) is required, the native Arduino 1.0 will not work. 
Follow the instuctions behind that link. A LM7805-regulator was used to give the display 5V, power supply
from Arduino Nano's (clone) vcc  was not enough.

Load cell amplifiers are used with this library: https://github.com/bogde/HX711

Connections / pins:   
 - rear load cell:   A0-A1
 - front load cell:  A2-A3
 - LCD i2c-bus:      A4-A5 (SDA-SCL)
 - battery voltage   A6


Calibration function
It is possible to set the calibration factors with 3 buttons. A known calibration weight is needed.

Altered values are saved to EEPROM.
 - navigation buttons / pins
    * btn 1 digital2 func
    * btn 2 digital3 -
    * btn 3 digital4 +

Hold btn 1 down a few seconds and release it to enter calibration menu. Select sensor and steps by clicking btn1, and 
make adjustments to scale factor by clicking buttons 2 and 3 until the weight is what it is supposed to be. Go through
all the steps, after the last one values are written to EEPROM.

Led or beeper is used to signal if a) battery is low or b) there is no activity after certain time, scale being in 
power save state and LCD backlight shut off.

Voltage divider with R1=10 R2=1 kohm resistors are used, and reference voltage is set to 1.1 V (ARef = internal).

Operating cycle is defined ONLY by how fast the sensors give the measurements. Quite slow
amplifiers were used. If faster ones are found, it might be necessary to add some delay()
to the end of the loop -block.

HX711 amplifiers and LCD seem to consume quite a lot. This is why update intervals are changed during the operation.

*/

//libraries
//display
#include <Wire.h> //for i2c-bus
#include <LiquidCrystal_I2C.h>

//load cell amplifiers
#include "HX711.h"


//load cell amplifiers - HOX! Prototype is just opposite... this is like in schema
HX711 front_scale(A2, A3);  
HX711 rear_scale(A0, A1); 


//eeprom
#include <EEPROM.h>

//pins
byte batRefPin = A6; //battery voltage measurement
byte button1Pin = 2;
byte button2Pin = 3;
byte button3Pin = 4;
byte beeperPin  = 5;

//variable
float battValue; //battery voltage
float eW,tW; //measurement results from front/rear load cells
float CGratio; //a ratio that is calculated from values above...
float CG; //final result

//physical dimensions (distances), make sure that these are coherent with the mechanical unit
float WingPegDist = 119.8; //calibration value in mm, projected distance between wing support points, measure with calliper
float LEstopperDist = 30.0; //calibration value in mm, projected distance from front wing support point to leading edge (stopper pin), measure with calliper

float CGoffset = ((WingPegDist / 2) + LEstopperDist);

//Lsome stuff for LCD
#define I2C_ADDR    0x3f // Display's i2c-address. Use i2c-scanner to search the correct one...
#define BACKLIGHT_PIN  3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

//display
LiquidCrystal_I2C  lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin); 
//*******************************************************************
//LOCALIZATION - modify these according to your preferences
bool weight_oz = false; //if for some strange reason ounces are preferred instead of grams

//splash text, shown after startup, eg "My own CG-scale"
String t0 = "Aaro's scale";

//battery status, eg "Battery:"
String t1 = "Battery:";

//What is hown during the initialization phase, eg. "Initializing.."
String t2 = "Initializing..";

//Label for weight, eg "Weight:"
String t3 = "Weight:";

//Label for CG result, eg "CG:"
String t4 = "CG:";

//Label for calibr. question, eg "Calibrate?";
String t5 = "Calibrate?";

//Label for calibr.question row2, eg "Btn 1 = Yes"
String t6 = "Btn 1 = Yes";
//********************************************************************
//other parameters
byte n_avg = 8; //number of measurements from which the average is calculated (by the HX711 library)
unsigned int lcd_min_update_interval_ms = 500; //to acoid flickering etc

//if sensor wires are crossed and scale shows negative values...
bool invert_front_sensor = true;
bool invert_rear_sensor  = true;

//calibration factors, modify these to adjust the output with known calibration weight...
float sens_cal_1 = 1000; //default value, this is overwritten with value stored by calibration function 
float sens_cal_2 = 1000; //default value, ...

float treshold = 2; //min weight, below this "0.0" is shown
float U_batt_low = 6; //low battery level, below this some info is shown

bool battAlarmShown = false; //flag to control batt info display usage (is shown only once per session)

//a counter to count.. something
byte counter = 0; //counter

//EEPROM addresses
unsigned int s1Addr=0;
unsigned int s2Addr=4;

//button/mode things
byte mode = 0; //0=norm 1,2,3... = calibration
byte btn = 0;
bool b1down,b2down,b3down; //button states
float adjStep = 10; //mode-dependent step
String adjSensor = "Front";

//scale sensor states, flags to avoid excess commands (and time consumption)
bool frontOn = false;
bool rearOn = false;

//lcd update timer (to prevent updating too often, f=0.5 Hz)
unsigned int t_prev_lcd_update = 0;
String prevL1, prevL2; //previous lines printed
byte prevLine;

//no activity --> turn off the LCD backlight and turn on the led?
byte n_zero_meas = 0;
byte n_sleep = 3; //after this time without any load on sensors LCD is turned off
unsigned int t_signal = 0; //millis() since las signaling
bool powerSave = false;

//-----------------------------------------------------------------
//setup
void setup() 
  {
  //button pin modes
  pinMode(button1Pin,INPUT_PULLUP); //button1 func
  pinMode(button2Pin,INPUT_PULLUP); //button2 +
  pinMode(button3Pin,INPUT_PULLUP); //button3 +

  //led or beeper
  pinMode(beeperPin,OUTPUT);

  //change reference voltage to 1.1 v (internal)
  analogReference(INTERNAL);
  
  lcd.begin (16,2); //  chars, lines
  lcd.setBacklightPin(3,POSITIVE);
  lcd.home (); 

  Serial.begin(9600); //for serial transmission

  //splash 
  print2lcd(1,t0);

  lcd.setBacklight(HIGH);
  print2lcd(2,t2);
  
  //HX711 init
  //read calibration factors from EEPROM 
  readFromEEPROM();
  
  front_scale.set_scale(sens_cal_1); 
  front_scale.tare(); 
  rear_scale.set_scale(sens_cal_2);
  rear_scale.tare(); 
  
  //power up the sensors
  front_scale.power_up();
  frontOn = true;
  rear_scale.power_up();
  rearOn = true;

  //sw operation mode
  mode = 0; //normal mode

  //button index, 0=no buttons pressed
  btn = 0;
  }
//-----------------------------------------------------------------
//main operation
void loop() 
  {
  //buttons?
  areButtonsPressed();
  
  if (mode == 0) //normal operation
    {
    //read sensor values
    readSensors();
  
    //send sensor and battery values to serial port [front rear U_batt]
    Serial.println(String(eW,2)+" "+String(tW,2)+" "+String(readBattVoltage(),2));
  
    //countdown during init
    if (counter < 6 && counter > 0) print2lcd(2,t2+String(5-counter));
    
    //set sensors to zero (tare)
    if (counter == 5) 
      {
      front_scale.tare();
      rear_scale.tare();
      giveSignal(2); //rdy
      }
    
    //update the counter
    counter++;
    
    //operation after init phase
    if (counter > 5) 
      {
      //total weight
      float mass = eW + tW;
      if (mass < treshold) mass = 0; //less than treshold = 0 g
      
      //calculate CG:
      float cog = calculateCG();
      
      //check activity --> powerSave 
      if (mass == 0)
        {
        n_zero_meas = n_zero_meas + 1;  
        if (n_zero_meas >= n_sleep && powerSave == false) 
          {
          lcd.setBacklight(LOW);
          powerSave = true;
          t_signal = (unsigned int)millis(); //reset timer
          }
        }
        else //some weight on scale
        {
        n_zero_meas = 0;  
        lcd.setBacklight(HIGH);
        powerSave = false;
        }
      
      //print total mass to lcd
      if (weight_oz == false) print2lcd(1,t3 +" "+String(mass,1)+" g");
      if (weight_oz == true)  print2lcd(1,t3 +" "+String(mass,2)+" oz");
  
      //print CG if feasible
      if (String(cog,1) != "0.0")
        {
        print2lcd(2,t4 +" "+String(cog,1)+" mm");  
        }
        else
        {
        print2lcd(2,t1+" "+String(readBattVoltage(),1)+"V");  
        }
  
      //counter rotation...
      if (counter > 20) //run between 10 and 20
        {
        counter = 10; //run between 10 and 20
        
        //voltage measurement
        if (readBattVoltage() < U_batt_low && battAlarmShown == false) //if lover than value set...
          {
          //...show info about battery voltage
          print2lcd(1,t1 +" < "+String(U_batt_low)+"V");
          giveSignal(3);
          
          delay(2000);
          battAlarmShown = true; //flag up
          t_signal = (unsigned int)millis(); //reset timer
          }
          
        //low battery, signal is given every 30 seconds
        if (readBattVoltage() < U_batt_low && battAlarmShown == true)
          {
          if ((unsigned int)millis() - t_signal > 30000)  
            {
            giveSignal(3);
            t_signal = (unsigned int) millis(); //update timestamp
            }
          }
        }
      }
    } //end of mode 0 
  
  //signal during power save state
  if (powerSave == true && ((unsigned int)millis() - t_signal) > 20000)
    {
    giveSignal(3);
    t_signal = (unsigned int)millis();  
    }
  
  //other modes
  //Question "calibrate?"
  if (mode == 1)
    {
    print2lcd(1,t5);
    print2lcd(2,t6);
    delay(500);
    }
  
  //cbr modes 1-7
  if (mode > 1 && mode < 8)
    {
    print2lcd(1,adjSensor+", step="+String(adjStep,2));
    
    if (mode > 1 && mode < 5) //front sensor
      {
      if (frontOn == false) {front_scale.power_up();  frontOn = true;}
      if (rearOn == true)   {rear_scale.power_down(); rearOn = false;}
      
      front_scale.set_scale(sens_cal_1); //update scaling factor   
      eW = front_scale.get_units(4)*-1; //-1 to invert if needed //n_avg
      if (weight_oz == true) eW = eW * 0.035274;
      
      //print factor and result to lcd
      if (weight_oz == false) print2lcd(2,String(sens_cal_1,2)+" "+String(eW,2)+" g");
      if (weight_oz == true)  print2lcd(2,String(sens_cal_1,2)+" "+String(eW,2)+" oz");
      }
      else //rear sensor
      {
      if (frontOn == true)  {front_scale.power_down();  frontOn = false;}
      if (rearOn == false)  {rear_scale.power_up();     rearOn = true;}
      
      rear_scale.set_scale(sens_cal_2); //update scaling factor   
      tW = rear_scale.get_units(4)*-1; //-1 to invert if needed
      if (weight_oz == true) tW = tW * 0.035274;
      
      //print factor and result to lcd
      if (weight_oz == false) print2lcd(2,String(sens_cal_2,2)+" "+String(tW,2)+" g");  
      if (weight_oz == true)  print2lcd(2,String(sens_cal_2,2)+" "+String(tW,2)+" oz");  
      }
    }

  if (mode == 8) //save values to eeprom
    {
    print2lcd(1,"Saving values...");
    print2lcd(2,"");
    
    saveToEEPROM();
        
    delay(1000);

    if (frontOn == false)  {front_scale.power_up();  frontOn = true;}
    if (rearOn == false)   {rear_scale.power_up();   rearOn = true;}
      
    mode = 0; // back to business
    }

  //power save state --> wait 5 sec after each loop
  if (counter > 5 && powerSave == true && mode == 0) delay(5000); 
  } //end of loop

//-----------------------------------------------------------------
//Functions
//printing to LCD
void print2lcd(int line_1_2,String text)
  {
  //to avoid updating too often
  bool update = false;
  if (line_1_2 != prevLine) update = true;
  if (line_1_2 == prevLine && ((unsigned int)millis()-t_prev_lcd_update) > lcd_min_update_interval_ms) update = true;

  //check if the contents is changed.. if not --> do not update LCD
  if (line_1_2 == 1 && text == prevL1) update = false;
  if (line_1_2 == 2 && text == prevL2) update = false;
  
  
  if (update == true)
    {
    //clear line
    lcd.setCursor(0,line_1_2-1);
    lcd.print("                "); //..stupid way

    //print mess
    lcd.setCursor(0,line_1_2-1);
    lcd.print(text);  
 
    //update timestanp, line number and such
    prevLine = line_1_2;
    t_prev_lcd_update = (unsigned int)millis();

    if (line_1_2 == 1) prevL1 = text;
    if (line_1_2 == 2) prevL2 = text;
    }
  }

//reading the sensor values
//values are put to global variables eW,tW
void readSensors()
  {
  //front 
  eW = front_scale.get_units(n_avg);
  if (invert_front_sensor == true) eW = eW * -1;

  if (eW < 0) eW = 0; //no negative values
  if (weight_oz == true) eW = eW * 0.035274; //oz?
  
  //rear
  tW = rear_scale.get_units(n_avg);
  if (invert_rear_sensor == true) tW = tW * -1;
  
  if (tW < 0) tW = 0; //no negative values
  if (weight_oz == true) tW = tW * 0.035274; //oz?
  }

//calculate CG
float calculateCG()  
  {
  if (eW > treshold && tW > treshold) //proceed only if there are relevant values
    {
    CGratio = tW / (eW + tW);
    return (((WingPegDist) * CGratio)) - ((WingPegDist) / 2) + CGoffset;
    }
    else
    {
    return 0;
    }
  }

//read battery voltage
float readBattVoltage() 
  {
  int val = analogRead(batRefPin);
  return val*1.1/1023/0.818*9; //multiplier 0.818 is pre-calculated from resistor values and ref voltage.
  }

//handle buttons
void areButtonsPressed()
  {  
  btn = 0; //button index, 0 = no buttons pressed
  //detect button up
  //b1
  if (b1down == false && digitalRead(2) == 0){b1down = true;}
  else{if (b1down == true && digitalRead(2) == 1){b1down = false;btn = 1;}} 
    
  //b2
  if (b2down == false && digitalRead(3) == 0){b2down = true;}
  else{if (b2down == true && digitalRead(3) == 1){b2down = false;btn = 2;}} 

   //b3
  if (b3down == false && digitalRead(4) == 0){b3down = true;}
  else{if (b3down == true && digitalRead(4) == 1){b3down = false;btn = 3;}} 

  //-------------------------------------------
  //mode-spesific operations
  //from basic operation to calibration mode, question?
  if (mode == 0 && btn == 1 && counter > 10)
    {
    mode = 1; //question 
    btn = 0;
    }
  
  //btn2 or 3 -> cancel, back to mode 0
  if (mode == 1 && btn > 1)
    {
    mode = 0;  
    btn = 0;
    }

  //btn1 = yes --> change to calibration mode a
  if (mode == 1 && btn == 1)
    {
    mode = 2;  
    btn = 0;
    }

  //front, step = 10
  if (mode == 2)
    {
    if (btn == 1) mode++; //next
    adjSensor = "Front";
    adjStep = 10;
    if (btn == 2) sens_cal_1 = sens_cal_1-adjStep;
    if (btn == 3) sens_cal_1 = sens_cal_1+adjStep;
    btn = 0;  
    }
  //front, step = 1
  if (mode == 3)
    {
    if (btn == 1) mode++; //next
    adjSensor = "Front";
    adjStep = 1;
    if (btn == 2) sens_cal_1 = sens_cal_1-adjStep;
    if (btn == 3) sens_cal_1 = sens_cal_1+adjStep;
    btn = 0;  
    }
  //front, step = 0.05
  if (mode == 4)
    {
    if (btn == 1) mode++; //next
    adjSensor = "Front";
    adjStep = 0.05;
    if (btn == 2) sens_cal_1 = sens_cal_1-adjStep;
    if (btn == 3) sens_cal_1 = sens_cal_1+adjStep;
    btn = 0;  
    }
  
  //rear, step = 10
  if (mode == 5)
    {
    if (btn == 1) mode++; //next
    adjSensor = "Rear";
    adjStep = 10;
    if (btn == 2) sens_cal_2 = sens_cal_2-adjStep;
    if (btn == 3) sens_cal_2 = sens_cal_2+adjStep;
    btn = 0;  
    }
  //rear, step = 1
  if (mode == 6)
    {
    if (btn == 1) mode++; //next
    adjSensor = "Rear";
    adjStep = 1;
    if (btn == 2) sens_cal_2 = sens_cal_2-adjStep;
    if (btn == 3) sens_cal_2 = sens_cal_2+adjStep;
    btn = 0;  
    }
  //rear, step = 0.05
  if (mode == 7)
    {
    if (btn == 1) mode++; //next
    adjSensor = "Rear";
    adjStep = 0.05;
    if (btn == 2) sens_cal_2 = sens_cal_2-adjStep;
    if (btn == 3) sens_cal_2 = sens_cal_2+adjStep;
    btn = 0;  
    }  
  }

//EEPROM
//read calibration factors
void readFromEEPROM()
  {
  //reset EEPROM if button1 = down
  if (digitalRead(2) == 0)
    {
    for (int i = 0;i<EEPROM.length();i++) EEPROM.write(i, 0xff);  
    print2lcd(1,"EEPROM RST");

    sens_cal_1 = 900;
    sens_cal_2 = 900;

    EEPROM.put(s1Addr,sens_cal_1);
    EEPROM.put(s2Addr,sens_cal_1);
    
    delay(1000);
    }
    else //read values from eeprom
    {
    EEPROM.get(s1Addr,sens_cal_1);
    EEPROM.get(s2Addr,sens_cal_2);  
    }
   
  Serial.println(sens_cal_1,HEX);
  Serial.println(sens_cal_2,HEX);
  }

//save calibration factors
void saveToEEPROM()
  {
  EEPROM.put(s1Addr,sens_cal_1);
  EEPROM.put(s2Addr,sens_cal_2);
  }

//signaling with led or beeper
void giveSignal(byte n)
  {
  byte t = 100;
  for (int i=0;i<n;i++)
    {
    digitalWrite(beeperPin,HIGH);
    delay(t);
    digitalWrite(beeperPin,LOW);
    delay(t);  
    }
  }
