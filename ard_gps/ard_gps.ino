// Copyright (C) 2015 Jason K Holden

// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program; if not, write to
// the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor
// Boston, MA  02110-1301, USA.

#include <SoftwareSerial.h>
#include "TinyGPS.h"
#include <Servo.h>


// Shenandoah National Park
// Hawksbill Peak
float dest_lat=38.555426;
float dest_lon=-78.395296;


const int MINUTES_TO_GIVE_UP=4;
int SER_LCD_PIN    = 8;
int PBTN_DSP_PIN   = 7;
int PDOWN_PIN      = 5;
int OVERRIDE_PIN   = 3;
int GPS_1PPS       = 2;
int SELECT_PIN     = 12;
int ENTER_PIN      = 13;
int SELECT_RDY_PIN = 6;
int ENTER_RDY_PIN  = 4;
int MIN_SATELLITES = 4;
int DESIRED_SATELLITES = 5;
const unsigned long SECS_IN_MIDRANGE = 20UL;

bool inGoodEnough = false;
bool goodEnough = false;
unsigned long startInRangeTime;
unsigned long stopInRangeTime;
            
SoftwareSerial LCD = SoftwareSerial(-1, SER_LCD_PIN);
TinyGPS gps_parser;
Servo latch;

void setup() {                
  // initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards:
  pinMode(PDOWN_PIN,      OUTPUT);
  digitalWrite(PDOWN_PIN, LOW);    // Keep us on
  pinMode(GPS_1PPS,       INPUT);  // Not used, here for completeness
  pinMode(OVERRIDE_PIN,   INPUT);  
  pinMode(SER_LCD_PIN,    OUTPUT);
  pinMode(PBTN_DSP_PIN,   OUTPUT);
  pinMode(SELECT_PIN,     INPUT);
  pinMode(ENTER_PIN,      INPUT);
  pinMode(SELECT_RDY_PIN, OUTPUT);
  pinMode(ENTER_RDY_PIN,  OUTPUT);  

  digitalWrite(PBTN_DSP_PIN,HIGH);  
  LCD.begin(9600);
  delay(1000);
  configureLcd();
  clearLcd();
  backlightOn();
  writeLcd();

  latch.attach(9);
  lock_latch();
  delay(100);

  // Setup GPS Receiver
  Serial.begin(4800);

  
}

long motorCount=0;
long overrideCount=0;
void loop() {
  
  // Toggle some pretty LED's based on switch state
  if(digitalRead(SELECT_PIN) == HIGH){
    digitalWrite(SELECT_RDY_PIN,HIGH); 
  } else {
    digitalWrite(SELECT_RDY_PIN,LOW);
  }
  if(digitalRead(ENTER_PIN) == HIGH){
    digitalWrite(ENTER_RDY_PIN,HIGH); 
  } else {
    digitalWrite(ENTER_RDY_PIN,LOW);
  }
  
  // Utilize override pin, which if continuously held, will
  // start the motor sweeping
  if(digitalRead(OVERRIDE_PIN) == HIGH){
    overrideCount++;
    if(overrideCount > 4){// Perform override functionality
      clearLcd();
      selectLineOne();
      LCD.print("Override engadged");
      sweep_motor();
    }
  } else {
    overrideCount = 0; 
  }
  
  unsigned long stop_time=1000UL * 60 * MINUTES_TO_GIVE_UP;
  if(millis() > stop_time){
     clearLcd();
     selectLineOne();
     LCD.print("Failed gps lock");
     selectLineTwo();
     LCD.print("Giving up");
     delay(5000);
     powerdown();
  }
  
  while(Serial.available()){
    char c = Serial.read();

    Serial.print(c);
    if(gps_parser.encode(c)){
      // Successful GPS decode

       int curSatellites=gps_parser.satellites();
       goodEnough = false;

       // We are definitely accurate
       if(curSatellites >= DESIRED_SATELLITES && curSatellites != 255){
         goodEnough = true;
         clearLcd();
         selectLineOne();
         LCD.print("GPS is great");
         delay(1500);  
       }
       
       // We are ok, may wait a little longer or give up and use it
       if(curSatellites < DESIRED_SATELLITES &&
          curSatellites >= MIN_SATELLITES){
          if(inGoodEnough == false){
            // first time in range
            inGoodEnough = true;
            startInRangeTime=millis();
            stopInRangeTime=startInRangeTime+1000UL*SECS_IN_MIDRANGE;
          }
          
          unsigned long curTime = millis();
          if (curTime > stopInRangeTime){
            // We've had enough satellites for long enough, lets see the answer
            goodEnough=true;
            clearLcd();
            selectLineOne();
            LCD.print("GPS is good");
            delay(1500); 
          }          
       }
       
       // We're definitely not good
       if(curSatellites < MIN_SATELLITES || curSatellites == 255){
         goodEnough = false;
         inGoodEnough = false;        
       }
       
       if(goodEnough == false){
            
         clearLcd();
         selectLineOne();
         LCD.print("GPS Locking..");
         selectLineTwo();
         LCD.print("sats=");
         LCD.print(curSatellites);
         delay(100);
       } else {
         // We've got good data
         processGps();      
       }
    }
  }
  
  if((motorCount % 32000) < 24000){
     digitalWrite(PBTN_DSP_PIN,LOW); 
  } else {
     digitalWrite(PBTN_DSP_PIN,HIGH); 
  }
  ++motorCount;
   
}

void sweep_motor(){
  int pos=0;
  for(pos = 45; pos < 165; pos += 1)  // goes from 0 degrees to 180 degrees
  {                                  // in steps of 1 degree
    latch.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for(pos = 165; pos>=45; pos-=1)     // goes from 180 degrees to 0 degrees
  {                                
    latch.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }    
}

void unlock_latch(){
  latch.write(125);
  delay(100); 
  latch.write(85);
  delay(100);
  latch.write(45); 
}

void lock_latch(){
  latch.write(165); 
}

void processGps(){
  // time in hhmmsscc, date in ddmmyy
  unsigned long fix_age, time, date, speed, course;

  // Output the Lat/Lon
  long lat, lon;
  gps_parser.get_position(&lat,&lon,&fix_age);

  float latf=lat/1000000.0;
  float lonf=lon/1000000.0;
  float distance_to_dest=gps_parser.distance_between(latf,lonf,dest_lat,dest_lon);
  long distance_to_dest_m=distance_to_dest;
  Serial.end();
  
  
  clearLcd();
  selectLineOne();
  LCD.print("Dist: ");
  if(distance_to_dest_m > 3201){
    // convert to miles
    float distance_to_dest_conv = distance_to_dest * 0.000621371; 
    long distance_to_dest_p=distance_to_dest_conv;
    LCD.print(distance_to_dest_p);
    LCD.print(" mi");
  } else {
    // print in feet
    float distance_to_dest_conv = distance_to_dest * 3.28084;
    long distance_to_dest_p=distance_to_dest_conv;
    LCD.print(distance_to_dest_p);
    delay(50);
    LCD.print(" feet");    
  }
  selectLineTwo();
  LCD.print("meters=");
  LCD.print(distance_to_dest_m);
  
  delay(10000);
  
  if(distance_to_dest < 1000){
    unlock_sequence();
  } else {
    powerdown_sequence(); 
  }
  
}

void powerdown_sequence(){
  clearLcd();
  selectLineOne();
  LCD.print("Not there yet!");
  selectLineTwo();
  LCD.print("Get closer!");
  delay(5000);
  powerdown(); 
}

void unlock_sequence(){

  // Let's do this!
  clearLcd();
  selectLineOne();
  LCD.print("Puzzle");
  selectLineTwo();
  LCD.print(" solved!!!");    
  delay(5000);

  clearLcd();
  selectLineOne();
  LCD.print("Jocelyn,");
  selectLineTwo();
  LCD.print("I love you");  
  delay(6000);
  
  clearLcd();
  selectLineOne();
  LCD.print("I want to spend");
  selectLineTwo();
  LCD.print("my life with you");  
  delay(6000);
  
  clearLcd();
  selectLineOne();
  LCD.print("  Will you");
  selectLineTwo();
  LCD.print("  marry me???");
  delay(9000);   
  
  clearLcd();
  selectLineOne();
  LCD.print("Unlocking!!!");
  delay(5000);
  
  // Open the box
  unlock_latch();
  delay(5000);
  
  // Finish up
  clearLcd();
  selectLineOne();
  LCD.print("Puzzle Unlocked!");
  delay(90000);
  for(int i=0;i<4;++i){
    sweep_motor();  
  }
  powerdown();
}

void configureLcd(){
    LCD.write(0x7C);   //command flag for backlight stuff
    LCD.write(0x04);   //16 chars wide (i.e. 16x2)
}

void powerdown(){
  digitalWrite(PDOWN_PIN,HIGH); // Keep us on
}

void writeLcd(){
  clearLcd();
  selectLineOne(); 

  LCD.print("Puzzle Starting");
  selectLineTwo(); 

  LCD.print("Use at own risk");
  //LCD.print(gps_parser.satellites());
  delay(5000);
  
  clearLcd();
  selectLineOne(); 
  LCD.print("Starting GPS...");
  delay(3000);
}

void selectLineOne(){ 
  //puts the cursor at line 0 char 0. 
  LCD.write(0xFE); //command flag 
  LCD.write(128); //position
  delay(100);  
} 

void selectLineTwo(){ 
  //puts the cursor at line 0 char 0. 
  LCD.write(0xFE); //command flag 
  LCD.write(192); //position
  delay(100); 
}

void clearLcd(){ 
  LCD.write(0xFE); //command flag 
  LCD.write(0x01); //clear command. 
  delay(50); 
}

void backlightOn(){  //turns on the backlight
    LCD.write(0x7C);   //command flag for backlight stuff
    LCD.write(157);    //light level.
    delay(50);
}
void backlightOff(){  //turns off the backlight
    LCD.write(0x7C);   //command flag for backlight stuff
    LCD.write(128);     //light level for off.
    delay(50);
}

