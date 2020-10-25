#include <Arduino.h>
#include <Wire.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Stepper.h>
#include <ds3231.h>

//screen settings
//#define SCREEN_WIDTH 128 
//#define SCREEN_HEIGHT 64

//stepper & rotation settings
const long stepsPerRevolution = 196600; //to get one full wheel cycle
const int motorInterfaceType = 1;
int rph = 1;

//pin definitions
const int waterSensor = A3;
const int encoderA = 2;
const int encoderB = 3;
const int waterSensorPower = 4;
const int lightA = 5;
const int lightB = 6;
const int stepperDisablePin = 7;
const int stepPin = 8;
const int dirPin = 9;
const int button = 10;
const int pump = 11;

//global variables
unsigned long startTime;

//objects
//Adafruit_SSD1306 screen(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Encoder knob(encoderA, encoderB);
Stepper motors = Stepper(3200, 8, 9);

//RTC
struct ts t;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  DS3231_init(DS3231_INTCN);
  //t.hour=19; 
  //t.min=21;
  //t.sec=0;
  //t.mday=27;
  //t.mon=8;
  //t.year=2020;
 
  //DS3231_set(t);
  
  pinMode(A3, INPUT);   //water sensor signal
  pinMode(2, INPUT);    //encoder interrupt A
  pinMode(3, INPUT);    //encoder interrrupt B
  pinMode(4, OUTPUT);   //water sensor power
  pinMode(5, OUTPUT);   //light relay A
  pinMode(6, OUTPUT);   //light relay B
  pinMode(7, OUTPUT);   //stepper EN
  pinMode(8, OUTPUT);   //stepper STEP
  pinMode(9, OUTPUT);   //steper DIR
  pinMode(10, INPUT);   //button
  pinMode(11, OUTPUT);  //pump
  pinMode(13, OUTPUT);  //LED builtin

   
//  screen.begin(SSD1306_SWITCHCAPVCC, 0x3C); //oLED screen init
  
  digitalWrite(pump, HIGH);  //set pump to off
  digitalWrite(lightA, HIGH);  //lights off
  digitalWrite(lightB, HIGH);  //lights off
  digitalWrite(stepperDisablePin, LOW); //enable stepper
  digitalWrite(dirPin, HIGH); //set default direction
  digitalWrite(waterSensorPower, LOW); //turn off water sensor
}

void loop() {
  
  digitalWrite(waterSensorPower, HIGH);
  bool waterLevel = digitalRead(waterSensor); 

  if (waterLevel == 1) digitalWrite(pump, LOW);
  if (waterLevel == 0) digitalWrite(pump, HIGH);
  
  motors.setSpeed(4.11*rph); 
  motors.step(-196600);

  digitalWrite(lightA, LOW);
  digitalWrite(lightB, LOW);

  DS3231_get(&t);
  if (t.hour <= 6) {
    digitalWrite(lightA, HIGH);
    digitalWrite(lightB, HIGH);
  }

}
