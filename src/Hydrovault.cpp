#include <Arduino.h>
#include <Wire.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <TMCStepper.h>
#include <RTClib.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_AM2320.h>

//Arduino Nano pin definitions

const int lightA = A0;
const int lightB = A1;
const int button = A3;
const int potA = A6;
const int potB = A7;
const int encoderA = 2;
const int encoderB = 3;
const int buzzer = 4;
const int vOutA = 5;
const int vOutB = 6;
const int stepPin = 7;
const int dirPin = 8;

//global variables
//unsigned long startTime;

//stepper & rotation settings
const long stepsPerRevolution = 196600; //to get one full wheel cycle
const int motorInterfaceType = 1;
int rph = 1;

//objects
//Adafruit_SSD1306 screen(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Encoder knob(encoderA, encoderB);
//Stepper motors = Stepper(3200, 8, 9);

//RTC
//struct ts t;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
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
  
  digitalWrite(pump, HIGH);  //set pump to off
  digitalWrite(lightA, HIGH);  //lights off
  digitalWrite(lightB, HIGH);  //lights off
  digitalWrite(stepperDisablePin, LOW); //enable stepper
  digitalWrite(dirPin, HIGH); //set default direction
  digitalWrite(waterSensorPower, LOW); //turn off water sensor
}

void loop() {
  
  //motors.setSpeed(4.11*rph); 
  //motors.step(-196600);

  digitalWrite(lightA, LOW);
  digitalWrite(lightB, LOW);

}
