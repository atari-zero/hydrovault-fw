#define ENCODER_OPTIMIZE_INTERRUPTS

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Encoder.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_AM2320.h>
#include <Buzzer.h>

//Arduino Nano pin definitions

const int relayA = A0;
const int relayB = A1;
const int button = A3;
const int potA = A6;
const int potB = A7;
const int encoderA = 2;
const int encoderB = 3;
const int buzzerPin = 4;
const int vOutA = 5;
const int vOutB = 6;
const int stepPin = 7;
const int dirPin = 8;
const int LEDPin = 9;
const int CSPin = 10;

//Encoder
  Encoder Enc (encoderA, encoderB);

//LCD 2004 I2C
  LiquidCrystal_I2C lcd(0x27,20,4);  // LCD address 0x27; 20 chars; 4 line display

//TMC2130 Steppers
  #define DIR_PIN          dirPin // Direction
  #define STEP_PIN         stepPin // Step
  #define CS_PIN           CSPin // Chip select

  #define R_SENSE 0.11f

  TMC2130Stepper driver = TMC2130Stepper(CS_PIN, R_SENSE); // Hardware SPI
  constexpr uint32_t steps_per_mm = 80;
  AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);


//RTC DS3107+
  RTC_DS1307 rtc;
  char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"}; //RTC Days

//Buzzer
  Buzzer buzzer(buzzerPin, LEDPin);

//global variables


void setup() {

//init
  Serial.begin(57600);
  Wire.begin();
  SPI.begin();
  rtc.begin();         // initialize clock
  lcd.init();          // initialize the lcd 
  lcd.backlight();     // turn on lcd backlight                  
  
//pin modes
  pinMode(A0, OUTPUT); // light relay A
  pinMode(A1, OUTPUT); // light relay B
  pinMode(A3, INPUT);  // encoder button
  pinMode(A6, INPUT);  // potentiometer 1
  pinMode(A7, INPUT);  // potentiometer 2
  pinMode(2, INPUT);   // encoder interrupt A
  pinMode(3, INPUT);   // encoder interrrupt B
  pinMode(4, OUTPUT);  // buzzer
  pinMode(5, OUTPUT);  // 12V Mosfet-regulated terminal A
  pinMode(6, OUTPUT);  // 12V Mosfet-regulated terminal B
  pinMode(7, OUTPUT);  // stepper STEP
  pinMode(8, OUTPUT);  // steper DIR 
  pinMode(9, OUTPUT);  // just a LED
  pinMode(10, OUTPUT); // SPI CS Pin

//RTC setup & adjustment
if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

//TMC Stepper init
  digitalWrite(CS_PIN, HIGH);
  driver.begin();             // Initiate pins and registeries
  driver.rms_current(600);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driver.en_pwm_mode(1);      // Enable extremely quiet stepping
  driver.pwm_autoscale(1);
  driver.microsteps(16);

  stepper.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();
}

void loop() {
  
  digitalWrite(relayA, LOW);
  digitalWrite(relayB, LOW);

}
