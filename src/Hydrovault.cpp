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

const int relayApin = A0;
const int relayBpin = A1;
const int buttonPin = A3;
const int potApin = A6;
const int potBpin = A7;
const int encoderA = 2;
const int encoderB = 3;
const int buzzerPin = 4;
const int vOutApin = 5;
const int vOutBpin = 6;
const int stepPin = 7;
const int dirPin = 8;
const int LEDPin = 9;
const int CSPin = 10;

//Global variables

int rpd;
int prg;
int day;
int tmp;
int hum;
int lux;
bool relayAStat;
bool relayBStat;

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

//Adafruit AM2320 Temp & Humidity sensor
  Adafruit_AM2320 am2320 = Adafruit_AM2320(); 

//Adafruit TSL2561 Luminosity sensor
  Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

void setup() {

//init
  Serial.begin(57600);
  Wire.begin();
  SPI.begin();
  rtc.begin();         // initialize clock
  lcd.init();          // initialize the lcd 
  lcd.backlight();     // turn on lcd backlight 
  am2320.begin();      // temp+humidity sensor init  
  tsl.begin();         // luminosity sensor init

//TSL config
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */

  //tsl.setIntegrpiationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */     
  
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

//TMC Stepper setup
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

//Welcome screen
  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("HYDROVAULT  v1.0");
  lcd.setCursor(2,1);
  lcd.print("----------------");
  lcd.setCursor(1,2);
  lcd.print("Open Source Rotary");
  lcd.setCursor(1,3);
  lcd.print("Hydroponic  System");
  delay(2000);
  lcd.clear();

}

void loop() {

  digitalWrite(relayApin, relayAStat); //Set lights to off
  digitalWrite(relayBpin, relayBStat);

  //Info Screen
  //Display Time & Date
  DateTime now = rtc.now();

  char buf1[] = "hh:mm:ss";
  char buf2[] = "DD/MM/YY";
  lcd.setCursor(0,2);
  lcd.print(now.toString(buf1));
  lcd.setCursor(0,3);
  lcd.print(now.toString(buf2));
  
  //Revolutions, day & program name
  lcd.setCursor(0,0);
  lcd.print("PRG:");
  lcd.print("None");
  lcd.setCursor(0,1);
  lcd.print("Day:");
  lcd.print(day);
  
  lcd.setCursor(12,0);
  lcd.print("RPD:");
  lcd.print(rpd);

  //Temperature, Humidity & Luminosity readings
  lcd.setCursor(12,1);
  lcd.print("Tmp:"); 
  lcd.print(tmp);
  lcd.print((char)223);

  lcd.setCursor(12,2);
  lcd.print("Hum:"); 
  lcd.print(hum);
  lcd.print("%");

  lcd.setCursor(12,3);
  lcd.print("Lux:"); 
  lcd.print(lux);

}
