#define ENCODER_OPTIMIZE_INTERRUPTS

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Encoder.h>
#include <TMC2130Stepper.h>
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
const int dirPin = 8;
const int stepPin = 7;
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

//TMC2130 Stepper
  TMC2130Stepper driver = TMC2130Stepper(LEDPin, dirPin, stepPin, CSPin);
  AccelStepper stepper = AccelStepper(stepper.DRIVER, stepPin, dirPin);

//RTC DS3107+
  RTC_DS1307 rtc;

//Buzzer
  Buzzer buzzer(buzzerPin);

//Adafruit AM2320 Temp & Humidity sensor
  Adafruit_AM2320 am2320 = Adafruit_AM2320(); 

//Adafruit TSL2561 Luminosity sensor
  Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

void setupTSL() {
  tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  // tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */

  //tsl.setIntegrpiationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */
}

void setupRTC() {
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0)); // January 21, 2014 at 3am
}

void setupTimer1() {
  cli();
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  OCR1A = 249; // set compare match register for 1hz increments // = (16*10^6) / (1*64) - 1 (must be <65536)
  TCCR1B |= (1 << WGM12); // turn on CTC mode
  TCCR1B |= (1 << CS11) | (1 << CS10); // Set CS11 and CS10 bits for 64 prescaler  
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}

ISR(TIMER1_COMPA_vect){ //timer1 interrupt 1kHz keeps motor running at constant speed no matter what's going on
  stepper.setSpeed(800);
  stepper.runSpeed();
}

void setupMotor(){  //TMC Stepper setup
  pinMode(CSPin, OUTPUT);
  digitalWrite(CSPin, HIGH);
  driver.begin();             // Initiate pins and registeries
  driver.rms_current(600);    // Set stepper current to 600mA.
  driver.stealthChop(1);      // Enable extremely quiet stepping
  driver.stealth_autoscale(1);
  driver.microsteps(16);

  stepper.setMaxSpeed(800);
  stepper.setAcceleration(3000);
  stepper.setEnablePin(LEDPin);
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();
}

void welcomeScreen(){
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

void readSensors(){
  tmp = am2320.readTemperature();
  hum = am2320.readHumidity();
  sensors_event_t event;
  tsl.getEvent(&event);
  lux = event.light;
}

void displayTimeDate(){
  DateTime now = rtc.now();
  char buf1[] = "hh:mm:ss";
  char buf2[] = "DD/MM/YY";
  lcd.setCursor(0,2);
  lcd.print(now.toString(buf1));
  lcd.setCursor(0,3);
  lcd.print(now.toString(buf2));
}

void setup() {

  SPI.begin();
  Wire.begin();
  Serial.begin(9600);
  rtc.begin();         // initialize clock
  lcd.init();          // initialize the lcd 
  lcd.backlight();     // turn on lcd backlight 
  am2320.begin();      // temp+humidity sensor init  
  tsl.begin();         // luminosity sensor init

  setupMotor();
  setupTimer1();
  setupTSL();  
  setupRTC();
  
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
  pinMode(9, OUTPUT);  // just a LED

  welcomeScreen();     //Show welcome screen
}


void loop() {

  readSensors();
  displayTimeDate();

  digitalWrite(relayApin, relayAStat); //Set lights to off
  digitalWrite(relayBpin, relayBStat);

  //Info Screen 
  
  //Revolutions, day & program name
  lcd.setCursor(0,0);
  lcd.print("PRG:");
  lcd.print("None");
  lcd.setCursor(0,1);
  lcd.print("Day:");
  lcd.print(day);
  
  lcd.setCursor(10,0);
  lcd.print("RPD:");
  lcd.print(rpd);

  //Temperature, Humidity & Luminosity readings
  lcd.setCursor(10,1);
  lcd.print("Tmp:"); 
  lcd.print(tmp);
  lcd.print((char)223);

  lcd.setCursor(10,2);
  lcd.print("Hum:"); 
  lcd.print(hum);
  lcd.print("%");
  
  lcd.setCursor(10,3);
  lcd.print("Lux:"); 
  lcd.setCursor(14,3);
  lcd.print(lux);
  lcd.print("     ");
}


