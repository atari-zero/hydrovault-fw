#define ENCODER_OPTIMIZE_INTERRUPTS

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SimpleRotary.h>
#include <TMC2130Stepper.h>
#include <AccelStepper.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_AM2320.h>
#include <Buzzer.h>
#include <Encoder.h>


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

enum state {
  INFO_SCREEN,
  MENU1,
  MENU2, 
  M_PROGRAM,
  M_SPEED,
  M_LIGHT_A,
  M_LIGHT_B,
  M_AUX_1,
  M_AUX_2,
  M_EEPROM,
};

enum selectorPosition {
  SET_PROGRAM,
  SET_SPEED,
  SET_LIGHT_A,
  SET_LIGHT_B,
  SET_AUX_A,
  SET_AUX_B,
  SET_EEPROM,
  BACK
};

int state = INFO_SCREEN;  //Screen state
int rpd;                  //Revolutions per day
int prg;                  //Program No
int day;                  //Days passed since beginning
int tmp;                  //Current temperature
int hum;                  //Current humidity  
int lux;                  //Current luminosity
int speed;                //Rotation speed
bool relayAStat;          //Light A 
bool relayBStat;          //Light B
bool auxAStat;
bool auxBStat;
unsigned long timing;     //timing for delays
int selectorPosition = 0;
long oldPosition  = -999;
byte dir;
byte push;

Encoder encoder(2, 3);
LiquidCrystal_I2C lcd(0x27,20,4);   // LCD 2004 I2C LCD address 0x27; 20 chars; 4 line display
TMC2130Stepper driver = TMC2130Stepper(LEDPin, dirPin, stepPin, CSPin);  //TMC2130 Stepper
AccelStepper stepper = AccelStepper(stepper.DRIVER, stepPin, dirPin); //Accel Stepper Library
RTC_DS1307 rtc; //Real Time Clock DS3107+
Buzzer buzzer(buzzerPin); //Buzzer
Adafruit_AM2320 am2320 = Adafruit_AM2320(); //Adafruit AM2320 Temp & Humidity sensor
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345); //Adafruit TSL2561 Luminosity sensor

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

void drawStaticInfoScreen(){
  lcd.setCursor(0,0);
  lcd.print("PRG:");  //Program - 4,0 for value
  lcd.setCursor(0,1);
  lcd.print("Day:");  //Day No - 4,1 for value
  lcd.setCursor(10,0);
  lcd.print("RPD:");  //Revs per day - 14,0 for value
  lcd.setCursor(10,1);
  lcd.print("Tmp:");  //Temperature - 14,1 for value
  lcd.setCursor(10,2);
  lcd.print("Hum:");  //Humidity - 14,2 for value
  lcd.setCursor(10,3);
  lcd.print("Lux:");  //Luminosity - 14,3 for value
}

void updateInfoScreen(){
  lcd.setCursor(4,0);
  lcd.print(prg);
  lcd.setCursor(4,1);
  lcd.print(dir);
  lcd.setCursor(14,0);
  lcd.print(rpd);
  lcd.setCursor(14,1);
  lcd.print(tmp); lcd.print((char)223);
  lcd.setCursor(14,2);
  lcd.print(hum); lcd.print("%");
  lcd.setCursor(14,3);
  lcd.print(lux); lcd.print("     ");
}

void drawStaticMenu(){
  if (state == MENU1){
    lcd.setCursor(2,0);
    lcd.print("Set Program");
    lcd.setCursor(2,1);
    lcd.print("Set Speed");
    lcd.setCursor(2,2);
    lcd.print("Set Light A");
    lcd.setCursor(2,3);
    lcd.print("Set Light B");
  }
  if (state == MENU2){
    lcd.setCursor(2,0);
    lcd.print("Set Aux A");
    lcd.setCursor(2,1);
    lcd.print("Set Aux B");
    lcd.setCursor(2,2);
    lcd.print("Set EEPROM");
    lcd.setCursor(2,3);
    lcd.print("Exit");
  }
  if (state == M_PROGRAM){
    lcd.setCursor(2,0);
    lcd.print("Program Selection");
    lcd.setCursor(2,1);
    lcd.print("Name: ");
    lcd.print(prg);
    lcd.setCursor(2,2);
    lcd.print("Start Program!");
    lcd.setCursor(2,3);
    lcd.print("Exit");
  }
  if (state == M_SPEED){
    lcd.setCursor(2,0);
    lcd.print("Rotation Speed");
    lcd.setCursor(2,1);
    lcd.print("Rev./Day: ");
    lcd.print(rpd);
    lcd.setCursor(2,2);
    lcd.print("Change");
    lcd.setCursor(2,3);
    lcd.print("Exit");
  }
  if (state == M_LIGHT_A){
    lcd.setCursor(2,0);
    lcd.print("Light A Schedule");
    lcd.setCursor(2,1);
    lcd.print("Status: ");
    if (relayAStat == true){
      lcd.print("ON");
    }
    else {
      lcd.print("OFF");
    }
    lcd.setCursor(2,2);
    lcd.print("Set Schedule:");
    lcd.setCursor(2,3);
    lcd.print("Exit");
  }
  if (state == M_LIGHT_B){
    lcd.setCursor(2,0);
    lcd.print("Light B Schedule");
    lcd.setCursor(2,1);
    lcd.print("Status: ");
    if (relayBStat == true){
      lcd.print("ON");
    }
    else {
      lcd.print("OFF");
    }
    lcd.setCursor(2,2);
    lcd.print("Set Schedule:");
    lcd.setCursor(2,3);
    lcd.print("Exit");
  }
  if (state == M_AUX_1){
    lcd.setCursor(2,0);
    lcd.print("Aux 12V A Schedule");
    lcd.setCursor(2,1);
    lcd.print("Status: ");
    if (auxAStat == true){
      lcd.print("ON");
    }
    else {
      lcd.print("OFF");
    }
    lcd.setCursor(2,2);
    lcd.print("Set Schedule:");
    lcd.setCursor(2,3);
    lcd.print("Exit");
  }
  if (state == M_AUX_2){
    lcd.setCursor(2,0);
    lcd.print("Aux 12V B Schedule");
    lcd.setCursor(2,1);
    lcd.print("Status: ");
    if (auxBStat == true){
      lcd.print("ON");
    }
    else {
      lcd.print("OFF");
    }
    lcd.setCursor(2,2);
    lcd.print("Set Schedule:");
    lcd.setCursor(2,3);
    lcd.print("Exit");
  }
  if (state == M_EEPROM){
    lcd.setCursor(2,0);
    lcd.print("EEPROM Settings");
    lcd.setCursor(2,1);
    lcd.print("Store to EEPROM");
    lcd.setCursor(2,2);
    lcd.print("Clear EEPROM");
    lcd.setCursor(2,3);
    lcd.print("Exit");
  }
}

void Selector(){ 
  /*int oldSelectorPosition;

  if (selectorPosition == 0 || selectorPosition == 4) {
    lcd.setCursor(0,0);
    lcd.print(">>");
    if (selectorPosition != oldSelectorPosition){
      lcd.setCursor(0,0);
      lcd.print("  ");
      oldSelectorPosition = selectorPosition;
    }
  }

  if (selectorPosition == 1 || selectorPosition == 5) {
    lcd.setCursor(0,1);
    lcd.print(">>");
    if (selectorPosition != oldSelectorPosition){
      lcd.setCursor(0,1);
      lcd.print("  ");
      oldSelectorPosition = selectorPosition;
    }
  }

  if (selectorPosition == 2 || selectorPosition == 6) {
    lcd.setCursor(0,2);
    lcd.print(">>");
    if (selectorPosition != oldSelectorPosition){
      lcd.setCursor(0,2);
      lcd.print("  ");
      oldSelectorPosition = selectorPosition;
    }
  }

  if (selectorPosition == 3 || selectorPosition == 7) {
    lcd.setCursor(0,3);
    lcd.print(">>");
    if (selectorPosition != oldSelectorPosition){
      lcd.setCursor(0,3);
      lcd.print("  ");
      oldSelectorPosition = selectorPosition;
    }
  }*/

  switch (selectorPosition){
    case 0 :
      lcd.setCursor(0,0);
      lcd.print("->");
      lcd.setCursor(0,1);
      lcd.print("  ");
      if (push == 1){
        delay(50);
        lcd.clear();
        state = M_PROGRAM;
      } 
    break;
    case 1 :
      lcd.setCursor(0,0);
      lcd.print("  ");
      lcd.setCursor(0,1);
      lcd.print("->");
      lcd.setCursor(0,2);
      lcd.print("  ");
      if (push == 1){
        delay(50);
        lcd.clear();
        state = M_SPEED;
      } 
    break;
    case 2 :
      lcd.setCursor(0,1);
      lcd.print("  ");
      lcd.setCursor(0,2);
      lcd.print("->");
      lcd.setCursor(0,3);
      lcd.print("  ");
      if (push == 1){
        delay(50);
        lcd.clear();
        state = M_LIGHT_A;
      } 
    break;
    case 3 : 
      lcd.setCursor(0,2);
      lcd.print("  ");                  
      lcd.setCursor(0,3);
      lcd.print("->");
      if (push == 1){
        delay(50);
        lcd.clear();
        state = M_LIGHT_B;
      } 
    break;
    case 4:
      lcd.setCursor(0,0);
      lcd.print("->");
      lcd.setCursor(0,1);
      lcd.print("  ");
      if (push == 1){
        delay(50);
        lcd.clear();
        state = M_AUX_1;
      } 
    break;
    case 5:
      lcd.setCursor(0,0);
      lcd.print("  ");
      lcd.setCursor(0,1);
      lcd.print("->");
      lcd.setCursor(0,2);
      lcd.print("  ");
      if (push == 1){
        delay(50);
        lcd.clear();
        state = M_AUX_2;
      } 
    break;
    case 6:
      lcd.setCursor(0,1);
      lcd.print("  ");
      lcd.setCursor(0,2);
      lcd.print("->");
      lcd.setCursor(0,3);
      lcd.print("  ");
      if (push == 1){
        delay(50);
        lcd.clear();
        state = M_EEPROM;
      } 
    break;
    case 7:
      lcd.setCursor(0,2);
      lcd.print("  ");
      lcd.setCursor(0,3);
      lcd.print("->");
      if (push == 1){
        delay(50);
        lcd.clear();
        state = INFO_SCREEN;
        selectorPosition = 0;
      } 
    break;              
  } 
}

void readEncoder(){
  long newPosition = encoder.read();
  if (newPosition != oldPosition) {
    if (newPosition > oldPosition+3){
    dir = 2;
    oldPosition = newPosition;
    } 
  if (newPosition < oldPosition-3) {
    dir = 1;
    oldPosition = newPosition;
    }  
  }
  else {
    dir = 0;
  }
}

void buttonPress(){
  int reading;           // the current reading from the input pin
  int previous = 1; 
  long time = 0;         // the last time the output pin was toggled
  long debounce = 200;   // the debounce time, increase if the output flickers  
  
  reading = digitalRead(buttonPin);

  if (reading == LOW && previous == HIGH && millis() - time > debounce) {

    push = 1;
    time = millis();    
  }
  else {
    push = 0;
  }
  previous = reading;
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
  pinMode(A3, INPUT_PULLUP);  // encoder button
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

  bool infoScreenDrawn;
  bool menuDrawn;
  readEncoder();
  buttonPress();
  if (dir == 1) selectorPosition++;
  if (dir == 2) selectorPosition--;
  if (selectorPosition < 0) selectorPosition = 0;
  if (selectorPosition > 7) selectorPosition = 7;

  switch (state){
    case INFO_SCREEN:
      if (infoScreenDrawn == false){
        drawStaticInfoScreen();
        infoScreenDrawn = true;
      }
      if (millis() - timing > 1000){ // read & display data on screen every second
        timing = millis();
        readSensors();
        displayTimeDate();
        updateInfoScreen();
      }
      if (push == 1){
        delay(50);
        lcd.clear();
        infoScreenDrawn = false;
        state = MENU1;
      } 
    break;
 
    case MENU1:
      Selector();
      if (menuDrawn == false){
        drawStaticMenu();
        menuDrawn = true;
      }
      if (selectorPosition > 3){
        delay(50);
        lcd.clear();
        menuDrawn = false;
        state = MENU2;
      }  
    break;

    case MENU2:
      Selector();
      if (menuDrawn == false){
        drawStaticMenu();
        menuDrawn = true;
      }   
      if (selectorPosition < 4){
        delay(50);
        lcd.clear();
        menuDrawn = false;
        state = MENU1;
      }   
    break;

    case M_PROGRAM:
    break;
  }
  digitalWrite(relayApin, relayAStat); //Set lights to off
  digitalWrite(relayBpin, relayBStat);
  
}


