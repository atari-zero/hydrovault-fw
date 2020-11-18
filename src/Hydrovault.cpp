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
//#include <Adafruit_BME280.h>
#include <Buzzer.h>
#include <Encoder.h>
#include <EEPROM.h>

//Arduino Nano pin definitions

const int relayApin = A0;
const int relayBpin = A1;
const int sensorPower = A2;
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

struct eepromData
{
  int storedPrg;
  int storedDay;
  int storedRpd;
  bool storedProgStarted;
  bool storedRelayA;
  bool storedRelayB;
  bool storedAuxA;
  bool storedAuxB;
  long storedDayCount;
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
bool programStarted;
unsigned long timing;     //timing for delays
unsigned long timingSensors;
unsigned long sensorWatchDogTimer;
int selectorPosition = 0;
int selectedPogram;
long oldPosition  = -999;
int eeAddress = 0;
unsigned long dayCount;
byte dir;
byte push;

Encoder encoder(2, 3);
LiquidCrystal_I2C lcd(0x27,20,4);   // LCD 2004 I2C LCD address 0x27; 20 chars; 4 line display
TMC2130Stepper driver = TMC2130Stepper(LEDPin, dirPin, stepPin, CSPin);  //TMC2130 Stepper
AccelStepper stepper = AccelStepper(stepper.DRIVER, stepPin, dirPin); //Accel Stepper Library
RTC_DS1307 rtc; //Real Time Clock DS3107+
Buzzer buzzer(buzzerPin); //Buzzer
//Adafruit_BME280 bme;
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
  tmp = am2320.readTemperature(); //bme.readTemperature(); //
  hum = am2320.readHumidity(); //bme.readHumidity(); //
  sensors_event_t event;
  tsl.getEvent(&event);
  lux = event.light;
  if (hum == 0 && tmp == 0 && (millis() - sensorWatchDogTimer > 10000)){
    digitalWrite (sensorPower, LOW);
    delay(500);
    digitalWrite (sensorPower, HIGH);
  }
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
  lcd.print(day);
  lcd.setCursor(14,0);
  lcd.print(rpd);
  lcd.setCursor(14,1);
  lcd.print(tmp); lcd.print((char)223);
  lcd.setCursor(14,2);
  lcd.print(hum); lcd.print("%");
  lcd.setCursor(14,3);
  lcd.print("      ");
  lcd.setCursor(14,3);
  lcd.print(lux);
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
    lcd.print("Program Selection:");
    lcd.setCursor(2,1);
    lcd.print("Name: ");
    lcd.setCursor(2,2);
    lcd.print("Restart Program!");
    lcd.setCursor(2,3);
    lcd.print("Exit");
  }
  if (state == M_SPEED){
    lcd.setCursor(2,0);
    lcd.print("Rotation Speed:");
    lcd.setCursor(2,1);
    lcd.print("Revs/Day: ");
    lcd.print(rpd);
    lcd.setCursor(2,2);
    lcd.print("Change");
    lcd.setCursor(2,3);
    lcd.print("Exit");
  }
  if (state == M_LIGHT_A){
    lcd.setCursor(2,0);
    lcd.print("Light A Control:");
    lcd.setCursor(2,1);
    lcd.print("Status: ");
    lcd.setCursor(2,2);
    lcd.print("Set Schedule:");
    lcd.setCursor(2,3);
    lcd.print("Exit");
  }
  if (state == M_LIGHT_B){
    lcd.setCursor(2,0);
    lcd.print("Light B Control:");
    lcd.setCursor(2,1);
    lcd.print("Status: ");
    lcd.setCursor(2,2);
    lcd.print("Set Schedule:");
    lcd.setCursor(2,3);
    lcd.print("Exit");
  }
  if (state == M_AUX_1){
    lcd.setCursor(2,0);
    lcd.print("Aux12V A Control:");
    lcd.setCursor(2,1);
    lcd.print("Status: ");
    lcd.setCursor(2,2);
    lcd.print("Set Schedule:");
    lcd.setCursor(2,3);
    lcd.print("Exit");
  }
  if (state == M_AUX_2){
    lcd.setCursor(2,0);
    lcd.print("Aux12V B Control:");
    lcd.setCursor(2,1);
    lcd.print("Status: ");
    lcd.setCursor(2,2);
    lcd.print("Set Schedule:");
    lcd.setCursor(2,3);
    lcd.print("Exit");
  }
  if (state == M_EEPROM){
    lcd.setCursor(2,0);
    lcd.print("EEPROM Settings:");
    lcd.setCursor(2,1);
    lcd.print("Store to EEPROM");
    lcd.setCursor(2,2);
    lcd.print("Clear EEPROM");
    lcd.setCursor(2,3);
    lcd.print("Exit");
  }
}

void drawSelector(){ 
  if (dir == 1) selectorPosition++;
  if (dir == 2) selectorPosition--;
  if (selectorPosition < -1) selectorPosition = 0;
  if (selectorPosition > 4) selectorPosition = 3;
  if (state != INFO_SCREEN){
    if (selectorPosition == 0) {
      lcd.setCursor(0,0);
      lcd.print(">>");
      lcd.setCursor(0,1);
      lcd.print("  ");
    }

    if (selectorPosition == 1) {
      lcd.setCursor(0,0);
      lcd.print("  ");
      lcd.setCursor(0,1);
      lcd.print(">>");
      lcd.setCursor(0,2);
      lcd.print("  ");
    }

    if (selectorPosition == 2) {
      lcd.setCursor(0,1);
      lcd.print("  ");
      lcd.setCursor(0,2);
      lcd.print(">>");
      lcd.setCursor(0,3);
      lcd.print("  ");
    }

    if (selectorPosition == 3) {
      lcd.setCursor(0,2);
      lcd.print("  ");
      lcd.setCursor(0,3);
      lcd.print(">>");
    }
  }
}

void clearSelector(){
  lcd.setCursor(0,0);
  lcd.print("  ");
  lcd.setCursor(0,1);
  lcd.print("  ");
  lcd.setCursor(0,2);
  lcd.print("  ");
  lcd.setCursor(0,3);
  lcd.print("  "); 
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
  static int previous = HIGH; 
  long time = 0;         // the last time the output pin was toggled
  long debounce = 200;   // the debounce time, increase if the output flickers  
  
  reading = digitalRead(buttonPin);

  if (reading == LOW && previous == HIGH && (millis() - time) > debounce) {
    push = 1;
    time = millis();    
  }
  else {
    push = 0;
  }
  previous = reading;
}

void beep(){
  buzzer.begin(100);
  buzzer.sound(NOTE_G7, 100);
  buzzer.end(100);
}

void beep2(){
  buzzer.begin(100);
  buzzer.sound(NOTE_B6, 100);
  buzzer.end(100);
}

void beepConfirm(){
  buzzer.begin(100);
  buzzer.sound(NOTE_GS4, 60);
  buzzer.sound(0, 10);
  buzzer.sound(NOTE_GS6, 80);
  buzzer.sound(0, 10);
  buzzer.sound(NOTE_GS5, 160);
  buzzer.sound(0, 10);
}

void clearEEPROM(){
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
}

void writeToEEPROM(){
  eepromData HVData = {
    prg,
    day,
    rpd,
    programStarted,
    relayAStat,
    relayBStat,
    auxAStat,
    auxBStat,
    dayCount
  };
  EEPROM.put(eeAddress, HVData);
}

void readFromEEPROM(){
  eepromData HVData;
  EEPROM.get(eeAddress, HVData);
  prg = HVData.storedPrg;
  day = HVData.storedDay;
  rpd = HVData.storedRpd;
  programStarted = HVData.storedProgStarted;
  relayAStat = HVData.storedRelayA;
  relayBStat = HVData.storedRelayB;
  auxAStat = HVData.storedRelayA;
  auxBStat = HVData.storedRelayB;
  dayCount = HVData.storedDayCount;
}

void runProgram(){
  digitalWrite(relayApin, relayAStat);
  digitalWrite(relayBpin, relayBStat);
  digitalWrite(vOutApin, auxAStat);
  digitalWrite(vOutBpin, auxBStat);
  if (programStarted == true){
    switch (prg){
      case 1 :
      {
      DateTime now = rtc.now();
      if (now.hour() > 6 && now.hour() < 23){
        relayAStat = true;
        relayBStat = true;
      }
      else {
        relayAStat = false;
        relayBStat = false;
      }
      day = (((now.unixtime() - dayCount) / 86400L) + 1);
      //if ((dayCount + 86400L) == now.unixtime()){
      //  day++;
      //  dayCount = now.unixtime();
      //  writeToEEPROM();
      //}
      break;
      }
      case 2 :
      {
      break;}
      case 3 :
      {
      break;}
    }
  }
}

void setup() {

  digitalWrite(sensorPower, HIGH);

  SPI.begin();
  Wire.begin();
  Serial.begin(9600);
  rtc.begin();         // initialize clock
  lcd.init();          // initialize the lcd 
  lcd.backlight();     // turn on lcd backlight 
  am2320.begin();      // temp+humidity sensor init  
  //bme.begin();  
  tsl.begin();         // luminosity sensor init

  setupMotor();
  setupTimer1();
  setupTSL();  
  setupRTC();
  
  pinMode(A0, OUTPUT); // light relay A
  pinMode(A1, OUTPUT); // light relay B
  pinMode(A2, OUTPUT); //AM2320 Power
  pinMode(A3, INPUT_PULLUP);  // encoder button
  pinMode(A6, INPUT);  // potentiometer 1
  pinMode(A7, INPUT);  // potentiometer 2
  pinMode(2, INPUT);   // encoder interrupt A
  pinMode(3, INPUT);   // encoder interrrupt B
  pinMode(4, OUTPUT);  // buzzer
  pinMode(5, OUTPUT);  // 12V Mosfet-regulated terminal A
  pinMode(6, OUTPUT);  // 12V Mosfet-regulated terminal B
  pinMode(9, OUTPUT);  // just a LED

  readFromEEPROM();

  //DateTime now = rtc.now();
  //if (now.unixtime() > dayCount + 86400) dayCount = now.unixtime();

  welcomeScreen();     //Show welcome screen
}

void loop() {

  bool infoScreenDrawn;
  bool menuDrawn;
  readEncoder();
  buttonPress();
  runProgram();

  switch (state){
    case INFO_SCREEN :
      if (infoScreenDrawn == false){
        drawStaticInfoScreen();
        selectorPosition = 0;
        infoScreenDrawn = true;
      }
      if (millis() - timingSensors > 6000) {
        readSensors();
        timingSensors = millis();
      }
      if (millis() - timing > 1000){ // read & display data on screen every second
        timing = millis();
        displayTimeDate();       
        updateInfoScreen();
      }
      if (push == 1){
        delay(50);
        beep();
        lcd.clear();
        infoScreenDrawn = false;
        state = MENU1;
      } 
      break;
 
    case MENU1 :
      drawSelector();
      if (menuDrawn == false){
        drawStaticMenu();
        menuDrawn = true;
      }
      if (selectorPosition == 4){
        if (dir == 1){
          delay(50);
          lcd.clear();
          menuDrawn = false;
          state = MENU2;
          selectorPosition = 0;         
        }
      }
      if (push == 1) lcd.clear();
      if (push == 1) beep();
      if (push == 1 && selectorPosition == 0) state = M_PROGRAM;
      if (push == 1 && selectorPosition == 1) state = M_SPEED;
      if (push == 1 && selectorPosition == 2) state = M_LIGHT_A;
      if (push == 1 && selectorPosition == 3) state = M_LIGHT_B;
      break;

    case MENU2 :
      drawSelector();
      if (menuDrawn == false){
        drawStaticMenu();
        menuDrawn = true;
      }   
      if (selectorPosition == -1){
        if (dir == 2){
          delay(50);
          lcd.clear();
          menuDrawn = false;
          state = MENU1;
          selectorPosition = 3;
        }
      } 
      if (push == 1) lcd.clear();
      if (push == 1) beep();
      if (push == 1 && selectorPosition == 0) state = M_AUX_1; 
      if (push == 1 && selectorPosition == 1) state = M_AUX_2;
      if (push == 1 && selectorPosition == 2) state = M_EEPROM;
      if (push == 1 && selectorPosition == 3) state = INFO_SCREEN;  
      break;

    case M_PROGRAM :
      drawSelector();
      if (menuDrawn == false){
        drawStaticMenu();
        menuDrawn = true;
      }
      if(programStarted == true){
        lcd.setCursor(8,1);
        lcd.print(prg);
        lcd.setCursor(11,1);
        lcd.print(" Running!");  
      }
      if(programStarted == false){
        lcd.setCursor(11,1);
        lcd.print("         ");  
      }
      if (push == 1){
        switch (selectorPosition){
          case 1 :
            {
            beep2();
            programStarted = false;
            lcd.setCursor(8,1);
            lcd.print("  ");
            selectedPogram++;
            if (selectedPogram > 3) selectedPogram = 1;
            lcd.setCursor(8,1);
            lcd.print(selectedPogram);
            break;
            }
          case 2 :
            {
            beepConfirm();
            programStarted = true;
            //day = 1;
            prg = selectedPogram; 
            DateTime now = rtc.now();
            dayCount = now.unixtime();
            lcd.setCursor(8,1);
            lcd.print("  ");
            lcd.setCursor(8,1);
            lcd.print(prg);          
            break;
            }
          case 3:
            {  
            beep();
            lcd.clear();
            state = MENU1;
            selectorPosition = 1;
            break; 
            }       
        }
      }         
    break;

    case M_SPEED :
      drawSelector();
      static int setRpd;
      if (menuDrawn == false){
        drawStaticMenu();
        menuDrawn = true;
      }
      if (setRpd != rpd){
        lcd.setCursor(12,1);
        lcd.print("  ");
        lcd.setCursor(12,1);
        lcd.print(setRpd);
      }
      if (setRpd == rpd){
        lcd.setCursor(12,1);
        lcd.print("  ");
        lcd.setCursor(12,1);
        lcd.print(rpd);
      }
      if (push == 1){
        switch (selectorPosition){
          case 1 :
            beep2();
            setRpd++;
            if (setRpd > 48) setRpd = 1;
            lcd.setCursor(12,1);
            lcd.print("  ");
            lcd.setCursor(12,1);
            lcd.print(setRpd); 
            break;
          case 2 :
            beepConfirm();
            rpd = setRpd;
            break;
          case 3:
            beep();
            lcd.clear();
            state = MENU1;
            selectorPosition = 1;
            break;       
        }
      }    
      break;

    case M_LIGHT_A :
      drawSelector();
      if (menuDrawn == false){
        drawStaticMenu();
        menuDrawn = true;
      }
      if (relayAStat == true){
        lcd.setCursor(10,1);
        lcd.print("ON ");
      }
      if (relayAStat == false){
        lcd.setCursor(10,1);
        lcd.print("OFF");
      }
        if (push == 1){
          switch (selectorPosition){
            case 1 :
              beep2();
              if (relayAStat == true) relayAStat = false;
              else {relayAStat = true;}
              break;
            case 2 :
              break;
            case 3:
              beep();
              lcd.clear();
              state = MENU1;
              selectorPosition = 2;
              break;
          }
        }  
      break;

    case M_LIGHT_B :
      drawSelector();
      if (menuDrawn == false){
        drawStaticMenu();
        menuDrawn = true;
      }
      if (relayBStat == true){
        lcd.setCursor(10,1);
        lcd.print("ON ");
      }
      if (relayBStat == false){
        lcd.setCursor(10,1);
        lcd.print("OFF");
      }
        if (push == 1){
          switch (selectorPosition){
            case 1 :
              beep2();
              if (relayBStat == true) relayBStat = false;
              else {relayBStat = true;}
              break;
            case 2 :
              break;
            case 3:
              beep();
              lcd.clear();
              state = MENU1;
              selectorPosition = 3;
              break;
          }
        }  
      break;

    case M_AUX_1 :
      drawSelector();
      if (menuDrawn == false){
        drawStaticMenu();
        menuDrawn = true;
      }
      if (auxAStat == true){
        lcd.setCursor(10,1);
        lcd.print("ON ");
      }
      if (auxAStat == false){
        lcd.setCursor(10,1);
        lcd.print("OFF");
      }
        if (push == 1){
          switch (selectorPosition){
            case 1 :
              beep2();
              if (auxAStat == true) auxAStat = false;
              else {auxAStat = true;}
              break;
            case 2 :
              break;
            case 3:
              beep();
              lcd.clear();
              state = MENU2;
              selectorPosition = 0;
              break;
          }
        }  
      break;

    case M_AUX_2 :
      drawSelector();
      if (menuDrawn == false){
        drawStaticMenu();
        menuDrawn = true;
      }
      if (auxBStat == true){
        lcd.setCursor(10,1);
        lcd.print("ON ");
      }
      if (auxBStat == false){
        lcd.setCursor(10,1);
        lcd.print("OFF");
      }
        if (push == 1){
          switch (selectorPosition){
            case 1 :
              beep2();
              if (auxBStat == true) auxBStat = false;
              else {auxBStat = true;}
              break;
            case 2 :
              break;
            case 3:
              beep();
              lcd.clear();
              state = MENU2;
              selectorPosition = 1;
              break;
          }
        }  
      break;

    case M_EEPROM :
      drawSelector();
      if (menuDrawn == false){
        drawStaticMenu();
        menuDrawn = true;
        if (push == 1){
          switch (selectorPosition){
            case 1 :
              writeToEEPROM();
              beepConfirm();
              break;
            case 2 :
              clearEEPROM();
              beepConfirm();
              break;
            case 3:
              beep();
              lcd.clear();
              state = MENU2;
              selectorPosition = 2;
              break;
          }
        } 
      } 
      break;
  }
}


