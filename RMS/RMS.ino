#include "sensitiveInformation.h"
#include <Wire.h>
#define FORMAT_SPIFFS_IF_FAILED true
#include "Adafruit_ADT7410.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include "Adafruit_miniTFTWing.h"


// Wifi & Webserver
#include "WiFi.h"
#include "SPIFFS.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include <Adafruit_MotorShield.h>
bool fanEnabled = false;            // If the fan is on or off.
bool automaticFanControl = true;    // Automatic or manual control

#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(4);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

AsyncWebServer server(80);

// ESP32Servo Start
#include <ESP32Servo.h>
Servo myservo;  // create servo object to control a servo
int servoPin = 12;
boolean blindsOpen = false;
// ESP32Servo End

//RFID
#include <SPI.h>
#include <MFRC522.h>

//LED
#define LEDRed 27
#define LEDGreen 33

// RTC Start - Remove if unnecessary
#include "RTClib.h"

RTC_PCF8523 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// RTC End

boolean LEDOn = false; // State of Built-in LED true=on, false=off.
#define LOOPDELAY 100

Adafruit_miniTFTWing ss;
#define TFT_RST    -1    // we use the seesaw for resetting to save a pin
#define TFT_CS   14
#define TFT_DC 32

// Create the ADT7410 temperature sensor object
Adafruit_ADT7410 tempsensor = Adafruit_ADT7410();
float fanTemperatureThreshold = 25.0;


Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);
//RFID
#define SS_PIN  21  // ES32 Feather
#define RST_PIN 17 // esp32 Feather. Could be others.

MFRC522 rfid(SS_PIN, RST_PIN);
bool safeLocked = true;


void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(10);
  }
  delay(1000);

  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    // Follow instructions in README and install
    // https://github.com/me-no-dev/arduino-esp32fs-plugin
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  // Wifi Configuration
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
    //tftDrawText("Connecting to WiFi..", ST77XX_RED);
  }
 // tft.fillScreen(ST77XX_BLACK);
  Serial.println();
  Serial.print("Connected to the Internet");
  Serial.print("IP address: ");
  // logEvent("Connected to Wifi");
  //String ip = WiFi.localIP().toString();
  Serial.println(WiFi.localIP());
   

  routesConfiguration(); // Reads routes from routesManagement

  server.begin();

  // RTC
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    //    abort();
  }

  // The following line can be uncommented if the time needs to be reset.
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  rtc.start();
  pinMode(LED_BUILTIN, OUTPUT);

  if (!ss.begin()) {
    Serial.println("seesaw init error!");
    while (1);
  }
  else Serial.println("seesaw started");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(255);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);

  //RFID
  SPI.begin(); // init SPI bus
  rfid.PCD_Init(); // init MFRC522
  pinMode(LEDRed, OUTPUT);
  pinMode(LEDGreen, OUTPUT);
  digitalWrite(LEDRed, LOW);
  digitalWrite(LEDGreen, LOW);

  ss.tftReset();
  ss.setBacklight(0x0); //set the backlight fully on

  // Use this initializer (uncomment) if you're using a 0.96" 180x60 TFT
  tft.initR(INITR_MINI160x80);   // initialize a ST7735S chip, mini display

  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);

  // ESP32Servo Start
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object
  // ESP32Servo End

  Serial.println("ADT7410 demo");

  // Make sure the sensor is found, you can also pass in a different i2c
  // address with tempsensor.begin(0x49) for example
  if (!tempsensor.begin()) {
    Serial.println("Couldn't find ADT7410!");
    while (1);
  }

  // sensor takes 250 ms to get first readings
  delay(250);
}


void loop() {
  builtinLED();
  rfidRead();
  tftDrawText(tempetureSens(), ST77XX_WHITE);
  fanControl();
  windowBlinds();
  safeStatusDisplay();
  safeLockout(); 
  tempetureSens();
  delay(LOOPDELAY); // To allow time to publish new code.
}


void builtinLED() {
  if (LEDOn) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void logEvent(String dataToLog) {
  /*
     Log entries to a file stored in SPIFFS partition on the ESP32.
  */
  // Get the updated/current time
  DateTime rightNow = rtc.now();
  char csvReadableDate[25];
  sprintf(csvReadableDate, "%02d,%02d,%02d,%02d,%02d,%02d,",  rightNow.year(), rightNow.month(), rightNow.day(), rightNow.hour(), rightNow.minute(), rightNow.second());

  String logTemp = csvReadableDate + dataToLog + "\n"; // Add the data to log onto the end of the date/time

  const char * logEntry = logTemp.c_str(); //convert the logtemp to a char * variable

  //Add the log entry to the end of logevents.csv
  appendFile(SPIFFS, "/logEvents.csv", logEntry);

  // Output the logEvents - FOR DEBUG ONLY. Comment out to avoid spamming the serial monitor.
  //  readFile(SPIFFS, "/logEvents.csv");

  Serial.print("\nEvent Logged: ");
  Serial.println(logEntry);
}

void tftDrawText(String text, uint16_t color) {
  
 // Draws the temperture and the IP address
  tft.setCursor(0, 0);
  tft.setTextSize(3);
  tft.setTextColor(color, ST77XX_BLACK);
  tft.setTextWrap(true);
  tft.print(text);
   tft.setCursor(0, 60);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextWrap(true);
  tft.print(WiFi.localIP());
}
// this function will allow for manual or automatic fan control from the website.
void automaticFan(float temperatureThreshold ) {
  float c = tempsensor.readTempC();
  myMotor->setSpeed(255);
  if (c < temperatureThreshold) {
 fanEnabled = false;
  } else {
    myMotor->run(FORWARD);
 fanEnabled = true;
  }


}
//this changes the fan from automatic to manual fan control
void fanControl() {
  if (automaticFanControl) {
    automaticFan(fanTemperatureThreshold);
  }
  if (fanEnabled) {
    myMotor->run(FORWARD);
  } else {
    myMotor->run(RELEASE);
  }
}

//uses the buttons on the LCD monitor to open and close the curtins 
void windowBlinds() {
  uint32_t buttons = ss.readButtons();
  if (! (buttons & TFTWING_BUTTON_A)) {
    if (blindsOpen) {
      myservo.write(0);
      logEvent("Blinds Closed");
    } else {
      myservo.write(180);
      logEvent("Blinds Open");
    }
    blindsOpen = !blindsOpen;
  }
}
// Takes the temperature
String tempetureSens() {

  // Read and print out the temperature, then convert to *F
  float c = tempsensor.readTempC();
  float f = c * 9.0 / 5.0 + 32;
  //Serial.print("Temp: "); Serial.print(c); Serial.print("*C\t");
  //Serial.print(f); Serial.println("*F");
  String tempC = String(c);
 // String tempInC = String(c);
 // tempInC += " C"; //shows degrees 
 // tftDrawText(tempInC, ST77XX_WHITE);
return tempC;
  delay(100);

}
// uses the rfid reader module to read the rfid cards and prints out their card ID
void rfidRead() {

  String uidOfCardRead = "";
  String validCardUID = "54 59 179 115";

  if (rfid.PICC_IsNewCardPresent()) { // new tag is available
    if (rfid.PICC_ReadCardSerial()) { // NUID has been readed
      MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
      for (int i = 0; i < rfid.uid.size; i++) {
        uidOfCardRead += rfid.uid.uidByte[i] < 0x10 ? " 0" : " ";
        uidOfCardRead += rfid.uid.uidByte[i];
      }
      Serial.println(uidOfCardRead);

      rfid.PICC_HaltA(); // halt PICC
      rfid.PCD_StopCrypto1(); // stop encryption on PCD
      uidOfCardRead.trim();
      if (uidOfCardRead == validCardUID) {
        safeLocked = false;
        logEvent("Safe Unlocked");
       
      } else {
        safeLocked = true;
        logEvent("Safe Locked");

      }
    }
  }

}
//this updates the trffic light module to display if the safe is locked or not.
void safeStatusDisplay() {
  /*
     Outputs the status of the Safe Lock to the LEDS
     Red LED = Locked
     Green LED = Unlocked.
  */
  if (safeLocked) {
    digitalWrite(LEDRed, HIGH);
    digitalWrite(LEDGreen, LOW);
  } else {
    digitalWrite(LEDRed, LOW);
    digitalWrite(LEDGreen, HIGH);
  }
}

//Make the safe turn off automaticly after a certain amount of time has passed
bool safeLockout(){
  unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 1000;           // interval at which to blink (milliseconds)

// here is where you'd put code that needs to be running all the time.

  // check to see if it's time to blink the LED; that is, if the difference
  // between the current time and last time you blinked the LED is bigger than
  // the interval at which you want to blink the LED.
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (safeLocked == false) {
      safeLocked = false;
    } else {
      safeLocked = true;
    }

//     set the LED with the ledState of the variable:
    digitalWrite(safeLocked, safeLocked);
  }
return safeLocked ;

  
}
