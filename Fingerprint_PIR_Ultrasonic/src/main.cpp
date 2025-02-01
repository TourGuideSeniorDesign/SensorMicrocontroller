#include <Arduino.h>
#include <Adafruit_Fingerprint.h>
#include "DFRobot_URM09.h"

#if (defined(__AVR__) || defined(ESP8266)) && !defined(__AVR_ATmega2560__)
// pin #2 is IN from sensor (GREEN wire)
// pin #3 is OUT from arduino  (WHITE wire)
// Set up the serial port to use softwareserial..
SoftwareSerial mySerial(0, 1);

#else
// On Leonardo/M0/etc, others with hardware serial, use hardware serial!
// #0 is green wire, #1 is white
#define mySerial Serial1

#endif

#define  URM09_MAX_RANGE      (520)//the max measurement vaule of the module is 520cm(a little bit longer than  effective max range)
#define  URM09_ADC_SOLUTION      (1023.0)//ADC accuracy of Arduino UNO is 10bit

/*
 *  Fingerprint Functions Start
 */

Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);
int redLED = 15;          // the pin that the red LED is attached to

uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      //Serial.println("No finger detected");
      digitalWrite(redLED, LOW);   // turn LED ON
      delay(10);
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      digitalWrite(redLED, LOW);   // turn LED ON
      delay(10);
      return p;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      digitalWrite(redLED, LOW);   // turn LED ON
      delay(10);
      return p;
    default:
      Serial.println("Unknown error");
      digitalWrite(redLED, LOW);   // turn LED ON
      delay(10);
      return p;
  }
  // OK success!
  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }
  // OK converted!
  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
    Serial.println("Found a print match!");
    digitalWrite(redLED, HIGH);   // turn LED ON
    delay(250);
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_NOTFOUND) {
    Serial.println("Did not find a match");
    return p;
  } else {
    Serial.println("Unknown error");
    return p;
  }

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);

  return finger.fingerID;
}

void setupFingerprint()
{
  while (!Serial);  // For Yun/Leo/Micro/Zero/...
  delay(100);
  Serial.println("\n\nAdafruit finger detect test");

  // set the data rate for the sensor serial port
  finger.begin(57600);
  delay(5);
  if (finger.verifyPassword()) {
    Serial.println("Found fingerprint sensor!");
  } else {
    Serial.println("Did not find fingerprint sensor :(");
    while (1) { delay(1); }
  }

  Serial.println(F("Reading sensor parameters"));
  finger.getParameters();
  Serial.print(F("Status: 0x")); Serial.println(finger.status_reg, HEX);
  Serial.print(F("Sys ID: 0x")); Serial.println(finger.system_id, HEX);
  Serial.print(F("Capacity: ")); Serial.println(finger.capacity);
  Serial.print(F("Security level: ")); Serial.println(finger.security_level);
  Serial.print(F("Device address: ")); Serial.println(finger.device_addr, HEX);
  Serial.print(F("Packet len: ")); Serial.println(finger.packet_len);
  Serial.print(F("Baud rate: ")); Serial.println(finger.baud_rate);

  finger.getTemplateCount();

  if (finger.templateCount == 0) {
    Serial.print("Sensor doesn't contain any fingerprint data. Please run the 'enroll' example.");
  }
  else {
    Serial.println("Waiting for valid finger...");
    Serial.print("Sensor contains "); Serial.print(finger.templateCount); Serial.println(" templates");
  }
}

void loopFingerprint()                     // run over and over again
{
  getFingerprintID();
  delay(50);            //don't ned to run this at full speed.
}

/*
 *  Fingerprint Functions End
 */

/*
 *  PIR Functions Start
 */
int blueLED = 14;          // the pin that the blue LED is attached to
int PIRout = 22;             // the pin that the sensor is attached to
int PIRstate = LOW;             // by default, no motion detected
int PIRval = 0;                 // variable to store the sensor status (value)

void setupPIR()
{
  pinMode(blueLED, OUTPUT);      // initialize LED as an output
  pinMode(PIRout, INPUT);    // initialize sensor as an input
  Serial.println("PIR detected");
}

void loopPIR(){
  PIRval = digitalRead(PIRout);   // read sensor value
  if (PIRval == HIGH) {           // check if the sensor is HIGH
    digitalWrite(blueLED, HIGH);   // turn LED ON
    delay(10);

    if (PIRstate == LOW) {
      Serial.println("Motion detected!");
      PIRstate = HIGH;       // update variable state to HIGH
    }
  }
  else {
    digitalWrite(blueLED, LOW); // turn LED OFF
    delay(10);

    if (PIRstate == HIGH){
      Serial.println("Motion stopped!");
      PIRstate = LOW;       // update variable state to LOW
    }
  }
}

/*
 *  PIR Functions End
 */

/*
 *  Ultrasonic Functions Start
 */

DFRobot_URM09 URM09;
int URM09Pin = A0;    // select the input pin
int sensor_t;
double dist_t;
void loopUltrasonic() {
  // read the value from the sensor:
  sensor_t = analogRead(URM09Pin);
  // turn the ledPin on
  dist_t = sensor_t * URM09_MAX_RANGE  / URM09_ADC_SOLUTION;//
  Serial.print(dist_t,0);
  Serial.println("cm");
  delay(10);
}

/*
 *  Ultrasonic Functions End
 */

// main functions go here
void setup() {
  Serial.begin(9600);
  setupFingerprint();
  setupPIR();
}

void loop() {
  loopFingerprint();
  loopPIR();
  loopUltrasonic();
}

// Enroll Fingerprint Functions
// uint8_t idEnroll;
//
// uint8_t getFingerprintEnroll() {
//   int p = -1;
//   Serial.print("Waiting for valid finger to enroll as #"); Serial.println(idEnroll);
//   while (p != FINGERPRINT_OK) {
//     p = finger.getImage();
//     switch (p) {
//     case FINGERPRINT_OK:
//       Serial.println("Image taken");
//       break;
//     case FINGERPRINT_NOFINGER:
//       Serial.print(".");
//       break;
//     case FINGERPRINT_PACKETRECIEVEERR:
//       Serial.println("Communication error");
//       break;
//     case FINGERPRINT_IMAGEFAIL:
//       Serial.println("Imaging error");
//       break;
//     default:
//       Serial.println("Unknown error");
//       break;
//     }
//   }
//
//   // OK success!
//
//   p = finger.image2Tz(1);
//   switch (p) {
//     case FINGERPRINT_OK:
//       Serial.println("Image converted");
//       break;
//     case FINGERPRINT_IMAGEMESS:
//       Serial.println("Image too messy");
//       return p;
//     case FINGERPRINT_PACKETRECIEVEERR:
//       Serial.println("Communication error");
//       return p;
//     case FINGERPRINT_FEATUREFAIL:
//       Serial.println("Could not find fingerprint features");
//       return p;
//     case FINGERPRINT_INVALIDIMAGE:
//       Serial.println("Could not find fingerprint features");
//       return p;
//     default:
//       Serial.println("Unknown error");
//       return p;
//   }
//
//   Serial.println("Remove finger");
//   delay(2000);
//   p = 0;
//   while (p != FINGERPRINT_NOFINGER) {
//     p = finger.getImage();
//   }
//   Serial.print("ID "); Serial.println(idEnroll);
//   p = -1;
//   Serial.println("Place same finger again");
//   while (p != FINGERPRINT_OK) {
//     p = finger.getImage();
//     switch (p) {
//     case FINGERPRINT_OK:
//       Serial.println("Image taken");
//       break;
//     case FINGERPRINT_NOFINGER:
//       Serial.print(".");
//       break;
//     case FINGERPRINT_PACKETRECIEVEERR:
//       Serial.println("Communication error");
//       break;
//     case FINGERPRINT_IMAGEFAIL:
//       Serial.println("Imaging error");
//       break;
//     default:
//       Serial.println("Unknown error");
//       break;
//     }
//   }
//
//   // OK success!
//
//   p = finger.image2Tz(2);
//   switch (p) {
//     case FINGERPRINT_OK:
//       Serial.println("Image converted");
//       break;
//     case FINGERPRINT_IMAGEMESS:
//       Serial.println("Image too messy");
//       return p;
//     case FINGERPRINT_PACKETRECIEVEERR:
//       Serial.println("Communication error");
//       return p;
//     case FINGERPRINT_FEATUREFAIL:
//       Serial.println("Could not find fingerprint features");
//       return p;
//     case FINGERPRINT_INVALIDIMAGE:
//       Serial.println("Could not find fingerprint features");
//       return p;
//     default:
//       Serial.println("Unknown error");
//       return p;
//   }
//
//   // OK converted!
//   Serial.print("Creating model for #");  Serial.println(idEnroll);
//
//   p = finger.createModel();
//   if (p == FINGERPRINT_OK) {
//     Serial.println("Prints matched!");
//   } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
//     Serial.println("Communication error");
//     return p;
//   } else if (p == FINGERPRINT_ENROLLMISMATCH) {
//     Serial.println("Fingerprints did not match");
//     return p;
//   } else {
//     Serial.println("Unknown error");
//     return p;
//   }
//
//   Serial.print("ID "); Serial.println(idEnroll);
//   p = finger.storeModel(idEnroll);
//   if (p == FINGERPRINT_OK) {
//     Serial.println("Stored!");
//   } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
//     Serial.println("Communication error");
//     return p;
//   } else if (p == FINGERPRINT_BADLOCATION) {
//     Serial.println("Could not store in that location");
//     return p;
//   } else if (p == FINGERPRINT_FLASHERR) {
//     Serial.println("Error writing to flash");
//     return p;
//   } else {
//     Serial.println("Unknown error");
//     return p;
//   }
//
//   return true;
// }
//
// void setupEnroll()
// {
//   Serial.begin(9600);
//   while (!Serial);  // For Yun/Leo/Micro/Zero/...
//   delay(100);
//   Serial.println("\n\nAdafruit Fingerprint sensor enrollment");
//
//   // set the data rate for the sensor serial port
//   finger.begin(57600);
//
//   if (finger.verifyPassword()) {
//     Serial.println("Found fingerprint sensor!");
//   } else {
//     Serial.println("Did not find fingerprint sensor :(");
//     while (1) { delay(1); }
//   }
//
//   Serial.println(F("Reading sensor parameters"));
//   finger.getParameters();
//   Serial.print(F("Status: 0x")); Serial.println(finger.status_reg, HEX);
//   Serial.print(F("Sys ID: 0x")); Serial.println(finger.system_id, HEX);
//   Serial.print(F("Capacity: ")); Serial.println(finger.capacity);
//   Serial.print(F("Security level: ")); Serial.println(finger.security_level);
//   Serial.print(F("Device address: ")); Serial.println(finger.device_addr, HEX);
//   Serial.print(F("Packet len: ")); Serial.println(finger.packet_len);
//   Serial.print(F("Baud rate: ")); Serial.println(finger.baud_rate);
// }
//
// uint8_t readnumber() {
//   uint8_t num = 0;
//
//   while (num == 0) {
//     while (! Serial.available());
//     num = Serial.parseInt();
//   }
//   return num;
// }
//
// void loopEnroll()                     // run over and over again
// {
//   Serial.println("Ready to enroll a fingerprint!");
//   Serial.println("Please type in the ID # (from 1 to 127) you want to save this finger as...");
//   idEnroll = readnumber();
//   if (idEnroll == 0) {// ID #0 not allowed, try again!
//      return;
//   }
//   Serial.print("Enrolling ID #");
//   Serial.println(idEnroll);
//
//   while (!getFingerprintEnroll() );
// }

// Empty Database Functions
// void setupEmptyDatabase()
// {
//     Serial.begin(9600);
//     while (!Serial);  // For Yun/Leo/Micro/Zero/...
//     delay(100);
//
//     Serial.println("\n\nDeleting all fingerprint templates!");
//     Serial.println("Press 'Y' key to continue");
//
//     while (1) {
//         if (Serial.available() && (Serial.read() == 'Y')) {
//             break;
//         }
//     }
//
//     // set the data rate for the sensor serial port
//     finger.begin(57600);
//
//     if (finger.verifyPassword()) {
//         Serial.println("Found fingerprint sensor!");
//     } else {
//         Serial.println("Did not find fingerprint sensor :(");
//         while (1);
//     }
//
//     finger.emptyDatabase();
//
//     Serial.println("Now database is empty :)");
// }
//
// void loopEmptyDatabase() {
// }

// Other Fingerprint Functions
// returns -1 if failed, otherwise returns ID #
// int getFingerprintIDez() {
//   uint8_t p = finger.getImage();
//   if (p != FINGERPRINT_OK)  return -1;
//
//   p = finger.image2Tz();
//   if (p != FINGERPRINT_OK)  return -1;
//
//   p = finger.fingerFastSearch();
//   if (p != FINGERPRINT_OK)  return -1;
//
//   // found a match!
//   Serial.print("Found ID #"); Serial.print(finger.fingerID);
//   Serial.print(" with confidence of "); Serial.println(finger.confidence);
//   return finger.fingerID;
// }
