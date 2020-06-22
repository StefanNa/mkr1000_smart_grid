#include "arduino_secrets.h"
#include <LiquidCrystal.h>
/* 
  Sketch generated by the Arduino IoT Cloud Thing "angle"
  https://create.arduino.cc/cloud/things/655ea764-13a9-435b-a574-57b75f11dfc4 

  Arduino IoT Cloud Properties description

  The following variables are automatically generated and updated when changes are made to the Thing properties

  int angle;
  bool light;
  bool toggle;

  Properties which are marked as READ/WRITE in the Cloud Thing will also have functions
  which are called when their values are changed from the Dashboard.
  These functions are generated with the Thing and added at the end of this sketch.
*/

#include "thingProperties.h"
#include <FTDebouncer.h>
#define LED_PIN 1
#define POTENTIOMETER_PIN A1
#define BUTTON_PIN 6
int btnState;
int btnPrevState = 0;
const int rs = 12, en = 11, d4 = 2, d5 = 3, d6 = 4, d7 = 5; 
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); 
void setup() {
  // Initialize serial and wait for port to open:
  analogWrite(A3, 0); // Set the brightness to its maximum value
  lcd.begin(16,2); //Tell Arduino to start your 16 column 2 row LCD
  lcd.setCursor(0,0);  //Set LCD cursor to upper left corner, column 0, row 0
  lcd.print("Pot value:");  //Print Message on First Row
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(1500); 

  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  
  /*
     The following function allows you to obtain more information
     related to the state of network and IoT Cloud connection and errors
     the higher number the more granular information you’ll get.
     The default is 0 (only errors).
     Maximum is 4
 */
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
}

void loop() {
  ArduinoCloud.update();
  int angleSensor = analogRead(POTENTIOMETER_PIN);
  angle = map(angleSensor, 0, 1023, 0, 255);
  lcd.setCursor(0,0);  //Set LCD cursor to upper left corner, column 0, row 0
  lcd.print(angle);  //Print Message on First Row
  btnState = digitalRead(BUTTON_PIN);
  if (btnPrevState == 0 && btnState == 1) {
   toggle = !toggle;
  }
  btnPrevState = btnState;
  if (light) {
//        Serial.println(light);
        analogWrite(LED_PIN, angle);
    } else {
//        Serial.println(light);
        analogWrite(LED_PIN, light);
    }
}



void onLightChange() {
  
    Serial.print("The light is ");
    if (light) {
        Serial.println("ON");
//        Serial.println(light);
    } else {
        Serial.println("OFF");
//        Serial.println(light);
    }
}
