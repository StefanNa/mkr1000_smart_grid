#include <LiquidCrystal.h>
#define POTENTIOMETER_PIN A1
int angle;
const int rs = 12, en = 11, d4 = 2, d5 = 3, d6 = 4, d7 = 5; 
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); 
void setup() {
  analogWrite(A3, 0); // Set the brightness to its maximum value
  // put your setup code here, to run once:
  lcd.begin(16,2); //Tell Arduino to start your 16 column 2 row LCD
  lcd.setCursor(0,0);  //Set LCD cursor to upper left corner, column 0, row 0
  lcd.print("Pot value:");  //Print Message on First Row
  Serial.begin(9600);
  delay(1500);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  int angleSensor = analogRead(POTENTIOMETER_PIN);
  angle = map(angleSensor, 0, 1023, 0, 255);
  if (angle>=100){
    lcd.setCursor(0,1);
    }
  else if (angle>=10){
    lcd.setCursor(0,1);
    lcd.print("    ");
    lcd.setCursor(1,1);
    }
  else {
    lcd.setCursor(0,1);
    lcd.print("    ");
    lcd.setCursor(2,1);
    }
  lcd.print(angle);  //Print Message on First Row
  delay(500);

}
