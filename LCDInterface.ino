#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 6, 4, 3, 2); //re,en,d4-7

void setup() {
Serial.begin(9600);
lcd.begin(16, 2);
lcd.print("Type: ");;

}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available())
  { lcd.clear();
    String str=Serial.readString();
    lcd.setCursor(0,1);
    lcd.print(str);
  }
  
}



