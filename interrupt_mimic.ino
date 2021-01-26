                /********** SLAVE  ***********/
/*** Module interface macros ****/

// LCD //
#define RS 13
#define EN 12
#define D4 11
#define D5 10
#define D6 9
#define D7 8

// BT_Slave AT+NAME=Bstronics AT+PSWD=abcd //
#define BT_Rx RX
#define BT_Tx TX
#define BT_State 4
#define BT_On 5


/*************************Protocols**************************/

#define TEMPERATURE 'A'
#define HUMIDITY 'B'
#define COUNT 'C'
#define LPG_gas 'D'
#define CH4_gas 'E'
#define LIGHT 'F'

/************************************************************/

/** Standard include files ***/
// LCD
#include <LiquidCrystal.h>
LiquidCrystal lcd(RS,EN,D4,D5,D6,D7); // rs en d4-7


/*******************************  Global Datas  *************************/

// Global datas
unsigned float temperature=0,humidity=0,LPG_concentration=0,CH4_concentration=0;
unsigned int peopleCount=0;
String dayStatus=""; 

/******************************************************/



void setup() {

//Serial.begin(9600);

lcd.begin(16, 2);

Serial.begin(38400);
pinMode(BT_On,OUTPUT);
pinMode(BT_State,INPUT);
digitalWrite(BT_On,HIGH);

lcd.print("BT Connecting.."); lcd.setCursor(0,1);
while(!digitalRead(BT_State));
lcd.print("Connected : Okay");delay(500);lcd.clear();lcd.home();

lcd.print("Initializing.."); lcd.setCursor(0,1);

for(int i=0;i<6;i++)
{while(!(Serial.available()>0));serialEvent();}

lcd.print("Connected : Okay");delay(500);lcd.clear();lcd.home();

}

void serialEvent()
{
  if(Serial.available())
  {
    String str=Serial.readString();
    char branch=str[0];
    str.remove(0,1);
 
    switch(branch)
    {
      case TEMPERATURE: temperature=str.toFloat();break;
      case HUMIDITY: humidity=str.toFloat();break;
      case COUNT: peopleCount=str.toInt();break;
      case LPG_gas: LPG_concentration=str.toFloat();break;
      case CH4_gas: CH4_concentration=str.toFloat();break;
      case LIGHT: dayStatus=str;break;
      case '$': lcd.clear();lcd.home();lcd.print("NOTE:");lcd.setCursor(0,1);lcd.print(str);delay(500);break;
    }
  
  }
}

void loop() {
  
long unsigned int previousMillis=millis();

lcd.clear();lcd.home();
lcd.print("Temp : ");lcd.print(String(temperature));lcd.write(0xCF);lcd.print("C");
lcd.setCursor(0,1);
lcd.print("Humi : ");lcd.print(String(humidity));lcd.print("RH");
while(millis()-previousMillis<1000){serialEvent();}//delay(1000);  
previousMillis=millis();

lcd.clear();lcd.home();
lcd.print("CH4 : ");lcd.print(String(CH4_concentration)+"ppm");
lcd.setCursor(0,1);
lcd.print("LPG : ");lcd.print(String(LPG_concentration)+"ppm");
while(millis()-previousMillis<1000){serialEvent();}//delay(1000);  
previousMillis=millis();

lcd.clear();lcd.home();
lcd.print("PeopleCount  : ");lcd.print(String(peopleCount));
lcd.setCursor(0,1);
while(millis()-previousMillis<1000){serialEvent();}//delay(1000);  
previousMillis=millis();

lcd.clear();lcd.home();
lcd.print("DayStatus  : ");
lcd.setCursor(0,1);lcd.print(dayStatus);
while(millis()-previousMillis<1000){serialEvent();}//delay(1000);  

}

