                                                                               /********** MASTER  ***********/

/*** Module interface macros ****/

/*************  SENSORS   **********/

#define PIR_in 4
#define PIR_out 5
#define PIR_INTin 2
#define PIR_INTout 3
#define MQ_PIN (A1) 
#define LDR_PIN (A0)
#define DHT11_PIN 31


/*============== MQ-6 Gas Sensor Module ======= */

#define         RL_VALUE                     (20)    
#define         RO_CLEAN_AIR_FACTOR          (10)    
 

#define         CALIBARAION_SAMPLE_TIMES     (50)    
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   
#define         READ_SAMPLE_INTERVAL         (50)    
#define         READ_SAMPLE_TIMES            (5)     
 

#define         GAS_LPG                      (0)
#define         GAS_CH4                      (1)
 
float           LPGCurve[3]  =  {3,   0,  -0.4};                                                        
float           CH4Curve[3]  =  {3.3, 0,  -0.38};   
float           Ro           =  10;              
   
/*================== LDR Spec ================ */
#define HIGH_intensity   70
#define MEDIUM_intensity  30

              /************* ACTUATORS   **********/
// LCD //

#define RS A9
#define EN A8
#define D4 A10
#define D5 A11
#define D6 A12
#define D7 A13
#define BackLight A15

// Buzzer
#define Buzzer 0

// Stepper Motor
#define S0 0
#define S1 0
#define S2 0
#define S3 0

// Relay Switch
#define Relay_OUT 0
#define Relay_IN 0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 /************ Communication  ************/

// AT+NAME=Bs_Master AT+PSWD=abcd //
#define BT_Rx RX1
#define BT_Tx TX1
#define BT_State 8

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
LiquidCrystal lcd(RS,EN,D4,D5,D6,D7); 

// Stepper Motor
#include <Stepper.h>
const int stepsPerRevolution = 200; 
Stepper myStepper(stepsPerRevolution, S0, S1, S2, S3);

//DHT11 Temperature cum Humidity
#include <dht.h>
dht DHT;

/*************************************************************/

                                                              /*******************************  Global Datas  *************************/

                                                              // Global datas
                                                              float temperature=0,humidity=0,LPG_concentration=0,CH4_concentration=0;
                                                              unsigned int peopleCount=0,LDR_intensity=0;
                                                              
                                                              float new_temperature=0,new_humidity=0,new_LPG_concentration=0,new_CH4_concentration=0;
                                                              volatile unsigned int new_peopleCount=0;
                                                              unsigned int new_LDR_intensity=0;
                                                              
                                                              String dayStatus=""; 

                            /***********************************************************************************************************************************************************/



void setup() {

Serial.begin(9600);

lcd.begin(16, 2);
pinMode(BackLight,OUTPUT);digitalWrite(BackLight,HIGH);

//Serial1.begin(38400);
//pinMode(BT_State,INPUT);

//lcd.print("BT Connecting.."); lcd.setCursor(0,1);
//Serial.println("BT Connecting..");
//while(!digitalRead(BT_State));
//Serial.println("Connected..");
//lcd.print("Connected : Okay");delay(500);lcd.clear();lcd.home();

lcd.print("Initializing.."); lcd.setCursor(0,1);
Serial.println("Initialization..");
Initialize_Sensors();
Serial.println("Done Initialization .....");
lcd.print("Done!");delay(500);lcd.clear();lcd.home();

pinMode(23,OUTPUT);
pinMode(24,OUTPUT);

Transmit_init();

}

/***********************************************************************************************************************************************************************************/

void loop() {
  
long unsigned int previousMillis=millis();
digitalWrite(BackLight,HIGH);

Sense_DHT();Transmit_Aquired_data();

lcd.clear();lcd.home();
lcd.print("Temp : ");lcd.print(String(temperature));lcd.write(0xDF);lcd.print("C");
lcd.setCursor(0,1);
lcd.print("Humi : ");lcd.print(String(humidity));lcd.print("RH");
while(millis()-previousMillis<2000){check_BT();}//delay(1000);  
previousMillis=millis();
digitalWrite(BackLight,HIGH);

Sense_Gas();Transmit_Aquired_data();
if(LPG_concentration>200){digitalWrite(23,HIGH);delay(2000);}

lcd.clear();lcd.home();
lcd.print("CH4 : ");lcd.print(String(CH4_concentration)+"ppm");
lcd.setCursor(0,1);
lcd.print("LPG : ");lcd.print(String(LPG_concentration)+"ppm");
while(millis()-previousMillis<2000){check_BT();Transmit_Aquired_data();}//delay(1000); 
previousMillis=millis();
digitalWrite(BackLight,HIGH);

if(peopleCount>0){digitalWrite(24,HIGH);}
else digitalWrite(24,LOW);

lcd.clear();lcd.home();
lcd.print("PeopleCount : ");lcd.print(String(peopleCount));
lcd.setCursor(0,1);
while(millis()-previousMillis<2000){check_BT();Transmit_Aquired_data();}//delay(1000);
previousMillis=millis();
digitalWrite(BackLight,HIGH);

Sense_LDR();Transmit_Aquired_data();

lcd.clear();lcd.home();
lcd.print("DayStatus: ");lcd.print(String(LDR_intensity)+'%');
lcd.setCursor(0,1);lcd.print(dayStatus);
while(millis()-previousMillis<2000){check_BT();}//delay(1000);
digitalWrite(BackLight,HIGH);


}

/***********************************************************************************************************************************************************************************/

                                                                      // **********    Sensing Function   ************ //
void Initialize_Sensors()
{ 
  init_MQ6(); // gas sensor calibration is around 25 seconds
  init_DHT();// temperature and humidity   
  init_LDR();
  init_PIR();// PIR
}

void Aquire_data()
{ 
  Sense_Gas();
  Sense_LDR();
  Sense_DHT();
}
//////////////////////////////////////////////////////////////////////////////

void init_MQ6(void)
{ 
  lcd.clear();lcd.home();
  lcd.print("MQ6 Init...");
  Serial.println("MQ6 Init..");
  lcd.setCursor(0,1);
  Ro = MQCalibration(MQ_PIN);
  Sense_Gas();
  Serial.println("Done..");
  lcd.print("Done!...");delay(100);
}

float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
 
float MQCalibration(int mq_pin) // calibrate in clean air
{ 
  int i;
  float val=0;
 
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 
 
  return val; 
}

float MQRead(int mq_pin)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CH4 ) {
      return MQGetPercentage(rs_ro_ratio,CH4Curve);
  }    
 
  return 0;
}
 
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10, (((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}

bool Sense_Gas()
{ 
  new_LPG_concentration=MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG);
  new_CH4_concentration=MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CH4);
 
}

///////////////////////////////////////////////////////////////////////////////////////////////


bool Sense_LDR(void)
{ 
  new_LDR_intensity=100-(100*(float(analogRead(LDR_PIN))/1023.0));
  if(LDR_intensity>HIGH_intensity){dayStatus="BRIGHT";}
  else if(LDR_intensity>MEDIUM_intensity){dayStatus="MEDIUM";}
  else dayStatus="DARK";
}

void init_LDR()
{ lcd.clear();lcd.home();
  lcd.print("LDR Init...");
  Serial.println("LDR Init..");
  lcd.setCursor(0,1);
  Sense_LDR();
  Serial.println("Done..");
  lcd.print("Done!");delay(500);
}
///////////////////////////////////////////////////////////////////////////////////////////////

bool Sense_DHT(void)
{
  while(1)
  { 
    int chk = DHT.read11(DHT11_PIN);
    if(chk==0)
    {
    new_temperature=DHT.temperature;
    new_humidity=DHT.humidity;
    break;
    }
    
  }
}

void init_DHT()
{ lcd.clear();lcd.home();
  lcd.print("DHT11 Init...");
  Serial.println("Init DHT..");
  lcd.setCursor(0,1);
  Sense_DHT();
  Serial.println("Done..");
  lcd.print("Done!");delay(500);
}
///////////////////////////////////////////////////////////////////////////////////////////////

// PIR init & interrupt Routine //

void init_PIR(void)
{ lcd.clear();lcd.home();
  lcd.print("PIR Init...");
  lcd.setCursor(0,1);
  pinMode(PIR_INTin, INPUT);pinMode(13,OUTPUT);
  pinMode(PIR_INTout, INPUT);
  pinMode(PIR_in,INPUT);pinMode(PIR_out,INPUT);
  while(digitalRead(PIR_in)==HIGH);while(digitalRead(PIR_out)==HIGH);
  delay(2000);
  digitalWrite(13,HIGH);
  attachInterrupt(digitalPinToInterrupt(PIR_INTin), entry_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(PIR_INTout), exit_ISR, RISING);
  Serial.println(COUNT+String(peopleCount));
  lcd.print("Done!");delay(500);
}

void entry_ISR() { 
  digitalWrite(13,LOW);
  while(digitalRead(PIR_in)==HIGH){if(digitalRead(PIR_out)==HIGH){new_peopleCount++;while(digitalRead(PIR_out)==HIGH);}}
  digitalWrite(13,HIGH);
}

void exit_ISR() {
  digitalWrite(13,LOW);
  while(digitalRead(PIR_out)==HIGH){if(digitalRead(PIR_in)==HIGH){new_peopleCount--;while(digitalRead(PIR_in)==HIGH);}}
  digitalWrite(13,HIGH);
}
                                                                      // ************   Actuator Functions  ***********/
#define OFF 0
#define ON 1

void Actuate_Stepper(void)
{
  
}

void Actuate_Buzzer(byte control)
{
  if(control==ON){}
  else{}
  
}

void Actuate_Relay(byte control,byte Switch)
{
  if(control==ON){}
  else{}
}

                                                                          // *********** Communication function ************* //

void check_BT(void)
{
  if(!digitalRead(BT_State))
  { Serial.println("BT Disconneced..");
    lcd.clear();lcd.home();lcd.print("BT Disconnected..");
    while(!digitalRead(BT_State));
    Serial.println("Conneced..");
    lcd.print("Connected : Okay");delay(500);lcd.clear();lcd.home();
  }
}

void Transmit_Aquired_data(void)
{
  if(new_temperature!=temperature){temperature=new_temperature;Serial.println(TEMPERATURE+String(temperature));Serial1.println(TEMPERATURE+String(temperature));delay(1000);}
  if(new_humidity!=humidity){humidity=new_humidity;Serial.println(HUMIDITY+String(humidity));Serial1.println(HUMIDITY+String(humidity));delay(1000);}
  if(new_LPG_concentration!=LPG_concentration){LPG_concentration=new_LPG_concentration;Serial.println(LPG_gas+String(LPG_concentration));Serial1.println(LPG_gas+String(LPG_concentration));delay(1000);}
  if(new_CH4_concentration!=CH4_concentration){CH4_concentration=new_CH4_concentration;Serial.println(CH4_gas+String(CH4_concentration));Serial1.println(CH4_gas+String(CH4_concentration));delay(1000);}
  if(new_peopleCount!=peopleCount){peopleCount=new_peopleCount;Serial.println(COUNT+String(peopleCount));Serial1.println(COUNT+String(peopleCount));delay(1000);}
  if(new_LDR_intensity!=LDR_intensity){LDR_intensity=new_LDR_intensity;Serial.println(LIGHT+String(LDR_intensity));Serial1.println(LIGHT+String(LDR_intensity));delay(1000);}
}

void Transmit_init(void)
{
  Serial1.println(TEMPERATURE+String(temperature));delay(1000);
  Serial.println(TEMPERATURE+String(temperature));
  Serial1.println(HUMIDITY+String(humidity));delay(1000);
  Serial.println(HUMIDITY+String(humidity));
  Serial1.println(LPG_gas+String(LPG_concentration));delay(1000);
  Serial.println(LPG_gas+String(LPG_concentration));
  Serial1.println(CH4_gas+String(CH4_concentration));delay(1000);
  Serial.println(CH4_gas+String(CH4_concentration));
  Serial1.println(COUNT+String(peopleCount));delay(1000);
  Serial.println(COUNT+String(peopleCount));
  Serial1.println(LIGHT+String(LDR_intensity));delay(1000);
  Serial.println(LIGHT+String(LDR_intensity));
}

void Flash_note(String note)
{
  Serial1.print('$'+note);
}

