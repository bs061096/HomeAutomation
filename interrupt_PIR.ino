
const byte interruptPin = 2;
volatile byte state = LOW;
unsigned int count=0;

void setup() {
  Serial.begin(9600);
  pinMode(interruptPin, INPUT_PULLUP);pinMode(13,OUTPUT);
  Serial.println("calibrating wait..");
  while(digitalRead(interruptPin)==HIGH);
  Serial.println("done..");
  attachInterrupt(digitalPinToInterrupt(interruptPin), interrupt_routine, RISING);
}

void loop() {
  digitalWrite(13,state);
  Serial.println("count = "+String(count));
  delay(1000);
}

void interrupt_routine() {
state=!state;Serial.println("interrupt");while(digitalRead(interruptPin)==HIGH);}
