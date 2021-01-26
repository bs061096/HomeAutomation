void setup()
{
    Serial.begin(9600);   //Sets the baud for serial data transmission                               
    pinMode(10, OUTPUT);  //Sets digital pin 13 as output pin
}
void loop()
{  
   if(Serial.available() > 0)      // Send data only when you receive data:
   {
      unsigned char data = Serial.read();        //Read the incoming data & store into data
      Serial.println(data);          //Print Value inside data in Serial monitor
      if(data==49) digitalWrite(10,HIGH);
      else if(data==48) digitalWrite(10,LOW);
      else digitalWrite(10,LOW);
   }
}
