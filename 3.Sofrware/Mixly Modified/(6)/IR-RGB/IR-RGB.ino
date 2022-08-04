#include <IRremote.h>
#define RECV_PIN 4    
IRrecv irrecv(RECV_PIN);  
decode_results results;

void setup() {
  Serial.begin(9600);  
  irrecv.enableIRIn();  
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
}

void loop() {
  //可以添加按键的功能
  if(irrecv.decode(&results))  
  {
    if(results.value==16724175)//按键1
    {
      digitalWrite(8,HIGH);
      digitalWrite(9,LOW);
      digitalWrite(10,LOW);
    }
    else if(results.value==16718055)//按键2
    {
      digitalWrite(8,LOW);
      digitalWrite(9,HIGH);
      digitalWrite(10,LOW);
    }
    else if(results.value==16743045)//按键3
    {
      digitalWrite(8,LOW);
      digitalWrite(9,LOW);
      digitalWrite(10,HIGH);
    }
    else//按下其他任意键，则关灯
    {
      digitalWrite(8,LOW);
      digitalWrite(9,LOW);
      digitalWrite(10,LOW);
    }
    Serial.println(results.value,DEC);
    irrecv.resume();  
  }
  else
  Serial.println("no signal");
//这里延时改的小一些可以使得RGB灯响应遥控器速度更快
  delay(200);   
}
