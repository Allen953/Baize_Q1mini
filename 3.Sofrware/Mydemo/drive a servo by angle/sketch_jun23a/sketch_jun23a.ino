#include <Wire.h>                    //16路舵机控制板头文件
#include <Adafruit_PWMServoDriver.h> //16路舵机控制板头文件

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();//以这种方式调用，它使用默认地址0x40。

#define SERVOMIN  150   //这是“最小”脉冲长度计数（在4096中）
#define SERVOMAX  600   //这是“最大”脉冲长度计数（在4096中）

byte pins[] = {4, 3, 11, 12,
               5, 2, 13, 10,
               6, 1, 14, 9,
               7, 0, 15, 8
              };

int S2P(int angle){
  float p=0.0;
  p=2.5*angle+150;
  return int(p);
}

void setup() {
  // put your setup code here, to run once:
   Serial.begin(57600);
   Serial.println("16 channel Servo test!");
   pwm.begin();
   pwm.setPWMFreq(60);  //设置频率60Hz  可用50Hz 40-1000//测试对电机的速度没有影响
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i=0;i<=180;i++)
  {
    pwm.setPWM(pins[2],0,S2P(i));
    delay(12);
  }
  delay(500);
  for(int i=180;i>=0;i--)
  {
    pwm.setPWM(pins[2],0,S2P(i));
    delay(12);
  }
  delay(500);
}
