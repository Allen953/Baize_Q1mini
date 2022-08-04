#include <Wire.h>                    //16路舵机控制板头文件
#include <Adafruit_PWMServoDriver.h> //16路舵机控制板头文件
//以这种方式调用，它使用默认地址0x40。
Adafruit_PWMServoDriver pwm=Adafruit_PWMServoDriver();

#define SERVOMIN  150   //这是“最小”脉冲长度计数（在4096中）
#define SERVOMAX  600   //这是“最大”脉冲长度计数（在4096中）

void setup() {
  // put your setup code here, to run once:
   Serial.begin(57600);
   Serial.println("16 channel Servo test!");
   pwm.begin();
   pwm.setPWMFreq(60);  //设置频率60Hz  可用50Hz 40-1000//测试对电机的速度没有影响
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i=SERVOMIN;i<=SERVOMAX;i++)
  {
    pwm.setPWM(11,0,i);
    delay(5);
  }
  delay(500);
  for(int i=SERVOMAX;i>=SERVOMIN;i--)
  {
    pwm.setPWM(11,0,i);
    delay(5);
  }
  delay(500);
}
