#include <Wire.h>                       //IIC通讯头文件
#include <Adafruit_PWMServoDriver.h> //16路舵机控制板头文件
//以这种方式调用，它使用默认地址0x40。
Adafruit_PWMServoDriver pwm=Adafruit_PWMServoDriver();

#define SERVOMIN  150//这是“最小”高电平占空比计数（在4096中）
#define SERVOMAX  600//这是“最大”高电平占空比计数（在4096中）
byte angle=90;//设置一个初始角度（输入值范围0-180）
//该函数用于将角度数据映射成PWM高电平占空比数据
int S2P(int angle){
  float p=0.0;
  p=2.5*angle+150;
  return int(p);
}
void setup() {
   Serial.begin(9600);//开启串口
   pwm.begin();  //初始化pca9685模块
   pwm.setPWMFreq(60);  //设置频率60Hz
   pwm.setPWM(11,0,S2P(angle));//舵机旋转到初试角度
   delay(300);
}
void loop() {
  int i=0;
  //如果串口缓冲区没有数据，则等待数据
  while(!(Serial.available()>0));
  i=Serial.parseInt();//读取串口输入的数字
  if((i<=180)&&(i>=0))
  {
    if(angle>i)
    {
      for(;angle>i;angle--)//逐步旋转
      {
        pwm.setPWM(11,0,S2P(angle));
        delay(10);
      }
    }
    else if(angle<i)
    {
      for(;angle<i;angle++)//逐步旋转
      {
        pwm.setPWM(11,0,S2P(angle));
        delay(10);
      }
    }
  }
}
