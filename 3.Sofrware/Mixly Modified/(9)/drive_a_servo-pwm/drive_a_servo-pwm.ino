#include <Wire.h>                       //IIC通讯头文件
#include <Adafruit_PWMServoDriver.h> //16路舵机控制板头文件
//以这种方式调用，它使用默认地址0x40。
Adafruit_PWMServoDriver pwm=Adafruit_PWMServoDriver();

#define SERVOMIN  150 //这是“最小”高电平占空比计数（在4096中）
#define SERVOMAX  600 //这是“最大”高电平占空比计数（在4096中）

void setup() {
   pwm.begin();  //初始化pca9685模块
   pwm.setPWMFreq(60);  //设置频率60Hz
}
void loop() {
  for(int i=SERVOMIN;i<=SERVOMAX;i++)//从0°-180°
  {
    pwm.setPWM(11,0,i);//关节2（尾巴）舵机接在11号PWM口
    delay(5);//舵机旋转需要时间,如果太快更新角度,舵机来不及反应
  }
  delay(500);
  for(int i=SERVOMAX;i>=SERVOMIN;i--)//从180°-0°
  {
    pwm.setPWM(11,0,i);
    delay(5);
  }
  delay(500);
}
