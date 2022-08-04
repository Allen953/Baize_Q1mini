#include <Wire.h>                    //16路舵机控制板头文件
#include <Adafruit_PWMServoDriver.h> //16路舵机控制板头文件
#include <IRremote.h>

#define IR_RECIEVER 4 //红外接收头信号线连接Arduino的Pin 4

IRrecv irrecv(IR_RECIEVER);
decode_results results;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();//以这种方式调用，它使用默认地址0x40。

#define SERVOMIN  150   //这是“最小”脉冲长度计数（在4096中）
#define SERVOMAX  600   //这是“最大”脉冲长度计数（在4096中）



int bk[45][8]= {
 30, 39,-57,-64,  6, -9, -6,  9,
 28, 49,-58,-56,  7,-11, -7, 11,
 25, 58,-59,-47,  9,-10, -9, 11,
 23, 65,-60,-37, 10, -8,-10,  8,
 20, 66,-61,-34, 12, -6,-12,  6,
 17, 66,-62,-31, 14, -3,-14,  3,
 14, 64,-63,-28, 16,  1,-16, -1,
 12, 61,-63,-27, 18,  5,-18, -5,
  9, 58,-64,-27, 20,  8,-20, -8,
  6, 57,-64,-30, 22,  7,-22, -7,
  3, 55,-64,-32, 25,  6,-25, -6,
  0, 54,-64,-35, 28,  5,-28, -5,
 -3, 52,-64,-37, 31,  4,-32, -4,
 -7, 50,-63,-40, 35,  4,-35, -4,
-10, 48,-64,-42, 38,  3,-38, -3,
-10, 46,-66,-44, 36,  3,-36, -3,
 -9, 44,-69,-46, 32,  3,-32, -3,
 -6, 42,-73,-48, 25,  3,-25, -3,
 -2, 40,-75,-50, 19,  3,-19, -4,
  3, 38,-76,-52, 14,  4,-14, -4,
 15, 35,-74,-53,  4,  5, -4, -5,
 26, 33,-71,-55, -3,  5,  3, -5,
 37, 30,-65,-57, -8,  6,  8, -6,
 47, 28,-58,-58,-11,  7, 11, -7,
 56, 25,-49,-59,-11,  9, 11, -9,
 64, 23,-39,-60, -9, 10,  9,-10,
 66, 20,-35,-61, -7, 12,  7,-12,
 66, 17,-31,-62, -4, 14,  4,-14,
 65, 14,-29,-63,  0, 16,  0,-16,
 62, 12,-27,-63,  4, 18, -5,-18,
 59,  9,-27,-64,  7, 20, -7,-20,
 57,  6,-29,-64,  7, 22, -7,-22,
 56,  3,-32,-64,  6, 25, -6,-25,
 54,  0,-34,-64,  5, 28, -5,-28,
 52, -3,-37,-64,  4, 31, -4,-32,
 51, -7,-39,-63,  4, 35, -4,-35,
 49,-10,-41,-64,  3, 38, -3,-38,
 47,-10,-43,-66,  3, 36, -3,-36,
 45, -9,-46,-69,  3, 32, -3,-32,
 43, -6,-48,-73,  3, 25, -3,-25,
 40, -2,-49,-75,  3, 19, -4,-19,
 38,  3,-51,-76,  4, 14, -4,-14,
 36, 15,-53,-74,  5,  4, -5, -4,
 33, 26,-55,-71,  5, -3, -5,  3,
 31, 37,-56,-65,  6, -8, -6,  8,
};

int shualai[3][16]={
  0,0,0,0,0,0,0,0,-80,-80,80,80,80,80,-80,-80,
  -30,-30,0,0,0,0,0,0,-90,-80,90,80,60,80,-60,-80,
  30,-30,0,0,0,0,0,0,-80,-90,80,90,80,60,-80,-60
};

byte pins[] = {4, 3, 11, 12,
               5, 2, 13, 10,
               6, 1, 14, 9,
               7, 0, 15, 8
              };
              
int8_t rotationDirections[] = {1, -1, 1, 1,
                               1, -1, 1, -1,
                               1, -1, -1, 1,
                               -1, 1, 1, -1
                              };

int S2P(int angle){
  float p=0.0;
  p=2.5*angle+150;
  return int(p);
}

byte translateIR() //将红外遥控器的键值信号转换为字符串命令（基于收到的红外键值动作）
//describing Remote IR codes.
{
  switch (results.value) {
    case 0xFFA25D:                        return (1);        //后退
    case 0xFFFFFFFF: return (0); //Serial.println(" REPEAT");
    //当遥控器实际键值与程序不匹配时，输出实际键值，用于修改程序
    default: {
        Serial.println(results.value, HEX);
      }
      return (0);
  }
  //delay(100); // Do not get immediate repeat //no need because the main loop is slow
  // The control could be organized in another way, such as:
  // forward/backward to change the gaits corresponding to different speeds.
}


void setup() {
  // put your setup code here, to run once:
   Serial.begin(57600);
   Serial.println("16 channel Servo test!");
   irrecv.enableIRIn();
   pwm.begin();
   pwm.setPWMFreq(60);  //设置频率60Hz  可用50Hz 40-1000//测试对电机的速度没有影响
}
byte cmd=0;
void loop() {
  // put your main code here, to run repeatedly:
  
 
  for(int i=0;i<16;i++)
  {
    pwm.setPWM(pins[i],0,S2P(shualai[0][i]+90*rotationDirections[i]));
  }
  delay(2000);
  for(int j=0;j<6;j++)
{
      for(int i=0;i<16;i++)
      {
        pwm.setPWM(pins[i],0,S2P(shualai[1][i]+90*rotationDirections[i]));
      }
      delay(150);
      for(int i=0;i<16;i++)
      {
        pwm.setPWM(pins[i],0,S2P(shualai[2][i]+90*rotationDirections[i]));
      }
      delay(150);
}
  
  

  
}
