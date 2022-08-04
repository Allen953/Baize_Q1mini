#include <Wire.h>                      //IIC通讯头文件
#include <Adafruit_PWMServoDriver.h> //16路舵机控制板头文件
#include <IRremote.h>                  //红外遥控头文件
//以这种方式调用，它使用默认地址0x40。
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  680   //这是“最小”高电平占空比计数（在4096中）
#define SERVOMAX  2200   //这是“最大”高电平占空比计数（在4096中）

#define RECV_PIN 4    
IRrecv irrecv(RECV_PIN);  
decode_results results;
//前进步态帧数据
char wk[43][8]= {
 12, 59,-55,-49, 23, 24, -2,-12,
 15, 59,-63,-47, 22, 27, -8,-11,
 18, 59,-67,-45, 20, 30,-20,-11,
 21, 59,-66,-43, 18, 34,-33,-10,
 24, 59,-64,-40, 16, 38,-37,-10,
 27, 58,-62,-37, 15, 43,-41,-11,
 30, 57,-60,-35, 13, 47,-45,-12,
 32, 58,-57,-32, 13, 47,-48,-13,
 35, 60,-57,-29, 12, 45,-47,-14,
 38, 62,-58,-26, 12, 41,-42,-15,
 40, 65,-59,-23, 11, 36,-37,-16,
 43, 66,-59,-20, 11, 32,-33,-18,
 45, 67,-59,-17, 11, 18,-30,-20,
 47, 62,-59,-14, 11,  7,-26,-22,
 49, 53,-59,-12, 12,  1,-24,-24,
 51, 40,-58,-12, 13,  2,-21,-22,
 52, 26,-57,-12, 14,  7,-19,-20,
 54, 17,-55,-14, 15, 13,-18,-16,
 55, 15,-54,-16, 17, 16,-16,-15,
 57, 13,-53,-23, 18, 19,-15, -9,
 58, 12,-51,-38, 21, 22,-13, -2,
 58, 12,-49,-51, 23, 24,-12, -1,
 59, 13,-47,-60, 26, 23,-11, -6,
 59, 17,-45,-66, 29, 20,-11,-15,
 59, 20,-43,-66, 32, 18,-10,-33,
 59, 23,-41,-65, 37, 17,-10,-35,
 58, 26,-38,-63, 41, 15,-11,-40,
 57, 29,-35,-61, 46, 14,-12,-44,
 58, 32,-33,-58, 47, 13,-13,-47,
 59, 34,-30,-57, 47, 12,-14,-48,
 61, 37,-27,-58, 43, 12,-15,-43,
 64, 40,-24,-59, 38, 11,-16,-38,
 65, 42,-21,-59, 34, 11,-17,-34,
 67, 44,-18,-59, 23, 11,-19,-31,
 64, 46,-15,-59, 10, 11,-21,-27,
 56, 48,-12,-59,  3, 12,-23,-24,
 45, 50,-12,-58,  1, 13,-23,-22,
 31, 52,-12,-57,  5, 14,-20,-19,
 18, 53,-14,-56, 13, 15,-17,-17,
 16, 55,-16,-55, 15, 17,-15,-16,
 14, 57,-18,-53, 17, 17,-13,-15,
 12, 57,-33,-52, 21, 20, -4,-14,
 12, 58,-47,-50, 23, 22,  0,-13,
};
//后退步态帧数据
char bk[45][8]={
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
//左转步态帧数据
char wkL[43][8]{
 40, 59,-55,-51,  8, 24, -2, -9,
 41, 59,-63,-50,  8, 27, -8, -9,
 42, 59,-67,-49,  7, 30,-20, -8,
 42, 59,-66,-48,  7, 34,-33, -8,
 43, 59,-64,-48,  7, 38,-37, -8,
 44, 58,-62,-47,  7, 43,-41, -8,
 45, 57,-60,-46,  8, 47,-45, -8,
 46, 58,-57,-46,  8, 47,-48, -8,
 46, 60,-57,-45,  8, 45,-47, -8,
 47, 62,-58,-44,  8, 41,-42, -8,
 48, 65,-59,-43,  8, 36,-37, -8,
 48, 66,-59,-42,  8, 32,-33, -8,
 49, 67,-59,-41,  8, 18,-30, -8,
 50, 62,-59,-41,  9,  7,-26, -8,
 50, 53,-59,-40,  9,  1,-24, -9,
 51, 40,-58,-40,  9,  2,-21, -8,
 52, 26,-57,-40, 10,  7,-19, -7,
 52, 17,-55,-41, 10, 13,-18, -6,
 53, 15,-54,-41, 10, 16,-16, -6,
 53, 13,-53,-44, 11, 19,-15, -4,
 54, 12,-51,-48, 11, 22,-13, -4,
 54, 12,-49,-51, 12, 24,-12, -5,
 55, 13,-47,-54, 12, 23,-11, -6,
 55, 17,-45,-57, 13, 20,-11, -8,
 56, 20,-43,-60, 13, 18,-10,-11,
 56, 23,-41,-59, 14, 17,-10,-13,
 57, 26,-38,-59, 14, 15,-11,-13,
 57, 29,-35,-59, 15, 14,-12,-14,
 58, 32,-33,-58, 15, 13,-13,-15,
 58, 34,-30,-57, 14, 12,-14,-15,
 59, 37,-27,-57, 14, 12,-15,-15,
 59, 40,-24,-57, 12, 11,-16,-14,
 59, 42,-21,-56, 12, 11,-17,-14,
 59, 44,-18,-56, 10, 11,-19,-13,
 56, 46,-15,-56,  7, 11,-21,-12,
 53, 48,-12,-55,  6, 12,-23,-12,
 50, 50,-12,-54,  5, 13,-23,-12,
 46, 52,-12,-54,  4, 14,-20,-11,
 42, 53,-14,-53,  5, 15,-17,-11,
 41, 55,-16,-53,  6, 17,-15,-10,
 41, 57,-18,-52,  6, 17,-13,-10,
 40, 57,-33,-52,  7, 20, -4,-10,
 40, 58,-47,-51,  8, 22,  0, -9,
};
//右转步态帧数据
char wkR[36][8]={
 12, 55,-53,-49, 23, 12, -5,-12,
 15, 55,-55,-47, 22, 12, -7,-11,
 18, 56,-58,-45, 20, 13, -9,-11,
 21, 56,-60,-43, 18, 13,-11,-10,
 24, 57,-59,-40, 16, 14,-13,-10,
 27, 57,-59,-37, 15, 15,-14,-11,
 30, 58,-58,-35, 13, 15,-15,-12,
 32, 58,-58,-32, 13, 15,-15,-13,
 35, 58,-57,-29, 12, 14,-15,-14,
 38, 59,-57,-26, 12, 14,-15,-15,
 40, 59,-56,-23, 11, 12,-14,-16,
 43, 60,-56,-20, 11, 11,-13,-18,
 45, 58,-56,-17, 11,  9,-13,-20,
 47, 55,-55,-14, 11,  7,-13,-22,
 49, 52,-55,-12, 12,  5,-12,-24,
 51, 49,-54,-12, 13,  4,-11,-22,
 52, 45,-54,-12, 14,  4,-11,-20,
 54, 42,-53,-14, 15,  5,-10,-16,
 55, 41,-53,-16, 17,  6,-10,-15,
 57, 40,-52,-23, 18,  7,-10, -9,
 58, 40,-51,-38, 21,  7, -9, -2,
 58, 40,-51,-51, 23,  8, -9, -1,
 59, 40,-50,-60, 26,  8, -9, -6,
 59, 41,-49,-66, 29,  8, -9,-15,
 59, 42,-49,-66, 32,  7, -8,-33,
 59, 43,-48,-65, 37,  7, -8,-35,
 58, 44,-47,-63, 41,  7, -8,-40,
 57, 44,-47,-61, 46,  8, -8,-44,
 58, 45,-46,-58, 47,  8, -8,-47,
 //59, 46,-45,-57, 47,  8, -8,-48,
 61, 47,-44,-58, 43,  8, -8,-43,
 //64, 47,-43,-59, 38,  8, -8,-38,
 65, 48,-43,-59, 34,  8, -8,-34,
 //67, 49,-42,-59, 23,  8, -8,-31,
 64, 49,-41,-59, 10,  8, -8,-27,
 //56, 50,-40,-59,  3,  9, -9,-24,
 45, 51,-40,-58,  1,  9, -8,-22,
 //31, 51,-40,-57,  5, 10, -7,-19,
 18, 52,-40,-56, 13, 10, -7,-17,
 //16, 53,-41,-55, 15, 10, -6,-16,
 14, 53,-42,-53, 17, 11, -5,-15,
 //12, 54,-46,-52, 21, 11, -4,-14,
 12, 54,-50,-50, 23, 11, -5,-13,
};

char balance[16]= { 
  0,  0,  0,  0,  0,  0,  0,  0, 30, 30,-30,-30, 30, 30,-30,-30};

//舵机校准偏角数据
char cal[16]={-6,-14,7,0,0,0,0,0,1,9,-3,3,6,-6,7,-6};
//舵机旋转方向
int8_t rotationDirections[] = {1, -1, 1, 1,
                               1, -1, 1, -1,
                               1, -1, -1, 1,
                               -1, 1, 1, -1
                              };
//关节映射表
byte pins[] = {4, 3, 11, 12,
               5, 2, 13, 10,
               6, 1, 14, 9,
               7, 0, 15, 8
              };
byte cmd=0;//命令状态：0为balance,1为前进，2为后退，3为左转，4为右转
//将角度数据转换成高电平占空比
int S2P(int angle){
  float p=0.0;
  p=(1520/150)*angle+760+680;
  return int(p);
}

void balan(){
  for(byte i=0;i<=15;i++)
    pwm.setPWM(pins[i],0,S2P((int(balance[i])+cal[i])*rotationDirections[i]));
  delay(500);
}
//前进函数封装起来，每次调用该函数，小猫前进一步
void qj()
{
  
  for(int i=0;i<=42;i++)//一共43帧
  {
    for(int j=0;j<=7;j++)//每帧8个舵机角度   
      //具体舵机角度=步态帧中舵机角度数据*旋转方向+舵机校准偏角+90
      //每次从8号关节舵机开始，执行到16号关节舵机，为一帧
      pwm.setPWM(pins[8+j],0,S2P((int(wk[i][j])+cal[8+j])*rotationDirections[j+8]));
    delay(50);
  }
}
//后退函数
void ht(){
  for(int i=0;i<=44;i++)//一共45帧
  {
    for(int j=0;j<=7;j++)//每帧8个舵机角度    
      //具体舵机角度=步态帧中舵机角度数据*旋转方向+舵机校准偏角+75
      //每次从8号关节舵机开始，执行到16号关节舵机，为一帧
      pwm.setPWM(pins[8+j],0,S2P((int(bk[i][j])+cal[8+j])*rotationDirections[j+8]));
    delay(50);
  }
}
//左转
void zz()
{
  for(int i=0;i<=42;i++)//一共43帧
  {
    for(int j=0;j<=7;j++)//每帧8个舵机角度   
      //具体舵机角度=步态帧中舵机角度数据*旋转方向+舵机校准偏角+75
      //每次从8号关节舵机开始，执行到16号关节舵机，为一帧
      pwm.setPWM(pins[8+j],0,S2P((int(wkL[i][j])+cal[8+j])*rotationDirections[j+8]));
    delay(50);
  }
}

//右转
void yz()
{
  for(int i=0;i<=35;i++)//一共43帧
  {
    for(int j=0;j<=7;j++)//每帧8个舵机角度
      //具体舵机角度=步态帧中舵机角度数据*旋转方向+舵机校准偏角+75
      //每次从8号关节舵机开始，执行到16号关节舵机，为一帧
      pwm.setPWM(pins[8+j],0,S2P((int(wkR[i][j])+cal[8+j])*rotationDirections[j+8]));
    delay(50);
  }
}

void setup() {
   Serial.begin(57600);//打开串口
   irrecv.enableIRIn(); //初始化红外遥控
   pwm.begin();       //初始化PCA9685
   pwm.setPWMFreq(240);  //设置频率60Hz  
   Serial.print(F("sf"));
   balan();//开机站好
}

void loop() {
  
  if(irrecv.decode(&results))  
  {
    switch(results.value)
    {
      case 0xFF629D:cmd=1;break;
      case 0xFFA856:cmd=2;break;
      case 0xFF22DD:cmd=3;break;
      case 0xFFC23D:cmd=4;break;
      default:cmd=0;
    }
    Serial.println(results.value,DEC);
    irrecv.resume();  
  }
  else
  Serial.println(F("no signal"));
  if(Serial.available()>0)
  {
    char cmd_c=Serial.read();
    Serial.print(cmd_c);
    switch(cmd_c)
    {
      case 'a':cmd=1;break;
      case 'b':cmd=2;break;
      case 'c':cmd=3;break;
      case 'd':cmd=4;break;
      default:cmd=0;
    }
  }
  //Serial.print(cmd,DEC);
  switch(cmd)
  {
    case 1:qj();break;
    case 2:ht();break;
    case 3:zz();break;
    case 4:yz();break;
    default:balan();
  }
}