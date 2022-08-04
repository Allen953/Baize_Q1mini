#include <EEPROM.h>
//简化
#define PT(s) Serial.print(s)  
#define PTL(s) Serial.println(s)
#define PTF(s) Serial.print(F(s))//节省动态内存
#define PTLF(s) Serial.println(F(s))

#define DOF 16                   //自由度16
#define MG92B_RANGE 150          //舵机角度范围150°
#define MG90D_RANGE 150          //舵机角度范围150°

//数据存储位置（在eeprom里的地址）
#define MELODY 1023 //声调数据存储在1023字节处，倒序存储.着意味着melody可以有不同的长度
#define PIN 0                 //关节映射表数据地址
#define CALIB 16              //关节舵机校准数据存储位置
#define MID_SHIFT 32          //中位转换数据地址
#define ROTATION_DIRECTION 48 //旋转方向数据地址
#define SERVO_ANGLE_RANGE 64  //舵机旋转范围数据地址

#define ADAPT_PARAM 160          //适应参数地址
#define NUM_ADAPT_PARAM  2       //适应参数数量

//下面的系数将会在adjust()函数里除以10，因此浮点数0.1可以被存储成整数1
//使用这种技巧可以用整数代替浮点数,节省96字节并且允许直接存储在eeprom上
#define panF 60
#define tiltF 60
#define sRF 50  //肩膀roll角因子
#define sPF 12 //肩膀pitch角因子
#define uRF 30 //大腿roll角因子
#define uPF 30 //大腿pitch角因子
#define lRF (-1.5*uRF) //小腿roll角因子
#define lPF (-1.5*uPF)//小腿leg角因子
//这是NYboard两个版本的关节映射表,ver2
byte pins[] = {4, 3, 11, 12,
               5, 2, 13, 10,
               6, 1, 14, 9,
               7, 0, 15, 8
              };
//音调数据
int8_t melody[] = {8, 13, 10, 13, 8,  0,  5,  8,  3,  5, 8,
                   8, 8,  32, 32, 8, 32, 32, 32, 32, 32, 8,
                  };

int8_t calibs[] = {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};

int8_t middleShifts[] = {0, 15, 0, 0,
                         -45, -45, -45, -45,
                         0, 0, 0, 0,
                         0, 0, 0, 0
                        };

//旋转方向
int8_t rotationDirections[] = {1, -1, 1, 1,
                               1, -1, 1, -1,
                               1, -1, -1, 1,
                               -1, 1, 1, -1
                              };

//每个舵机的角度旋转范围         
byte servoAngleRanges[] = {
  MG90D_RANGE, MG90D_RANGE, MG90D_RANGE, MG90D_RANGE,
  MG92B_RANGE, MG92B_RANGE, MG92B_RANGE, MG92B_RANGE,
  MG92B_RANGE, MG92B_RANGE, MG92B_RANGE, MG92B_RANGE,
  MG92B_RANGE, MG92B_RANGE, MG92B_RANGE, MG92B_RANGE
                           };

int8_t adaptiveParameterArray[16][NUM_ADAPT_PARAM] = {
  { -panF, 0}, { -panF, -tiltF}, { -2 * panF, 0}, {0, 0},
  {sRF, -sPF}, { -sRF, -sPF}, { -sRF, sPF}, {sRF, sPF},
  {uRF, uPF}, {uRF, uPF}, { -uRF, uPF}, { -uRF, uPF},
  {lRF, lPF}, {lRF, lPF}, { -lRF, lPF}, { -lRF, lPF}
};
