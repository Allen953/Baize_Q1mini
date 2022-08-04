/*
    Skill class holds only the lookup information of joint angles.
    One frame of joint angles defines a static posture, while a series of frames defines a periodic motion, usually a gait.
    Skills are instantiated as either:
      instinct  (trained by Rongzhong Li, saved in external i2c EERPOM) or
      newbility (taught by other users, saved in PROGMEM)
    A well-tuned (finalized) newbility can also be saved in external i2c EEPROM. Remember that EEPROM has very limited (1,000,000) write cycles!

    SkillList (inherit from QList class) holds a mixture of instincts and newbilities.
    It also provides a dict(key) function to return the pointer to the skill.
    Initialization information(individual skill name, address) for SkillList is stored in on-board EEPROM

    Behavior list (inherit from QList class) holds a time dependent sequence of multiple skills, triggered by certain perceptions.
    It defines the order, speed, repetition and interval of skills。
    (Behavior list is yet to be implemented)

    Motion class uses the lookup information of a Skill to construct a Motion object that holds the actual angle array.
    It also implements the reading and writing functions in specific storage locations.
    Considering Arduino's limited SRAM, you should create only one Motion object and update it when loading new skills.

    instinct(external EEPROM) \
                                -- skill that contains only lookup information
    newbility(progmem)        /

    Skill list: skill1, skill2, skill3,...
                              |
                              v
                           motion that holds actual joint angle array in SRAM

    Behavior list: skill3(speed, repetition and interval), skill1(speed, repetition and interval), ...

    **
    Updates: One Skill object in the SkillList takes around 20 bytes in SRAM. It takes 200+ bytes for 15+ skills.
    On a tiny atmega328 chip with only 2KB SRAM, I'm implementing the Skills and SkillList in the on-board EEPROM。
    Now the skill list starts from on-board EEPROM address SKILLS.
    Format:
    1 byte skill_1 nameLength + char string name1 + 1 char skillType1 + 1 int address1,
    1 byte skill_2 nameLength + char string name2 + 1 char skillType2 + 1 int address2,
    ...
    The iterator can traverse the list with the string length of each skill name.

    The Skill and SkillList classes are obsolete in the atmega328 implementation but are still included in this header file.
*/

#include "Instinct.h" //postures and movements trained by RongzhongLi
//#define DEVELOPER
#ifdef DEVELOPER
#include <MemoryFree.h> //http://playground.arduino.cc/Code/AvailableMemory
#include <QList.h> //https://github.com/SloCompTech/QList
#endif

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>  //通过PCA9685控制舵机，PCA9685可以控制16路舵机，采用iic与主机通信
#include <EEPROM.h>

//简化
#define PT(s) Serial.print(s)  
#define PTL(s) Serial.println(s)
#define PTF(s) Serial.print(F(s))//trade flash memory for dynamic memory with F() function
#define PTLF(s) Serial.println(F(s))

//主板设置
#define INTERRUPT 0
#define IR_RECIEVER 4 //红外接收头信号线连接Arduino的Pin 4
#define BUZZER 5
#define GYRO
#define ULTRA_SOUND
#define BATT A0

#ifdef ULTRA_SOUND
#define VCC 8
#define TRIGGER 9
#define ECHO 10
#define LONGEST_DISTANCE 200 // 200 cm = 2 meters
float farTime =  LONGEST_DISTANCE * 2 / 0.034;  //计算超声波往返最长时间（微秒）
#endif

void beep(int8_t note, float duration = 10, int pause = 0, byte repeat = 1 ) {
  if (note == 0) {//rest note，空拍，不会响，仅仅起到延时作用
    analogWrite(BUZZER, 0);
    delay(duration);
    return;
  }
  int freq = 220 * pow(1.059463, note - 1); // 1.059463 comes from https://en.wikipedia.org/wiki/Twelfth_root_of_two
  float period = 1000000.0 / freq;
  for (byte r = 0; r < repeat; r++) {
    for (float t = 0; t < duration * 1000; t += period) {
      analogWrite(BUZZER, 150);      // Almost any value can be used except 0 and 255，150 experiment to get the best tone
      delayMicroseconds(period / 2);        // 高电平半个周期
      analogWrite(BUZZER, 0);               //低电平半个周期
      delayMicroseconds(period / 2);        // down for half period
    }
    delay(pause);
  }
}

void playMelody(int start) {   //奏乐
  byte len = (byte)EEPROM.read(start) / 2;
  for (int i = 0; i < len; i++)
    beep(EEPROM.read(start - 1 - i), 1000 / EEPROM.read(start - 1 - len - i), 100);
    //Serial.print(EEPROM.read(start - 1 - i),DEC),Serial.print(',');
    //Serial.print(EEPROM.read(start - 1 - len - i),DEC),Serial.println(); 
}
//喵叫，当不带参数调用该函数时，默认不重复喵叫，即repeat=0
void meow(int repeat = 0, int pause = 200, int startF = 50,  int endF = 200, int increment = 5) {
  for (int r = 0; r < repeat + 1; r++) {
    for (int amp = startF; amp <= endF; amp += increment) {
      analogWrite(BUZZER, amp);//用0替代amp ，关闭喵叫
      delay(15); // 等待15毫秒以便蜂鸣器在该电压下震动发声
    }
    delay(100 + 500 / increment);
    analogWrite(BUZZER, 0);//喵叫一次之后，延时一段时间，关闭蜂鸣器
    if (repeat)delay(pause);//如果repeat为真，则停顿pause毫秒，继续下次喵叫
  }
}

//这是NYboard两个版本的关节映射表,ver2
byte pins[] = {4, 3, 11, 12,
               5, 2, 13, 10,
               6, 1, 14, 9,
               7, 0, 15, 8
              };

#define HEAD
#define TAIL
#define X_LEG
#define WALKING_DOF 8

//remap pins for different walking modes, pin4 ~ pin15
byte fast[] = {
  4, 4, 7, 7,
  8, 8, 11, 11,
  12, 12, 15, 15
};
byte slow[] = {
  5, 5, 8, 8,
  9, 9, 10, 10,
  13, 13, 14, 14
};
byte left[] = {
  5, 4, 7, 6,
  9, 8, 11, 10,
  13, 12, 15, 14
};
byte right[] = {
  4, 5, 6, 7,
  8, 9, 10, 11,
  12, 13, 14, 15
};

//数据存储位置（在eeprom里的地址）
#define MELODY 1023           //声调数据存储在1023字节处，倒序存储.着意味着melody可以有不同的长度
#define PIN 0                 // 16 byte array
#define CALIB 16              // 关节舵机校准数据存储位置
#define MID_SHIFT 32          // 16 byte array
#define ROTATION_DIRECTION 48 // 16 byte array
#define SERVO_ANGLE_RANGE 64  // 16 byte array
#define MPUCALIB 80           // 16 byte array
#define FAST 96               // 16 byte array
#define SLOW 112              // 16 byte array
#define LEFT 128              // 16 byte array
#define RIGHT 144             // 16 byte array

#define ADAPT_PARAM 160          // 16 x NUM_ADAPT_PARAM byte array
#define NUM_ADAPT_PARAM  2    // number of parameters for adaption
#define SKILLS 200         //一个字节的技能名字长度, followed by the char array for skill name
// then followed by i(nstinct) on progmem, or n(ewbility) on progmem

//舵机常量
#define DOF 16                   //自由度16
#define PWM_FACTOR 4             //PWM因数为4
#define MG92B_MIN 170*PWM_FACTOR
#define MG92B_MAX 550*PWM_FACTOR
#define MG92B_RANGE 150          //舵机角度范围150°

#define MG90D_MIN 158*PWM_FACTOR //因此，在同一信号下，mg92b和mg90不会同时在中位
#define MG90D_MAX 515*PWM_FACTOR
#define MG90D_RANGE 150          //舵机角度范围150°

// PCA9685默认地址0x40，
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//你也可以用不同的地址（自己设置的地址）调用
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

//取决于你的舵机制造商,脉宽的最大值和最小值可能不同, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!

#define SERVOMIN  MG92B_MIN // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  MG92B_MAX // this is the 'maximum' pulse length count (out of 4096)
#define SERVO_ANG_RANGE MG92B_RANGE

#define PWM_RANGE (SERVOMAX - SERVOMIN)

float degPerRad = 180 / M_PI;
float radPerDeg = M_PI / 180;

// tone: pause,1,  2,  3,  4,  5,  6,  7,  1,  2, 3,
// code: 0,    1,  3,  5,  6,  8,  10, 12, 13, 15, 17
//音调数据
int8_t melody[] = {8, 13, 10, 13, 8,  0,  5,  8,  3,  5, 8,
                   8, 8,  32, 32, 8, 32, 32, 32, 32, 32, 8,
                  };

//byte pins[] = {11, 12, 4, 16, 16, 16, 16, 16, 9,14,1,6,8,15,0,7};//tail version

//舵机校准数据出厂设置值，用于恢复出厂设置
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
byte servoAngleRanges[] =  {MG90D_RANGE, MG90D_RANGE, MG90D_RANGE, MG90D_RANGE,
                            MG92B_RANGE, MG92B_RANGE, MG92B_RANGE, MG92B_RANGE,
                            MG92B_RANGE, MG92B_RANGE, MG92B_RANGE, MG92B_RANGE,
                            MG92B_RANGE, MG92B_RANGE, MG92B_RANGE, MG92B_RANGE
                           };
                           
float pulsePerDegree[DOF] = {};//每一度对应的PWM高电平宽度
int8_t servoCalibs[DOF] = {};//舵机校准角度数据
char currentAng[DOF] = {};
int calibratedDuty0[DOF] = {};//舵机转到中位时候的PWM值

/*
 * eeprom这个库的库函数无论是write还是update函数都只能写入一个字节的变量，因此变量
 * 最大值为255，而当我们想要写入一个大于255的变量的时候，需要做相应的处理才行。
 * 函数功能：在eeprom的p_address地址和下一个字节地址存入一个2字节的整型数据
*/
void EEPROMWriteInt(int p_address, int p_value)
{
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);
  EEPROM.update(p_address, lowByte);
  EEPROM.update(p_address + 1, highByte);
}

//函数功能：从e2prom的p_address地址和下个字节地址读取一个2字节的整型数据
int EEPROMReadInt(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);
  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

#define DEVICE_ADDRESS 0x50    //（AT24C32D存储芯片(eeprom)的I2C通讯地址)共4x1024 bytes（4k字节）
#define WIRE_BUFFER 30         //Arduino wire allows 32 byte buffer, with 2 byte for address.
#define WIRE_LIMIT 16          //That leaves 30 bytes for data. use 16 to balance each writes
#define PAGE_LIMIT 32          //AT24C32D 32-byte Page Write Mode. Partial Page Writes Allowed
#define EEPROM_SIZE (65536/8)
#define SKILL_HEADER 3

bool EEPROMOverflow = false;
//把PROGMEM中的数据拷到E2Prom中
void copyDataFromPgmToI2cEeprom(unsigned int &eeAddress, unsigned int pgmAddress) {
  uint8_t period = pgm_read_byte(pgmAddress);//automatically cast to char*
  byte frameSize = period > 1 ? WALKING_DOF : 16;//判断，如果帧数大于1，则单帧大小WALKING_DOF，否则单帧大小16
  int len = period * frameSize + SKILL_HEADER;   //步态或姿势数据总的大小
  int writtenToEE = 0;
  while (len > 0) {
    Wire.beginTransmission(DEVICE_ADDRESS);
    Wire.write((int)((eeAddress) >> 8));   // MSB
    Wire.write((int)((eeAddress) & 0xFF)); // LSB
    
    byte writtenToWire = 0;
    do {
      if (eeAddress == EEPROM_SIZE) {
        PTL();
        PTL("I2C EEPROM overflow! You must reduce the size of your instincts file!\n");
        EEPROMOverflow = true;
#ifdef BUZZER
        meow(3);
#endif
        return;
      }
      
      Wire.write((byte)pgm_read_byte(pgmAddress + writtenToEE++));//为什么这里转成byte？
      writtenToWire++;
      eeAddress++;
    } while ((--len > 0 ) && (eeAddress  % PAGE_LIMIT ) && (writtenToWire < WIRE_LIMIT));//be careful with the chained conditions
    //self-increment may not work as expected
    Wire.endTransmission();
    delay(6);  // needs 5ms for page write
    //PTL("\nwrote " + String(writtenToWire) + " bytes.");
  }
  //PTLF("finish copying to I2C EEPROM");
}

//Motion类，
class Motion {
  public:
    byte pins[DOF];
    uint8_t period;
    float expectedRollPitch[2];
    char* dutyAngles;//目标角度指针
    Motion() {
      period = 0;
      expectedRollPitch[0] = 0;
      expectedRollPitch[1] = 0;
      dutyAngles = NULL;
    }

    int lookupAddressByName(char* skillName) {//通过技能名字寻找地址
      int skillAddressShift = 0;
      for (byte s = 0; s < NUM_SKILLS; s++) {//在片上e2prom里存储技能信息，将技能加载进SkillList
        byte nameLen = EEPROM.read(SKILLS + skillAddressShift++);
        char* readName = new char[nameLen + 1];
        for (byte l = 0; l < nameLen; l++) {
          readName[l] = EEPROM.read(SKILLS + skillAddressShift++);
        }
        readName[nameLen] = '\0';
        /*
         * C/C++函数，比较两个字符串设这两个字符串为str1，str2， 
         * 若str1=str2，则返回零； 
         * 若str1<str2，则返回负数； 
         * 若str1>str2，则返回正数
         */
        if (!strcmp(readName, skillName)) {//如果在E2Prom里读取到的名字与skillname相等，则返回在E2Prom里读取到的名字的地址
          delete[]readName;
          return SKILLS + skillAddressShift;
        }
        delete[]readName;
        skillAddressShift += 3;//1 byte type, 1 int address
      }
      PTLF("wrong key!");
      return -1;//如果没找到参数传过来的技能名字，则返回-1，说明技能输入错误
    }
    //从PROGMEM中加载数据
    void loadDataFromProgmem(unsigned int pgmAddress) {
      period = pgm_read_byte(pgmAddress);//automatically cast to char*
      for (int i = 0; i < 2; i++)
        expectedRollPitch[i] = radPerDeg * (int8_t)pgm_read_byte(pgmAddress + 1 + i);
      byte frameSize = period > 1 ? WALKING_DOF : 16;
      int len = period * frameSize;
      //delete []dutyAngles; //check here
      dutyAngles = new char[len];
      for (int k = 0; k < len; k++) {
        dutyAngles[k] = pgm_read_byte(pgmAddress + SKILL_HEADER + k);
      }
    }
    //从外挂i2c的eeprom中加载数据
    void loadDataFromI2cEeprom(unsigned int &eeAddress){
      Wire.beginTransmission(DEVICE_ADDRESS);
      Wire.write((int)((eeAddress) >> 8));   // 地址高位
      Wire.write((int)((eeAddress) & 0xFF)); // 地址低位
      Wire.endTransmission();
      Wire.requestFrom(DEVICE_ADDRESS, 3);
      period = Wire.read();                 //period为读取帧数
      //PTL("read " + String(period) + " frames");
      for (int i = 0; i < 2; i++)
        expectedRollPitch[i] = radPerDeg * (int8_t)Wire.read();//读取roll和pitch角度并转换成弧度
      byte frameSize = period > 1 ? WALKING_DOF : 16;  //每帧数据量，如果帧数大于1，则单帧大小WALKING_DOF，否则单帧大小16
      int len = period * frameSize;          //总的角度数据量
      dutyAngles = new char[len];//建立dutyAngles字符数组存储读取到的角度数据
      int readFromEE = 0;
      int readToWire = 0;
      while (len > 0) {
        //PTL("request " + String(min(WIRE_BUFFER, len)));
        Wire.requestFrom(DEVICE_ADDRESS, min(WIRE_BUFFER, len));//单次让iic eeprom芯片传输数据量
        readToWire = 0;
        do {
          if (Wire.available())//如果iic总线有数据
          dutyAngles[readFromEE++] = Wire.read();
          //将读取到的角度数据存入dutyAngles字符数组之中
        } while (--len > 0 && ++readToWire<WIRE_BUFFER);
      }//读数据结束
    }
    
//通过板载eeprom地址加载里面的数据，数据存储的时候先存储了技能的类别，然后下一个字节开始为技能的数据地址
    void loadDataByOnboardEepromAddress(int onBoardEepromAddress) {
      char skillType = EEPROM.read(onBoardEepromAddress);//读取技能类型
      unsigned int dataArrayAddress = EEPROMReadInt(onBoardEepromAddress + 1);
      delete[] dutyAngles;
#ifdef DEVELOPER
      PTF("free memory: ");
      PTL(freeMemory());//打印出来freememory，剩余内存
#endif
#ifdef I2C_EEPROM
      //如果技能类型为本能，则从I2CE2Prom里加载数据（根据数据指针地址，在技能类型的下一个）
      if (skillType == 'I') { //copy instinct data array from external i2c eeprom
        loadDataFromI2cEeprom(dataArrayAddress);
      }
      else                 //从progmem里面拷新技数据指针。
#endif                        
      {
        loadDataFromProgmem(dataArrayAddress) ;
      }
#ifdef DEVELOPER
      PTF("free memory: ");
      PTL(freeMemory());
#endif
    }
    //通过技能名字寻找数据地址
    void loadBySkillName(char* skillName) {//get lookup information from on-board EEPROM and read the data array from storage
      int onBoardEepromAddress = lookupAddressByName(skillName);
      if (onBoardEepromAddress == -1)
        return;
      loadDataByOnboardEepromAddress(onBoardEepromAddress);//从片上E2Prom里面加载数据（地址）
    }

    void info() {
      PTL("period: " + String(period) + ",\tdelayBetweenFrames: " + ",\texpected (pitch,roll): (" + expectedRollPitch[0]*degPerRad + "," + expectedRollPitch[1]*degPerRad + ")");
      for (int k = 0; k < period * (period > 1 ? WALKING_DOF : 16); k++) {
        PT(String((int8_t)dutyAngles[k]) + ", ");
      }
      PTL();
    }
};

Motion motion;

//在板载eeprom上分配技能地址，为社么这样做？
void assignSkillAddressToOnboardEeprom() {
  int skillAddressShift = 0;
  PT("\n* Assigning ");
  PT(sizeof(progmemPointer) / 2);
  PTL(" skill addresses...");
  for (byte s = 0; s < sizeof(progmemPointer) / 2; s++) { //save skill info to on-board EEPROM, load skills to SkillList
    PTL(s);
    byte nameLen = EEPROM.read(SKILLS + skillAddressShift++); //不带最后一个类型字符
    skillAddressShift += nameLen;
    char skillType = EEPROM.read(SKILLS + skillAddressShift++);
    if (skillType == 'N') // the address of I(nstinct) has been written in previous operation: saveSkillNameFromProgmemToOnboardEEPROM() in instinct.ino
      // if skillType == N(ewbility), save pointer address of progmem data array to onboard eeprom.
      // it has to be done for different sketches because the addresses are dynamically assigned

      EEPROMWriteInt(SKILLS + skillAddressShift, (int)progmemPointer[s]);
#if defined(I2C_EEPROM) && defined (MAIN_SKETCH)  //此处编译
    else
      s--;
#endif
    skillAddressShift += 2;
  }
  PTLF("Finished!");
}

inline byte pin(byte idx) {
  return EEPROM.read(PIN + idx);
}
inline byte remapPin(byte offset, byte idx) {
  return EEPROM.read(offset + idx);
}
//舵机角度范围
inline byte servoAngleRange(byte idx) {
  return EEPROM.read(SERVO_ANGLE_RANGE + idx);
}
//中位转换
inline int8_t middleShift(byte idx) {
  return EEPROM.read( MID_SHIFT + idx);
}

inline int8_t rotationDirection(byte idx) {
  return EEPROM.read(ROTATION_DIRECTION + idx);
}
//该函数用于读取舵机校准角度数据
inline int8_t servoCalib(byte idx) {
  return EEPROM.read( CALIB + idx);
}

// balancing parameters
#define ROLL_LEVEL_TOLERANCE 2//the body is still considered as level, no angle adjustment
#define PITCH_LEVEL_TOLERANCE 1
float levelTolerance[2] = {ROLL_LEVEL_TOLERANCE * radPerDeg, PITCH_LEVEL_TOLERANCE * radPerDeg}; //the body is still considered as level, no angle adjustment
#define LARGE_PITCH 75
//the following coefficients will be divided by 10.0 in the adjust() function. so (float) 0.1 can be saved as (int8_t) 1
//this trick allows using int8_t array insead of float array, saving 96 bytes and allows storage on EEPROM
#define panF 60
#define tiltF 60
#define sRF 50   //shoulder roll factor
#define sPF 12 //shoulder pitch factor
#define uRF 30 //upper leg roll factor
#define uPF 30 //upper leg pitch factor
#define lRF (-1.5*uRF) //lower leg roll factor 
#define lPF (-1.5*uPF)//lower leg pitch factor
#define LEFT_RIGHT_FACTOR 2
#define FRONT_BACK_FACTOR 2
#define POSTURE_WALKING_FACTOR 0.5
#ifdef POSTURE_WALKING_FACTOR
float postureOrWalkingFactor;
#endif

#ifdef X_LEG
int8_t adaptiveParameterArray[16][NUM_ADAPT_PARAM] = {
  { -panF, 0}, { -panF, -tiltF}, { -2 * panF, 0}, {0, 0},
  {sRF, -sPF}, { -sRF, -sPF}, { -sRF, sPF}, {sRF, sPF},
  {uRF, uPF}, {uRF, uPF}, { -uRF, uPF}, { -uRF, uPF},
  {lRF, lPF}, {lRF, lPF}, { -lRF, lPF}, { -lRF, lPF}
};
#else // >> leg
int8_t adaptiveParameterArray[16][NUM_ADAPT_PARAM] = {
  { -panF, 0}, { -panF / 2, -tiltF}, { -2 * panF, 0}, {0, 0},
  {sRF, -sPF}, { -sRF, -sPF}, { -sRF, sPF}, {sRF, sPF},
  {uRF, uPF}, {uRF, uPF}, {uRF, uPF}, {uRF, uPF},
  {lRF, lPF}, {lRF, lPF}, {lRF, lPF}, {lRF, lPF}
};
#endif

float RollPitchDeviation[2];
int8_t slope = 1;
inline int8_t adaptiveCoefficient(byte idx, byte para) {
  return EEPROM.read(ADAPT_PARAM + idx * NUM_ADAPT_PARAM + para);
}

float adjust(byte i) {
  float rollAdj, pitchAdj;
  if (i == 1 || i > 3)  {//check idx = 1
    bool leftQ = (i - 1 ) % 4 > 1 ? true : false;
    //bool frontQ = i % 4 < 2 ? true : false;
    //bool upperQ = i / 4 < 3 ? true : false;
    float leftRightFactor = 1;
    if ((leftQ && RollPitchDeviation[0]*slope  > 0 )
        || ( !leftQ && RollPitchDeviation[0]*slope  < 0))
      leftRightFactor = LEFT_RIGHT_FACTOR;
    rollAdj = fabs(RollPitchDeviation[0]) * adaptiveCoefficient(i, 0) * leftRightFactor;

  }
  else
    rollAdj = RollPitchDeviation[0] * adaptiveCoefficient(i, 0) ;

  return (
#ifdef POSTURE_WALKING_FACTOR
           (i > 3 ? postureOrWalkingFactor : 1) *
#endif
           // rollAdj + adaptiveCoefficient(i, 1) * ((i % 4 < 2) ? RollPitchDeviation[1] : abs(RollPitchDeviation[1])));
           rollAdj + RollPitchDeviation[1] * adaptiveCoefficient(i, 1) );
}
//该函数用于存储舵机校准校准角度数据
void saveCalib(int8_t *var) {
  for (byte i = 0; i < DOF; i++) {
    EEPROM.update(CALIB + i, var[i]);
    calibratedDuty0[i] = SERVOMIN + PWM_RANGE / 2 + float(middleShift(i) + var[i]) * pulsePerDegree[i] * rotationDirection(i);
  }
}
//此函数负责让i号关节舵机转到给定angle角度
void calibratedPWM(byte i, float angle) {
  currentAng[i] = angle;
  int duty = calibratedDuty0[i] + angle * pulsePerDegree[i] * rotationDirection(i);
  duty = max(SERVOMIN , min(SERVOMAX , duty));//保证duty在SERVOMIN和SERVOMAX之间，如果超出范围，按照MIN和MAX处理
  pwm.setPWM(pin(i), 0, duty);
  /*Serial.print(i);
  Serial.print(":");
  Serial.print(duty);
  Serial.print(",");*/
}
//此函数可一次性让小喵16个舵机转到目标角度
void allCalibratedPWM(char * dutyAng) {
  for (int8_t i = DOF - 1; i >= 0; i--) {
    calibratedPWM(i, dutyAng[i]);
  }
}

//关闭舵机，舵机靠PWM波驱动，当高电平占空比为4096时，意味着长为高电平
void shutServos() {
  delay(100);
  for (int8_t i = DOF - 1; i >= 0; i--) {
    pwm.setPWM(i, 0, 4096);
  }
}
//这个函数的作用是让小喵从当前的Posture移动到目标的Posture
void transform(char * target,  float speedRatio = 1, byte offset = 0){
  char *diff = new char[DOF - offset], maxDiff = 0;
  for (byte i = offset; i < DOF; i++) {
    diff[i - offset] =   currentAng[i] - target[i - offset];
    maxDiff = max(maxDiff, abs( diff[i - offset]));
  }
  byte steps = byte(round(maxDiff / 1.0/*degreeStep*/ / speedRatio));//默认速度是一步一度
  for (byte s = 0; s <= steps; s++)
    for (byte i = offset; i < DOF; i++) {
      float dutyAng = (target[i - offset] + (steps == 0 ? 0 : (1 + cos(M_PI * s / steps)) / 2 * diff[i - offset]));
      calibratedPWM(i,  dutyAng);
      delayMicroseconds(100);
    }
  delete [] diff;
  //  printList(currentAng);
  //  PTL();
}
//此函数用于执行一连串技能（posture)
void behavior(int n, char** skill, float *speedRatio, int *pause){
  for (byte i = 0; i < n; i++) {
    motion.loadBySkillName(skill[i]);
    transform( motion.dutyAngles, speedRatio[i]);
    delay(pause[i]);
  }
}

//short tools
template <typename T> int8_t sign(T val) {
  return (T(0) < val) - (val < T(0));
}

template <typename T> void printList(T * arr, byte len = DOF) {
  String temp = "";
  for (byte i = 0; i < len; i++) {
    temp += String(arr[i]);
    temp += '\t';
    //PT((T)(arr[i]));
    //PT('\t');
  }
  PTL(temp);
}
//这个函数是干嘛用的？
template <typename T> void printEEPROMList(int EEaddress, byte len = DOF) {
  for (byte i = 0; i < len; i++) {
    PT((T)(EEPROM.read(EEaddress + i)));
    PT('\t');
  }
  PTL();
}
//获取用户输入的一个字符
char getUserInput() {//limited to one character
  while (!Serial.available());
  return Serial.read();
}
