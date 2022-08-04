#define MAIN_SKETCH   //条件编译预备
#include "OpenCat.h"

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define PACKET_SIZE 42           //MPU FIFO缓存大小
#define OVERFLOW_THRESHOLD 128

//#if OVERFLOW_THRESHOLD>1024-1024%PACKET_SIZE-1   // when using (1024-1024%PACKET_SIZE) as the overflow resetThreshold, the packet buffer may be broken
// and the reading will be unpredictable. it should be replaced with previous reading to avoid jumping
#define FIX_OVERFLOW
//#endif
#define HISTORY 2
int8_t lag = 0;
//yaw,pitch,roll.这3个角度分别是绕Y,X,Z这3个轴的倾角
float ypr[3];
//重力向量
float yprLag[HISTORY][2];

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
// MPU control/status vars
bool dmpReady = false;  //如果DMP（数字运动处理器）初始化成功则为true
uint8_t mpuIntStatus;   //holds actual interrupt status byte from MPU
uint8_t devStatus;      //在操作每个设备后返回状态值(0 = success, !0 = error)
uint16_t packetSize;    //预计DMP（数字运动处理器）包大小(默认42字节)
uint16_t fifoCount;     //查看当前FIFO缓存数据大小（字节）
uint8_t fifoBuffer[PACKET_SIZE]; // FIFO缓存

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {//DMP数字运动处理器
  mpuInterrupt = true;
}

// https://brainy-bits.com/blogs/tutorials/ir-remote-arduino
#include <IRremote.h>
/*-----( 声明对象 )-----*/
IRrecv irrecv(IR_RECIEVER);     //创建'irrecv'对象
decode_results results;        //创建'decode_results'对象
String translateIR() //将红外遥控器的键值信号转换为字符串命令（基于收到的红外键值动作）
{
  switch (results.value) {
    case 0xFFA25D:                        return (F("sit"));        //坐
    case 0xFF629D:                        return (F("d"));          //趴着休息
    case 0xFFE21D:                        return (F("hi"));         //打招呼


    case 0xFF22DD:                        return (F("buttUp"));     //翘屁股
    case 0xFF02FD:                        return (F("balance"));    //站立平衡
    case 0xFFC23D:                        return (F("str"));        //伸懒腰

    case 0xFFE01F:                        return (F("pee"));        //撒尿
    case 0xFFA856:                        return (F("tr"));         //小跑
    case 0xFF906E:                        return (F("pu"));         //俯卧撑

    case 0xFF6897:                        return (F("wkL"));        //向左前进（walk）
    case 0xFF9867:                        return (F("wk"));         //前进
    case 0xFFB04E:                        return (F("wkR"));        //向右前进

    case 0xFF30CF:                        return (F("crL"));        //向左爬行
    case 0xFF18E7:                        return (F("cr"));         //爬行
    case 0xFF7A85:                        return (F("crR"));        //向右爬行

    case 0xFF10EE:                        return (F("bkL"));        //向左后退
    case 0xFF38C6:                        return (F("bk"));         //后退
    case 0xFF5AA4:                        return (F("bkR"));        //向右后退

    case 0xFF42BC:                        return (F("tb"));         //燃烧生命模式
    case 0xFF4AB5:                        return (F("zero"));       //舵机回中（自定义技能）
    case 0xFF52AC:                        return (F("rc"));         //翻倒复位
    case 0xFFFFFFFF: return (""); //Serial.println(" REPEAT");
    //当遥控器实际键值与程序不匹配时，输出实际键值，用于修改程序
    default: {
        Serial.println(results.value, HEX);
      }
      return ("");
  }
  //delay(100); // 不需要马上重复，由于主loop()函数比较慢
  // 这种控制方式可以以其他方式规划，:
  // 可以在按键上以不同的速度规划前进/后退步态
}

char token;
#define CMD_LEN 10
char *lastCmd = new char[CMD_LEN];
char *newCmd = new char[CMD_LEN];
byte newCmdIdx = 0;
byte hold = 0;
int8_t offsetLR = 0;
bool checkGyro = true;
int8_t countDown = 0;

uint8_t timer = 0;

byte firstMotionJoint;
byte jointIdx = 0;


unsigned long usedTime = 0;

void setup() {
  pinMode(BUZZER, OUTPUT);
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();//初始化wire库,并且加入到I2C网络
  //Wire.setClock(400000);
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(57600);
  Serial.setTimeout(10);
  while (!Serial);//while (Serial) Arduino会在串口通讯准备好的时候返回真

  while (Serial.available() && Serial.read()); // 清空串口缓存
  delay(100);
  PTLF("\n* Start *");
  PTLF("Initialize I2C");
  PTLF("Connect MPU6050");
  mpu.initialize();
  //do
  {
    delay(500);
    PTLF("Test connection");//连接确认
    PTL(mpu.testConnection() ? F("MPU successful") : F("MPU failed"));//sometimes it shows "failed" but is ok to bypass.
  } //while (!mpu.testConnection());
  
  do {//加载并配置DMP
    PTLF("Initialize DMP");
    devStatus = mpu.dmpInitialize();//初始化MPU6050并获得初始化结果
    delay(500);
    // supply your own gyro offsets here, scaled for min sensitivity
    for (byte i = 0; i < 4; i++) {
      PT(EEPROMReadInt(MPUCALIB + 4 + i * 2));
      PT(" ");
    }
    PTL();
    mpu.setZAccelOffset(EEPROMReadInt(MPUCALIB + 4));//校正mpu6050，详见https://blog.csdn.net/wulala789/article/details/99618189?web=web
    mpu.setXGyroOffset(EEPROMReadInt(MPUCALIB + 6));
    mpu.setYGyroOffset(EEPROMReadInt(MPUCALIB + 8));
    mpu.setZGyroOffset(EEPROMReadInt(MPUCALIB + 10));

    //确认mpu6050是否正常工作，如果正常工作，则devStatus的值为0，否则打印错误代号
    if (devStatus == 0) {
      PTLF("Enable DMP");// 既然已经准备好了，那就打开DMP
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      PTLF("Enable interrupt");
      attachInterrupt(INTERRUPT, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      PTLF("DMP ready!");//设置DMP的准备状态，这样loop()函数知道是否能用
      dmpReady = true;

      // get expected DMP packet size for later comparison
      //获得预期的DMP数据包大小，以便以后进行比较
      packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      //ERROR!
      //1 = initial memory load failed
      //2 = DMP配置更新失败
      //(如果设备无法工作,通常错误代码为1)
      PTLF("DMP failed (code ");
      PT(devStatus);
      PTLF(")");
      PTL();
    }
  } while (devStatus);
  //打开音乐
  playMelody(MELODY);
  
    irrecv.enableIRIn(); // Start the receiver;开启红外接收头
  
  assignSkillAddressToOnboardEeprom();//在芯片内部E2prom里面分配技能地址
  PTL();
  // 开启PCA9685，并初始化舵机
  { pwm.begin();
    pwm.setPWMFreq(60 * PWM_FACTOR); // Analog servos run at ~60 Hz updates
    delay(200);

    strcpy(lastCmd, "rest");//将先前命令设置为复位
    motion.loadBySkillName("rest");
    for (int8_t i = DOF - 1; i >= 0; i--) {
      pulsePerDegree[i] = float(PWM_RANGE) / servoAngleRange(i);
      servoCalibs[i] = servoCalib(i);
      calibratedDuty0[i] =  SERVOMIN + PWM_RANGE / 2 + float(middleShift(i) + servoCalibs[i]) * pulsePerDegree[i]  * rotationDirection(i) ;
      calibratedPWM(i, motion.dutyAngles[i]);
      delay(100);
    }
    randomSeed(analogRead(0));//use the fluctuation of voltage caused by servos as entropy pool
    shutServos();
    token = 'd';
  }
  beep(30);

  pinMode(BATT, INPUT);
  pinMode(VCC, OUTPUT);
  pinMode(TRIGGER, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO, INPUT); // Sets the echoPin as an Input
  digitalWrite(VCC, HIGH);
  int t = 0;
  int minDist, maxDist;
  meow();
  delay(500);
  //meow();
}

void loop() { 
    newCmd[0] = '\0';
    newCmdIdx = 0;

    //命令接受模块
    if (irrecv.decode(&results)) {
      String IRsig = translateIR();//将红外信号转化成字符串命令
      //接收到非空信号
      if (IRsig != "") {
        strcpy(newCmd, IRsig.c_str());//将字符串转化成字符数组给newCmd

        Serial.print("newCmd=");
        Serial.print(newCmd);
      
        if (!strcmp(newCmd, "d"))//如果新命令为d（c语言中strcmp比较两个字符串的大小，两个字符串相同时返回0）
          token = 'd';
        else if (!strcmp(newCmd, "tb")) {//强化模式
          if (checkGyro)
            countDown = 4;
          checkGyro = !checkGyro;
        }
        else if (!strcmp(newCmd, "hi")) {//如果新命令为hi
          motion.loadBySkillName("sit");
          transform(motion.dutyAngles);
          char **bList = new char*[2];
          bList[0] = "hi";
          bList[1] = "hi2";
          float speedRatio[2] = {1, 1};
          int pause[2] = {0, 0};
          for (byte i = 0; i < 4; i++)
            behavior(2, bList, speedRatio, pause);
          meow();
          delete []bList;
          delay(200);
          motion.loadBySkillName("sit");
          transform( motion.dutyAngles);
          strcpy(newCmd, "rest");
        }
        else if (!strcmp(newCmd, "rc")) {
          char **bList = new char*[10];
          bList[0] = "rc1";
          bList[1] = "rc2";
          bList[2] = "rc3";
          bList[3] = "rc4";
          bList[4] = "rc5";
          bList[5] = "rc6";
          bList[6] = "rc7";
          bList[7] = "rc8";
          bList[8] = "rc9";
          bList[9] = "rc10";
          float speedRatio[10] = {2, 2, 2, 10, 5, 10, 5, 5, 5, 2};
          int pause[10] = {500, 500, 500, 0, 0, 0, 0, 0, 0, 0};
          behavior( 10, bList, speedRatio, pause);
          strcpy(newCmd, "rest");
          delete []bList;
        }
        else if (!strcmp(newCmd, "pu")) {
          char **bList = new char*[2];
          bList[0] = "pu1";
          bList[1] = "pu2";
          float speedRatio[2] = {2, 2};
          int pause[2] = {0, 0};
          for (byte i = 0; i < 3; i++)
            behavior(2, bList, speedRatio, pause);
          strcpy(newCmd, "rest");
          meow();
          delete []bList;
        }
        else
          token = 'k';
        newCmdIdx = 2;

      Serial.print(",token=");
      Serial.print(token);
      Serial.print(",lastCmd=");
      Serial.print(lastCmd);
      Serial.print(",newCmdIdx=");
      Serial.println(newCmdIdx);
      delay(5);
      }
      irrecv.resume(); // receive the next value
    }
    
    
}
