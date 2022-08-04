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
//检查身体运动
void checkBodyMotion()  {
  if (!dmpReady) return;
  // 等待MPU中断或额外的数据包可用
  if (mpuInterrupt || fifoCount >= packetSize)
  { //重置中断标志并获取INT_STATUS字节
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    // 获取当前FIFO大小
    fifoCount = mpu.getFIFOCount();
    //PTL(fifoCount);
    //检查是否有溢出（除非我们的代码效率太低，否则永远不会发生）
    if ((mpuIntStatus & 0x10) || fifoCount > OVERFLOW_THRESHOLD) {
      //重置，以便我们可以干净地继续
      mpu.resetFIFO();
      //否则，检查DMP数据就绪中断（应该经常发生）
      lag = (lag - 1 + HISTORY) % HISTORY;
    }
    else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      // read a packet from FIFO（FIFO是一种先进先出的数据缓存器，）
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      /* 如果有> 1个数据包可用，在此跟踪FIFO计数
      （这使我们无需等待中断即可立即阅读更多内容）*/
      fifoCount -= packetSize;

      //以度数显示欧拉角
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      ypr[1] = -ypr[1] ;

      // overflow is detected after the ypr is read. it's necessary to keep a lag recrod of previous reading.  -- RzLi --
      for (byte g = 0; g < 2; g++) {
        yprLag[lag][g] = ypr[g + 1];
        ypr[g + 1] = yprLag[(lag - 1 + HISTORY) % HISTORY][g] ;
      }
      lag = (lag + 1) % HISTORY;
      // --
      //deal with accidents
      //如果实际读取pitch角度（前后倾角）大于75度
      if (fabs(ypr[1])*degPerRad > LARGE_PITCH) {
        if (!hold) {
          token = 'k';
          strcpy(newCmd, ypr[1]*degPerRad > LARGE_PITCH ? "lifted" : "dropped");
          newCmdIdx = 1;
        }
        hold = 10;
      }
      // recover
      else if (hold) {
        if (hold == 10) {
          token = 'k';
          strcpy(newCmd, "balance");
          newCmdIdx = 1;
        }
        hold --;
        if (!hold) {
          char temp[CMD_LEN];
          strcpy(temp, newCmd);
          strcpy(newCmd, lastCmd);
          strcpy(lastCmd, temp);
          newCmdIdx = 1;
          meow();
        }
      }
      //计算偏差，roll角和pitch角
      for (byte i = 0; i < 2; i++) {
        RollPitchDeviation[i] = ypr[2 - i] - motion.expectedRollPitch[i];
        //PTL(RollPitchDeviation[i]);
        RollPitchDeviation[i] = sign(ypr[2 - i]) * max(fabs(RollPitchDeviation[i]) - levelTolerance[i], 0);//filter out small angles
      }
      //PTL(jointIdx);
    }
  }
}

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
    // MPU block
#ifdef GYRO //if opt out the gyro, the calculation can be really fast
    if (checkGyro && countDown == 0)
      checkBodyMotion();
#endif

      Serial.print("token=");
      Serial.print(token);
      Serial.print(",newCmd=");
      Serial.print(newCmd);
      Serial.print(",lastCmd=");
      Serial.print(lastCmd);
      Serial.print(",newCmdIdx=");
      Serial.println(newCmdIdx);
      delay(5);
}
