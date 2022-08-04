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
    //accident block
    //for obstacle avoidance and auto recovery

    //命令接受模块
    if (irrecv.decode(&results)) {
      String IRsig = translateIR();//将红外信号转化成字符串命令
      //接收到非空信号
      if (IRsig != "") {
        strcpy(newCmd, IRsig.c_str());//将字符串转化成字符数组给newCmd
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
      }
      irrecv.resume(); // receive the next value
    }
    
    if ( Serial.available() > 0) {
      token = Serial.read();
      newCmdIdx = 3;
    }
    if (newCmdIdx) {
      PTL(token);
      beep(newCmdIdx * 4);
      // this block handles argumentless tokens
      switch (token) {
        case 'h': {   PTLF("* Help Info *");  break;}
        case 'd': {
            motion.loadBySkillName("rest");
            transform( motion.dutyAngles);
            PTLF("shut down servos");
            shutServos();
            break;
          }
        case 's': {   PTLF("save calibration");     saveCalib(servoCalibs);  break;}
        case 'a': {//加载校准数据（从eeprom加载到内存）
            PTLF("abort calibration");
            for (byte i = 0; i < DOF; i++) {
              servoCalibs[i] = servoCalib(i);
            }
            break;
          }

        // this block handles array like arguments
        case 'i': //indexed joint motions: joint0, angle0, joint1, angle1, ...
        case 'l': //list of all 16 joint: angle0, angle2,... angle15
          //case 'o': //for melody
          {
            String inBuffer = Serial.readStringUntil('~');
            int8_t numArg = inBuffer.length();
            char* list = inBuffer.c_str();
            if (token == 'i') {
              for (int i = 0; i < numArg; i += 2) {
                calibratedPWM(list[i], list[i + 1]);
              }
            }
            else if (token == 'l') {
              allCalibratedPWM(list);
            }
            break;
          }
        case 'j': { //show the list of current joint anles
            printList((int8_t*)currentAng);
            break;
          }
        case 'c': //校准
        case 'm': //move joint to angle
        case 'u': //meow (repeat, increament)
        case 'b': //beep(tone, duration): tone 0 is pause, duration range is 0~255
          {
            int8_t target[2] = {};
            String inBuffer = Serial.readStringUntil('\n');
            byte inLen = 0;
            strcpy(newCmd, inBuffer.c_str());
            char *pch;
            pch = strtok (newCmd, " ,");
            for (byte c = 0; pch != NULL; c++)
            {
              target[c] = atoi(pch);
              pch = strtok (NULL, " ,");
              inLen++;
            }
            if (token == 'c') {
              //PTLF("calibrating [ targetIdx, angle ]: ");
              if (strcmp(lastCmd, "c")) { //first time entering the calibration function
                motion.loadBySkillName("calib");
                transform( motion.dutyAngles);
              }
              if (inLen == 2)
                servoCalibs[target[0]] = target[1];
              PTL();
              for (byte i = 0; i < DOF; i++) {
                PT(i);
                PT(",\t");
              }
              PTL();
              printList(servoCalibs);
              yield();
            }
            else if (token == 'm') {
              //SPF("moving [ targetIdx, angle ]: ");
              motion.dutyAngles[target[0]] = currentAng[target[0]] = target[1];
            }
            else if (token == 'u') {
              meow(target[0], 0, 50, 200, 1 + target[1]);
            }
            else if (token == 'b') {
              beep(target[0], (byte)target[1]);
            }
            PT(token);
            printList(target, 2);
            if (token == 'c' || token == 'm') {
              int duty = SERVOMIN + PWM_RANGE / 2 + float(middleShift(target[0])  + servoCalibs[target[0]] + motion.dutyAngles[target[0]]) * pulsePerDegree[target[0]] * rotationDirection(target[0]);
              pwm.setPWM(pin(target[0]), 0,  duty);
            }
            break;
          }
        default: if (Serial.available() > 0) {
            String inBuffer = Serial.readStringUntil('\n');
            strcpy(newCmd, inBuffer.c_str());
          }
      }
      while (Serial.available() && Serial.read()); //刷新剩余的串行缓冲区，以防命令解析错误
      //如果新的命令不为空而且与旧命令不同
      if (strcmp(newCmd, "") && strcmp(newCmd, lastCmd) ) {
        //      PT("compare lastCmd ");
        //      PT(lastCmd);
        //      PT(" with newCmd ");
        //      PT(token);
        //      PT(newCmd);
        //      PT("\n");
        if (token == 'w') {}; //some words for undefined behaviors

        if (token == 'k') { //validating key
          motion.loadBySkillName(newCmd);
          char lr = newCmd[strlen(newCmd) - 1];//找到新命令字符数组的最后一个字母
          offsetLR = (lr == 'L' ? 15 : (lr == 'R' ? -15 : 0));
          //如果lr为L，则offsetLR为15，如果lr为R，则offsetLR为-15；否则为0
          //motion.info();
          timer = 0;
          //如果新命令不是balance(站立）,lifted(俯卧撑姿势）,dropped（伸懒腰）
          if (strcmp(newCmd, "balance") && strcmp(newCmd, "lifted") && strcmp(newCmd, "dropped") )
            strcpy(lastCmd, newCmd);

          postureOrWalkingFactor = (motion.period == 1 ? 1 : POSTURE_WALKING_FACTOR);
          
          //如果是posture指令，则从0号关节开始
          //如果是步态指令,如果腿部自由度为8,从8号关节开始
          //               如果腿部自由度为12,从4号关节开始
          firstMotionJoint = (motion.period == 1) ? 0 : DOF - WALKING_DOF;
          transform( motion.dutyAngles,  1, firstMotionJoint);
          jointIdx = DOF;
          //如果新命令为rest，则关闭舵机
          if (!strcmp(newCmd, "rest")) {
            shutServos();
            token = 'd';
          }
          
        }
        else {
          lastCmd[0] = token;
          memset(lastCmd + 1, '\0', CMD_LEN - 1);
        }
      }
    }



    //运动模块
                        {
                          if (token == 'k') {
                            if (jointIdx == DOF) {
                                timer++;
                                if (timer == motion.period) {
                                  timer = 0;
                                  if (countDown == 0)
                                    checkGyro = true;
                                  else countDown--;
                                }
                              jointIdx =0;
                            }
                           //jointIdx为当前关节号，firstMotionJoint为起始运动关节号，motion.period为数据帧数
                            if (jointIdx < firstMotionJoint && motion.period > 1) {
                              calibratedPWM(jointIdx, (jointIdx != 1 ? offsetLR : 0) //看是向左还是向右
                                            + 10 * sin (timer * (jointIdx + 2) * M_PI / motion.period) //look around
                                            + (checkGyro ? adjust(jointIdx) : 0)
                                           );
                            }
                            else if (jointIdx >= firstMotionJoint) {
                              int dutyIdx = timer * WALKING_DOF + jointIdx - firstMotionJoint;
                              calibratedPWM(jointIdx, motion.dutyAngles[dutyIdx]+ adjust(jointIdx)
                                           );
                            }
                            jointIdx++;
                          }
                        }
}
