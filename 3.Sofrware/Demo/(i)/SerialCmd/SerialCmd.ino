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



}
