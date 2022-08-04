#include "OpenCat.h"//在EEPROM保存标定数据和常数

void setup() {
  Serial.begin(57600);//初始化并启用串口
  Serial.setTimeout(5);//setTimeout函数用于设置设备等待数据流的最大时间间隔
  delay(1);
  while (!Serial);//while (Serial) Arduino会在串口通讯准备好的时候返回真

  //更新音符数据
  EEPROM.update(MELODY, sizeof(melody));//第一个字节存储的还是音乐的长度（音符数量） 在1023字节处
  for (byte i = 0; i < sizeof(melody); i++)
    EEPROM.update(MELODY - 1 - i, melody[i]);//从1022字节开始
    
  PTLF("Reset joint calibration? (Y/n)");
  while (!Serial.available());
  char resetJointCalibrationQ = Serial.read();
  //如果输入大写字母Y，则更新所有关节校准角度值为0度
  for (byte i = 0; i < DOF; i++) {
    if (resetJointCalibrationQ == 'Y')
      EEPROM.update(CALIB + i, calibs[i]);        //存储舵机角度校准数据
    EEPROM.update(PIN + i, pins[i]);              //将关节映射表写入e2prom的前16个字节
    EEPROM.update(MID_SHIFT + i, middleShifts[i]);              //中位转换，存储中位转换数据
    EEPROM.update(ROTATION_DIRECTION + i, rotationDirections[i]);//将舵机旋转方向数据写入e2prom
    EEPROM.update(SERVO_ANGLE_RANGE + i, servoAngleRanges[i]);   //将舵机角度范围数据写入e2prom
    for (byte para = 0; para < NUM_ADAPT_PARAM; para++) {         //存储平衡适应参数数据
    EEPROM.update(ADAPT_PARAM + i * NUM_ADAPT_PARAM + para, round(adaptiveParameterArray[i][para]));
    }
  }
}

void loop() {
}
