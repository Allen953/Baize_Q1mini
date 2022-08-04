#include "OpenCat.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  /*
  int p_address=204,p_value=0;
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);
  Serial.print(lowByte,DEC);
  Serial.print(',');
  Serial.print(highByte,DEC);
  Serial.println(',');
  EEPROM.update(p_address, lowByte);
  EEPROM.update(p_address + 1, highByte);


  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);
  Serial.print(lowByte,DEC);
  Serial.print(',');
  Serial.print(highByte,DEC);
  Serial.print(',');
  p_value=((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
  Serial.print(p_value,DEC);
*/

 //把PROGMEM中的数据拷到E2Prom中
  unsigned int eeAddress=0,pgmAddress=progmemPointer[0];
  uint8_t period = pgm_read_byte(pgmAddress);//读取数据帧数
  byte frameSize = period > 1 ? WALKING_DOF : 16;//判断，如果帧数大于1，则单帧大小WALKING_DOF，否则单帧大小16
  int len = period * frameSize + SKILL_HEADER;   //步态或姿势数据总的大小
  int writtenToEE = 0;
  while (len > 0) {
    Wire.beginTransmission(DEVICE_ADDRESS);
    Wire.write((int)((eeAddress) >> 8));   // MSB
    Wire.write((int)((eeAddress) & 0xFF)); // LSB
    byte writtenToWire = 0;
        do {  if (eeAddress == EEPROM_SIZE) {
              PTL();
              PTL("I2C EEPROM overflow! You must reduce the size of your instincts file!\n");
              EEPROMOverflow = true;
              meow(3);
              return; }
        Wire.write((byte)pgm_read_byte(pgmAddress + writtenToEE++));//
        writtenToWire++;
        eeAddress++;
    } while ((--len > 0 ) && (eeAddress  % PAGE_LIMIT ) && (writtenToWire < WIRE_LIMIT));//be careful with the chained conditions,self-increment may not work as expected
    Wire.endTransmission();
    delay(6);  // needs 5ms for page write
    //PTL("\nwrote " + String(writtenToWire) + " bytes.");
  }
  //PTLF("finish copying to I2C EEPROM");
}

void loop() {
  // put your main code here, to run repeatedly:

}
