#include "OpenCat.h"

void setup() {
  Serial.begin(57600);//初始化并启用串口
  Serial.setTimeout(5);//setTimeout函数用于设置设备等待数据流的最大时间间隔
  delay(1);
  while (!Serial);//while (Serial) Arduino会在串口通讯准备好的时候返回真
  
  int skillAddressShift = 0;
  unsigned int i2cEepromAddress = 0; //won't hurt if unused

  PTLF("\n* Update Instincts? (Y/n)");
  while (!Serial.available());
  char choice = Serial.read();
  PT(choice == 'Y' ? "Will" : "Won't");
  PTL(" overwrite Instincts on external I2C EEPROM!");
  PTLF("Saving skill info...");
  for (byte s = 0; s < NUM_SKILLS; s++) {//save skill info to on-board EEPROM
    byte len = strlen(skillNameWithType[s]);
    EEPROM.update(SKILLS + skillAddressShift++, len - 1); //技能名字中最后一个字符为类型,本能在外挂eeprom,新技在progmem
    PT(skillNameWithType[s][len - 1] == 'I' ? "I nstinct:\t" : "N ewbility:\t");//如果类型为本能则打印I nstinct:，否则打印N ewbility:
    for (byte l = 0; l < len; l++) {
      PT(skillNameWithType[s][l]);
      EEPROM.update(SKILLS + skillAddressShift++, skillNameWithType[s][l]);
    }
    PTL();
    //PTL("Current EEPROM address is " + String(SKILLS + skillAddressShift));
    if (!EEPROMOverflow)
      if (skillNameWithType[s][len - 1] == 'I' && choice == 'Y') { //  if there's instinct and there's i2c eeprom, and user decide to update.
        // save the data array to i2c eeprom. its address will be saved to onboard eeprom
        EEPROMWriteInt(SKILLS + skillAddressShift, i2cEepromAddress);
        copyDataFromPgmToI2cEeprom(i2cEepromAddress,  (unsigned int) progmemPointer[s]);
      }
    skillAddressShift += 2; // 地址为整形数据（2字节）
  }
  PTLF("  *********** Notice! *********");
  PTLF("    Maximal storage of onboard EEPROM is 1024 bytes.");
  PTF("\tInstinctive dictionary used ");
  PT(SKILLS + skillAddressShift);
  PT(" bytes (");
  PT(float(100) * (SKILLS + skillAddressShift) / 1024);
  PTLF(" %)!");
  if (choice == 'Y') {
    PTF("    Maximal storage of external I2C EEPROM is ");
    PT(EEPROM_SIZE);
    PTLF(" bytes.");
    PT("\tInstinctive data used ");
    PT(i2cEepromAddress);
    PT(" bytes (");
    PT(float(100)*i2cEepromAddress / EEPROM_SIZE);
    PTLF(" %)!");
  }
  PTLF("  *****************************");
  PTLF("Finished!");
}

void loop() {
  // put your main code here, to run repeatedly:

}
