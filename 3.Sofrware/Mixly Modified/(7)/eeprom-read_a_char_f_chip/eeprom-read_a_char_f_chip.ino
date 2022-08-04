#include <EEPROM.h>  //引入库文件
void setup() {
  //在eeprom地址0处读出字符
  Serial.begin(9600);
  Serial.println(char(EEPROM.read(0)));
}
void loop() {
}
