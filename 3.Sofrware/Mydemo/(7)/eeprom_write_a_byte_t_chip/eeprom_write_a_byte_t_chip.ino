#include <EEPROM.h>  //引入库文件
void setup() {
  //在eeprom地址0处写入字符'I'
  EEPROM.write(0,35);
}
void loop() {
}
