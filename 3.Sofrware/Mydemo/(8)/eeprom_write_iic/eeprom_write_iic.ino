#include <Wire.h>
#define ADDRESS_AT24C32 0x50
word wordAddress=0x0F00;//数据存储地址
char dar[] = "darling"; //存储字符数组
char buffer[30]; 
int i;
void setup()
{
     Wire.begin();
     Serial.begin(9600);
     Wire.beginTransmission(ADDRESS_AT24C32);
     Wire.write(highByte(wordAddress));
     Wire.write(lowByte(wordAddress));
     for (i = 0; i < sizeof(dar); i++)
         Wire.write(dar[i]);//将字符逐个写入芯片
     Wire.endTransmission();    
     delay(10); //等待写入完成
}
 
void loop()
{
     Wire.beginTransmission(ADDRESS_AT24C32);
     Wire.write(highByte(wordAddress));
     Wire.write(lowByte(wordAddress));
     Wire.endTransmission();
     Wire.requestFrom(ADDRESS_AT24C32, sizeof(dar));
     if(Wire.available() >= sizeof(dar))
     {
         for (i = 0; i < sizeof(dar); i++)
             buffer[i] = Wire.read();
     }
     Serial.print(buffer);//打印字符数组
     delay(200000);
}
