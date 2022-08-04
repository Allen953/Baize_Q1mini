#include <Wire.h>//调用iic通讯库
//AT24C32D存储芯片(eeprom)的I2C通讯地址
#define ADDRESS_AT24C32 0x50
int i;
word wordAddress = 0x0000;//读取数据地址
char buffer[5]; //定义一个数组存储读取到的数据
void setup()
{
     Wire.begin();//初始化iic接口
     Serial.begin(9600);//初始化串口
}
void loop()
{
     //开始一次传输数据，开始给从机发送一个地址
     Wire.beginTransmission(ADDRESS_AT24C32);
//写地址高低位，每次只能写入一个字节，（地址占两个字节）分两次写入
     Wire.write(highByte(wordAddress));//写地址高位
     Wire.write(lowByte(wordAddress));//写地址地位
     Wire.endTransmission();
//主设备请求从设备5个字节数据
//小于arduino的iic通讯缓存-32字节,防止溢出
     Wire.requestFrom(ADDRESS_AT24C32,5);
     if(Wire.available()>0)//如果缓存有数据
     {
         for (i = 0;i<5; i++)
             buffer[i] = Wire.read();//读取
     }
     //打印数据
     for(i = 0;i<5; i++)
     {
         Serial.print((buffer[i]),DEC);//打印
         Serial.print(", ");
     }
     //再次请求数据
     Wire.requestFrom(ADDRESS_AT24C32,5);
     if(Wire.available()>0)
     {
         for (i = 0; i<5; i++)
             buffer[i] = Wire.read();
     }
     for(i = 0; i<5; i++)
     {
         Serial.print((buffer[i]),DEC);
         Serial.print(", ");
     }
     Serial.println();
     delay(200000);
}
