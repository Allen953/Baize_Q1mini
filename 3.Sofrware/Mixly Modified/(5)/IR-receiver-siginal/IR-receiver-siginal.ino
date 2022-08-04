#include <IRremote.h>  //引入红外遥控库文件
#define RECV_PIN 4    //将数字4用RECV_PIN替代，方便辨认使用
IRrecv irrecv(RECV_PIN);  //将RECV_PIN号引脚用于接收红外信号
decode_results results;

void setup() {
  //开启串口用于打印红外接收头接收到的键值
  Serial.begin(9600);  
  irrecv.enableIRIn();  //初始化红外遥控器
}

void loop() {
  /*irrecv.decode(&results)函数对
   *接收到的IR信号进行解码并将其存储在变量中
   *没有收到任何内容时返回0
  */
  if(irrecv.decode(&results))  //判断如果接收到红外遥控按键键值
  {
//每次接收到信号，则从串口打印信号值（DEC为10进制，HEX为16进制）
    Serial.println(results.value,DEC);
    irrecv.resume();  //清空原有信号，接收下一个指令
  }
  else
  Serial.println("no signal");//如果没有信号接收到，则在串口打印无信号
  delay(1000);   //每隔1000ms接受一次，防止串口刷新过快
}
