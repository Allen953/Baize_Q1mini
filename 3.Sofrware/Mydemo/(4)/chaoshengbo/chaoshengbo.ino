#define VCC 8  //定义8号IO口为VCC（电源）
#define Trig 9 //定义9号IO口连接Trig
#define Echo 10 //定义10号IO口连接Echo
float cm; //定义全局变量cm存储距离
void setup() 
{ 
  Serial.begin(9600); //打开串口
  pinMode(VCC,OUTPUT); //初始化引脚模式
  digitalWrite(VCC,HIGH); //给模块上电
  pinMode(Trig, OUTPUT); 
  pinMode(Echo, INPUT); 
} 
void loop() 
{ 
  //发一个10ms的高脉冲去触发TrigPin 
  digitalWrite(Trig, LOW); 
  delayMicroseconds(2); 
  digitalWrite(Trig, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(Trig, LOW); 
  //pulseIn函数可以检测出高低电平脉冲宽度
  cm=pulseIn(Echo,HIGH)/58.0; //算成厘米 
  cm=(int(cm * 100.0))/100.0;//保留两位小数 
  Serial.print(cm); 
  Serial.print("cm"); 
  Serial.println(); 
  delay(100); 
}

