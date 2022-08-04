#define beep 5//定义蜂鸣器宏
void setup() {
  pinMode(beep,OUTPUT);
}

void loop() {//设置蜂鸣器以200ms为周期bb响
  digitalWrite(beep,HIGH);
  delay(100);
  digitalWrite(beep,LOW);
  delay(100);
}
