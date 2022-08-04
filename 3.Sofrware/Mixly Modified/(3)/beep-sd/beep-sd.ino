#define beep 5
void setup() {
  Serial.begin(57600);
}
//7个低音的震动频率：1，2，3，4，5，6，7
const int toned[7] = {262,294,330,349,392,440,494};
//7个中音的振动频率
const int tonez[7] = {523,587,659,698,784,880,988};
//7个高音的振动频率
const int toneg[7] = {1046,1175,1318,1397,1568,1760,1967};
void loop() {
  //依次发出低音的7个音调
  for(int i=0;i<7;i++)
  {
    tone(beep,toned[i],250);
    Serial.println(toned[i],DEC);
    delay(500);
  }
  //依次发出中音的7个音调
  for(int i=0;i<7;i++)
  {
    tone(beep,tonez[i],250);
    Serial.println(tonez[i],DEC);
    delay(500);
  }
  //依次发出高音的7个音调
  for(int i=0;i<7;i++)
  {
    tone(beep,toneg[i],250);
    Serial.println(toneg[i],DEC);
    delay(500);
  }
  delay(2000);
}
