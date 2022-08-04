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
//两只老虎简谱的前16个音调
byte lh[16]={1,2,3,1,1,2,3,1,3,4,5,5,3,4,5,5};
void loop() {
  //依次发出两只老虎的16个音调
  for(int i=0;i<15;i++)
  {
    tone(beep,toned[lh[i]-1],250);
    Serial.println(toned[lh[i]-1],DEC);
    delay(250);
  }
  delay(200000);
}
