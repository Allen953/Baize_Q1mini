#define dt 45.0  //定义大腿长度
#define xt 50.0  //定义小腿长度

#define x_b -60.0  //轨迹坐标变换x轴
#define y_b 0.0   //轨迹坐标变换y轴

#define pi 3.1415  //圆周率Π
//将轨迹离散化处理后找出替代点的坐标
float xy[20][2]={
  75.0,55.0,
  75.0,60.0,
  75.0,65.0,
  75.0,70.0,
  75.0,75.0,
  75.0,80.0,
  75.0,75.0,
  75.0,70.0,
  75.0,65.0,
  75.0,60.0,
  75.0,55.0,
  75.0,50.0,
  75.0,45.0,
  75.0,40.0,
  75.0,35.0,
  75.0,30.0,
  75.0,35.0,
  75.0,40.0,
  75.0,45.0,
  75.0,50.0,
  };
//定义角度变量，存储8和12关节舵机角度数据
float angle[20][2]={
  0.0,0.0,
  0.0,0.0,
  0.0,0.0,
  0.0,0.0,
  0.0,0.0,
  0.0,0.0,
  0.0,0.0,
  0.0,0.0,
  0.0,0.0,
  0.0,0.0,
  0.0,0.0,
  0.0,0.0,
  0.0,0.0,
  0.0,0.0,
  0.0,0.0,
  0.0,0.0,
  0.0,0.0,
  0.0,0.0,
  0.0,0.0,
  0.0,0.0
};
//主要算法，将文档中的算法公式用程序代码实现。
void sf(){
//变量a,b分别存储8和12号舵机的关节角度数据
//，其他为中间变量，存储计算过程中的数据
  float xb2=0.0,a=0.0,a1=0.0,a2=0.0;
  float b=0.0;
  for(int i=0;i<=19;i++)//对现在坐标进行坐标系变换（平移操作）
  {
    xy[i][0]=xy[i][0]+x_b;
    xy[i][1]=xy[i][1]+y_b;
  }
//具体算法实现，程序反三角函数计算出来的
//角度表示方法为弧度，要切换成角度
  for(int i=0;i<=19;i++)
  {
    xb2=xy[i][0]*xy[i][0]+xy[i][1]*xy[i][1];
    a1=acos((dt*dt+xb2-xt*xt)/(2*sqrt(xb2)*dt))*180/pi;
    a2=atan(xy[i][1]/xy[i][0])*180/pi;
    a=a1+a2-90.0;
    b=acos((dt*dt+xt*xt-xb2)/(2*dt*xt))*180/pi;
    b=b-90.0;
    delay(3);
    angle[i][0]=round(a);
    angle[i][1]=round(b);
  }
}
//输出：将角度数据依照每帧8个数据的格式输出，一共20帧
void sc(){
  for(int i=0;i<=19;i++)
  {
    Serial.print(int(angle[i][0]),DEC);
    Serial.print(",");
    Serial.print(int(angle[i][0]),DEC);
    Serial.print(",");
    Serial.print(-int(angle[i][0]),DEC);
    Serial.print(",");
    Serial.print(-int(angle[i][0]),DEC);
    Serial.print(",");
    Serial.print(int(angle[i][1]),DEC);
    Serial.print(",");
    Serial.print(int(angle[i][1]),DEC);
    Serial.print(",");
    Serial.print(-int(angle[i][1]),DEC);
    Serial.print(",");
    Serial.print(-int(angle[i][1]),DEC);
    Serial.print(",");
    Serial.println();
    
  }
}
void setup() {
  Serial.begin(57600);
  sf();//计算轨迹离散化之后的舵机角度偏差值
  sc();
}

void loop() {
}
