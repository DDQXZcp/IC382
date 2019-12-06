#define pi 3.1415926

//DEFINE FOR motor control////
#define MAXSPEED 2000 //mm per second 
#define MINSPEED 20 //mm per second

////////////L298N part////////
// connect motor controller pins to Arduino digital pins
// motor one
int enA = 10;
int in1 = 9;
int in2 = 8;
// motor two
int enB = 5;
int in3 = 7;
int in4 = 6;
// motor three
int enC = 10;
int in5 = 9;
int in6 = 8;
// motor four
int enD = 5;
int in7 = 7;
int in8 = 6;


//////////chassis part////////
  int Gvx = 0;//Global x velocity unit: mm/s
  int Gvy = 0;//Global y velocity unit: mm/s
  double theta = 0; //current angular biased from the original (in radian)
  double AngularSpeed = 0; //unit: radian/s
  int Rspeed[4]={0,0,0,0};//REAL SPEED, The speed of 4 motors : unit: mm/s
  int Mspeed[4]={0,0,0,0};//MAPPED SPEED, which falled in 0-255
  float r = 0;//radius of the wheel


void setup() {
  // put your setup code here, to run once:
  
  //////////L298N part/////////
  // set all the motor control pins to outputs
pinMode(enA, OUTPUT);
pinMode(enB, OUTPUT);
pinMode(in1, OUTPUT);
pinMode(in2, OUTPUT);
pinMode(in3, OUTPUT);
pinMode(in4, OUTPUT);
}

void loop() {
///////////bluetooth part/////////

    
  /////////chassis part//////////
  /////////The kinematics of the chassis///
    Rspeed[0] = Gvx * (-1) * sin(theta + pi / 4) + Gvy * cos(theta + pi / 4) + r * theta;
    Rspeed[1] = Gvx * (-1) * sin(theta + 3 * pi / 4) + Gvy * cos(theta + 3 * pi / 4) + r * theta;
    Rspeed[2] = Gvx * (-1) * sin(theta + 5 * pi / 4) + Gvy * cos(theta + 5 * pi / 4) + r * theta;
    Rspeed[3] = Gvx * (-1) * sin(theta + 7 * pi / 4) + Gvy * cos(theta + 7 * pi / 4) + r * theta;
 //////// Mspeed is the absolute value of Rspeed, and is mapped into 0~255 for pwm output////
    Mspeed[0] = map(abs(Rspeed[0]),0,MAXSPEED,0,255);
    Mspeed[1] = map(abs(Rspeed[1]),0,MAXSPEED,0,255);
    Mspeed[2] = map(abs(Rspeed[2]),0,MAXSPEED,0,255);
    Mspeed[3] = map(abs(Rspeed[3]),0,MAXSPEED,0,255);
    //////////First identify the direction of each motor and modify the H-bridge, then output the mapped pwm value, no pid control///
    if(Rspeed[0]> 0)
    {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(enA, Mspeed[0]);
      }
     if(Rspeed[0]< 0)
     {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enA, Mspeed[0]);
      }
     if(Rspeed[1]> 0)
    {
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enB, Mspeed[0]);
      }
     if(Rspeed[1]< 0)
     {
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enB, Mspeed[0]);
      }
      if(Rspeed[2]> 0)
    {
      digitalWrite(in5, HIGH);
      digitalWrite(in6, LOW);
      analogWrite(enC, Mspeed[0]);
      }
     if(Rspeed[2]< 0)
     {
      digitalWrite(in5, LOW);
      digitalWrite(in6, HIGH);
      analogWrite(enC, Mspeed[0]);
      }
     if(Rspeed[3]> 0)
    {
      digitalWrite(in7, HIGH);
      digitalWrite(in8, LOW);
      analogWrite(enD, Mspeed[0]);
      }
     if(Rspeed[3]< 0)
     {
      digitalWrite(in7, LOW);
      digitalWrite(in8, HIGH);
      analogWrite(enD, Mspeed[0]);
      }
//// 

 
}
// Below is the demo funtion from L298N module tutorial, just for reference
/*
  void demoOne()
{
// this function will run the motors in both directions at a fixed speed
// turn on motor A
digitalWrite(in1, HIGH);
digitalWrite(in2, LOW);
// set speed to 200 out of possible range 0~255
analogWrite(enA, 200);
// turn on motor B
digitalWrite(in3, HIGH);
digitalWrite(in4, LOW);
// set speed to 200 out of possible range 0~255
analogWrite(enB, 200);
delay(2000);
// now change motor directions
digitalWrite(in1, LOW);
digitalWrite(in2, HIGH);
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
delay(2000);
// now turn off motors
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(in3, LOW);
digitalWrite(in4, LOW);
}
void demoTwo()
{
// this function will run the motors across the range of possible speeds
// note that maximum speed is determined by the motor itself and the operating voltage
// the PWM values sent by analogWrite() are fractions of the maximum speed possible
// by your hardware
// turn on motors
digitalWrite(in1, LOW);
digitalWrite(in2, HIGH);
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
// accelerate from zero to maximum speed
for (int i = 0; i < 256; i++)
{
analogWrite(enA, i);
analogWrite(enB, i);
delay(20);
}
// decelerate from maximum speed to zero
for (int i = 255; i >= 0; --i)
{
analogWrite(enA, i);
analogWrite(enB, i);
delay(20);
}
// now turn off motors
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(in3, LOW);
digitalWrite(in4, LOW);
}
void loop()
{
demoOne();
delay(1000);
demoTwo();
delay(1000);
}
*/
