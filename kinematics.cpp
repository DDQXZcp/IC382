// Return a struct that contains pid

int Gvx = 0;//Global x velocity unit: mm/s
int Gvy = 0;//Global y velocity unit: mm/s
double theta = 0; //current angular biased from the original (in radian)
double AngularSpeed = 0; //unit: radian/s
int Rspeed[4]={0,0,0,0};//REAL SPEED, The speed of 4 motors : unit: mm/s
int Mspeed[4]={0,0,0,0};//MAPPED SPEED, which falled in 0-255
float r = 0;//radius of the wheel



Rspeed[0] = (-sin(theta)*cos(theta)*Gvx + cos(theta)*cos(theta)*Gvy + R*angular_speed) / r
Rspeed[1] = (-sin(theta+A2)*cos(theta)*Gvx + cos(theta+A2)*cos(theta)*Gvy + R*angular_speed) / r
Rspeed[2] = (-sin(theta+A3)*cos(theta)*Gvx + cos(theta+A3)*cos(theta)*Gvy + R*angular_speed) / r
