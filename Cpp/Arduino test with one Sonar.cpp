/**
 * @file              OneSonarObstacleAvoiding
 * @author            Ucef Mountacer
 * @license           open
 *  
 * @date              april 2019
 * @brief             This file define the obstacle avoiding using potential field
 *                    and one sonar
 * @Important         add Brainzy.h and BrainzyPinout.h libraries. However the code can be used with any robot
 *                    Code lacks control module over mototrs because it depends on the application
 */


#include <BRAINZY.h>
#include <BrainzyPinout.h>
#include <math.h>

#define D1 28
#define D2 30

// 1 sonar version 
const int trigPin = 28;
const int echoPin = 30;
// Variables for the duration and the distance
long duration;
float distance;
// potential variables
int Ka=10;
int Kr=5;
float F[]={};
//Vector<int> F;   //total force
float Fax,Fay,Frx,Fry,Fx,Fy;  //x and y constitutives
float angle=0;
int velocity=20;
float Xt=1000;
float Yt=1000;  
float Xo,Yo;
float distance_to_target=0;  
float Xrb,Yrb; //robot position
float R = 0;
float x,y;
const float pi = 3.14;
int i=0;

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600);   
  
}

void loop() {
     
  distance = calculateDistance(); //constantly calculate distance
  angle = PotentialMethod(distance); // obstacle detection function. See below 
  angle = angle *180 / pi;
  Serial.print(angle);
  Serial.print('\t');
  Serial.println(distance);
  delay(100);
  
 
}


float PotentialMethod(float dist){
  //F[]={};
  float theta;
  if(dist<40){
        //this is where the obstacle is near
        Xrb = Robby.xPositionRead();
        Yrb = Robby.yPositionRead();
        distance_to_target = sqrt(pow((Xt - Xrb),2)+pow((Yt - Yrb),2));
        Fax=-Ka * (Xrb - Xt) / distance_to_target;
        Fay=-Ka * (Yrb - Yt) / distance_to_target;

        ////////////////////////////////
        R=pi * Robby.angularPositionRead() / 180;
        x=dist * sin(R);
        y=dist * cos(R);
        Xo=Xrb + x;
        Yo=Yrb + y;
        /////////////////////////////////
        Frx=-Kr * (Xo - Xrb)/pow(dist,3); 
        Fry=-Kr * (Yo - Yrb)/pow(dist,3);;   
        F[0]=Fax+Frx;   
        F[1]=Fay+Fry;                        
        //F.PushBack(Fax+Frx);               //because the sonar is on the Y axis
        //F.PushBack(Fay+Fry);
        Fx=(float)F[0];
        Fy=(float)F[1];
        theta=atan(Fy/Fx);
  }
  else{
        theta = 0.0;
  }
  return theta;
  
}

//works
int calculateDistance(){ 
  
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
  distance= duration*0.034/2;
  return distance;
}
