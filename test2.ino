#include <BRAINZY.h>
#include <Vector.h>
#include <math.h>


// Defines Trig and Echo pins of the Ultrasonic Sensor
const int trigPin = 3;
const int echoPin = 2;
// Variables for the duration and the distance
long duration;
float distance;
int Ka;
int Kr;
Vector<int> Fa;  //attractive force   with module=Ka
Vector<int> Fr;  //repulsive force    with module=Kr
Vector<int> F;   //total force
float Fax,Fay,Frx,Fry,Fx,Fy;  //x and y constitutives
float theta;
int velocity=20;
float Xt=2000;
float Yt=2000;  //(2m,2m)
float distance_to_target=0;  
float Xrb,Yrb;



void setup() {
  // put your setup code here, to run once:
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600);
  Robby.setMode(DIFFERENTIAL);
  
}

void loop() {
  //  Robby.leftMotorWrite(velocity);
  //  Robby.rightMotorWrite(velocity);

  Robby.goTo(Xt,Yt); //pointer le robot dans une cible 
                 
  distance = calculateDistance();
  Serial.println(distance);
  delay(100);

  detection();
  
  Ka=10;   
  Kr=1;    
  theta=0; 
  //Fax=0;
  //Fay=Ka;  //the force initially in one direction: Y
  ///the robot is moving in Y direction
  ///the robot has no target, when it turns it define a new direction (always Y for the robot) 
  ///and start again with theta=0 and Fa=...
  
  Fa.PushBack(Fax); //remplir le vecteur
  Fa.PushBack(Fay);
  Fr.PushBack(Fry); 
  Fr.PushBack(Fry); //maintenant Fa=[Fax,Fay]  and Fr=[Frx,Fry]
  
  
  
  //Robby.turn(theta);
  Robby.pointTo(theta);
  
}

void detection(){
  
  if(distance<30){
    
    //this is where the obstacle is near

        distance_to_target = sqrt(pow((Xt - Xrb),2)+pow((Yt - Yrb),2));
        Fax=-Ka * (Xrb - Xt) / distance_to_target;
        Fay=-Ka * (Yrb - Yt) / distance_to_target;
        Fry=-Kr/(pow(distance,2)); 
        Frx=0;                             //dans ce cas l'obstacle est supposé détecté dans l'axe Y du robot
        F.PushBack(Fax+Frx);
        F.PushBack(Fay+Fry);
        Fx=(float)F[0];
        Fy=(float)F[1];
        theta=atan(Fy/Fx);
  }
}

/////////à implementer dans le sonar
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
