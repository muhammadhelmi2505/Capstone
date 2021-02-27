#include<Servo.h> 
// -------------------------------------------- Pins declarations ------------------------------------------   
// For the IR Sensor Output    
int IRSen1 = A0;  // LEFT sensor    
int IRSen2 = A1;  // Right sensor    
 
//left motor 
const int in1L = 12; //pin 5 of H-Bridge --> pin 12 of arduino 
const int in2L = 13; //pin 7 of H-Bridge --> pin 13 of arduino 
const int enAL = 11; //pin 6 of H-Bridge (to control speed) --> pin 11 of arduino 
 
//right motor 
const int in3R = 8; //pin 10 of H-Bridge --> pin 8 of arduino 
const int in4R = 7; //pin 12 of H-Bridge -->pin 7 of arduino 
const int enBR = 9; //pin 11 of H-Bridge (to control speed) --> pin 9 of arduino 
 

////ultrasonic sensor obstacle & flower pot 
const int echoPin = 4; 
const int trigPin = 5; 
 
//pump sensor 
const int pump=6; 
 
//bluetooth pin (0,1) 
 

//PIR 
int PIR= A3; 
 
//servo 
int servoPin = 10; 
Servo servo; 

 //buzzer 
#define SPEAKERS A4 

//------------------------------------------- --- Variables --- ---------------------------------------- 
int threshold = 500;   
float brg; 
int IR1 = 0;    
int IR2 = 0;    
int sec=0;
char data; 
#define BEATTIME 500 //Length of the generated tone (msec) 
#define BEATTIMES 500 

//---------------------------------------------movement motor------------------------------------------- 
void forward(){   
 
digitalWrite(in1L,HIGH);    
digitalWrite(in2L,LOW);    
analogWrite(enAL,200);    
digitalWrite(in3R,HIGH);    
digitalWrite(in4R,LOW);    
analogWrite(enBR,200);    
}    
    
void backward(){    //move backward
 
digitalWrite(in1L,LOW);    
digitalWrite(in2L,HIGH);    
analogWrite(enAL,200);    
digitalWrite(in3R,LOW);    
digitalWrite(in4R,HIGH);    
analogWrite(enBR,200);     
}    
    
void right(){ //turn right
     
digitalWrite(in1L,HIGH);    
digitalWrite(in2L,LOW);    
analogWrite(enAL,70);    
digitalWrite(in3R,HIGH);    
digitalWrite(in4R,LOW);    
analogWrite(enBR,200);    
}    
    
void left(){    //turn left
   digitalWrite(in1L,HIGH);    
  digitalWrite(in2L,LOW);    
  analogWrite(enAL,200);    
  digitalWrite(in3R,HIGH);    
digitalWrite(in4R,LOW);    
analogWrite(enBR,70);     
}    
  
void stops(){    //stop robot
digitalWrite(in1L,LOW);    
digitalWrite(in2L,LOW);    
analogWrite(enAL,0);    
digitalWrite(in3R,LOW);    
digitalWrite(in4R,LOW);    
analogWrite(enBR,0);     
}    

 //----------------------------------------------waterpump------------------------------------ 
 
               
 
 void pumping(){                            //check for present of flowerpot in front, if theres flower pot move forward or else reverse and continue moving.
  long duration, cm; 
   digitalWrite(trigPin, LOW); 
   delayMicroseconds(2); 
   digitalWrite(trigPin, HIGH); 
   delayMicroseconds(10); 
   digitalWrite(trigPin, LOW); 
   duration = pulseIn(echoPin, HIGH); 
   cm = duration / 29/2; 
   if (28<cm<31) 
   { 
    while(cm>5){                      ///move forward until it is near to the flower pot.
      forward();
      digitalWrite(trigPin, LOW); 
   delayMicroseconds(2); 
   digitalWrite(trigPin, HIGH); 
   delayMicroseconds(10); 
   digitalWrite(trigPin, LOW); 
   duration = pulseIn(echoPin, HIGH); 
   cm = duration / 29/2; 
    
    } 
     
    servo.write(45); // aim the hose to be slanted down to the flower pot.
     delay(1000);     // Wait 1 second 
  
    digitalWrite(pump,HIGH); //pump water out
    delay (3000);
    digitalWrite(pump,LOW); //stop pumpingg water
     servo.write(90);//// move the hose to original position

       while(cm<30){ //move backward back into the patrol zone
      backward();

 
digitalWrite(trigPin, LOW); 
   delayMicroseconds(2); 
   digitalWrite(trigPin, HIGH); 
   delayMicroseconds(10); 
   digitalWrite(trigPin, LOW); 
   duration = pulseIn(echoPin, HIGH); 
   cm = duration / 29/2; 
    
    } 
   } 
 
 
} 
 
 

 
//-----------------------------------------obstacle-------------------------------------- 
void obstacle(){ // detect for any obstacle
  long duration, cm; 
   digitalWrite(trigPin, LOW); 
   delayMicroseconds(2); 
   digitalWrite(trigPin, HIGH); 
   delayMicroseconds(10); 
   digitalWrite(trigPin, LOW); 
   duration = pulseIn(echoPin, HIGH); 
   cm = duration / 29/2; 
  brg=cm; 
} 
 
//------------------------------------motion-------------------------------------- 
void motion(){ //detect any human motion
  int val= analogRead(PIR); 
 
 
   while(val>300){ //if there is any motion stop and turn on  buzzer.
      stops(); 
     alarm(); 
      delay(1000); 
      val= analogRead(PIR); 
  } 
} 
//--------------------------------------------alarm------------------------------------ 
void alarm(){ 
                                                ///alarm sound
 tone(SPEAKERS,1000,BEATTIME) ; // Do 
delay(BEATTIMES) ; 
tone(SPEAKERS,500,BEATTIME) ; // Re
delay(BEATTIMES) ; 
  } 
 
//----------------check sensor----------------------------------- 
void checksensor(){ 
                                      //detect the black tape
IR1 = analogRead(IRSen1);   
   IR2 = analogRead(IRSen2);   
} 
 
//-----------------------------automode----------------------------------------------------- 
void automode(){ //the robot in the auto mode
  char q;   
  for(;;)    
  {   
    if(sec=1){ //if security mode on detect any  human motion
      motion(); 
    } 
     
    
     
    checksensor(); 
      
    if(Serial.available())   //check whether there user want to stop, activate or deactivate security mode
        {q=Serial.read();   
          if(q=='s')   
          {stops();   
          break;}} 
          if(q=='y') 
          {sec=1; } 
          if(q=='z') 
          { sec=0;} 
     else if ((IR1 > threshold) && (IR2 > threshold)) {   ///check whethe the robot has any obstacle and in patrol zone, else move forward
       obstacle(); 
  if(brg<10){ 
    stops();  
    delay(1000); 
    while(brg<20){ 
      backward();}
      right();  
     delay(500);
  } 
else{ 
        forward(); 
           
          } }
      
    
     else if ((IR1 < threshold) && (IR2 < threshold)) {    // detect black tape and flowerpot, else reverse
     pumping(); 
    backward();   
    delay(5000);  
    right();  
    delay(1000);  
  }     
    
else if ((IR1 > threshold) && (IR2 < threshold)) {    // detect black tape and flowerpot, else reverse
  while((IR1 > threshold)){ 
    left(); 
      checksensor();  
    } 
    pumping(); 
    backward();   
    delay(500);  
    right();  
   delay(1000);   
  }     
    
else if ((IR1 < threshold) && (IR2 > threshold)) {    // detect black tape and flowerpot, else reverse
  while(IR2 > threshold){ 
    right(); 
     checksensor();  
    } 
    pumping(); 
  
 backward();   
  
    delay(500);  
    left();  
    delay(1000);  
  }     
else    
      {   
        stops();   
      }   
      }   
} 
 
//--------------------------setup---------------------------------------------------------------   
void setup()    
{   
  pinMode(in1L, OUTPUT);    
  pinMode(in2L, OUTPUT);    
  pinMode(enAL, OUTPUT);    
  pinMode(in3R, OUTPUT);    
  pinMode(in4R, OUTPUT);    
  pinMode(enBR, OUTPUT);  
 
  pinMode(IRSen1, INPUT); 
  pinMode(IRSen2, INPUT);  
 
pinMode(pump,OUTPUT); 
 

 
pinMode(trigPin, OUTPUT); 
pinMode(echoPin, INPUT); 
 
servo.attach(servoPin); 
servo.write(90); // set the hose to be 0 degree as original position
   
   
  // set the data rate for the SoftwareSerial port   
  Serial.begin(9600);      
}   
   
//----------------------------------------------------------------------loop----------------------------------   
void loop()    
{    

stops();   //initally the robot stop

if (Serial.available())    //wait for user input via bluetooth
{   
    data=Serial.read();   
 if(data=='a')   
 {   
  automode(); 
 }   
    
 else if(data=='b')   
 {   
  forward();    
  delay(2000);   
 }   
   
 else if(data=='c')   
 {   
  backward();    
  delay(2000);   
 }   
   
 else if(data=='d')   
 {   
  right();   
  delay(1500);   
 }   
   
 else if(data=='e')   
 {   
  left();   
  delay(1500);   
 }  
 
  else if(data=='y'){ 
    sec=1; 
  } 
 
  else if(data=='z'){ 
    sec=0; 
  } 
   
 else    
 {   
  stops;   
 }   
}   
}
