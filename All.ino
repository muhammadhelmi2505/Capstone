
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#define I2C_ADDR    0x27 //Add your address here.  Find it from I2C Scanner
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7
#define led_pin 13
LiquidCrystal_I2C lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);  
int pr = 2;               // choose the input pin (for PIR sensor)  
const int trigPin = 12;
const int echoPin = 11;
const int buzzer = 13;
float duration;
float distance;
const int in1 = 4; //pin 5 of H-Bridge --> pin 4 of arduino
const int in2 = 7; //pin 7 of H-Bridge --> pin 7 of arduino
const int enA = 5; //pin 6 of H-Bridge (to control speed) --> pin 5 of arduino

//right motor
const int in3 = 8; //pin 10 of H-Bridge --> pin 8 of arduino
const int in4 = 9; //pin 12 of H-Bridge -->pin 9 of arduino
const int enB = 6; //pin 11 of H-Bridge (to control speed) --> pin 6 of arduino

//left sensor
const int sensor1 = 0;       //Analog input IR Sensor 1 -- Controls left motor
const int sensor2 = 1;
const int battery = 3;//set right sensor --> pin 1 arduino
boolean forwardEnable = true;
//declare variables to store analog value of IR sensors
int nr1 = 0, nr2 = 0;
int pirState = LOW;             // we start, assuming no motion detected
int val = 0;  
char data;
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);

pinMode(in1,OUTPUT);  //set left and right motor (with PWM) as output
pinMode(in2,OUTPUT);
pinMode(in3,OUTPUT);
pinMode(in4,OUTPUT);
pinMode(enA,OUTPUT);
pinMode(enB,OUTPUT);
pinMode(sensor1, INPUT);
pinMode(sensor2, INPUT);
pinMode(pr,INPUT);
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);
pinMode(buzzer,OUTPUT);
pinMode(battery, INPUT)
digitalWrite(enA,HIGH);   //set enA and enB at '1'
digitalWrite(enB,HIGH);
lcd.begin (16,2); //My LCD was 16x2
lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
lcd.setBacklight(HIGH);
lcd.home (); // go home

}

void detectSensor(){
 nr1= analogRead( sensor1);
nr2 = analogRead( sensor2);
 // Serial.println(nr2);
  //Serial.println("\t");
  //Serial.println(nr1);
  //Serial.println("\n");

  
}
void moveForward(){
  Serial.println("Forward");
  
  digitalWrite(in1,HIGH);  //set left motor forward (clockwise)
  digitalWrite(in2,LOW);
 // set the PWM of motors at 100
  analogWrite(enA, 100);

  digitalWrite(in3,HIGH);  //set right motor forward (clockwise)
  digitalWrite(in4,LOW);
  analogWrite(enB, 100);
  
}

void moveReverse(){
   Serial.println("Reverse");
  
  digitalWrite(in1,LOW);  //set left motor reverse (counter-clockwise)
  digitalWrite(in2,HIGH);
  analogWrite(enA, 100);  // set the PWM of motors at 100

  digitalWrite(in3,LOW);  //set right motor reverse (counter-clockwise)
  digitalWrite(in4,HIGH);
  analogWrite(enB, 100);
}

void turnLeft(){
   Serial.println("Turn Left");
  
  digitalWrite(in1,HIGH);  //set left motor static
  digitalWrite(in2,HIGH);

  digitalWrite(in3,LOW);  //set right motor forward (clockwise)
  digitalWrite(in4,HIGH);
  analogWrite(enB, 100);  //set pwm of right motor at 100
}

void turnRight(){
   Serial.println("Turn Right");
  
  digitalWrite(in1,LOW);  //set left motor forward (clockwise)
  digitalWrite(in2,HIGH);
  analogWrite(enA, 500);  // set the PWM of motors at 200

  digitalWrite(in3,HIGH);  //set right motor static
  digitalWrite(in4,HIGH);
}

void Brake(){
   Serial.println("Stop");
  
  digitalWrite(in1,HIGH);  //set left motor static
  digitalWrite(in2,HIGH);

  digitalWrite(in3,HIGH);  //set right motor static
  digitalWrite(in4,HIGH);
}
void  air(){
 float x;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration=pulseIn(echoPin, HIGH);
  distance=(duration*0.034)/2;
  Serial.print("Ditances: ");
  lcd.setCursor(0,0);
  lcd.print("Distances: ");
  x=16.0-distance; 
  Serial.print(x);
  lcd.print(x);
  delay(500);
  Serial.print("\n");
  if(x<4){
   Serial.print("Please refill\n");
  lcd.setCursor(0,1);
   lcd.print("Please refill ");
   delay(500);
   lcd.clear();
   Serial.print("A");
  }
  
  else if((x<8)&&(x>4)){
   Serial.print("Moderate level \n");
     lcd.setCursor(0,1);
   lcd.print("Moderate  ");
   delay(500);
   lcd.clear();
  Serial.print("B");
  
  }

  else if (x>8){
   Serial.print("More water left\n");
   lcd.setCursor(0,1);
  lcd.print("Almost full ");
  delay(500);
  lcd.clear();
    Serial.print("C");
  }
 
}
void pirSensor(){
  val=digitalRead(pr);
  if(val==HIGH){
  tone(buzzer,1000); 
  Brake();
  delay(1000);
  moveForward();
  if(pirState==LOW)
  pirState = HIGH;
  }
  else{
    noTone(buzzer);
    if(pirState==HIGH)
    pirState =LOW; 
    
  }
  
}
void voltage{
  int sensorValue = analogRead(battery); //read the A3 pin value
  float voltage = sensorValue * (5.00 / 1023.00)*2; //convert the value to a true voltage.
  lcd.setCursor(0,0);
  lcd.print("voltage = ");
  lcd.print(voltage); //print the voltage to LCD
  lcd.print(" V");

}

void loop() {
voltage();  
air();
pirSensor();
detectSensor();

  if(nr1>400||nr2>500){
  digitalWrite(in1, HIGH);  //Stop Motor A
  digitalWrite(in2, HIGH);
   
  digitalWrite(in3, HIGH);  //Set Motor B forward
  digitalWrite(in4, HIGH);
   forwardEnable = false;
 }

//char data
     if(Serial.available()>0){
      data= Serial.read(); // reading the data received from the bluetooth module
     }
     detectSensor();
      if(nr1>400||nr2>500){
  digitalWrite(in1, HIGH);  //Stop Motor A
  digitalWrite(in2, HIGH);
   
  digitalWrite(in3, HIGH);  //Set Motor B forward
  digitalWrite(in4, HIGH);
   forwardEnable = false;
 }
      switch(data)
      {
        case 'A': 
        forwardEnable = true;
        Serial.println("Left");
      moveForward();
  
        break; // when a is pressed on the app on your smart phone

         case 'S': 
        Serial.println("stop");
        Brake();
        break; // when a is pressed on the app on your smart phone
 case 'G': 
  if(forwardEnable =false)
 ;
 else{
        Serial.println("Go");
  digitalWrite(in1, HIGH);  //Stop Motor A
  digitalWrite(in2, LOW);
   
  digitalWrite(in3, HIGH);  //Set Motor B forward
  digitalWrite(in4, LOW);
 }
        break; // when a is pressed on the app on your smart phone


         case 'R': 
           forwardEnable = true;
        Serial.println("REVERSE");
  moveReverse();
 
        break; // when a is pressed on the app on your smart phone
        
        case ('D'||'d'):
        forwardEnable = true; 
        Serial.println("Right");
  digitalWrite(in1, HIGH);  //Set Motor A forward
  digitalWrite(in2, HIGH);
   
  digitalWrite(in3, HIGH);  //Stop Motor B
  digitalWrite(in4, LOW); 

        break; // when d is pressed on the app on your smart phone

case ('O'): 

do {
 if(Serial.available()>0){
    data = Serial.read();
 }
detectSensor();
         
 
if ((nr1 > 400) && (nr2 <600)&&(nr3 > 750)){ 
   Brake();
    delay(1000);
    moveReverse();
    delay(1500);
    turnLeft();
    delay(1500);
    moveForward();
      }

  
  
else if ((nr1 <300) && (nr2 > 500)&&(nr3 > 750)){ 
     Brake();
    delay(1000);
   moveReverse();
    delay(1500);
    turnRight();
    delay(1500);
    moveForward();
      }
  
  
 else if ((nr1 >300)&& (nr2 > 500)&&(nr3 > 750)){ 
   Brake();
    delay(1000);
   moveReverse();
    delay(2000);
    turnLeft();
    delay(1000);
    moveForward();
   }
   
   
else if ((nr1< 300)&& (nr2 < 500)&& (nr3<200)){ 
   Brake();
    delay(2000);
  
  
  
   moveReverse();
    delay(2000);
    turnLeft();
    delay(1000);
    moveForward();
   }
  else{
    moveForward();
  }
}while(data == 'O');
  break;
  default: break;
 
        
   
   
}
}



//if ((nr1>400) || (nr2>600)){  
  //Brake();  //robot stop for 1s
  //delay(2000);
  
 // moveReverse();  //robot reverse for 0.5s
  //delay(2000);
  
//Brake();  //robot stop for 1s
  //delay(1000);
  
 // turnRight();  //robot turns right 
  //delay(2000);
//}

//else
//moveForward();  //robot moves forward if sensor not detect
//delay(3000);
//}
