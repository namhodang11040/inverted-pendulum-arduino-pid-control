#include <PID_v1.h>

const int encoderPinA = 3;  // Pin A of the encoder is connected to pin 2 on the Arduino
const int encoderPinB = 2;  // Pin B of the encoder is connected to pin 3 on the Arduino

const int motorPin1 = 4;   // pin IN1 of the L298N module
const int motorPin2 = 5;  // pin IN2 of the L298N module
const int enablePin = 10;  // Enable pin of L298N module

const int setzero = 11; //set zero
const int switchenable = 12; //set enable
bool enable = false;

const int switchPin1 = 21;  // interrupt pin 1
const int switchPin2 = 20;  // interrupt pin 2

const int DCencoderPinA = 18;  // Pin A of the encoder is connected to pin 2 on the Arduino
const int DCencoderPinB = 19;  // Pin B of the encoder is connected to pin 3 on the Arduino

const int pulsesPerRevolution = 1200; // pulses per cycle of the encoder
volatile long encoderValue = 0;
volatile long position = 0;

volatile long encoderPosition = 0; // Variable position of the encoder
long targetAngle = 600.0;              
double input, output, setpoint;     
double Kp =54.3718, Ki = 225.7454, Kd =1.1415;  

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
  Serial.begin(9600);
  //L298
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  //CTHT
  pinMode(switchPin1, INPUT_PULLUP);
  pinMode(switchPin2, INPUT_PULLUP);
  //DC
  analogWrite(enablePin, 255);  
  digitalWrite(motorPin1, HIGH);  
  digitalWrite(motorPin2, LOW);
  //Encoder
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  //setzero và enable
  pinMode(setzero, INPUT);
  pinMode(switchenable, INPUT_PULLUP);
  //interrupt
  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DCencoderPinA), DCencoder, CHANGE);
  
  //PID
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);  
  pid.SetOutputLimits(-255, 255);  // Set output value limit
  //set home
  home();
}

void loop() {
  limit();
  if(digitalRead(setzero) == LOW) encoderValue=0;
  if(digitalRead(switchenable) == LOW) {
    while (digitalRead(switchenable) == LOW){
    }
    enable= !enable;
  } 
  if (position < 2000) setpoint = 601.0;
  else if (position > 3000) setpoint = 603.0;
  else if (position <=  3000 && position >= 2000) setpoint = 602.0;
  
  double degrees = map(encoderValue, 0, pulsesPerRevolution, 0, 1200);
  //Holds value between 0 and 1200 pulses
  degrees = fmod(degrees, 1200);
  input = degrees;

    // Calculate PID output
  if (enable == 1){
    if (input>400 && input <800){
      pid.Compute();
      controlMotor(output);
      
    }
    else {
      analogWrite(enablePin, 0);  // Set motor speed
      digitalWrite(motorPin1,LOW);
      digitalWrite(motorPin2,LOW);
    }
  }
  Serial.print("Angle: ");
  Serial.print(degrees, 0);
  Serial.print("   | Position: ");
  Serial.print(position);
  Serial.print("   | Enable: ");
  Serial.print(enable);
  Serial.print("   | Target angle: ");
  Serial.print(setpoint);
  Serial.print("   | Output");
  Serial.println(output);
  delay(10);
}

void handleEncoder() { //The function reads the angle value of the pendulum
  int encoderStateA = digitalRead(encoderPinA);
  int encoderStateB = digitalRead(encoderPinB);

  if (encoderStateA == encoderStateB) {
    encoderValue++;
  } else {
    encoderValue--;
  }

  // Adjust the value when below 0 degrees
  if (encoderValue < 0) {
    encoderValue += pulsesPerRevolution;
  }
}
void DCencoder() { //function reads the position of the pendulum
  if (digitalRead(DCencoderPinA) == digitalRead(DCencoderPinB)) {
    position++;
  } else {
    position--;
  }
}
void controlMotor(int n) {
  if (n>0){
  analogWrite(enablePin, n);  
  digitalWrite(motorPin1, HIGH);  
  digitalWrite(motorPin2, LOW);
  }
  if (n<0){
  analogWrite(enablePin, abs(n));  
  digitalWrite(motorPin1, LOW);  
  digitalWrite(motorPin2, HIGH);
  }

}

void home() {
  // set home
  while (digitalRead(switchPin2) != LOW) {
    analogWrite(enablePin, 150);  
  digitalWrite(motorPin1,LOW);  
  digitalWrite(motorPin2,HIGH);

  }
  position = 0;
  while (position<2100)
  {
    analogWrite(enablePin, 120);  
    digitalWrite(motorPin1,HIGH);  
    digitalWrite(motorPin2,LOW);
  }
    analogWrite(enablePin, 0);  
    digitalWrite(motorPin1,LOW);  
    digitalWrite(motorPin2,LOW);
}
void limit (){
  if (digitalRead(switchPin2) == LOW){
  analogWrite(enablePin, 255);  
  digitalWrite(motorPin1,HIGH);  
  digitalWrite(motorPin2,LOW);
  delay(400);
  analogWrite(enablePin, 0);  
  digitalWrite(motorPin1,LOW);  
  digitalWrite(motorPin2,LOW);
  }
  if (digitalRead(switchPin1) == LOW){
  analogWrite(enablePin, 255);  
  digitalWrite(motorPin1,LOW);  
  digitalWrite(motorPin2,HIGH); 
  delay(400);  
  analogWrite(enablePin, 0);  
  digitalWrite(motorPin1,LOW);  
  digitalWrite(motorPin2,LOW);
  }
}