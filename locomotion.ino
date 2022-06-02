#include "Arduino.h"
#include "Bounce2.h"

int rEncoder = 2;
int lEncoder = 3;

volatile uint16_t rCounter = 0;
volatile uint16_t lCounter = 0;

volatile uint16_t rFlag = 0;
volatile uint16_t lFlag = 0;

byte debounceTime = 0.5;
Bounce rInput;
Bounce lInput;

int rMotor1 = 4;
int rMotor2 = 7;
int rMotor = 5;

int lMotor1 = 8;
int lMotor2 = 11;
int lMotor = 6;

int RMotor1 = 12;
int RMotor2 = 13;
int RMotor = 9;

int LMotor1 = A0;
int LMotor2 = A1;
int LMotor = 10;

int rPower = 150;
int lPower = 150;

void setup() {
  Serial.begin(9600);

  pinMode(rMotor1, OUTPUT);
  pinMode(rMotor2, OUTPUT);
  pinMode(lMotor1, OUTPUT);
  pinMode(lMotor2, OUTPUT);

  pinMode(rMotor, OUTPUT);
  pinMode(lMotor, OUTPUT);
  
  pinMode(RMotor1, OUTPUT);
  pinMode(RMotor2, OUTPUT);
  pinMode(LMotor1, OUTPUT);
  pinMode(LMotor2, OUTPUT);

  pinMode(RMotor, OUTPUT);
  pinMode(LMotor, OUTPUT);

  rInput.attach(rEncoder, INPUT_PULLUP);
  lInput.attach(lEncoder, INPUT_PULLUP);
  rInput.interval(debounceTime);
  lInput.interval(debounceTime);

  attachInterrupt(digitalPinToInterrupt (rEncoder), rCount, RISING);
  attachInterrupt(digitalPinToInterrupt (lEncoder), lCount, RISING);

  //forward();
  //right();
  //left();
  //uTurn();
  //reverse();
}

void loop() {
  // put your main code here, to run repeatedly:

}

int16_t rCount() {
  static int16_t rPosition = 0;

  rInput.update();

  if (rInput.rose()) {
    rPosition++;
  }
  return rPosition;
}

int16_t lCount() {
  static int16_t lPosition = 0;

  lInput.update();

  if (lInput.rose()) {
    lPosition++;
  }
  return lPosition;
}

void forward() {
  Serial.println("forward");
    
  digitalWrite(rMotor1, HIGH);
  digitalWrite(rMotor2, LOW);
  digitalWrite(lMotor1, HIGH);
  digitalWrite(lMotor2, LOW);
  
  digitalWrite(RMotor1, HIGH);
  digitalWrite(RMotor2, LOW);
  digitalWrite(LMotor1, HIGH);
  digitalWrite(LMotor2, LOW);
  
  while(true){
    int16_t rCounter = rCount();
    int16_t lCounter = lCount();
    
    if (rCounter < 18) {
      Serial.println("right counter rn is: ");
      Serial.println(rCounter);
      Serial.println("left counter rn is: ");
      Serial.println(lCounter);
      
      if (lCounter < rCounter) {
        Serial.println("left is smaller");
        lPower += 10;
      }
      else if (lCounter > rCounter) {
        Serial.println("right is smaller");
        lPower -= 10;
      }
      analogWrite(RMotor, rPower);
      analogWrite(LMotor, lPower);
    }
    
    else{
      Serial.println("right counter rn is: ");
      Serial.println(rCounter);
      Serial.println("left counter rn is: ");
      Serial.println(lCounter);
    
      Serial.println("brake");
    
      brake();
      break;
    }
  }
  }
  
void right() {
  Serial.println("right");
  
  digitalWrite(rMotor1, LOW);
  digitalWrite(rMotor2, HIGH);
  digitalWrite(lMotor1, HIGH);
  digitalWrite(lMotor2, LOW);
  
  while(true){
    int16_t rCounter = rCount();
    int16_t lCounter = lCount();
    
    if (rCounter < 5) {
      Serial.println("right counter rn is: ");
      Serial.println(rCounter);
      Serial.println("left counter rn is: ");
      Serial.println(lCounter);
      
      if (lCounter < rCounter) {
        Serial.println("left is smaller");
        lPower += 10;
      }
      else if (lCounter > rCounter) {
        Serial.println("right is smaller");
        lPower -= 10;
      }
      analogWrite(rMotor, rPower);
      analogWrite(lMotor, lPower);
    }
    
    else{
      Serial.println("right counter rn is: ");
      Serial.println(rCounter);
      Serial.println("left counter rn is: ");
      Serial.println(lCounter);
    
      Serial.println("brake");
    
      brake();
      break;
    }
  }
  }
  
void left() {
  Serial.println("left");
  
  digitalWrite(rMotor1, HIGH);
  digitalWrite(rMotor2, LOW);
  digitalWrite(lMotor1, LOW);
  digitalWrite(lMotor2, HIGH);
  
  while(true){
    int16_t rCounter = rCount();
    int16_t lCounter = lCount();
    
    if (rCounter - rFlag < 5) {
      Serial.println("right counter rn is: ");
      Serial.println(rCounter - rFlag);
      Serial.println("left counter rn is: ");
      Serial.println(lCounter - lFlag);
      
      if (lCounter < rCounter) {
        Serial.println("left is smaller");
        lPower += 10;
      }
      else if (lCounter > rCounter) {
        Serial.println("right is smaller");
        lPower -= 10;
      }
      analogWrite(rMotor, rPower);
      analogWrite(lMotor, lPower);
    }
    
    else{
      Serial.println("right counter rn is: ");
      Serial.println(rCounter - rFlag);
      Serial.println("left counter rn is: ");
      Serial.println(lCounter - lFlag);
      
      rFlag = rCounter;
      lFlag = lCounter;
    
      Serial.println("brake");
    
      brake();
      break;
    }
  }
  }
  
void uTurn(){
  Serial.println("uTurn");
  
  left();
  left();
}

void reverse() {
  Serial.println("reverse");
  
  digitalWrite(rMotor1, LOW);
  digitalWrite(rMotor2, HIGH);
  digitalWrite(lMotor1, LOW);
  digitalWrite(lMotor2, HIGH);
  
  while(true){
    
    int16_t rCounter = rCount();
    int16_t lCounter = lCount();
    
    if (rCounter < 15) {
      Serial.println("right counter rn is: ");
      Serial.println(rCounter);
      Serial.println("left counter rn is: ");
      Serial.println(lCounter);
      
      if (lCounter < rCounter) {
        Serial.println("left is smaller");
        lPower += 10;
      }
      else if (lCounter > rCounter) {
        Serial.println("right is smaller");
        lPower -= 10;
      }
      analogWrite(rMotor, rPower);
      analogWrite(lMotor, lPower);
    }
    
    else{
      Serial.println("right counter rn is: ");
      Serial.println(rCounter);
      Serial.println("left counter rn is: ");
      Serial.println(lCounter);
    
      Serial.println("brake");
    
      brake();
      break;
    }
  }
  }

void brake() {
  digitalWrite(rMotor1, LOW);
  digitalWrite(rMotor2, LOW);
  digitalWrite(lMotor1, LOW);
  digitalWrite(lMotor2, LOW);
  analogWrite(rMotor, 0);
  analogWrite(lMotor, 0);
  
  digitalWrite(RMotor1, LOW);
  digitalWrite(RMotor2, LOW);
  digitalWrite(LMotor1, LOW);
  digitalWrite(LMotor2, LOW);
  analogWrite(RMotor, 0);
  analogWrite(LMotor, 0);
}

