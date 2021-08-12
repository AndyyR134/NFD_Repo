// Transmitter Code [Updated]
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "BTS7960.h"

RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"1nodeA", "2nodeB"};
//int button = 0; 
const int AIN1 = 14;
const int AIN2 = 15;
const int PWMA = 9;
const int BIN1 = 16;
const int BIN2 = 17;
const int PWMB = 10;
const int R_IS = 3;
const int R_EN = 2;
const int R_PWM = 6;
const int L_IS = 19;
const int L_EN = 4;
const int L_PWM = 5;

const int IRSensor = 18;
int done = 6;
int sensorStatus = 1;
void setup() {
  Serial.begin(9600);
  
  pinMode(IRSensor, INPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); 
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  //
  pinMode(R_IS, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_IS, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  digitalWrite(R_IS, LOW);
  digitalWrite(L_IS, LOW);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00002
  radio.openReadingPipe(1, addresses[1]); // 00001
  radio.setPALevel(RF24_PA_MIN);
}


void Motor(int motorSpeed){  //THIS IS TO MOVE BOTH MOTORS IN THE SAME DIRECTION WITHOUT NEEDING ANOTHER FUNCTION
  if(motorSpeed > 0){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (motorSpeed < 0){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  analogWrite(PWMA, abs(motorSpeed));
  analogWrite(PWMB, abs(motorSpeed));
}

void expandWings(){  
  Serial.println("Expanding Wings");
   analogWrite(R_PWM, 255);
   analogWrite(L_PWM, 0);
   delay(4000);
   analogWrite(R_PWM, 0);
   analogWrite(L_PWM, 0);
}

void retractWings(){
  Serial.println("Retracting Wings...");
   analogWrite(R_PWM, 0);
   analogWrite(L_PWM, 255);
   delay(1000);
   analogWrite(R_PWM, 0);
   analogWrite(L_PWM, 0);
}
void broadcast(int sensorStatus){
    Serial.println("Running Code");  
    radio.write(&sensorStatus, sizeof(sensorStatus));
}

void loop() {
  radio.stopListening();
  sensorStatus = digitalRead(IRSensor);
  
  // Print to Serial for Debug
  Serial.print("Sensor Status: ");
  Serial.println(sensorStatus);
  //  Write the sensor status over the radio
  
  radio.write(&sensorStatus, sizeof(sensorStatus));
  
  // match the activate with the sensor status
  int activate = sensorStatus;
  int beginBroadcast = sensorStatus;
  
  // print for debug
  Serial.print("Activate: ");
  Serial.println(int(activate));
  
  delay(200);
  // if activated start
  if(activate == 0){
  Serial.print("Activating...");
    expandWings();
    if(beginBroadcast == 0){      // if button LOW -> sending info
      broadcast(sensorStatus);
      beginBroadcast = 1;
    }
    // switch the listening state
    Serial.println("Switching Listening states...");
    while(done != 4){
      radio.startListening();
      radio.read(&done, sizeof(done));
      Serial.println(done);
     }
    //delay(500);
    if (done == 4 ) {  // if done = 4 -> receive data
      Serial.println("Moving Motors");
      radio.stopListening();  
      sensorStatus = 1;
      radio.write(&sensorStatus, sizeof(sensorStatus));
      Motor(255);
      delay(2500);
      Motor(0);
      delay(1000);
      Motor(-255);
      delay(2500);
      retractWings();
      Motor(0);
      done = 6;    
    }
    activate = 1;
    sensorStatus = 1; 
  }
}
