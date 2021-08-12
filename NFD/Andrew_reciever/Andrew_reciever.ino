
//receiver code
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"1nodeA", "2nodeB"}; // adjusted
const int AIN1 = 14; // AIN1 pin #
const int AIN2 = 15; // AIN2 pin #
const int PWMA = 3; // PWM pin # 
const int BIN1 = 16;
const int BIN2 = 17;
const int PWMB = 5;
const int CIN1 = 18; // AIN1 pin #
const int CIN2 = 19; // AIN2 pin #
const int PWMC = 6; // PWM pin # 
const int DIN1 = 2;
const int DIN2 = 4;
const int PWMD = 9;
int irSensor;
int done = 0;
int sensorState = 1;
boolean start = false;


void setup(){
  Serial.begin(9600);
  pinMode(AIN1, OUTPUT); 
  pinMode(AIN2, OUTPUT); 
  pinMode(PWMA, OUTPUT); 
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(CIN1, OUTPUT); 
  pinMode(CIN2, OUTPUT); 
  pinMode(PWMC, OUTPUT); 
  pinMode(DIN1, OUTPUT);
  pinMode(DIN2, OUTPUT);
  pinMode(PWMD, OUTPUT); 
  radio.begin(); 
  radio.openWritingPipe(addresses[1]); // "1nodeA"
  radio.openReadingPipe(0, addresses[0]); // "2nodeB"
  radio.setPALevel(RF24_PA_MIN);
}

void Motor(int motorSpeed) //function driving a single motor, if we need to turn around a seperate function is needed. Four motors also require another driver
{
 if(motorSpeed > 0){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    digitalWrite(CIN1, HIGH);
    digitalWrite(CIN2, LOW);
    digitalWrite(DIN1, HIGH);
    digitalWrite(DIN2, LOW);
  } else if (motorSpeed < 0){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    digitalWrite(CIN1, LOW);
    digitalWrite(CIN2, HIGH);
    digitalWrite(DIN1, LOW);
    digitalWrite(DIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    digitalWrite(CIN1, LOW);
    digitalWrite(CIN2, LOW);
    digitalWrite(DIN1, LOW);
    digitalWrite(DIN2, LOW);
  }
  analogWrite(PWMA, abs(motorSpeed));
  analogWrite(PWMB, abs(motorSpeed));
  analogWrite(PWMC, abs(motorSpeed));
  analogWrite(PWMD, abs(motorSpeed));
}

boolean executeOrder66(){
  Serial.print("Sensor State: ");
  // read the radio for the new Sensor state & print it
  radio.read(&sensorState, sizeof(sensorState));
  if(radio.available()){
    sensorState = 1;
  }
  if(sensorState == 1) {
    return false;
  } else {
    return true;
  }
}

void loop() {
  radio.startListening();

  unsigned long started_waiting_at = millis();
  bool timeout = false;
  while ( ! radio.available() && ! timeout )
    if (millis() - started_waiting_at > 200 )
      timeout = true;
  // Describe the results
  if ( timeout )
  {
    printf("Failed, response timed out.\n\r");
  }
  else
  {
      // Grab the response, compare, and send to debugging spew
    unsigned long got_time;
    radio.read( &got_time, sizeof(unsigned long) );
    // Spew it
    printf("Got response %lu, round-trip delay: %lu\n\r",got_time,millis()-got_time);
  }
  // Try again 1s later
  delay(1000);
  
  
  start = executeOrder66();
  //const char text[32];
  // print the sensor state (which should be 1)
  sensorState = 1;
  if(start == true){
    Serial.print("Time to start!");
  }
  Serial.println(int(sensorState));
  Serial.print("Active: ");
  Serial.println(bool(start));
  // wait for .8s
  delay(200);
  
  // if the sensor returns 1, begin program
  
    if(start == true){
      Serial.println("Moving motor");
      Motor(255);
      delay(5000);
      Serial.println("Finished!");
      Motor(0); 
      done = 4;
      if(done == 4){
        Serial.println("Sending Command..");
        radio.stopListening();
        radio.write(&done, sizeof(done));
        Serial.print("Done = " );
        Serial.println(done);
        Serial.println("Done moving motor.");
        Serial.println("Sending Done Command");
        done = 7;
        delay(10000);
      }
      if (done == 7){
        delay(10000); //wait for table bot
        Motor(-255); //baclwars
        delay(5000); //wait for 
        Motor(0);
        done = 6; 
      }
      sensorState = 1; 
    }
    start = false;
    radio.startListening();
}
