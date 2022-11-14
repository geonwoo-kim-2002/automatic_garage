#include <SoftwareSerial.h>
#include <Stepper.h>

#define TRIG_DOOR 6
#define ECHO_DOOR 7

#define TRIG_GARAGE 12
#define ECHO_GARAGE 13

const int stepsPerRevolution = 2048;
char cmd;
bool car_in = false;
bool door_open = false;
bool in = false;

Stepper myStepper(stepsPerRevolution, 11, 9, 10, 8);

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_DOOR, OUTPUT);
  pinMode(ECHO_DOOR, INPUT);

  pinMode(TRIG_GARAGE, OUTPUT);
  pinMode(ECHO_GARAGE, INPUT);

  myStepper.setSpeed(14);
}

void loop() {
  long duration_door, distance_door, duration_garage, distance_garage;
  delayMicroseconds(2);
  digitalWrite(TRIG_DOOR, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_DOOR, LOW);

  duration_door = pulseIn(ECHO_DOOR, HIGH);

  delayMicroseconds(2);
  digitalWrite(TRIG_GARAGE, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_GARAGE, LOW);

  duration_garage = pulseIn(ECHO_GARAGE, HIGH);

  distance_door = duration_door * 17 / 1000;
  distance_garage = duration_garage * 17 / 1000;

  if(car_in == false){
    Serial.println(distance_door);

    delay(50);

    if(Serial.available()){
      cmd = Serial.read();
    }
    if(cmd == 'a'){
      if(!door_open){
        myStepper.step(stepsPerRevolution);
        car_in = true;
        door_open = true;
        in = true;
      }
    }
  }
  else{
    if(door_open){
      if(distance_garage <= 10 && distance_door > 10){
        if(in == true){
          myStepper.step(-stepsPerRevolution);
          door_open = false;
          in = false;
        }
      }
      else if(distance_garage > 10 && distance_door <= 10){
        if(in == false){
          myStepper.step(-stepsPerRevolution);
          door_open = false;
          //in = true;
          Serial.println(10000);
          car_in = false;
        }
      }
    }
    else{
      if(distance_garage > 10){
        myStepper.step(stepsPerRevolution);
        door_open = true;
      }
    }
  }
  
  delay(500);
}
