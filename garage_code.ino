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
  delayMicroseconds(2); //문 초음파 센서 거리 감지
  digitalWrite(TRIG_DOOR, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_DOOR, LOW);

  duration_door = pulseIn(ECHO_DOOR, HIGH);

  delayMicroseconds(2); //차고 초음파 센서 거리 감지
  digitalWrite(TRIG_GARAGE, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_GARAGE, LOW);

  duration_garage = pulseIn(ECHO_GARAGE, HIGH);

  distance_door = duration_door * 17 / 1000;
  distance_garage = duration_garage * 17 / 1000;

  if(car_in == false){ //차가 차고에 없다면
    Serial.println(distance_door); //문 초음파 센서 거리 출력

    delay(50);

    if(Serial.available()){
      cmd = Serial.read();
    }
    
    if(cmd == 'a'){ //a가 읽히면
      if(!door_open){ //차고의 문을 연다
        for(int i = 0;i<9;i++)
        {
          myStepper.step(stepsPerRevolution);  
        }
        
        car_in = true;
        door_open = true;
        in = true;
      }
    }
  }
  else{ //차가 차고에 있다면
    if(door_open){ //문이 열려있다면
      if(distance_garage <= 10 && distance_door > 10){ //차가 문에서 감지되지 않고 차고에서 감지됐을 때
        if(in == true){ //차가 들어올 때
          for(int i = 0;i<9;i++)
        {
          myStepper.step(-stepsPerRevolution);  
        } //문을 닫는다
          door_open = false;
          in = false;
        }
      }
      else if(distance_garage > 10 && distance_door <= 10){ //차가 차고에서 감지되지 않고 문에서 감지됐을 때
        if(in == false){ //차가 나갈 때
          for(int i = 0;i<9;i++)
        {
          myStepper.step(-stepsPerRevolution);  
        }
          door_open = false;
          Serial.println(10000); //10000을 출력한다
          car_in = false;
        }
      }
    }
    else{ //문이 닫혀있을 때
      if(distance_garage > 10){ //차가 차고에서 감지되지 않으면
        for(int i = 0;i<9;i++)
        {
          myStepper.step(stepsPerRevolution);  
        } //문을 연다
        door_open = true;
      }
    }
  }
  
  delay(5 0);
}
