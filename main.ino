// Created by Sergiy&Danila inc.



#include<AFMotor.h>
class USsensor{
  private:
    int trig = 24;          //default value for trig of UltraSonic
    int echo = 25;          //default value for echo of UltraSonic
    float newDistance = 0;  
    float oldDistance = 0;
    float duration;         //Duration of sound wave flight
    bool first = true;      //First read (see delta check)
    bool core = 0;          //Corection value fro delta check)
    float delta = 50;       //Default value for delta 
    float startDelta = 50;  //Default vaule for start delta
   
  public:
    USsensor(){
      // constuctor for lazy people with default settings 
      pinMode(trig,OUTPUT);
      pinMode(echo, INPUT);
    }
    USsensor(int trig, int echo, float delta){
      //constructor with everything
      this->trig = trig;
      this->echo = echo;
      this->delta = delta;
      this->startDelta = delta;
      pinMode(trig,OUTPUT);
      pinMode(echo, INPUT);
    }
    float getDistance(){
      return newDistance;
    }
    void readSensor(){
      //get data from UltraSonic sensor 
      digitalWrite(trig, LOW);
      delayMicroseconds(5);
      digitalWrite(trig, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig, LOW); 
      oldDistance = newDistance;
      duration = pulseIn(echo, HIGH);
      newDistance = ((duration)/2)/29.1;
      delayMicroseconds(500);
    }
    void deltaCheck(){
      //check for miss-readed value (noise filtering)
      if ((abs(oldDistance-newDistance))>delta){
        newDistance = oldDistance;
        delta = delta + delta*core*0.5;
        core ++;
       // Serial.print("auto correction");
      }else{
        delta = startDelta;
        core = 0;
      }
    }
};


class motionControl{
  // 4 указателя на 4 мотора + всякая фигня 
  private:
    int speed = 255;                  //default max speed
    int IRPIN = 22;                   //Pin for Infra-Red sensor 
    bool isObstacle = 0;              //0 - No obstacle, 1 - obstacel 
    char input;                       //Input value(letter from bluetooth)
    char dir;                         //check for last action 
    bool IR = 0;                      //Input from Infra-red Sensor 
    float lastk = 0;                  //last proportional/integrational k 
    float k = 0;                      //present proportional/integrational k 
    float leftMotors;                 //speed for left pair of motors
    float rightMotors;                //speed for right pair of motors
    AF_DCMotor *motor1;               //pointers for all the motors
    AF_DCMotor *motor2;
    AF_DCMotor *motor3;
    AF_DCMotor *motor4;
    float pk = 0; 
  public:
  //constuctor with everything
    motionControl(int IRPIN, int speed ,AF_DCMotor *motor1,AF_DCMotor *motor2, AF_DCMotor *motor3, AF_DCMotor *motor4 ){
      this->IRPIN = IRPIN;
      this->speed = speed;
      pinMode(IRPIN, INPUT);
      this->motor1 = motor1;
      this->motor2 = motor2;
      this->motor3 = motor3;
      this->motor4 = motor4;
      pinMode(IRPIN, INPUT);
 
    }
    //constructor with just motors assigning 
    motionControl(AF_DCMotor *motor1,AF_DCMotor *motor2, AF_DCMotor *motor3, AF_DCMotor *motor4){
      //default constructor 
      pinMode(IRPIN, INPUT);
      this->motor1 = motor1;
      this->motor2 = motor2;
      this->motor3 = motor3;
      this->motor4 = motor4;
    }
    //destructor with deleting motors
    ~motionControl(){
      delete motor1;
      delete motor2;
      delete motor3;
      delete motor4;
    }
    //input value setting (bluetooth)
    void setInput(char input){
      this->input = input;
              
    }
    //if IR see an object (for input value 0-object, 1 - nothing) (for output 1- object, 0 -nothing)
    void stopIR(){
      IR = digitalRead(IRPIN);
      if (IR == 0){
        this->isObstacle = 1;
        //Serial.print(isObstacle);
      }
      else{
        this->isObstacle = 0;
        //Serial.print(isObstacle);
      }
      if (isObstacle == 1 && input == 'f'){
        //updated version 
        }
     }
    //setting speed for the motors
    void setMotorsSpeed(){
       motor1->setSpeed(this->speed);
       motor2->setSpeed(this->speed);
       motor3->setSpeed(this->speed);
       motor4->setSpeed(this->speed);
    }
    //changing speed of the motors
    void changeSpeed(){
      //increasing 
      if (input =='U'){
        if (speed <= 245){
          speed = speed + 10;
        }else{
          speed = 255;
        }
      }
      //decreasing 
      else if(input == 'D'){
        if (speed >= 10){
          speed = speed - 10;
        }
        else {
          speed  = 0;
        }
      }
    }
  //  a lot of stupied code for controling motors
  void movmentControl(){
    if (input == 'f'){
      // forward
      if (isObstacle != 1){
        dir = input;
        motor1->run(FORWARD);
        motor2->run(FORWARD);
        motor3->run(FORWARD);
        motor4->run(FORWARD);
      }else{
        motor1->run(RELEASE);
        motor2->run(RELEASE);
        motor3->run(RELEASE);
        motor4->run(RELEASE);
      }
      //to the right
    }else if (input == 'r'){
      dir = input;
      motor1->run(FORWARD);
      motor2->run(FORWARD);
      motor3->run(BACKWARD);
      motor4->run(BACKWARD);
      // to the left
    }else if(input == 'l'){
      dir = input;
      motor1->run(BACKWARD);
      motor2->run(BACKWARD);
      motor3->run(FORWARD);
      motor4->run(FORWARD);
      //backwards
    }else if(input == 'b'){
      dir = input;
      motor1->run(BACKWARD);
      motor2->run(BACKWARD);
      motor3->run(BACKWARD);
      motor4->run(BACKWARD); 
    }else if(input == 's'){     //stop
      motor1->run(RELEASE);
      motor2->run(RELEASE);
      motor3->run(RELEASE);
      motor4->run(RELEASE); 
    }
  }
  void autoDrive(float left, float right, float middle){
   // autoDrive using walls and 3 UltraSonic sensors
    motor1->run(FORWARD);
    motor2->run(FORWARD);
    motor3->run(FORWARD);
    motor4->run(FORWARD);
    //changing last error
    lastk = k;  
    pk = (50-middle)/25+1; //increase k based on how wall is far from us
    if (pk <= 1){
      pk = 1; // if less than 0 - assigning two one, which means that object is further than 50 sm and we do not need to care about it
    }else{
      //everything !OK, object is less than 50 sm ahead
    }
    Serial.println(pk);
    k = (left - right)*3*pk+(k-lastk)*5+(k+lastk)*0.1; //PID regulator formula (proprotional part)*k1*(correction pk)+(differential part)*k2+(integral part)*k3;
    // speed of the left motor based on the PID regulator 
    leftMotors = 125 -k;
    // speed of the right motor based on the PID regulator 
    rightMotors = 125 + k;
    // error avoiding in case of assigning speed > 255 or less than 0
    if (leftMotors > 255){
      leftMotors = 255;
    }else if(leftMotors<0){
      leftMotors = 0 ;
    }
    if (rightMotors > 255){
      rightMotors = 255;
    }else if(rightMotors<0){
      rightMotors = 0 ;
    }
    //Setting speed
    motor1->setSpeed(leftMotors);
    motor2->setSpeed(leftMotors);
    motor3->setSpeed(rightMotors);
    motor4->setSpeed(rightMotors);
 
  }
  void driveBack(float dist){
    // TODO 
}
};

// new objects for 4 motors
AF_DCMotor *motor1 = new AF_DCMotor(1); 
AF_DCMotor *motor2 = new AF_DCMotor(2); 
AF_DCMotor *motor3 = new AF_DCMotor(3);
AF_DCMotor *motor4 = new AF_DCMotor(4);
// new object for control 
motionControl *control = new motionControl(motor1, motor2, motor3, motor4);
// objects for ultrasonic reading 
USsensor *UltraSonicL = new USsensor();
USsensor *UltraSonicR = new USsensor(26, 27, 50);
USsensor *UltraSonicM = new USsensor(28, 29, 50);
//bluetooth input value
char input;

void setup() {
  //Serial communication 
  Serial.begin(9600);
  Serial1.begin(9600);
  // Assigning 1 value for ultrasonic sensor, so we will always have delta and last delta 
  UltraSonicL->readSensor();  
  UltraSonicR->readSensor();
  UltraSonicM->readSensor();  
}

void loop() {
  //reading from all 3 sensors
  UltraSonicR->readSensor();
  UltraSonicL->readSensor();
  UltraSonicM->readSensor();
  //checking if data is appropriate 
  UltraSonicR->deltaCheck();
  UltraSonicL->deltaCheck();
  UltraSonicM->deltaCheck();

  //control->driveBack(UltraSonicM->getDistance());
  //Serial.println(UltraSonicM->getDistance());
  //Autodrive
  control->autoDrive(UltraSonicL->getDistance(),UltraSonicR->getDistance(), UltraSonicM->getDistance());
  /*
  if (Serial1.available()) // проверяем, поступают ли какие-то команды
  {    
      control->setInput(input);
      control->changeSpeed();
      control->setMotorsSpeed();
      control->movmentControl();
      control->stopIR();
      
    }
  }else{
    control->stopIR();
  }
 */
}
