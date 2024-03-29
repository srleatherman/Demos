#include "Encoder.h" //this gives us access to the encoder.h files 
#define output2A 1 //this pin is output A for the encoder motor 2
#define output2B 3 //this pin is output B for the encoder motor 2
#define outputA 2 //this pin is output A for the encoder motor 1
#define outputB 5 // this pin is output B for the encoder motor 1
#define motorVoltageRight 9 //this is one of the two voltage pwm pins we could have picked
#define motorVoltageLeft 10//thi is for the second motor voltage and a pwm pin
#define button 4 // this pin is for setting the interupt
#define signOfVoltage 7 // this is what direction the wheel will turn motor 1
#define signOfVoltLeft 8
#define flagIndicator 12 // this pin is if there is a flag

//the following line of code sets up the encoder to read from outputs a and b and will be used throughout the code
//using the encoder.h file allows us to simplify the code
Encoder motorRight(outputA, outputB);
Encoder motorLeft(output2A, output2B);

float controlSignal; //this variable is used in controller function
float desiredVoltage; //this is the desired voltage not in PWM

float desiredVoltagePWM; //this is the desired voltage in PWM

float Kp = 5; // design value based on simulation
float Ki = 0; // design value based on simulation
int overallTime; // this keeps track of the overall time it takes the loop to complete
int startTime=0; // this is the time the loop starts
int startPosition; // this is the start position of the encoder at the beginning of the loop
int currentCountsRight; // this is the counts that the encoder is currently at in the loop
int currentCountsLeft;
float currentAngleRight; // this the angle that the encoder is currently at in the loop
float currentAngleLeft;
int dr;
int dl;
int drold;
int dlold;
int angRightVel;
int angLeftVel;
float startAngle; // this is the starting angle that the encoder is at the beginning of the loop
int FIXED_CYCLE_TIME = 10; // unit is in ms
float pi = 3.1415; // known constant
int cpr = 3200; // counts per rotation known from the motor data sheet
float desiredAngle; // stores what the input from the PI means  
String data; // input from the PI
int dataInt;
bool DataRead; // makes sure that the data read from the PI is being sent correctly


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //set up all of my output pins 
  pinMode(button, OUTPUT);
  digitalWrite(button, HIGH);
  
  pinMode(signOfVoltage,OUTPUT);
  digitalWrite(signOfVoltage, HIGH);

  pinMode(signOfVoltLeft,OUTPUT);
  digitalWrite(signOfVoltLeft, HIGH);
  
  pinMode(motorVoltageRight, OUTPUT);
  pinMode(motorVoltageLeft, OUTPUT);
  
  //set all of my input pins
  pinMode(flagIndicator, INPUT); 
}

void loop() {
  // put your main code here, to run repeatedly:
  //reads time at begining of loop
  startTime = millis();

  //calls the function that determines what the desired angle is based on what is read in from the PI
  dataInt = data.toInt();
  desiredAngle = pi*dataInt/2;
  Serial.print(desiredAngle);
  //reads the current state of the encoder in counts
  currentCountsRight = motorRight.read(); // this is reading the current counts from the encoder
  currentCountsLeft = motorLeft.read();
  
  //the equation to get angle from counts is counts*2*pi/counts per rotation
  //the final units are radians
  currentAngleRight = (float)currentCountsRight*2*pi/(float)cpr;
  currentAngleLeft = (float)currentCountsLeft*2*pi/(float)cpr;
  
  controllerOne(desiredAngle, currentAngleRight);
  controllerTwo(desiredAngle, currentAngleLeft);
  
  //reads time at end of loop
  overallTime = millis();

  dr = motorRight.read();
  angRightVel = (dr-drold)/(overallTime-startTime);  
  drold = dr;
  
  dl = motorLeft.read();
  angLeftVel = (dl-dlold) / (overallTime-startTime);
  dlold = dl;
}

//the contoller function implements the controller designed in the arduino code
void controllerOne(float desiredAngle,float currentAngle){
  //the function takes in the desired angle and the current angle that the encoder is at

  //the desired voltage is determined by how large the difference between the desired and current angle
  //multiplied by the Kp value found. Since our KI value from our controller was zero that part is not implemented
  desiredVoltage = (float)Kp*(desiredAngle - currentAngle);

  //the desired boltage must be converted to a PWM value because that is what the motorVoltage can interpret
  desiredVoltagePWM = (float)desiredVoltage*255/7.5;

  //this checks the sign of the voltage
  if( desiredVoltagePWM <0){
    //if it is less than zero it is negative
    digitalWrite(signOfVoltage,false);
  }else{
    //if it is greater than zero it is positive
    digitalWrite(signOfVoltage, true);
  } 

  //this takes the unsigned PWM value
  controlSignal = abs(desiredVoltagePWM);

  //In the chance the the desired voltage exceeds 7.5 volts we can not meet those requirements, so we set
  //the voltage to the maximum possible PWM value of 255.
  if(controlSignal >255){
    controlSignal = 255;
  }
  //this writes to the pin that controls the voltage the voltage we derived.  
  analogWrite(motorVoltageRight, controlSignal);
  startPosition = currentAngle;
}
void controllerTwo(float desiredAngle,float currentAngle){
  //the function takes in the desired angle and the current angle that the encoder is at

  //the desired voltage is determined by how large the difference between the desired and current angle
  //multiplied by the Kp value found. Since our KI value from our controller was zero that part is not implemented
  desiredVoltage = (float)Kp*(desiredAngle - currentAngle);

  //the desired boltage must be converted to a PWM value because that is what the motorVoltage can interpret
  desiredVoltagePWM = (float)desiredVoltage*255/7.5;

  //this checks the sign of the voltage
  if( desiredVoltagePWM <0){
    //if it is less than zero it is negative
    digitalWrite(signOfVoltage,false);
  }else{
    //if it is greater than zero it is positive
    digitalWrite(signOfVoltage, true);
  } 

  //this takes the unsigned PWM value
  controlSignal = abs(desiredVoltagePWM);

  //In the chance the the desired voltage exceeds 7.5 volts we can not meet those requirements, so we set
  //the voltage to the maximum possible PWM value of 255.
  if(controlSignal >255){
    controlSignal = 255;
  }
  //this writes to the pin that controls the voltage the voltage we derived.  
  analogWrite(motorVoltageLeft, controlSignal);
  startPosition = currentAngle;
}
