// C++ code
// 1&2 for left motor
//3&4 for right motor

// defining bluetooth pins
const int BTRX  = 1;
const int BTTX = 0;

// We use software serial to avoid conflicts
// with the default RX/TX  pins of the Arduino board
#include <SoftwareSerial.h>
SoftwareSerial SerialBT(BTRX,  BTTX);  // to establish the bluetooth
//define the used pins
# define E_R 11
# define E_L 10
# define I1 9
# define I2 8
# define I3 12
# define I4 13
# define T_L 2
# define T_R A2
# define T_F A5
# define Echo_R A3
# define Echo_F A4
# define Echo_L 3
# define Red 5
# define Blue 6
# define Green 7
# define C_S_Pin A1
# define V_S_Pin A0
const float referenceVoltage = 5000.0; // 5V to be used for voltage sensor
int speed = 0; // t initiallize the speed the modify it to elect it later 
long duration_R, duration_L, duration_F, distance_F, distance_R, distance_L; //to be used with ultrasonic
int direction;  //1for left ,2 for right because tinker doesn't compare strings
float vcc = 5.0;     // Voltage of the Arduino board
float adcResolution = 1024.0;  // ADC resolution (10-bit)
float currentSensorOffset = 2.5; // Sensor offset voltage
float currentSensorScale = 0.185; // Sensor scaling factor (185 mV/A)
void setup()
{
  SerialBT.begin(9600);//to set up the bluetooth
  pinMode(E_R,OUTPUT);
  pinMode(E_L,OUTPUT);
  pinMode(I1,INPUT);
  pinMode(I2,INPUT);
  pinMode(I3,INPUT);
  pinMode(I4,INPUT);
  pinMode(T_L,OUTPUT);
  pinMode(T_F,OUTPUT);
  pinMode(T_R,OUTPUT);
  pinMode(Echo_R,INPUT);
  pinMode(Echo_F,INPUT);
  pinMode(Echo_L,INPUT);
  pinMode(Red,OUTPUT);
  pinMode(Blue,OUTPUT);
  pinMode(Green,OUTPUT);
  Serial.begin(9600);
}
void speed_colour(){//function to set the color of RGB LED according to speed
  if (speed==1){
    analogWrite(Green,255);
    analogWrite(Blue,0);
    analogWrite(Red,0);
  }
  else if (speed==2){
    analogWrite(Green,0);
    analogWrite(Blue,255);
    analogWrite(Red,0);
  }
  else if (speed==3){
    analogWrite(Green,0);
    analogWrite(Blue,0);
    analogWrite(Red,255);
  }
  else{
    analogWrite(Green,0);
    analogWrite(Blue,0);
    analogWrite(Red,0);
  }
}
void Forward_speed(){ //function to set the speed of motors according to speed
  if(speed==1){ 
    digitalWrite(I1,HIGH); 
    digitalWrite(I2,LOW); 
    digitalWrite(I3,HIGH); 
    digitalWrite(I4,LOW);
    analogWrite(E_R,115);
    analogWrite(E_L,85);
  }
  else if(speed==2){
    digitalWrite(I1,HIGH); 
    digitalWrite(I2,LOW); 
    digitalWrite(I3,HIGH); 
    digitalWrite(I4,LOW);
    analogWrite(E_R,200);
    analogWrite(E_L,170);
  }
  else if(speed==3){
    digitalWrite(I1,HIGH); 
    digitalWrite(I2,LOW); 
    digitalWrite(I3,HIGH); 
    digitalWrite(I4,LOW);
    analogWrite(E_R,255);
    analogWrite(E_L,225);
  }
  else {
    digitalWrite(I1,LOW); 
    digitalWrite(I2,LOW); 
    digitalWrite(I3,LOW); 
    digitalWrite(I4,LOW);
    analogWrite(E_R,0);
    analogWrite(E_L,0);
  }
}
void stop(){ //function to stop the motors
  digitalWrite(I1,LOW); 
  digitalWrite(I2,LOW); 
  digitalWrite(I3,LOW); 
  digitalWrite(I4,LOW);
  analogWrite(E_R,0);
  analogWrite(E_L,0);
  analogWrite(Green,0);
  analogWrite(Blue,0);
  analogWrite(Red,0);
}
double kp_R = 1.0, ki_R = 0.1, kd_R = 0.05; // PID constants without testing , should be modified by testing
double setpoint_R = 35.0; // Desired distance in cm
double input_R, output_R, error_R, last_error_R = 0.0;
double integral_R = 0.0, derivative_R = 0.0;
    
void Right_distance(){// function to calculate distance from ultrasonic on the right
  digitalWrite(T_R, HIGH);
  delayMicroseconds(10);
  digitalWrite(T_R, LOW);
  duration_R = pulseIn(Echo_R, HIGH);
  distance_R=(duration_R/2.0)*0.0343;
  SerialBT.print("distance_R: "); //used BT to show results on bluetooth
  SerialBT.print(distance_R);
}
double kp_L = 1.0, ki_L = 0.1, kd_L = 0.05; // PID constants without testing , should be modified by testing
double setpoint_L = 35.0; // Desired distance in cm
double input_L, output_L, error_L, last_error_L = 0.0;
double integral_L = 0.0, derivative_L = 0.0;
void Left_distance(){// function to calculate distance from ultrasonic on the left
  digitalWrite(T_L, HIGH);
  delayMicroseconds(10);
  digitalWrite(T_L, LOW);
  duration_L = pulseIn(Echo_L, HIGH);
  distance_L=(duration_L/2.0)*0.0343;
  SerialBT.print("distance_L: ");//used BT to show results on bluetooth
  SerialBT.print(distance_L);
}
double kp_F = 1.0, ki_F = 0.1, kd_F = 0.05; // PID constants  without testing , should be modified by testing
double setpoint_F = 35.0; // Desired distance in cm
double input_F, output_F, error_F, last_error_F = 0.0;
double integral_F = 0.0, derivative_F = 0.0;
void Forward_distance(){// function to calculate distance from ultrasonic in front of the car
  digitalWrite(T_F, HIGH);
  delayMicroseconds(10);
  digitalWrite(T_F, LOW);
  duration_F = pulseIn(Echo_F, HIGH);
  distance_F=(duration_F/2.0)*0.0343;
  SerialBT.print("distance_F: ");//used BT to show results on bluetooth
  SerialBT.print(distance_F);
}
void Turn_Right(){ // function to move the left motor and stop the right one to turn right
  digitalWrite(I1,HIGH); 
  digitalWrite(I2,LOW); 
  digitalWrite(I3,LOW); 
  digitalWrite(I4,LOW);
  analogWrite(E_R,0);
  analogWrite(E_L,255);
  delay(100);
  
}
void Turn_Left(){// function to move the right motor and stop the left one to turn left
  digitalWrite(I1,LOW); 
  digitalWrite(I2,LOW); 
  digitalWrite(I3,HIGH); 
  digitalWrite(I4,LOW);
  analogWrite(E_R,255);
  analogWrite(E_L,0);
  delay(100);
  
}
    
void move_back(){ // function to reverse directions of motors to move back
  digitalWrite(I1,LOW); 
  digitalWrite(I2,HIGH); 
  digitalWrite(I3,LOW); 
  digitalWrite(I4,HIGH);
  analogWrite(E_R,200);
  analogWrite(E_L,170);
}


void Auto_motion(){// function for autonomous motion
  Forward_distance();//called it to know the forward space
  Left_distance();//called it to know the left space
  Right_distance();//called it to know the right space
  Forward_speed();//to move in straight line
  delay(250);
  if(distance_F<35){ // check forward limits if smaller than 35 it moves back and controlled by PID
    error_F = setpoint_F - distance_F;
  
  // Calculate the PID terms
 	integral_F += error_F * ki_F; // Integral term (0.1 is the time step in seconds)
  	derivative_F = (error_F - last_error_F) / 0.1; // Derivative term (0.1 is the time step in seconds)
  
  // Calculate the PID output
  	output_F = (kp_F * error_F) + (ki_F * integral_F) + (kd_F * derivative_F);
    distance_F=output_F;
  // Update the last error
  	last_error_F = error_F;
    move_back();
  }
  else if (distance_R<35){// check right limits if smaller than 35 it moves left and controlled by PID
    error_R = setpoint_R - distance_R;
  
  // Calculate the PID terms
 	integral_R += error_R * ki_R; // Integral term (0.1 is the time step in seconds)
  	derivative_R = (error_R - last_error_R) / 0.1; // Derivative term (0.1 is the time step in seconds)
  
  // Calculate the PID output
  	output_R = (kp_R * error_R) + (ki_R * integral_R) + (kd_R * derivative_R);
    distance_R=output_R;
  // Update the last error
  	last_error_R = error_R;
    Turn_Left();
  }
  else if(distance_L<35){// check left limits if smaller than 35 it moves right and controlled by PID
    error_L = setpoint_L - distance_L;
  
  // Calculate the PID terms
 	integral_L += error_L * ki_L; // Integral term (0.1 is the time step in seconds)
  	derivative_L = (error_L - last_error_L) / 0.1; // Derivative term (0.1 is the time step in seconds)
  
  // Calculate the PID output
  	output_L = (kp_L * error_L) + (ki_L * integral_L) + (kd_L * derivative_L);
    distance_L=output_L;
  // Update the last error
  	last_error_L = error_L;
    Turn_Right();
  }
}
void current_sensor(){//reads the current sensor
  int adcValue = analogRead(C_S_Pin);
  float currentValue = (adcValue / adcResolution * vcc - currentSensorOffset) / currentSensorScale;// Calculate the current value
  SerialBT.print(currentvalue)
}
void volt_sensor(){// reads the volt sensor
  int sensorValue =analogRead(V_S_Pin);

  // Convert the sensor value to voltage
  float voltage = (sensorValue * referenceVoltage) / 1024.0;
  SerialBT.print(voltage);
}


int pro=0; 
void loop() {
  int fun = SerialBT.parseInt();//takes input from bluetooth to call a function later from switch case
  if (fun!=0){//to save the called function in varialble(pro) untill calling another 
    pro=fun;
  }
  SerialBT.println(pro);// to print which process we are calling
    // Handle the different cases non-blockingly
  switch (pro) {
    case 1:
      speed = 1;
      speed_colour();
      break;
    case 2:
      speed = 2;
      speed_colour();
      break;
    case 3:
      speed = 3;
      speed_colour();
      break;
    case 4:
      Auto_motion();
      current_sensor();
      volt_sensor();
      break;
    case 5:
      Forward_speed();
      break;
    case 6:
      move_back();
      break;
    case 7:
      Turn_Right();
      break;
    case 8:
      Turn_Left();
      break;
    case 9:
      stop();
      break;
  }
  delay(100);
}

