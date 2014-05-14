//

// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the right 
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
//int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the left 
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168

#include <Wire.h>

// LSM303 accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
#define M_X_MIN -649
#define M_Y_MIN -504
#define M_Z_MIN -270
#define M_X_MAX -434
#define M_Y_MAX 224
#define M_Z_MAX -109

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

//***************************************************************************
#include <SoftwareSerial.h>
#include <Servo.h> 
//********************************************************************
/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data, 
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

//#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 1 //Will print the analog raw data
#define PRINT_EULER 0   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 13 

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values 
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
}; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here
float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};
 
//******************************************************************
Servo leftwing;
Servo rightwing;
int pos;
int posl;
int posr;
const int rxPin = 5; //SoftwareSerial RX pin, connect to JY-MCY TX pin
const int txPin = 6; //SoftwareSerial TX pin, connect to JY-MCU RX pin
// For the Micro board it is backward which is the following
// level shifting to 3.3 volts may be needed

SoftwareSerial mySerial( rxPin,txPin); // RX, TX

const int ledPin = 13;  // led pin
const int lw=8;
const int rw=11;
const int lr=9;
const int rr=10;
int lw0= 180;
int rw0= 17;
int state = 0;        // if state is 1, the LED will turn on and
// if state is 0, the LED will turn off
int flag = 0;         // a flag to prevent duplicate messages

void setup() {
  // sets the pins as outputs: 
  pinMode(ledPin, OUTPUT);
  pinMode(lw, OUTPUT);
  pinMode(rw, OUTPUT);
  mySerial.begin(9600);
  digitalWrite(ledPin, LOW); // LED is initially off
//**********************I2C IMU measurement
 // pinMode (STATUS_LED,OUTPUT);  // Status LED
  //

  I2C_Init();
  if(mySerial.available() > 0){
  mySerial.println("Pololu MinIMU-9 + Arduino AHRS");
  }
  //digitalWrite(STATUS_LED,LOW);
  delay(1500);
  mySerial.println("accel_init");
  Accel_Init();
  mySerial.println("compass_init");
  Compass_Init();
  mySerial.println("gyro_init");
  Gyro_Init();
  
  delay(20);
  
  for(int i=0;i<32;i++)    // We take some readings...
    {
    Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
    }

  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;
    
  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];
  
  //Serial.println("Offset:");
  for(int y=0; y<6; y++)
    Serial.println(AN_OFFSET[y]);
  
  delay(2000);
  //digitalWrite(ledPin,HIGH);
    
  timer=millis();
  delay(20);
  counter=0;
 mySerial.println("Done intitializing IMU");
 
  //*******************************************************
}

void loop() {
  //**************IMU reading and bluetooth
    //*******BLUETOOTH TERMAINAL COMMAND
 //reads serial input and saves it in the state variable
  if(mySerial.available() > 0){
    // mySerial.println("d: leftwing decrease; f:leftwing increase;j: leftwing -;k: leftwing +;1 to 9: set both wings to 10 to 90 degrees");
    state = mySerial.read();   
    flag=0; //clear the flag so we can print the state
  }
  // if the state is '0' the LED will turn off
  if (state == 'q') {
    lw0=180;
    rw0=17;
    if(flag == 0){
      mySerial.println("Reset angles");
      flag = 1;
    }
  }
  // if the state is '1' the led will turn on
  else if (state == 'o') {
    digitalWrite(ledPin, HIGH);
    if(flag == 0){
      mySerial.println("LED: on");
      flag = 1;
    }
  }
    else if (state == 'p') {
    digitalWrite(ledPin, LOW);
    if(flag == 0){
      mySerial.println("LED: off");
      flag = 1;
    }
  }
  else if (state == 'g')
  { 
      leftwing.attach(lw);
      pos=leftwing.read();
       if(flag == 0){
      mySerial.println("left wing position is ");
      mySerial.println(pos);
      flag=1;
    }
      leftwing.write(pos-1);
       
  while(state!='d')
  {
     if(mySerial.available() > 0){
    // mySerial.println("d: leftwing decrease; f:leftwing increase;j: leftwing -;k: leftwing +;1 to 9: set both wings to 10 to 90 degrees");
    state = mySerial.read();   
    flag=0; //clear the flag so we can print the state
  }
  }
  if(flag == 0){
      mySerial.println("detached");
      flag = 1;
    }  
  }
  else if (state == 'h')
  { 
      leftwing.attach(lw);
      pos=leftwing.read();
       if(flag == 0){
      mySerial.println("left wing position is ");
      mySerial.println(pos);
      flag=1;
    }
      leftwing.write(pos+1);
  while(state!='d')
  {
     if(mySerial.available() > 0){
    // mySerial.println("d: leftwing decrease; f:leftwing increase;j: leftwing -;k: leftwing +;1 to 9: set both wings to 10 to 90 degrees");
    state = mySerial.read();   
    flag=0; //clear the flag so we can print the state
  }
  }
  if(flag == 0){
      mySerial.println("detached");
      flag = 1;
    }  
  }
  else if (state == 'j')
  { 
      rightwing.attach(rw);
      pos=rightwing.read();
       if(flag == 0){
      mySerial.println("right wing position is ");
      mySerial.println(pos);
      flag=1;
    }
      rightwing.write(pos-1);
  while(state!='d')
  {
     if(mySerial.available() > 0){
    // mySerial.println("d: leftwing decrease; f:leftwing increase;j: leftwing -;k: leftwing +;1 to 9: set both wings to 10 to 90 degrees");
    state = mySerial.read();   
    flag=0; //clear the flag so we can print the state
  }
  }
  if(flag == 0){
      mySerial.println("detached");
      flag = 1;
    }  
  }
    else if (state == 'k')
  { 
      rightwing.attach(rw);
      pos=rightwing.read();
       if(flag == 0){
      mySerial.println("right wing position is ");
      mySerial.println(pos);
      flag=1;
    }
      rightwing.write(pos+1);
  while(state!='d')
  {
     if(mySerial.available() > 0){
    // mySerial.println("d: leftwing decrease; f:leftwing increase;j: leftwing -;k: leftwing +;1 to 9: set both wings to 10 to 90 degrees");
    state = mySerial.read();   
    flag=0; //clear the flag so we can print the state
  }
  }
  if(flag == 0){
      mySerial.println("detached");
      flag = 1;
    }  
  }
    else if (state == '0') {
    leftwing.attach(lw);
    rightwing.attach(rw);
    leftwing.write(lw0);
    rightwing.write(rw0); 
    if(flag == 0){
      mySerial.println("Servo angle: +0");
      flag = 1;
    }   
  }
      else if (state == '1') {
    leftwing.attach(lw);
    rightwing.attach(rw);
    leftwing.write(lw0-10);
    rightwing.write(rw0+10); 
    if(flag == 0){
      mySerial.println("Servo angle: +10");
      flag = 1;
    }   
  }
      else if (state == '2') {
    leftwing.attach(lw);
    rightwing.attach(rw);
    leftwing.write(lw0-20);
    rightwing.write(rw0+20); 
    if(flag == 0){
      mySerial.println("Servo angle: +20");
      flag = 1;
    }   
  }
    else if (state == '3') {
    leftwing.attach(lw);
    rightwing.attach(rw); 
    leftwing.write(lw0-30);
    rightwing.write(rw0+30);
    if(flag == 0){
      mySerial.println("Servo angle: +30");
      flag = 1;
    }   
  }
    else if (state == '4') {
    leftwing.attach(lw);
    rightwing.attach(rw);
    leftwing.write(lw0-40);
    rightwing.write(rw0+40); 
    if(flag == 0){
      mySerial.println("Servo angle: +40");
      flag = 1;
    }   
  }
      else if (state == '5') {
    leftwing.attach(lw);
    rightwing.attach(rw);
    leftwing.write(lw0-50);
    rightwing.write(rw0+50); 
    if(flag == 0){
      mySerial.println("Servo angle: +50");
      flag = 1;
    }   
  }
  else if (state == '6') {
    leftwing.attach(lw);
    rightwing.attach(rw);
    leftwing.write(lw0-60);
    rightwing.write(rw0+60); 
    if(flag == 0){
      mySerial.println("Servo angle: +60 degrees");
      flag = 1;
    }   
  }
    else if (state == '7') {
    leftwing.attach(lw);
    rightwing.attach(rw);
    leftwing.write(lw0-70);
    rightwing.write(rw0+70); 
    if(flag == 0){
      mySerial.println("Servo angle: +70 degrees");
      flag = 1;
    }   
  }
      else if (state == '8') {
    leftwing.attach(lw);
    rightwing.attach(rw);
    leftwing.write(lw0-80);
    rightwing.write(rw0+80); 
    if(flag == 0){
      mySerial.println("Servo angle: +80 degrees");
      flag = 1;
    }   
  }
    else if (state == '9') {
    leftwing.attach(lw);
    rightwing.attach(rw);
    leftwing.write(lw0-90);
    rightwing.write(rw0+90); 
    if(flag == 0){
      mySerial.println("Servo angle: +90 degrees");
      flag = 1;
    }   
  }
   else if (state == 'p') {
    posl=leftwing.read();
    posr=rightwing.read();
    leftwing.attach(lw);
    rightwing.attach(rw);
    leftwing.write(posl-5);
    rightwing.write(posr+5); 
    posl=leftwing.read();
    posr=rightwing.read();
    if(flag == 0){
      mySerial.println("Servo angle: +5 degrees");
      mySerial.println("Left wing:");
       mySerial.println(posl);
      mySerial.println("Right wing:");
       mySerial.println(posr);
      flag = 1;
    }  
    
    while(state!='d')
  {
     if(mySerial.available() > 0){
    // mySerial.println("d: leftwing decrease; f:leftwing increase;j: leftwing -;k: leftwing +;1 to 9: set both wings to 10 to 90 degrees");
    state = mySerial.read();   
    flag=0; //clear the flag so we can print the state
  }
  } 
  }
     else if (state == 'm') {
     posl=leftwing.read();
    posr=rightwing.read();
    leftwing.attach(lw);
    rightwing.attach(rw);
    leftwing.write(posl+5);
        rightwing.write(posr-5); 
    if(flag == 0){
      mySerial.println("Servo angle: +5 degrees");
      mySerial.println("Left wing:");
       mySerial.println(posl);
      mySerial.println("Right wing:");
       mySerial.println(posr);
      flag = 1;
    }   
    
    while(state!='d')
  {
     if(mySerial.available() > 0){
    // mySerial.println("d: leftwing decrease; f:leftwing increase;j: leftwing -;k: leftwing +;1 to 9: set both wings to 10 to 90 degrees");
    state = mySerial.read();   
    flag=0; //clear the flag so we can print the state
  }
  }

  }
  else if (state == 's') {
    leftwing.attach(lw);
    rightwing.attach(rw);
    leftwing.write(lw0);
    rightwing.write(rw0); 
    if(flag == 0){
     mySerial.println("sweep");
     flag=1;
    }
    for(pos = 0; pos < 60; pos += 1)  // goes from 0 degrees to rw0 degrees 
    {                                  // in steps of 1 degree 
      leftwing.write(lw0-pos);
      rightwing.write(rw0+pos); 

      // tell servo to go to position in variable 'pos' 
      delay(1000);                       // waits 15ms for the servo to reach the position 
        if(mySerial.available() > 0){
          state = mySerial.read();
          flag=0; //clear the flag so we can print the state
        }
      if (state=='d')
      {
         leftwing.detach();
          rightwing.detach();
        if(flag == 0)
        {
          mySerial.println("detached");
          flag=1;
          break;
        }
      }
          mySerial.println("testpoint1");
          flag=1;
    }
    for(pos = 60; pos > 0; pos -= 1)  // goes from 0 degrees to rw0 degrees 
    { 
    
      leftwing.write(lw0-pos);
        rightwing.write(rw0+pos); 
      // tell servo to go to position in variable 'pos' 
      delay(1000);                       // waits 15ms for the servo to reach the position 
      if(mySerial.available() > 0){
        state = mySerial.read();
        flag=0; //clear the flag so we can print the state
      }
      if (state=='d')
      {
         leftwing.detach();
         rightwing.detach();
         if(flag == 0)
         {
           mySerial.println("detached");
           flag=0;
         }
         break;
      }
   }
    leftwing.detach();
    rightwing.detach();
 mySerial.println("testpoint2");
   flag=1;
  }
   else if (state=='d')
      {
         leftwing.detach();
         rightwing.detach();
         if(flag == 0)
         {
           mySerial.println("detached");
           flag=1;
         }
      }
     else if (state == 'r') {
    posl=leftwing.read();
    posr=rightwing.read();
    if(flag == 0){
      mySerial.println("Left wing:");
       mySerial.println(posl);
      mySerial.println("Right wing:");
       mySerial.println(posr);
      flag = 1;
    }
      }   
       else if (state == 'z') {
    lw0=leftwing.read();
    rw0=rightwing.read();
    if(flag == 0){
      mySerial.println("Set zero conditions");
      flag = 1;
    }
      }  
       else if (state == 'a') 
       {           
         while(state =='a') 
         {
 /*
     if(flag == 0){
      mySerial.println("Pitch =");
      mySerial.println(pitch);
      mySerial.println("Roll = ");
      mySerial.println(roll);
      mySerial.println("Yaw =");
      mySerial.println(yaw);
      flag = 1;
    }
    */  
    if((millis()-timer)>=20)  // Main loop runs at 50Hz
  {
    
    counter++;
    timer_old = timer;
    timer=millis();
    if (timer>timer_old)
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;
    
    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer
    
    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
      {
      counter=0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading  
      }
    
    // Calculations...
    Matrix_update(); 
    Normalize();
    Drift_correction();
    Euler_angles();
    // ***
   
    printdata();
    if(mySerial.available() > 0){
    // mySerial.println("d: leftwing decrease; f:leftwing increase;j: leftwing -;k: leftwing +;1 to 9: set both wings to 10 to 90 degrees");
    state = mySerial.read();   
    flag=0; //clear the flag so we can print the state
  }
  }
    
         }
      }  
     
     //****************IMU READING
     /*
     counter++;
    timer_old = timer;
    timer=millis();
    if (timer>timer_old)
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;
    
    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer
    
    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
      {
      counter=0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading  
      }
    
    // Calculations...
    Matrix_update(); 
    Normalize();
    Drift_correction();
    Euler_angles();
    // ***
   
    printdata(); 
    */
}
  //**************
