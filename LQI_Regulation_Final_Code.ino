//LQI control method or (PID control based on LQR for elevation and travel)for regulating the Quansers 3DOF Helicopter
//In this method the control theory is valid at equilibrium point only.
// Main objective is to control or regulate the angular position to zero i.e equilibrium point.
// Motors max voltage is +/-24V and continous or rated voltage +/-12V.
// The velocities are estimated by Discrete High Pass Filter.
// counterweight is positioned at 5th slot.

#include <SPI.h>
#include "Wire.h"

//Sampling time
unsigned int T = 10 ;
unsigned long currentMillis1 = 0;
unsigned long currentMillis2 = 0;
unsigned long currentMillis3 = 0;
long lasttime1 =0;
long lasttime2 =0;
long previousMillis = 0;
long previousMillis1 = 0;
long previousMillis2 = 0;
long previousMillis3 = 0;
long previousMillis4 = 0;
long previousMillis5 = 0;
long previousMillis6 = 0;

float val1    = 0;   // analog value 
float val2    = 0;   // analog value 
float val3    = 0;   // analog value 
float oldVal1 = 0;
float oldVal2 = 0;
float oldVal3 = 0;

float xi1 =0;
float xi2 =0;
float zi1 =0;
float zi2 =0;

 //Motor PWM output Pins
 int PWM1 = 5;         
 int PWM2 = 6;
 int PWM_Vf,PWM_Vb;
 // Encoder resolution
 int Enc_res1= 1024;//Enc_res2= 8192;
 // Motor Direction Pins
 int Dir_pin1 = 44;
 int Dir_pin2 = 46;
 unsigned long dt1=0;
 float val11=0;


// Encoder pins
const int slaveSelectEnc1 = 53;   // Slave Select pins for Elevation Encoder
const int slaveSelectEnc2 = 49;   // Slave Select pins for Pitch Encoder
const int slaveSelectEnc3 = 48; // Slave Select pins for Travel Encoder

// These hold the current encoder count.
signed long enc1count = 0;
signed long enc2count = 0;
signed long enc3count = 0;

// states of the system
float x1=0;
float y1=0;
float z1=0;
float x2=0;
float y2=0;
float z2=0;

//Desired position
float xd1=0;
float xd3=0;

//error
float err1 = 0;
float err2 = 0;

const float pi = 3.14159267;

// Feedback gains or LQI gains
const float k11=  10.4875;                           
const float k12= 18.2538;
const float k13= -5.4912;
const float k14= 11.0660 ;
const float k15= 5.6261;
const float k16= -4.6096;
const float k17= 3.1623;
const float k18= -3.1623;
const float k21= k11;
const float k22= -k12 ;
const float k23= -k13 ;
const float k24= k14;
const float k25= -k15;
const float k26= -k16 ;
const float k27= k17;
const float k28= -k18;

float U1,U2;
float Vf,Vb ;

// defined variable which are used for discrete High-pass Filter 
float E1=0;
float E2=0;
float E3=0;
float F1=0;
float F2=0;
float F3=0;
int i =0;


// Void setup
void setup() 
{
  // put your setup code here, to run once:
   Wire.begin(); 
   Serial.begin(57600);
   pinMode(PWM1 , OUTPUT);
   pinMode(PWM2 , OUTPUT);
   pinMode(Dir_pin1, OUTPUT);
   pinMode(Dir_pin2, OUTPUT);
   pinMode(28, OUTPUT);
   pinMode(30, OUTPUT);
   pinMode(32, OUTPUT);
   pinMode(34, OUTPUT);
   
   initEncoders();       
   Serial.println("Encoders Initialized...");  
   clearEncoderCount();  
   Serial.println("Encoders Cleared...");
}

void loop() 
{
    unsigned long currentMillis = millis();  
    float Time1 = float(currentMillis - previousMillis);
   unsigned long Time = currentMillis - previousMillis; 
    if( Time > T)
    {
      
    previousMillis = currentMillis; 

    // Calulating the position of elevation x1, pitch y1 and travel z1.
    x1 = readValue1();
    y1 = readValue2();
    z1 = readValue3();

    // Saturating the angular positon or providing the limits.
    
    if (x1 > 0.78)
    {
      x1 = 0.78;
    }
    else if (x1<-0.78)
    {
      x1 = -0.78;
    }
    if (y1 > 0.78)
    {
      y1 = 0.78;
    }
    else if (y1<-0.78)
    {
      y1 = -0.78;
    }
    if (z1 > 0.78)
    {
      z1 = 0.78;
    }
    else if (z1<-0.78)
    {
      z1 = -0.78;
    }
   
    //Serial.println("First Angular position");
    //Serial.print(" x1: "); 
    Serial.print(x1);  Serial.print(" ");
    //Serial.print(" y1: "); 
    Serial.print(y1);  Serial.print(" ");
    //Serial.print(" z1: "); 
    Serial.print(z1); Serial.print(" ");

   //angular velocities of all three degree of freedom
   
   x2 = Cal_Elev_vel(x1);
   y2 = Cal_Pitch_vel(y1);
   z2 = Cal_Travel_vel(z1);
     
   //Serial.println("First Angular velocities");
   //Serial.print(" x2: "); 
    Serial.print(x2); Serial.print(" ");
  //Serial.print(" y2: "); 
    Serial.print(y2);  Serial.print(" ");
  //Serial.print(" z2: "); 
    Serial.print(z2);  Serial.print(" ");

   // Vf and Vb are integral voltage values of the elevation and travel 
   
    Vf = Integral1();
    Vb = Integral2();

    Serial.print(Vf); Serial.print(" ");
    Serial.print(Vb); Serial.print(" ");

    // feedback control signal calculated from LQR method 
    // where 7.5V is bias control voltage.
    
    U1 = -(x1*k11+y1*k12+z1*k13+x2*k14+y2*k15+z2*k16+Vf)+7.5;//+7.5;
    U2 = -(x1*k21+y1*k22+z1*k23+x2*k24+y2*k25+z2*k26+Vb)+7.5;//+7.5;
      
    //Serial.print("Controller1: "); 
    Serial.print(U1); Serial.print(" ");   // Control signal for Front motor
   // Serial.print("Controller2: "); 
    Serial.println(U2);     // Control signal for Back motor

    int lim =24;
    if (U1 > lim)
    {
      U1 = lim;
    }
    else if (U1 < -lim)
    {
      U1 = -lim;
    }
    if (U2 > lim)
    {
      U2 = lim;
    }
    else if (U2 < -lim)
    {
      U2 = -lim;
    }

    PWM_Vf = map(U1,0,24,0,255);
    analogWrite(PWM1,PWM_Vf);               // analog output which is given to front motor  
    digitalWrite(Dir_pin1,LOW);             // digital pin for defining the direction of rotation of the motor in this case propeller or blades rotate in anti-clockwise 
    PWM_Vb = map(U2,0,24,0,255);            // define the PWM signal to volage range.
    analogWrite(PWM2,PWM_Vb);               // analog output which is given to back motor
    digitalWrite(Dir_pin2,LOW);
   
    
    i = i+1;  // just to check no. of loops

 }
}
