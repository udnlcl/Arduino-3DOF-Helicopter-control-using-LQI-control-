// function for reading Elevation angular position

float readValue1()
{
  float u;
  currentMillis1 = millis();
  enc1count = 131+readEncoder(1);         // intial condition is 131 or 0.2 radians.
  u = ((-enc1count*360)/(4*1024))*pi/180; //elevation
  val1 = u;
  return val1;
  previousMillis1 = currentMillis1;
  oldVal1 = val1;
}


//----------------------------------------------------------------------
// function for reading Pitch angular position

float readValue2()
{
  float u;
  unsigned long currentMillis2 = millis();
  enc2count = readEncoder(2);                  // intial condition is 0 radians.
  u = ((enc2count*360)/(4*1024))*pi/180; //elevation
  val2 = u;
  return val2;
  previousMillis2 = currentMillis2;
  oldVal2 = val2;
}

//-----------------------------------------------------------------------
// function for reading Travel angular position

float readValue3()
{
  float u;
  unsigned long currentMillis3 = millis();
  enc3count = readEncoder(3);                             // intial condition is 0 radians.
  u = ((enc3count*360)/(8*1024))*pi/180; //elevation
  val3 = u;
  return val3;
  previousMillis3 = currentMillis3;
  oldVal3 = val3;
}

//--------------------------------------------------------------------------
// Integral function for calculating Vf.

float Integral1()
{
float front;
float dt = (millis()- lasttime1);
lasttime1 = millis();  
enc1count = 131+readEncoder(1);
xi1 = ((-enc1count*360)/(4*1024))*pi/180;
enc3count = readEncoder(3);
zi1 = ((enc3count*360)/(8*1024))*pi/180; //elevation
err1 = xi1-xd1;
err2 = zi1-xd3;
front += (k17*err1+k18*err2)*(dt);
return front;
}

//--------------------------------------------------------------------------
// Integral function for calculating Vb.


float Integral2()
{
float back;
float dt = (millis()- lasttime2);
lasttime2 = millis();  
enc1count = 131+readEncoder(1);
xi2 = ((-enc1count*360)/(4*1024))*pi/180;
enc3count = readEncoder(3);
zi2 = ((enc3count*360)/(8*1024))*pi/180; //elevation
err1 = xi2-xd1;
err2 = zi2-xd3;
back += (k27*err1+k28*err2)*(dt);
return back;
}


