float Cal_Elev_vel(float xn1)
{
    float xn;
    

    //xn = -0.9421*H1+1.9391*F1+0.34572*E1-0.34572*G1; 5 and 35  0.006561
    xn = 0.6049*F1 - 50.27*E1 + 50.27*xn1;   // sampling of 0.01
    //xn = 0.951*F1 - 50.27*E1 + 50.27*xn1; // sampling of 0.001
    E1= xn1;
    F1= xn;
    
    return xn;

}

float Cal_Pitch_vel(float yn1)
{
    float yn;

   yn = (0.6049*F2 - 50.27*E2 + 50.27*yn1);
   //yn = (0.951*F2 - 50.27*E2 + 50.27*yn1);
   E2= yn1;
   F2= yn;
  return yn;

}


float Cal_Travel_vel(float zn1)
{
  
    float zn;
    
    zn = (0.6049*F3 -50.27*E3 +50.27*zn1); //for 0.01
    //zn = (0.951*F3 -50.27*E3 +50.27*zn1);  //for 0.001
    //zn = (0.778*F3 -50.27*E3 +50.27*zn1);   //for 0.005
    E3= zn1;
    F3= zn;
   return zn;

}
