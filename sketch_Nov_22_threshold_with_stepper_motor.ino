

#include "LSM6DS3.h"
#include "Wire.h"


/*
  
*/


// Constants
const int  Psele1 = 9;    // the pin that the first selector LSB of motor controller is attached to
const int  Psele2 = 10;    // // the pin that the second selector LSB of motor controller is attached to
const int  Psele3 = 11;    // the pin that the third selector LSB of motor controller is attached to
const int  Psele4 = 12;    // the pin that the fourth selector LSB of motor controller is attached to
const int  Pstart = 13;    // the pin that the initiate motion of the motor  is attached to
const int  Pservo = 2;    // the pin that the power-on to the motor  is attached to


// Stepper motor coמstants


const int SwitchPin = 4;
const int refPin = 5;
const int stepPin = 6; 
const int dirPin = 7;



const int SOFT_POT_PIN_F = A3; // Pin connected to front softpot wiper
const int SOFT_POT_PIN_R = A5; // Pin connected to rear softpot wiper

//Flexiforce parameters

//Front Sensor parameters
const int ffs_F = A0; // Pin connected to front softpot wiper
float cf_F = 19.5;
int ffsdata_F = 0;
float vout_F;

//Rear Sensor parameters
const int ffs_R = A1; // Pin connected to front softpot wiper
float cf_R =1000;
//float cf_R = 19.5;
int ffsdata_R = 0;
float vout_R;

//Extreme thresholds initialization 

//float thrshigh;
//float thrslow;

//

//SoftPot parameters
int softPotADC_F = 0;
int softPotADC_R = 0;
const int GRAPH_LENGTH = 40; // Number of Positions- 0.022/(40)=0.00055m between points



LSM6DS3 myIMU(I2C_MODE, 0x6A);

// Variables will change:
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button

int lastButtonState = 0;     // previous state of the button

const int cycle = 9300;//2000 originaly
const int speed = 1500;//800;
int switchstate=1;
int refstate=1;
int lastswitchstate=0;
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
unsigned long debouncecounter = 0;


void setup() {
  // 
  pinMode(SOFT_POT_PIN_F, INPUT);
  pinMode(SOFT_POT_PIN_R, INPUT);
  pinMode(ffs_F, INPUT);
  pinMode(ffs_R, INPUT);

  
  
  pinMode(Psele1, OUTPUT);
  pinMode(Psele2, OUTPUT);
  pinMode(Psele3, OUTPUT);
  pinMode(Psele4, OUTPUT); 
  pinMode(Pstart, OUTPUT);
  pinMode(Pservo, OUTPUT);

  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(SwitchPin,INPUT);
  pinMode(refPin,OUTPUT);
  
digitalWrite(dirPin,HIGH);
digitalWrite(stepPin,HIGH);
digitalWrite(refPin,HIGH);

digitalWrite(Psele1,HIGH);
digitalWrite(Psele2,HIGH);
digitalWrite(Psele3,HIGH);
digitalWrite(Psele4,HIGH);
digitalWrite(Pstart,HIGH);
digitalWrite(Pservo,HIGH);

  // initialize serial communication:
 
  Serial.begin(9600);
//  Serial1.begin(9600);

while (!Serial);
    //Call .begin() to configure the IMUs
    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }


Serial.println("Choose which Mode the motor should operate: ");
  Serial.println("1 -  Light Beating ");
  Serial.println("2 -  Moderate Beating ");
  Serial.println("3 -  Intense Beating ");
  Serial.println("4 -  Extreme Beating ");
  
}



void loop() {

  


char val1=1;  //auto entry mode
char val= Serial.read();
//char val1= Serial1.read();




if (val== '1')

  {

                              //     for (int j=0; j<10 ; j++)
                              //  {  
    digitalWrite(Psele1,LOW);
    digitalWrite(Psele2,HIGH);
    digitalWrite(Psele3,HIGH);
    digitalWrite(Psele4,HIGH);
    
// digitalWrite(Pservo,LOW);
//
 delay(5000);
//    digitalWrite(Pstart,LOW);


//Sensor Reading

for(int j=0;j<2500;j++)
{
ffsdata_F = analogRead(ffs_F);
  ffsdata_R = analogRead(ffs_R);
  // Read in the soft pot's ADC value
  softPotADC_F = analogRead(SOFT_POT_PIN_F);
  softPotADC_R = analogRead(SOFT_POT_PIN_R);
  // Map the 0-1023 value to 0-40
  int softPotPosition_F = map(softPotADC_F, 0, 1023, 0, GRAPH_LENGTH);
  int softPotPosition_R = map(softPotADC_R, 0, 1023, 0, GRAPH_LENGTH);
  vout_F = (ffsdata_F * 5.0) / 1023.0;
  vout_F = vout_F * cf_F;

  vout_R = (ffsdata_R * 5.0) / 1023.0;
  vout_R = vout_R * cf_R;

//Extreme values case //if((vout_F || vout_R) > thrshigh )
//
//{
//
//   delay(1000);
//    digitalWrite(Pstart,HIGH);
//    digitalWrite(Pservo,HIGH);
//Serial.println("Extreme threshold reached. Motor power cut-off ");
//Serial1.println('b');//end of motion transmission to computer vision module
//
//
//  
//}
//
////if((vout_F < thrslow) and (vout_R< thrslow ))
//
//{
//
//   delay(1000);
//    digitalWrite(Pstart,HIGH);
//    digitalWrite(Pservo,HIGH);
//Serial.println("Low threshold reached. Stronger profile required ");
//Serial1.println('b');//end of motion transmission to computer vision module
//
//
//  
//}

//end of extreme values case

//Datastreamer recording of parameters including both rear and front FlexiForce A201 force sensors, rear and front SoftPot SPL02001031ST position and force sensors and the Grove 6 axis Accelerometer & Gyroscope module

  //Serial.println("FlexiForce Front Sensor Force:   ");
  Serial.print(",");
  Serial.print(vout_F, 3);
  Serial.print("N");
  Serial.print(",");
  Serial.print(softPotPosition_F);
  Serial.print(',');
  //Serial.println("FlexiForce Rear Sensor Force:   ");
  Serial.print(vout_R, 6);
  Serial.print("N");
  Serial.print(",");
  Serial.print(softPotPosition_R);

// //Accelerometer
////    Serial.print("Ani lo");
////    Serial.print("\nAccelerometer:\n");
////    Serial.print(" X1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatAccelX(), 4);
//         Serial.print(" Y1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatAccelY(), 4);
//    //Serial.print(" Z1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatAccelZ(), 4);
//    //Gyroscope
//   // Serial.print("\nGyroscope:\n");
//        Serial.print(" X1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatGyroX(), 4);
//    //Serial.print(" Y1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatGyroY(), 4);
//    //Serial.print(" Z1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatGyroZ(), 4);
       Serial.println();

 

}
 

 
    
    delay(1000);
    digitalWrite(Pstart,HIGH);
    digitalWrite(Pservo,HIGH);

Serial1.println('b');//end of motion transmission to computer vision module



delay(7000); 
 digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
      // Makes 200 pulses for making one full cycle rotation
      for(int i=0; i<=cycle/4; i++){
     
         
         digitalWrite(stepPin,HIGH); 
         delayMicroseconds(speed); 
         digitalWrite(stepPin,LOW);
         delayMicroseconds(speed);
        
      }
delay(1000);

for(int i=0; i<=(cycle/11)+100; i++){
     
         
         digitalWrite(stepPin,HIGH); 
         delayMicroseconds(speed); 
         digitalWrite(stepPin,LOW);
         delayMicroseconds(speed);
        
      }



                             //   }
    
    
}


 if (val1== '2')

  {
  


    digitalWrite(Psele1,LOW);
    digitalWrite(Psele2,LOW);
    digitalWrite(Psele3,HIGH);
    digitalWrite(Psele4,HIGH);
    
    digitalWrite(Pservo,LOW);


    delay(500);
    digitalWrite(Pstart,LOW);

//Sensor Reading

for(int j=0;j<25000;j++)
{
ffsdata_F = analogRead(ffs_F);
  ffsdata_R = analogRead(ffs_R);
  // Read in the soft pot's ADC value
  softPotADC_F = analogRead(SOFT_POT_PIN_F);
  softPotADC_R = analogRead(SOFT_POT_PIN_R);
  // Map the 0-1023 value to 0-40
  int softPotPosition_F = map(softPotADC_F, 0, 1023, 0, GRAPH_LENGTH);
  int softPotPosition_R = map(softPotADC_R, 0, 1023, 0, GRAPH_LENGTH);
  vout_F = (ffsdata_F * 5.0) / 1023.0;
  vout_F = vout_F * cf_F;

  vout_R = (ffsdata_R * 5.0) / 1023.0;
  vout_R = vout_R * cf_R;

  //Extreme values case //if((vout_F || vout_R) > thrshigh )
//
//{
//
//   delay(1000);
//    digitalWrite(Pstart,HIGH);
//    digitalWrite(Pservo,HIGH);
//Serial.println("Extreme threshold reached. Motor power cut-off ");
//Serial1.println('b');//end of motion transmission to computer vision module
//
//
//  
//}
//
////if((vout_F < thrslow) and (vout_R< thrslow ))
//
//{
//
//   delay(1000);
//    digitalWrite(Pstart,HIGH);
//    digitalWrite(Pservo,HIGH);
//Serial.println("Low threshold reached. Stronger profile required ");
//Serial1.println('b');//end of motion transmission to computer vision module
//
//
//  
//}

//end of extreme values case


  //Serial.println("FlexiForce Front Sensor Force:   ");
  Serial.print(",");
  Serial.print(vout_F, 3);
  Serial.print("N");
  Serial.print(",");
  Serial.print(softPotPosition_F);
  Serial.print(',');
  //Serial.println("FlexiForce Rear Sensor Force:   ");
  Serial.print(vout_R, 3);
  Serial.print("N");
  Serial.print(",");
  Serial.print(softPotPosition_R);

// //Accelerometer
////    Serial.print("Ani lo");
////    Serial.print("\nAccelerometer:\n");
////    Serial.print(" X1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatAccelX(), 4);
//         Serial.print(" Y1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatAccelY(), 4);
//    //Serial.print(" Z1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatAccelZ(), 4);
//    //Gyroscope
//   // Serial.print("\nGyroscope:\n");
//        Serial.print(" X1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatGyroX(), 4);
//    //Serial.print(" Y1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatGyroY(), 4);
//    //Serial.print(" Z1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatGyroZ(), 4);
       Serial.println();

 

}
    
    delay(1000);
    digitalWrite(Pstart,HIGH);
    digitalWrite(Pservo,HIGH);

 Serial1.println('b');

delay(7000); 
 digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
      // Makes 200 pulses for making one full cycle rotation
      for(int i=0; i<=cycle/4; i++){
     
         
         digitalWrite(stepPin,HIGH); 
         delayMicroseconds(speed); 
         digitalWrite(stepPin,LOW);
         delayMicroseconds(speed);
        
      }
delay(1000);

for(int i=0; i<=(cycle/11)+100; i++){
     
         
         digitalWrite(stepPin,HIGH); 
         delayMicroseconds(speed); 
         digitalWrite(stepPin,LOW);
         delayMicroseconds(speed);
        
      }
    
    
}

 if (val1== '3')

  {
    
    
    digitalWrite(Psele1,HIGH);
    digitalWrite(Psele2,LOW);
    digitalWrite(Psele3,LOW);
    digitalWrite(Psele4,HIGH);
    
    digitalWrite(Pservo,LOW);
    
    
    delay(500);
    digitalWrite(Pstart,LOW);

//Sensor Reading

for(int j=0;j<250;j++)
{
ffsdata_F = analogRead(ffs_F);
  ffsdata_R = analogRead(ffs_R);
  // Read in the soft pot's ADC value
  softPotADC_F = analogRead(SOFT_POT_PIN_F);
  softPotADC_R = analogRead(SOFT_POT_PIN_R);
  // Map the 0-1023 value to 0-40
  int softPotPosition_F = map(softPotADC_F, 0, 1023, 0, GRAPH_LENGTH);
  int softPotPosition_R = map(softPotADC_R, 0, 1023, 0, GRAPH_LENGTH);
  vout_F = (ffsdata_F * 5.0) / 1023.0;
  vout_F = vout_F * cf_F;

  vout_R = (ffsdata_R * 5.0) / 1023.0;
  vout_R = vout_R * cf_R;

//Extreme values case //if((vout_F || vout_R) > thrshigh )
//
//{
//
//   delay(1000);
//    digitalWrite(Pstart,HIGH);
//    digitalWrite(Pservo,HIGH);
//Serial.println("Extreme threshold reached. Motor power cut-off ");
//Serial1.println('b');//end of motion transmission to computer vision module
//
//
//  
//}
//
////if((vout_F < thrslow) and (vout_R< thrslow ))
//
//{
//
//   delay(1000);
//    digitalWrite(Pstart,HIGH);
//    digitalWrite(Pservo,HIGH);
//Serial.println("Low threshold reached. Stronger profile required ");
//Serial1.println('b');//end of motion transmission to computer vision module
//
//
//  
//}

//end of extreme values case

  //Serial.println("FlexiForce Front Sensor Force:   ");
  Serial.print(",");
  Serial.print(vout_F, 3);
  Serial.print("N");
  Serial.print(",");
  Serial.print(softPotPosition_F);
  Serial.print(',');
  //Serial.println("FlexiForce Rear Sensor Force:   ");
  Serial.print(vout_R, 3);
  Serial.print("N");
  Serial.print(",");
  Serial.print(softPotPosition_R);

// //Accelerometer
////    Serial.print("Ani lo");
////    Serial.print("\nAccelerometer:\n");
////    Serial.print(" X1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatAccelX(), 4);
//         Serial.print(" Y1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatAccelY(), 4);
//    //Serial.print(" Z1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatAccelZ(), 4);
//    //Gyroscope
//   // Serial.print("\nGyroscope:\n");
//        Serial.print(" X1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatGyroX(), 4);
//    //Serial.print(" Y1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatGyroY(), 4);
//    //Serial.print(" Z1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatGyroZ(), 4);
       Serial.println();

 

}
    
    delay(1000);
    digitalWrite(Pstart,HIGH);
    digitalWrite(Pservo,HIGH);

   Serial1.println('b');

delay(7000); 
 digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
      // Makes 200 pulses for making one full cycle rotation
      for(int i=0; i<=cycle/4; i++){
     
         
         digitalWrite(stepPin,HIGH); 
         delayMicroseconds(speed); 
         digitalWrite(stepPin,LOW);
         delayMicroseconds(speed);
        
      }
delay(1000);

for(int i=0; i<=(cycle/11)+100; i++){
     
         
         digitalWrite(stepPin,HIGH); 
         delayMicroseconds(speed); 
         digitalWrite(stepPin,LOW);
         delayMicroseconds(speed);
        
      }
    
}

if (val1==4)

  {
//    digitalWrite(Psele1,HIGH);
//    digitalWrite(Psele2,HIGH);
//    digitalWrite(Psele3,HIGH);
//    digitalWrite(Psele4,LOW);


    
   
    digitalWrite(Psele1,LOW);
    digitalWrite(Psele2,HIGH);
    digitalWrite(Psele3,HIGH);
    digitalWrite(Psele4,LOW);
    
     digitalWrite(Pservo,LOW);
    delay(500);
    digitalWrite(Pstart,LOW);


   //Sensor Reading

for(int j=0;j<250;j++)
{
ffsdata_F = analogRead(ffs_F);
  ffsdata_R = analogRead(ffs_R);
  // Read in the soft pot's ADC value
  softPotADC_F = analogRead(SOFT_POT_PIN_F);
  softPotADC_R = analogRead(SOFT_POT_PIN_R);
  // Map the 0-1023 value to 0-40
  int softPotPosition_F = map(softPotADC_F, 0, 1023, 0, GRAPH_LENGTH);
  int softPotPosition_R = map(softPotADC_R, 0, 1023, 0, GRAPH_LENGTH);
  vout_F = (ffsdata_F * 5.0) / 1023.0;
  vout_F = vout_F * cf_F;

  vout_R = (ffsdata_R * 5.0) / 1023.0;
  vout_R = vout_R * cf_R;

//Extreme values case //if((vout_F || vout_R) > thrshigh )
//
//{
//
//   delay(1000);
//    digitalWrite(Pstart,HIGH);
//    digitalWrite(Pservo,HIGH);
//Serial.println("Extreme threshold reached. Motor power cut-off ");
//Serial1.println('b');//end of motion transmission to computer vision module
//
//
//  
//}
//
////if((vout_F < thrslow) and (vout_R< thrslow ))
//
//{
//
//   delay(1000);
//    digitalWrite(Pstart,HIGH);
//    digitalWrite(Pservo,HIGH);
//Serial.println("Low threshold reached. Stronger profile required ");
//Serial1.println('b');//end of motion transmission to computer vision module
//
//
//  
//}

//end of extreme values case

  //Serial.println("FlexiForce Front Sensor Force:   ");
  Serial.print(",");
  Serial.print(vout_F, 3);
  Serial.print("N");
  Serial.print(",");
  Serial.print(softPotPosition_F);
  Serial.print(',');
  //Serial.println("FlexiForce Rear Sensor Force:   ");
  Serial.print(vout_R, 3);
  Serial.print("N");
  Serial.print(",");
  Serial.print(softPotPosition_R);

// //Accelerometer
////    Serial.print("Ani lo");
////    Serial.print("\nAccelerometer:\n");
////    Serial.print(" X1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatAccelX(), 4);
//         Serial.print(" Y1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatAccelY(), 4);
//    //Serial.print(" Z1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatAccelZ(), 4);
//    //Gyroscope
//   // Serial.print("\nGyroscope:\n");
//        Serial.print(" X1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatGyroX(), 4);
//    //Serial.print(" Y1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatGyroY(), 4);
//    //Serial.print(" Z1 = ");
        Serial.print(',');
        Serial.print(myIMU.readFloatGyroZ(), 4);
       Serial.println();

 

}

    delay(1000);
    digitalWrite(Pstart,HIGH);
    digitalWrite(Pservo,HIGH);

Serial1.println('b');

delay(7000); 
 digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
      // Makes 200 pulses for making one full cycle rotation
      for(int i=0; i<=cycle/4; i++){
     
         
         digitalWrite(stepPin,HIGH); 
         delayMicroseconds(speed); 
         digitalWrite(stepPin,LOW);
         delayMicroseconds(speed);
        
      }
delay(1000);

for(int i=0; i<=(cycle/11)+100; i++){
     
         
         digitalWrite(stepPin,HIGH); 
         delayMicroseconds(speed); 
         digitalWrite(stepPin,LOW);
         delayMicroseconds(speed);
        
      }

  }
















  
}
  
