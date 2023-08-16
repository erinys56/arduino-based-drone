#include <Wire.h>
#include "Adafruit_VL53L1X.h"
#include "SparkFunMPL3115A2.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
/////////////mpu variable//////////////
float gyroRoll,gyroPitch, gyroYaw;
float gyroRollCal,gyroPitchCal, gyroYawCal;
float Groll,Gpitch,Gyaw;
float tempGroll,tempGpitch,tempGyaw;
float accX,accY,accZ;
float gyroX,gyroZ,gyroY;
float Aroll,Apitch;
float tempAroll,tempApitch;
float roll,pitch,yaw;
float timeprev,time,elapsedtime;
float test;
float accXCal,accYCal,accZCal;
float zaxis;
float gyroRollAngle,gyroPitchAngle;
float rolltemp, pitchtemp;
float rollnew, pitchnew;
Adafruit_MPU6050 mpu;
//-----------------------------------------------//
/////////////NRF24L01 variable//////////////
RF24 radio(7, 8); // CE, CSN
struct Data_Package {
  float yaw;
  float roll = 0; // roll -30 ~ 30
  float pitch = 0; // pitch -30 ~ 30
  float throttle = 0;
  boolean button1 = false;
  boolean button2 = false;
  boolean button3 = false;
};
Data_Package data; //Create a variable with the above structure
struct Data_Package2 {
  float droneyaw=0;
  float dronepitch=0;
  float droneroll=0;
  float Altitude=0;
  float distance_from_ground = 0;
  float hover_distance_from_ground = 0;
};
Data_Package2 data2; //Create a variable with the above structure

const byte address[6] = "00001";
const byte address2[6] = "00011";

boolean button1;
boolean prevbutton1;
float ptimer, timer;
float battery;
boolean button2, prevbutton2 = false;
boolean button3, prevbutton3 = false;
//-----------------------------------------------//
/////////////VL53L1X variable//////////////
Adafruit_VL53L1X vl53;
float distance;
float distance_from_ground;
float hover_distance_from_ground;
//-----------------------------------------------//
/////////////MPL3115A2 variable//////////////
MPL3115A2 mpl;
float Altitude;
float AltitudeCal;
float prevAltitude;
int count2, maxcount2,prevmaxcount2;
float newAltitude, pastAltitude,Alt;
//-----------------------------------------------//
/////////////Motors variable//////////////
Servo FL;
Servo FR;
Servo BL;
Servo BR;
float throttle;
float throttleFL;
float throttleFR;
float throttleBR;
float throttleBL;
//-----------------------------------------------//
/////////////PID variable//////////////
float rollPID,pitchPID,yawPID, rollerror, rollprevious_error, pitcherror, pitchprevious_error,yawerror, yawprevious_error;
float rollpid_p=0;
float rollpid_i=0;
float rollpid_d=0;
float pitchpid_p=0;
float pitchpid_i=0;
float pitchpid_d=0;
float yawpid_p=0;
float yawpid_i=0;
float yawpid_d=0;
float altpid_p=0;
float altpid_i=0;
float altpid_d=0;
double pitchkp=2 ;//3
double pitchki=0.005;//0.01
double pitchkd=0.3;//0.35
double rollkp=1.7 ;//3
double rollki=0.005;//0.01
double rollkd=0.27;//0.35
double yawkp=3 ;//3
double yawki=0.005;//0.01
double yawkd=0.45;//0.35
double altkp=0.13 ;//3
double altki=0.0001;//0.01
double altkd=0.045;//0.35
float altprevious_error,alterror;
float altdesired;
float rolldesired_angle,pitchdesired_angle,yawdesired_angle;
float prevrolldesired_angle,prevpitchdesired_angle,prevyawdesired_angle;
float altPID;
int AltOn;
//-----------------------------------------------//
float currenttime;
///---------------------------------------------------------------------//////////////
///---------------------------------------------------------------------//////////////
///---------------------------------------------------------------------//////////////
///---------------------------------------------------------------------//////////////
///---------------------------------------------------------------------//////////////
void setup() {
  // put your setup code here, to run once:
  startmotor();
  Wire.begin();
  radio.begin();
  Serial.begin(115200);
  mpusetup();
  VL53L1Xsetup();
  MPL3115A2setup();


}

void loop() {
  // put your main code here, to run repeatedly:
  ReadWriteNRF24L01();
  readVL53L1X();
  readMPL3115A2();
  readmpu();
  PIDcalculate();
  writeMotor();


}
////---------------------------------------------------------------------//////////////
///---------------------------------------------------------------------//////////////
///---------------------------------------------------------------------//////////////
///---------------------------------------------------------------------//////////////





void ReadWriteNRF24L01(void){
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
  if (radio.available() ) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    radio.stopListening();
    radio.openWritingPipe(address2);
    radio.setPALevel(RF24_PA_MAX);
    radio.write(&data2,sizeof(Data_Package2));
   
    ptimer = timer;
  }
  data2.droneroll = roll;
  data2.dronepitch = pitch;
  data2.droneyaw = yaw;
  data2.distance_from_ground = distance_from_ground;
  data2.Altitude = Alt;
  //data2.hover_distance_from_ground = hover_distance_from_ground;
  battery = (analogRead(A2));
  battery = battery *(8.25/693.00);
  data2.hover_distance_from_ground = battery;


  throttle = data.throttle;

  prevrolldesired_angle = rolldesired_angle;
  prevpitchdesired_angle = pitchdesired_angle;
  prevyawdesired_angle = yawdesired_angle;
  rolldesired_angle = data.roll;
  pitchdesired_angle = data.pitch;
  yawdesired_angle = data.yaw;
   
  prevbutton1 = button1;
  button1 = data.button1;
  prevbutton2 = button2;
  button2 = data.button2;
  prevbutton3 = button3;
  button3 = data.button3;
  
  if(button1 == 1 && prevbutton1 == 0){
    hover_distance_from_ground = distance_from_ground;
  }
  if(button1 == 0){
    hover_distance_from_ground = 0;
  }
  timer = millis();
  if(timer-ptimer > 3000) throttle = 1000;

  if(button2 == 1 && prevbutton2 == 0){

  }
  if(button1 == 1 && button3 == 1 && distance_from_ground >= 0){
    if(distance_from_ground > 500){
      hover_distance_from_ground = hover_distance_from_ground - 200*elapsedtime;
    }
    if(distance_from_ground < 500){
      hover_distance_from_ground = hover_distance_from_ground - 150*elapsedtime;
    }
    if(distance_from_ground < 300){
      hover_distance_from_ground = hover_distance_from_ground - 100*elapsedtime;
    }
    if(distance_from_ground < 150){
      hover_distance_from_ground = hover_distance_from_ground - 50*elapsedtime;
    }
    if(distance_from_ground <20) {
        FR.writeMicroseconds(1000);
        FL.writeMicroseconds(1000);
        BR.writeMicroseconds(1000);
        BL.writeMicroseconds(1000);
        delay(1000);
      throttle = 1000;
    }
  }


  
}





void mpusetup(void){
  mpu.begin();
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);; // 5, 10, 21, 44, 94, 184, 260
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG); // 250deg/s = 131LSB, 500deg/s = 65.5LSB, 1000deg/s = 32.8LSB, 2000deg/s = 16.4LSB   0, 1, 2 ... order
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G); // 2g = 16384LSB, 4g = 8192LSB, 8g = 4096LSB, 16g = 2048LSB
  for(int i = 0; i<1000;i++){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);  
  gyroRollCal += g.gyro.y;
  gyroPitchCal += g.gyro.x;
  gyroYawCal += g.gyro.z;
  }
  gyroRollCal /= 1000;
  gyroPitchCal /= 1000;
  gyroYawCal /= 1000;
  
}

void readmpu(void){
  timeprev = time;
  time = millis();
  elapsedtime = (time-timeprev)/1000;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);  
  gyroRoll = (g.gyro.y-gyroRollCal)*(180/PI)*elapsedtime;
  gyroPitch = (g.gyro.x-gyroPitchCal)*(180/PI)*elapsedtime;
  gyroYaw = (g.gyro.z-gyroYawCal)*(180/PI)*elapsedtime;
  gyroRollAngle += gyroRoll;
  gyroPitchAngle += gyroPitch;

  

  rolltemp += gyroRoll;
  pitchtemp += 0-gyroPitch;
  Gyaw += gyroYaw;
  
  accX = (float)a.acceleration.y/8192;
  accY = (float)a.acceleration.x/8192;
  accZ = (float)a.acceleration.z/8192;
  Aroll = -atan(accY/sqrt(pow(accX,2)+pow(accZ,2)))*180/PI;
  Apitch = -atan(accX/sqrt(pow(accY,2)+pow(accZ,2)))*180/PI;
  zaxis = acos(cos(roll*PI/180)*cos(pitch*PI/180))*180/PI;

  rolltemp += pitchtemp * sin(gyroYaw * PI/180);
  pitchtemp -= rolltemp * sin(gyroYaw * PI/180);
  Serial.print(rolltemp);
  Serial.print("  ");
  Serial.println(pitchtemp);
  
/////////////////////////complementary filter///////////////////////////////////////
rolltemp = 0.999*(rolltemp) + 0.001*Aroll;
pitchtemp = 0.999*(pitchtemp) + 0.001*Apitch;
yaw = 0-Gyaw;
roll =  rolltemp;
pitch = pitchtemp;


/////////////////////////complementary filter///////////////////////////////////////
 
}

void VL53L1Xsetup(void){
  vl53.begin(0x29, &Wire);
  vl53.startRanging();
  vl53.setTimingBudget(15); // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
}
float count,prevdistance,saveddistance;
float newdistance, pastdistance;
void readVL53L1X(void){
  prevdistance = distance;
  distance = vl53.distance();
  if (prevdistance == distance){
    count ++;
  }
  if(prevdistance != distance){
    count = 0;
    newdistance = distance;
    pastdistance = prevdistance;
  }
  distance_from_ground = (pastdistance+((count/9)*(newdistance-pastdistance)))*cos(zaxis*PI/180);
}

void  MPL3115A2setup(void){
  mpl.begin();
  mpl.setModeAltimeter(); // Measure altitude above sea level in meters
  mpl.setOversampleRate(7); // Set Oversample to the recommended 128
  mpl.enableEventFlags(); // Enable all three pressure and temp event flags 
  for(int b = 0; b<1000 ; b++){
  AltitudeCal += mpl.readAltitude();
  }
  AltitudeCal = AltitudeCal/1000;
}


void readMPL3115A2(void){
  prevAltitude = Altitude;
  Altitude = mpl.readAltitude()-AltitudeCal;


  if (prevAltitude == Altitude){
    count2 ++;
  }
  if(prevdistance != distance){
    count2 = 0;
    newAltitude = Altitude;
    pastAltitude = prevAltitude;
  }
  if(count2>maxcount2){
    maxcount2 = count2;
    
  }
  Alt = (pastAltitude+((count/maxcount2)*(newAltitude-pastAltitude)));

}

void startmotor(){
  BL.attach(3);
  FL.attach(5);
  BR.attach(6);
  FR.attach(9);
 
  FL.writeMicroseconds(2000); 
  FR.writeMicroseconds(2000);
  BL.writeMicroseconds(2000);
  BR.writeMicroseconds(2000);
  delay(5000);
  FL.writeMicroseconds(1000); 
  FR.writeMicroseconds(1000);
  BL.writeMicroseconds(1000);
  BR.writeMicroseconds(1000);
  delay(5000);
}

void writeMotor(){
  throttleFR = throttle-rollPID-pitchPID+yawPID+altPID;
  throttleFL = throttle+rollPID-pitchPID-yawPID+altPID;
  throttleBR = throttle-rollPID+pitchPID-yawPID+altPID;
  throttleBL = throttle+rollPID+pitchPID+ yawPID+altPID;

if(throttleFR>2000) throttleFR = 2000;
if(throttleFL>2000) throttleFL = 2000;
if(throttleBR>2000) throttleBR = 2000;
if(throttleBL>2000) throttleBL = 2000;
if(throttleFR<1000) throttleFR = 1000;
if(throttleFL<1000) throttleFL = 1000;
if(throttleBR<1000) throttleBR = 1000;
if(throttleBL<1000) throttleBL = 1000;  

  FR.writeMicroseconds(throttleFR);
  FL.writeMicroseconds(throttleFL);
  BR.writeMicroseconds(throttleBR);
  BL.writeMicroseconds(throttleBL);
}

void PIDcalculate(){
rollprevious_error = rollerror; //Remember to store the previous error.
pitchprevious_error = pitcherror; //Remember to store the previous error.
yawprevious_error = yawerror; //Remember to store the previous error.
altprevious_error = alterror;
alterror = hover_distance_from_ground - distance_from_ground;
rollerror = rolldesired_angle - roll;
pitcherror = pitchdesired_angle - pitch;
yawerror = yawdesired_angle - yaw;



rollpid_p = rollkp*rollerror;
rollpid_i = rollpid_i+(rollki*rollerror);
rollpid_d = rollkd*((rollerror - rollprevious_error)/elapsedtime);
if(prevrolldesired_angle != rolldesired_angle) rollpid_d = 0;
rollPID = rollpid_p + rollpid_i + rollpid_d;

pitchpid_p = pitchkp*pitcherror;
pitchpid_i = pitchpid_i+(pitchki*pitcherror);
pitchpid_d = pitchkd*((pitcherror - pitchprevious_error)/elapsedtime);
if(prevpitchdesired_angle != pitchdesired_angle) pitchpid_d = 0;
pitchPID = pitchpid_p + pitchpid_i + pitchpid_d;

yawpid_p = yawkp*yawerror;
yawpid_i = yawpid_i+(yawki*yawerror);  
yawpid_d = yawkd*((yawerror - yawprevious_error)/elapsedtime);
if(prevyawdesired_angle != yawdesired_angle) yawpid_d = 0;
yawPID = yawpid_p  + yawpid_d + yawpid_i;

altpid_p = altkp*alterror;
altpid_i = altpid_i+(altki*alterror);
altpid_d = altkd*((alterror - altprevious_error)/elapsedtime);
if(button1 == 1 && prevbutton1 == 0){
  altpid_d = 0;
}
//if(abs(alterror) > abs(altprevious_error)) altpid_d = 0;
altPID = altpid_p + altpid_i + altpid_d;

if(data.button1 == false || distance_from_ground < 0){
  altpid_p = 0;
  altpid_i = 0;
  altpid_d = 0;
  altPID = 0;
}
if(throttle <1050){
rollpid_p = 0;
rollpid_i = 0;
rollpid_d = 0;
rollPID = 0;
pitchpid_p = 0;
pitchpid_i = 0;
pitchpid_d = 0;
pitchPID = 0;
yawpid_p = 0;
yawpid_i = 0;
yawpid_d = 0;
yawPID = 0;
altpid_p = 0;
altpid_i = 0;
altpid_d = 0;
altPID = 0;
}
}
