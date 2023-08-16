#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
RF24 radio(7, 8); // CE, CSN
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
const byte address[6] = "00001";
const byte address2[6] = "00011";
// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {
  float yaw = 0;
  float roll = 0;
  float pitch = 0;
  float throttle = 0;
  boolean button1 = false;
  boolean button2 = false;
  boolean button3 = false;
};

struct Data_Package2 {
  float droneyaw;
  float dronepitch;
  float droneroll;
  float Altitude;
  float distance_from_ground;
  float hover_distance_from_ground;
};
Data_Package2 data2; //Create a variable with the above structure

float yaw;
float syaw = 0;
float tempx,x,y,z=0;
double test;
Data_Package data; // Create a variable with the above structure
int pb = 0;
int pbprev = 0;
int AltOn = 0;
float check;
float altadj;
int count=0;
int c = 0;
float stickroll;
float stickpitch;
float stickyaw;
float stickthrottle;
float throttle;
float pitch;
float roll;
float potenthrottle;
float b1,b2,b3,prevb1,prevb2,prevb3;
boolean button1,button2,button3;

void setup() {
  Serial.begin(115200); 
  radio.begin();
  bno.begin();
  
  bno.setExtCrystalUse(true);
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(25,28);
  display.println("Initializing!");
  display.display();
  delay(500);
  display.clearDisplay();
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  pinMode(A6,INPUT);
  pinMode(A7,INPUT);
}

void loop() {
   //Send the whole data from the structure to the receiver
  
  sensors_event_t event;
  bno.getEvent(&event);
  tempx = event.orientation.x;
  pitch = 0-event.orientation.z; // pitch
  roll = event.orientation.y; // roll
  x = map(tempx,0,360,-180,180); // yaw


  stickyaw += 2*(map(analogRead(A1),0,1023,5,-5)-1);
  stickthrottle += map(analogRead(A0),0,1023,20,-20)-1;
  potenthrottle = map(analogRead(A2),0,1023,1000,2000);
  throttle = stickthrottle + potenthrottle;
  if(potenthrottle == 1000) stickthrottle = 0;
  prevb1 = b1;
  prevb2 = b2;
  prevb3 = b3;
  b1 = analogRead(A3);
  b2 = analogRead(A6);
  b3 = analogRead(A7);

  if(button1 == false && prevb1 != 0 && b1 == 0){
    button1 = true;
  }else if(button1 == true && prevb1 != 0 && b1 == 0){
    button1 = false;
  }

  if(button2 == false && prevb2 != 0 && b2 == 0){
    button2 = true;
  }else if(button2 == true && prevb2 != 0 && b2 == 0){
    button2 = false;
  }

  if(button3 == false && prevb3 != 0 && b3 == 0 && b2 != 0){
    button3 = true;
  }else if(button3 == true && prevb3 != 0 && b3 == 0 && b2 != 0){
    button3 = false;
  }
  data.button1 = button1;
  data.button2 = button2;
  data.button3 = button3;
Serial.print(button1);
Serial.print(" ");
Serial.print(button2);
Serial.print(" ");
Serial.println(data2.distance_from_ground);


data.roll = roll;
data.pitch = pitch;
data.throttle = throttle + stickthrottle;
data.yaw = stickyaw;





//data.altadj = altadj;

  

  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();
  radio.write(&data, sizeof(Data_Package));
  radio.openReadingPipe(0, address2);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
  if (radio.available() ) {
  radio.read(&data2, sizeof(Data_Package2)); // Read the whole data and store it into the 'data' structure
  }



  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("        Y    P    R ");
  display.setCursor(0,16);
  display.println("Remote:");
  display.setCursor(45,16);
  display.println(stickyaw,0);
  display.setCursor(75,16);
  display.println(pitch,0);
  display.setCursor(105,16);
  display.println(roll,0);
  display.setCursor(0,25);
  display.println("Drone:");
  display.setCursor(45,25);
  display.println(data2.droneyaw,0);
  display.setCursor(75,25);
  display.println(data2.dronepitch,0);
  display.setCursor(105,25);
  display.println(data2.droneroll,0);
  display.setCursor(0,0);
  display.println("T:");
  display.setCursor(11,0);
  display.println((map(throttle,1000,2000,0,100)));
  display.setCursor(25,0);
  display.println("%");
  //display Altitude//
  display.setCursor(0,40);
  display.println("A:");
  display.setCursor(12,40);
  display.println(data2.Altitude,1);
  if(data2.Altitude>10 || data2.Altitude<0) display.setCursor(38,40);
  else display.setCursor(33,40);
  display.println("m");
  //display distance//
  display.setCursor(48,40);
  display.println("D:");
  display.setCursor(60,40);
  display.println(data2.distance_from_ground/1000,2);
  display.setCursor(85,40);
  display.println("m");
  //display hovermode//
  display.setCursor(95,40);
  display.println("H:");
  display.setCursor(110,40);
  if(data.button1 == 0) display.println("OFF");
  else display.println("ON");


  
  display.setCursor(0,55);
  display.println("Hover at:");
  display.setCursor(65,55);
  display.println(data2.hover_distance_from_ground,2);
  

  display.display();

};
