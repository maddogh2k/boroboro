// laduje biblioteki
#include <ezButton.h>
#include <DFRobot_TCS3430.h>
#include "AS726X.h"
#include "Wire.h"
#include "SPI.h"
#include <DFRobot_I2C_Multiplexer.h>
#include "Adafruit_VL6180X.h"

//ustawienie guzikow
#define LOOP_STATE_STOPPED 0
#define LOOP_STATE_STARTED 1
ezButton program1(3);  // guzik jest podlaczony do pinu wyjscia 3, uruchamia program 1
int loopState = LOOP_STATE_STOPPED;

//ustawienie multipleksera

DFRobot_I2C_Multiplexer I2CMultiplexer(&Wire, 0x70);
Adafruit_VL6180X vl = Adafruit_VL6180X();
Adafruit_VL6180X v2 = Adafruit_VL6180X();

const unsigned long eventInterval = 500; // dlugosc przerwy
const unsigned long czasMigniecia = 100;
unsigned long previousTime = 0;
 // cos od millis()

uint16_t errorsAndWarnings = 0;

#define LED_R 10
#define LED_G 11
#define LED_B 12
#define zawor_1 13

AS726X sensor;
DFRobot_TCS3430 TCS3430;

void setup(void) {
  Serial.begin(115200);
  program1.setDebounceTime(50); //do guzika pierwszego
  I2CMultiplexer.begin();
  
  // wait for serial port to open on native usb devices
  while (!Serial) {
    delay(1);
  }

  pinMode(LED_R, OUTPUT); //Poszczególne piny sterujące diodą jako wyjścia
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(zawor_1, OUTPUT);

  digitalWrite(LED_R, LOW); //Dioda wyłączona
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  digitalWrite(zawor_1, LOW);

  
  I2CMultiplexer.selectPort(0);

unsigned long currentTime = millis();

  while (!TCS3430.begin()){ 
   if (currentTime - previousTime >= eventInterval) {
   Serial.println("Please check that the IIC device is properly connected");
    }
  }
  I2CMultiplexer.selectPort(1);
  sensor.begin();
 // sensor.enableBulb(); // dioda led miernika NIR
  previousTime = currentTime;

  I2CMultiplexer.selectPort(7);

  Serial.println("Adafruit VL6180x test!");
  if (! vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");

 I2CMultiplexer.selectPort(5);

  Serial.println("Adafruit 2 VL6180x test!");
  if (! v2.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
 Serial.println("Sensor found!");
}

void loop() {

  unsigned long currentTime = millis();

if (currentTime - previousTime >= eventInterval) {

program1.loop(); // MUST call the loop() function first

  if (program1.isPressed()) {
    if (loopState == LOOP_STATE_STOPPED)
      loopState = LOOP_STATE_STARTED;
    else if(loopState == LOOP_STATE_STARTED)
      loopState = LOOP_STATE_STOPPED;
      Serial.print("start"); 
  }

  if (loopState == LOOP_STATE_STARTED) {
 
  digitalWrite(LED_R, LOW); //Dioda wyłączona
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  digitalWrite(zawor_1, LOW);

I2CMultiplexer.selectPort(7); 
  
  uint8_t range = vl.readRange();  // mierzymy odleglosc od czujnika
  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
   // Serial.print("Range: "); Serial.println(range);
  }

  if (range <= 50) { // uruchamia dalsze pomiary gdy borowka jest blizej niz 5 cm

   I2CMultiplexer.selectPort(0);

  uint16_t XData = TCS3430.getXData();
  uint16_t YData = TCS3430.getYData();
  uint16_t ZData = TCS3430.getZData();
  uint16_t IR1Data = TCS3430.getIR1Data();
  uint16_t IR2Data = TCS3430.getIR2Data();
   String str = "" + String(XData) + " " + String(YData) + " " + String(ZData) + " " + String(IR1Data) + " " + String(IR2Data);
   Serial.println(str);

  float xy = (XData / YData) * 100;
  float xz = (XData / ZData) * 100;
  float yz = (YData / ZData) * 100;
  
 // String str = " xy : " + String(xy) + " xz : " + String(xz) + " yz : " + String(yz) + "    IR1 : " + String(IR1Data) + "    IR2 : " + String(IR2Data);
 // Serial.println(str);

  // Serial.print(" XY: ");
  // Serial.print(xy);
  // Serial.print(" XZ: ");
  // Serial.print(xz);
  // Serial.print(" YZ: ");
  // Serial.print(yz);
  // Serial.println(str);

  if (xy >= 0 && xz > 300 && yz > 200){
    
    digitalWrite (LED_R, HIGH);
    digitalWrite (LED_G, LOW);
    digitalWrite (LED_B, LOW);
    //delay (50);
   // digitalWrite (LED_R, LOW);
   // digitalWrite (LED_G, LOW);
   // digitalWrite (LED_B, LOW);
   }
 

 
 if (xy < 100 && xz >= 300 && yz >= 400){
  digitalWrite (LED_R, LOW);
  digitalWrite (LED_G, HIGH);
  digitalWrite (LED_B, LOW);
  digitalWrite (zawor_1, HIGH);
  //delay (50);
 // digitalWrite (LED_R, LOW);
 // digitalWrite (LED_G, LOW);
 // digitalWrite (LED_B, LOW);
 // digitalWrite (zawor_1, LOW);
  }


  if (xy <= 100 && xz <= 200 && yz <= 200){
  digitalWrite (LED_R, LOW);
  digitalWrite (LED_G, LOW);
  digitalWrite (LED_B, HIGH);
  //delay (50);
 // digitalWrite (LED_R, LOW);
 // digitalWrite (LED_G, LOW);
 // digitalWrite (LED_B, LOW);
  }



  I2CMultiplexer.selectPort(1);
    sensor.takeMeasurements();
  //Prints all measurements
  if (sensor.getVersion() == SENSORTYPE_AS7262)
  {
    //Visible readings
    Serial.print(" Reading: ;V[");
    Serial.print(sensor.getCalibratedViolet(), 2);
    Serial.print("] ;B[");
    Serial.print(sensor.getCalibratedBlue(), 2);
    Serial.print("] ;G[");
    Serial.print(sensor.getCalibratedGreen(), 2);
    Serial.print("] ;Y[");
    Serial.print(sensor.getCalibratedYellow(), 2);
    Serial.print("] ;O[");
    Serial.print(sensor.getCalibratedOrange(), 2);
    Serial.print("] ;R[");
    Serial.print(sensor.getCalibratedRed(), 2);
  }
  else if (sensor.getVersion() == SENSORTYPE_AS7263)
  {
    //Near IR readings
   // Serial.print(" R");
  //  Serial.print(sensor.getCalibratedR(), 0);
    //Serial.print(" S");
  //  Serial.print(" ");
  //  Serial.print(sensor.getCalibratedS(), 0);
    //Serial.print(" T");
  //  Serial.print(" ");
  //  Serial.print(sensor.getCalibratedT(), 0);
    //Serial.print(" U");
  //  Serial.print(" ");
  //  Serial.print(sensor.getCalibratedU(), 0);
    //Serial.print(" V");
  //  Serial.print(" ");
  //  Serial.print(sensor.getCalibratedV(), 0);
    //Serial.print(" W");
  //  Serial.print(" ");
  //  Serial.print(sensor.getCalibratedW(), 0);
  }
 
  Serial.println();}
  
//stad nie dziala, zawiesza sie i nie powtarza petli

I2CMultiplexer.selectPort(5); 
  
  uint8_t range2 = v2.readRange();  // mierzymy odleglosc od czujnika 2
  uint8_t status2 = v2.readRangeStatus();
  if (status2 == VL6180X_ERROR_NONE) {
   // Serial.print("Range dwa: "); Serial.println(range2);


  }
  
previousTime = currentTime;
}

}
}    
