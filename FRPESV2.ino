#include <Adafruit_LSM6DSO32.h>
#include <BMP280_DEV.h>      
#include <SPI.h>
#include <LoRa.h>    
#include <SD.h>  
#include <SoftwareSerial.h>
#include <TinyGPS++.h>         
#include <pico/multicore.h>    

//FLIGHT RECORDER AND PARACHUTE EJECTION SYSTEM V2 DEMO
//-----------------User Configuration---------------------
float mainParachuteAltitude = 200;
float maximumDescentVelocity = -50;
float accelerationTrigger = 40;
float velocityTrigger = 10;
bool debugMode = false;
bool parachuteTestMode = false;
//--------------------------------------------------------

//Zrób szyfrowanie
//Zrób zapisywanie danych na karcie SD (np. kiedy bmp wykrył start)
//Ustaw warunki wykrycia startu i opadania
//Przerób Measurements
//Przerób flight controller
//Przerób SD saving
//Przerób DataTransmission


//Digital pins
int led1 = 20;
int led2 = 21;
int buzzer = 22;
int drogueChuteIgniter = 26;
int mainChuteIgniter = 27;
SoftwareSerial gpsSerial(0,1);
const int sdCS = 9;

//BMP280
float temperature, pressure, altitude;
BMP280_DEV bmp280;  
float ground_Altitude;
bool bmp_setup = false;

//LSM6DSO32
Adafruit_LSM6DSO32 dso32;

//GPS
TinyGPSPlus gps;
float lattitude,longitude;

//SD
File myFile;

//Launch Detection
bool bmp280launch = false;
bool lsm6launch = false;
bool bmp280descent = false;
bool lsm6descent = false;

//Flight Data
float last_time;
float new_time;
bool launch_time_lock = false;
float launch_time;
float flight_time;
float my_time;
float Apogee;
float Velocity;
float final_Altitude;
float previous_Altitude;
float Acceleration;
float Rotation;
float roll;
float tiltY;
float tiltZ;
bool landed = false;
bool burnout = false;
float maxAcceleration;
float maxVelocity;
bool drougeDeployed = false;
bool mainDeployed = false;

int mode = 0;
//0 - Standby no go
//1 - Standby go for launch
//2 - Flight
//3 - Descent
//4 - Landing

//Configuration
bool igniters_go = false;
bool lsm6_go = false;
bool bmp280_go = false;

void setup(void) {
  Serial.begin(115200);
  Serial.println("Starting FRPES V2");
  

  //Led1
  pinMode(led1, OUTPUT);
  //Led2
  pinMode(led2, OUTPUT);  
  //Buzzer
  pinMode(buzzer, OUTPUT);
  //Drouge Chute Igniter
  pinMode(drogueChuteIgniter, OUTPUT);  
  //Main Chute Igniter
  pinMode(mainChuteIgniter, OUTPUT);
  igniters_go = true;
  

  //SD card initialization
  SPI1.setRX(12);
  SPI1.setTX(11);
  SPI1.setSCK(10);
  SPI1.setCS(9);
  Serial.print("Initializing SD card...");
  if (!SD.begin(sdCS, SPI1)) {
    Serial.println("SD initialization failed.");
    while (true);
  }
  myFile = SD.open("Flight.txt", FILE_WRITE);
  if (myFile) {
    myFile.println("Time,Mode,Flight_time,Apogee,Acceleration,Altitude,Velocity,Roll,tiltY,tiltZ");
    myFile.close();
  } else {
    Serial.println("Error occured while opening flight data file.");
  }
  Serial.println("SD configuration was successful.");


  //LoRa Initialization
  LoRa.setPins(17, 14, 13); 
  Serial.println("LoRa Sender Initialization");
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setCodingRate4(8);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.enableCrc();
  LoRa.setSpreadingFactor(12);
  LoRa.setTxPower(10); 
  Serial.println("LoRa configuration was successful.");
  //Special settings:
  //LoRa.setTxPower(20);
  //LoRa.setSpreadingFactor(7);
  //LoRa.setSignalBandwidth(125E3);

  
  //GPS Initialization
  gpsSerial.begin(9600);
  

  //LSM6DSO32 Initialization
  Serial.println("Initializing LSM6DSO32.");
  if (!dso32.begin_I2C(0x6a)) {
    while (1) {
      delay(10);
    }
  }
  Serial.println("LSM6DSO32 Found!");
  dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G);
  dso32.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  dso32.setAccelDataRate(LSM6DS_RATE_104_HZ);
  dso32.setGyroDataRate(LSM6DS_RATE_104_HZ);
  lsm6_go = true;
  Serial.println("LSM6DSO32 configuration was successful.");
  

  //BMP280 Initialization
  Serial.println("Initializing BMP280.");
  bmp280.begin(0x76);    
  bmp280.startNormalConversion();
  bmp280_go = true;
  Serial.println("BMP280 configuration was successful.");
  //Special settings:
  //bmp280.setPresOversampling(OVERSAMPLING_X4);  
  //bmp280.setTempOversampling(OVERSAMPLING_X1);  
  //bmp280.setIIRFilter(IIR_FILTER_4); 
  

  //Multicore setup
  multicore_launch_core1(DataTransmission);
}

void Debug(){
  if((millis()/1000)>=42){
    mode = 2;    
  }
  if((millis()/1000)>=52){
    mode = 3;    
  }
}

void loop() {
  Measurements();
  Controller();
  SDDataSaving(); 
  if(debugMode == true){
    Debug();    
  }   
  delay(20);
}

void Debug(){
  if(parachuteTestMode == true){
    delay(5000);
    DrougeParachuteDeployment();
    delay(5000);
    MainParachuteDeployment();
    delay(20000);    
  }
  
}

void Measurements(){ 
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  dso32.getEvent(&accel, &gyro, &temp);

  Acceleration = sqrt(pow(accel.acceleration.x, 2) + pow(accel.acceleration.y, 2) + pow(accel.acceleration.z, 2)); 
  
  if (bmp280.getMeasurements(temperature, pressure, altitude))  
  {
    if(bmp_setup == false){
      ground_Altitude = altitude;
      bmp_setup = true;      
    }
    //Altitude and Apogee
    final_Altitude = altitude - ground_Altitude;
    if(final_Altitude>Apogee){
      Apogee = final_Altitude;
    } 
    
    //Time
    my_time = millis() / 1000;
    new_time = (millis()/1000)-last_time;
    last_time = millis() / 1000;
    flight_time = my_time-launch_time; 
    
    //Velocity
    Velocity = (final_Altitude-previous_Altitude)/0.20;
    previous_Altitude = final_Altitude;
  
    //Roll and tilt
    roll = gyro.gyro.x;
    tiltY = accel.acceleration.y;
    tiltZ = accel.acceleration.z;  

    //GPS Measurements
    while (gpsSerial.available())
    {
      int data = gpsSerial.read();
      if (gps.encode(data))
      {
        lattitude = (gps.location.lat());
        longitude = (gps.location.lng());
      }
    }

    //Serial Display
    Serial.print("Time: ");
    Serial.print(my_time);
    Serial.print(" , ");
    Serial.print("Mode: ");
    Serial.print(mode);
    Serial.print(" , ");
    Serial.print("Flight Time: ");
    Serial.print(flight_time);
    Serial.print(" , ");    
    Serial.print("Apogee: ");
    Serial.print(Apogee);
    Serial.print(" , ");
    Serial.print("Acceleration: ");
    Serial.print(Acceleration);
    Serial.print(" , ");
    Serial.print("Altitude: ");
    Serial.print(final_Altitude);
    Serial.print(F("m , "));  
    Serial.print("Velocity: ");
    Serial.print(Velocity);
    Serial.print(" , ");
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print("r/s");
    Serial.print("    ,    ");
    Serial.print("TiltY: ");
    Serial.print(tiltY);    
    Serial.print(" , ");
    Serial.print("TiltZ: ");
    Serial.print(tiltZ);  
    Serial.print(" , ");
    Serial.print("lattitude: ");
    Serial.print(lattitude, 6);
    Serial.print(" , ");
    Serial.print("longitude: ");
    Serial.println(longitude, 6);
    Serial.println("---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
  }
}

void Encryption(){
  
}

void DataTransmission(){
  while (true) {
    //Transmission
    //H-altitude
    //A-acceleration
    //V-velocity
    //T-flight time
    //N-north position
    //W-west position
    //R-roll speed
    //M-mode  

    LoRa.beginPacket();
    LoRa.print("H");
    LoRa.print(final_Altitude);
    LoRa.print(" ");
    LoRa.endPacket();

    LoRa.beginPacket();
    LoRa.print("V");
    LoRa.print(Velocity, 1);
    LoRa.print(" ");
    LoRa.endPacket();
    
    LoRa.beginPacket();
    LoRa.print("A");
    LoRa.print(Acceleration, 1); 
    LoRa.print(" ");
    LoRa.endPacket();

    LoRa.beginPacket();    
    LoRa.print("T");
    LoRa.print(flight_time);
    LoRa.print(" ");
    LoRa.endPacket();

    LoRa.beginPacket(); 
    LoRa.print("N");
    LoRa.print(lattitude, 6);
    LoRa.print(" ");
    LoRa.endPacket();

    LoRa.beginPacket(); 
    LoRa.print("W");
    LoRa.print(longitude, 6);
    LoRa.print(" ");
    LoRa.endPacket();
    
    LoRa.beginPacket(); 
    LoRa.print("R");
    LoRa.print(roll, 1);
    LoRa.print(" ");
    LoRa.endPacket();

    LoRa.beginPacket(); 
    LoRa.print("M");
    LoRa.print(mode, 1);
    LoRa.print(" ");
    LoRa.endPacket();

    if(drougeDeployed==true){
      LoRa.beginPacket(); 
      LoRa.print("DP");
      LoRa.print("1");
      LoRa.print(" ");
      LoRa.endPacket();      
    }
    else{
      LoRa.beginPacket(); 
      LoRa.print("DP");
      LoRa.print("0");
      LoRa.print(" ");
      LoRa.endPacket();     
    }

    if(mainDeployed==true){
      LoRa.beginPacket(); 
      LoRa.print("MP");
      LoRa.print("1");
      LoRa.print(" ");
      LoRa.endPacket();      
    }
    else{
      LoRa.beginPacket(); 
      LoRa.print("MP");
      LoRa.print("0");
      LoRa.print(" ");
      LoRa.endPacket();     
    }
    delay(2000);
  }
}

void SDDataSaving(){
  myFile = SD.open("Flight.txt", FILE_WRITE);
  if (myFile) {
    myFile.print(my_time);
    myFile.print(",");
    myFile.print(mode);
    myFile.print(",");
    myFile.print(flight_time);
    myFile.print(",");
    myFile.print(Apogee);
    myFile.print(",");
    myFile.print(Acceleration);
    myFile.print(",");
    myFile.print(final_Altitude);
    myFile.print(",");
    myFile.print(Velocity);
    myFile.print(",");
    myFile.print(roll);
    myFile.print(",");
    myFile.print(tiltY);
    myFile.print(",");
    myFile.println(tiltZ);
    myFile.close();
  } else {
    Serial.println("Error opening Flight.txt");
  }
}

void Controller(){
  switch(mode){
    //Mode 0
    case 0:
      if(igniters_go == true && lsm6_go == true && bmp280_go == true){
        tone(buzzer, 600);
        delay(300);
        noTone(buzzer);
        delay(300);
        tone(buzzer, 600);
        delay(300);
        noTone(buzzer);
        delay(300);
        tone(buzzer, 600);
        delay(300);
        noTone(buzzer);
      
        Serial.println("Go for launch.");         
        mode = 1;
      } 
      break;
    //Mode 1
    case 1:
      if(bmp280launch == false){
        if(Velocity >= velocityTrigger){
          Serial.println("BMP280 DETECTED LAUNCH");     
          digitalWrite(led1, HIGH);
          bmp280launch = true;
        } 
      }  
      if(Acceleration >= accelerationTrigger){
        Serial.println("LAUNCH DETECTED");
        digitalWrite(led1, HIGH);
        tone(buzzer, 500);
        mode = 2;
      }
      break;
    //Mode 2
    case 2:
      if(launch_time_lock==false){
        launch_time = millis()/1000;        
        launch_time_lock = true;
      }
      if(Acceleration <= 3){
        if(burnout == false){
          Serial.println("Engine burnout."); 
          burnout = true;
        }    
      }
      if(bmp280descent == false){
        if(Velocity <= -3){
          Serial.println("BMP280 DETECTED DESCENT");     
          bmp280descent = true;
        }      
      }
      if(lsm6descent == false){        
        if(Acceleration <= 1){
          Serial.println("LSM6DSO32 DETECTED DESCENT");     
          lsm6descent = true;
        }       
      }
      if(mainDeployed == false){
        if(Velocity <= 10 && Acceleration <= 1){
          Serial.println("DESCENT DETECTED");
          DrougeParachuteDeployment();
          digitalWrite(led2, HIGH);
          mode = 3;           
        }     
      }  
      break;
    //Mode 3
    case 3:
      if(Velocity<=0.5 && Velocity>=-0.5){
        if(landed == false){
          Serial.println("Rocket has landed.");
          landed = true;
        }
      }
      else if(final_Altitude <= 150){
        if(mainDeployed == false){
          Serial.println("Deploying Main Parachute at 150 meters.");
          MainParachuteDeployment();
        }
      }
      else if(Velocity<=-maximumDescentVelocity){
        if(mainDeployed == false){
          Serial.println("Drouge parachute failure. Main parachute ejection charge fired.");
          MainParachuteDeployment();
        }      
      }
      break;
           
  }
}

void DrougeParachuteDeployment(){
  if(drougeDeployed==false){
    digitalWrite(drogueChuteIgniter, HIGH);
    delay(1000);
    digitalWrite(drogueChuteIgniter, LOW);    
    drougeDeployed = true;    
  }
}

void MainParachuteDeployment(){
  if(mainDeployed==false){
    digitalWrite(mainChuteIgniter, HIGH);
    delay(1000);
    digitalWrite(mainChuteIgniter, LOW);
    mainDeployed = true;    
  }
}

void InitializationFailure(){
  
}