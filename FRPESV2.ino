#include <Adafruit_LSM6DSO32.h>
#include <BMP280_DEV.h>      
#include <SPI.h>
#include <LoRa.h>    
#include <SD.h>  
#include <SoftwareSerial.h>
#include <TinyGPS++.h>         
#include <pico/multicore.h>    

//---------------------------
//THIS IS NOT FINISHED CODE
//---------------------------

//FLIGHT RECORDER AND PARACHUTE EJECTION SYSTEM V2 DEMO
//-----------------Configuration---------------------
float mainParachuteAltitude = 200;
float maximumDescentVelocity = -50;
float accelerationTrigger = 40;
bool debugMode = false;
bool enableSerialDisplay = true;
//---------------------------------------------------

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
int fileNumber = 1;

//Flight Data
//Altitude and velocity calculations
float vel_d_time2;
float vel_d_time1;
float Apogee;
float Velocity = 0;
float final_Altitude;
float previous_Altitude;
float maxVelocity;
//Time calculations
bool launch_time_lock = false;
float launch_time;
float flight_time;
float total_time;
//Acceleration and gyroscope calculations
float Acceleration;
float Rotation;
float roll;
float tiltY;
float tiltZ;
float maxAcceleration;
//Others
bool landed = false;
bool burnout = false;
bool drougeDeployed = false;
bool mainDeployed = false;
bool transmissionLock = false;

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
    InitializationFailure();
    while (true);
  }
  while(SD.exists("Flight" + String(fileNumber) + ".csv")){
    fileNumber++;    
  }
  myFile = SD.open("Flight" + String(fileNumber) + ".csv", FILE_WRITE);
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
  bmp280.setTimeStandby(TIME_STANDBY_500MS);
  bmp280_go = true;
  Serial.println("BMP280 configuration was successful.");
  //Special settings:
  //bmp280.setPresOversampling(OVERSAMPLING_X4);  
  //bmp280.setTempOversampling(OVERSAMPLING_X1);  
  //bmp280.setIIRFilter(IIR_FILTER_4); 

  //Multicore setup
  multicore_launch_core1(DataTransmission);
}

void loop() {
  if(debugMode == true){
    Debug();    
  }
  Measurements();    
  Controller();   
  delay(20);
}

void Debug(){
  delay(5000);
  DrougeParachuteDeployment();
  delay(5000);
  MainParachuteDeployment();
  delay(20000);    
}

void Measurements(){ 
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  dso32.getEvent(&accel, &gyro, &temp);

  //Time data
  float timeMillis = millis();
  total_time = (float)timeMillis / 1000;
  flight_time = ((float)timeMillis / 1000)-launch_time; 
  
  //Acceleration Data
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
    
    //Velocity
    float valueMillis = millis();
    vel_d_time1 = (float)valueMillis / 1000;
    Velocity = (final_Altitude-previous_Altitude)/(vel_d_time1-vel_d_time2);
    valueMillis = millis();
    vel_d_time2 = (float)valueMillis / 1000;
    previous_Altitude = final_Altitude;
  }

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
  if(enableSerialDisplay == true){
    Serial.print("Time: ");
    Serial.print(total_time);
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

  myFile = SD.open("Flight" + String(fileNumber) + ".csv", FILE_WRITE);
  if (myFile) {
    myFile.print(total_time, 2);
    myFile.print(",");
    myFile.print(mode);
    myFile.print(",");
    myFile.print(flight_time, 2);
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
    myFile.print(tiltZ); 
    if(drougeDeployed==true){ 
      myFile.print(",1"); 
    }
    else{
      myFile.print(",0"); 
    }

    if(mainDeployed==true){
      myFile.println(",1");     
    }
    else{
      myFile.println(",0");    
    }
    myFile.close();

  } else {
    Serial.println("Error opening flight file");
  }
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
    if(transmissionLock==false){
      transmissionLock = true;
      LoRa.beginPacket();
      LoRa.print(final_Altitude);
      LoRa.print(",");
      LoRa.print(Velocity, 1);
      LoRa.print(",");
      LoRa.print(Acceleration, 1); 
      LoRa.print(",");
      LoRa.print(flight_time);
      LoRa.print(",");
      LoRa.print(lattitude, 6);
      LoRa.print(",");
      LoRa.print(longitude, 6);
      LoRa.print(",");
      LoRa.print(roll, 1);
      LoRa.print(",");
      LoRa.print(mode, 1);
      LoRa.print(",");
      if(drougeDeployed==true){ 
        LoRa.print("1");
        LoRa.print(",");    
      }
      else{
        LoRa.print("0");
        LoRa.print(",");  
      }

      if(mainDeployed==true){
        LoRa.println("1");
        LoRa.endPacket();      
      }
      else{
        LoRa.println("0");
        LoRa.endPacket();     
      }
      transmissionLock = false;
    }
    delay(4000);
  }
}

void Controller(){
  switch(mode){
    //Mode 0
    case 0:
      if(igniters_go == true && lsm6_go == true && bmp280_go == true){
        //Startup melody
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
      else{
        InitializationFailure();
      }
      break;
    //Mode 1
    case 1:  
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
        launch_time = (float)millis()/1000;        
        launch_time_lock = true;
      }
      if(Acceleration <= 3){
        if(burnout == false){
          Serial.println("Engine burnout."); 
          burnout = true;
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
      if(Velocity<=1 && Velocity>=-1 && final_Altitude<30){
        if(landed == false){
          Serial.println("Rocket has landed.");
          landed = true;
        }
      }
      else if(final_Altitude <= mainParachuteAltitude){
        if(mainDeployed == false){
          Serial.println("Deploying Main Parachute");
          MainParachuteDeployment();
        }
      }
      else if(Velocity<=maximumDescentVelocity){
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
    Serial.println("Deploying Drogue Parachute.");
    drougeDeployed = true;    
  }
}

void MainParachuteDeployment(){
  if(mainDeployed==false){
    digitalWrite(mainChuteIgniter, HIGH);
    delay(1000);
    digitalWrite(mainChuteIgniter, LOW);
    Serial.println("Deploying Main Parachute.");
    mainDeployed = true;    
  }
}

void InitializationFailure(){
  digitalWrite(led2, HIGH);
  Serial.println("Initialization failure. Check modules and restart FRPES V2.");    
}