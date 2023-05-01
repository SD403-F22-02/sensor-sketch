#include "SensorFusion.h"
#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include "ArduinoBLE.h"

// IMU Variables
#define maxEntryLength 50
#define maxSessionLength 20000 //keep an eye on the amount of dynamic memory being used in the compiler message
LSM6DSO imu;
SF fusion;

float gx, gy, gz, ax, ay, az;
float pitch, roll, yaw;
float deltat;
int imu_data;

unsigned long sessionStartMillis, timeMillis;
bool recordingSession = false;
unsigned int sessionDataLength = 0;
// char *sessionData[maxEntryLength] = {};
struct dataEntry {
  unsigned int time_ms;
  float roll;
  float pitch;
  float yaw;
};
struct dataEntry sessionData[maxSessionLength];

// Bluetooth Variables
BLEService sensorService("580A");
BLEStringCharacteristic dataChar("580E", BLERead | BLEWrite | BLENotify, maxEntryLength);
BLEStringCharacteristic commandChar("580F", BLERead | BLEWrite | BLENotify, maxEntryLength);

void setup() {
  Serial.begin(115200);
  while(!Serial);

  // IMU Setup
  Wire.begin();
  delay(10);
  if(imu.begin())
    Serial.println(F("IMU Ready"));
  else
    Serial.println(F("Could not connect to IMU :("));

  if(imu.initialize(SOFT_INT_SETTINGS))
    Serial.println(F("Loaded IMU settings"));
  
  imu.setHighPerfAccel(true);
  imu.setHighPerfGyro(true);

  // Bluetooth Setup
  if(!BLE.begin()){
    Serial.println(F("Starting BLE module failed!"));
    while(1);
  }

  BLE.setLocalName("TheraTec Sensor");
  BLE.setAdvertisedService(sensorService);

  sensorService.addCharacteristic(commandChar);
  sensorService.addCharacteristic(dataChar);
  BLE.addService(sensorService);

  BLE.advertise();
  Serial.println(F("Bluetooth Ready!"));
  commandChar.setValue(F("Ready!"));
};

void loop() {
  BLEDevice central = BLE.central();
  // time = millis();

  if(central){
    Serial.print(F("Connected to central: "));
    Serial.println(central.address());
    while(central.connected()){
      sensorData();

      if(commandChar.written()){
        String command = commandChar.value();
        Serial.print(F("Recieved command: "));
        Serial.println(command);

        // uses the first char in command string
        switch(command[0]){
          case 's':
            // start recording data
            Serial.println(F("start recording session"));
            recordingSession = true;
            sessionStartMillis = millis();
            sessionDataLength = 0;
            commandChar.setValue(F("started session"));
            break;

          case 'x':
            // stop recording session
            Serial.println(F("stop recording session"));
            recordingSession = false;
            commandChar.setValue(F("stopped session"));
            break;
            
          case 'g':
            // send session data over dataChar
            Serial.println(F("send session data"));
            commandChar.setValue(F("Sending values"));
            sendSessionData();
            break;
        }
      }
    }
  }
}

String entryToString(struct dataEntry entry){
  String outString = "";
  int temp_roll = entry.roll * 100;
  int temp_pitch = entry.pitch * 100;
  int temp_yaw = entry.yaw * 100;
  
  outString += entry.time_ms;
  outString += ",";
  outString.concat(String(temp_roll/100) + "." + String(temp_roll%100)); outString += ",";
  outString.concat(String(temp_pitch/100) + "." + String(temp_pitch%100)); outString += ",";
  outString.concat(String(temp_yaw/100) + "." + String(temp_yaw%100));
  return outString;
}

void sendSessionData(){
  for(unsigned int i=0; i<sessionDataLength; i++){
    struct dataEntry entry = sessionData[i];
    // String entryToString = String(entry.time_ms) + "," + String(entry.roll) + "," + String(entry.pitch) + "," + String(entry.yaw);
    String entryString = entryToString(entry);


    Serial.println(entryString);
    dataChar.writeValue(entryString);
  }
  commandChar.setValue(F("done"));
}

// add to session data with time,roll,pitch,yaw
void sensorData(){

  imu_data = imu.listenDataReady();

  if(imu_data == ALL_DATA_READY){
    timeMillis = millis() - sessionStartMillis;
    gx = imu.readFloatGyroX() * DEG_TO_RAD;
    gy = imu.readFloatGyroY() * DEG_TO_RAD;
    gz = imu.readFloatGyroZ() * DEG_TO_RAD;
    ax = imu.readFloatAccelX();
    ay = imu.readFloatAccelY();
    az = imu.readFloatAccelZ();

    deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
    fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow

    pitch = fusion.getPitch();
    roll = fusion.getRoll();    //you could also use getRollRadians() ecc
    yaw = fusion.getYaw();

    struct dataEntry newEntry = {timeMillis, roll, pitch, yaw};
    if(recordingSession){
      sessionData[sessionDataLength] = newEntry;
      sessionDataLength += 1;
      Serial.print(F("Entry:")); Serial.print(sessionDataLength); Serial.print(",");

      // if(sessionDataLength >= maxSessionLength){
      //   commandChar.setValue(F("Max session length reached"));
      //   recordingSession = false;
      // }
    }
    
    Serial.print(F("Time:")); Serial.print(newEntry.time_ms); Serial.print(",");
    Serial.print(F("Roll:")); Serial.print(newEntry.roll); Serial.print(",");
    Serial.print(F("Pitch:")); Serial.print(newEntry.pitch); Serial.print(",");
    Serial.print(F("Yaw:")); Serial.print(newEntry.yaw); Serial.println();

    delay(50);
  }
}