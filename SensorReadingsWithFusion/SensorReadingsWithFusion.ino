#include "SensorFusion.h"
#include "SparkFunLSM6DSO.h"
#include "Wire.h"
//#include "SPI.h"

LSM6DSO imu; //Default constructor is I2C, addr 0x6B
SF fusion;

float gx, gy, gz, ax, ay, az;
float pitch, roll, yaw;
float deltat;
int imu_data; 

void setup() {


  Serial.begin(115200);
  delay(500); 
  
  Wire.begin();
  delay(10);
  if( imu.begin() )
    Serial.println("Ready.");
  else { 
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  if( imu.initialize(SOFT_INT_SETTINGS) )
    Serial.println("Loaded Settings.");

  imu.setHighPerfAccel(true);
  imu.setHighPerfGyro(true);
}


void loop(){
  imu_data = imu.listenDataReady();

  if(imu_data == ALL_DATA_READY){
    gx = imu.readFloatGyroX() * DEG_TO_RAD;
    gy = imu.readFloatGyroY() * DEG_TO_RAD;
    gz = imu.readFloatGyroZ() * DEG_TO_RAD;
    ax = imu.readFloatAccelX();
    ay = imu.readFloatAccelY();
    az = imu.readFloatAccelZ();

    deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
    //choose only one of these two:
    fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow

    pitch = fusion.getPitch();
    roll = fusion.getRoll();    //you could also use getRollRadians() ecc
    yaw = fusion.getYaw();

    Serial.print("Pitch:"); Serial.print(pitch); Serial.print(",");
    Serial.print("Roll:"); Serial.print(roll); Serial.print(",");
    Serial.print("Yaw:"); Serial.print(yaw); Serial.println();

    // Serial.print("gX:");Serial.print(gx);Serial.print(",gY:");Serial.print(gy);Serial.print(",gZ:");Serial.println(gz);
    // Serial.print("aX:");Serial.print(ax);Serial.print(",aY:");Serial.print(ay);Serial.print(",aZ:");Serial.println(az);
  }



  // //Get all parameters
  // Serial.print("\nAccelerometer:\n");
  // Serial.print(" X = ");
  // Serial.println(myIMU.readFloatAccelX(), 3);
  // Serial.print(" Y = ");
  // Serial.println(myIMU.readFloatAccelY(), 3);
  // Serial.print(" Z = ");
  // Serial.println(myIMU.readFloatAccelZ(), 3);

  // Serial.print("\nGyroscope:\n");
  // Serial.print(" X = ");
  // Serial.println(myIMU.readFloatGyroX(), 3);
  // Serial.print(" Y = ");
  // Serial.println(myIMU.readFloatGyroY(), 3);
  // Serial.print(" Z = ");
  // Serial.println(myIMU.readFloatGyroZ(), 3);

  // Serial.print("\nThermometer:\n");
  // Serial.print(" Degrees F = ");
  // Serial.println(myIMU.readTempF(), 3);
  
  delay(50);
}
