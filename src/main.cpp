#include <Arduino.h>
// System libraries
#include "SPI.h"
#include "Wire.h"
// utility
#include "i2cscan.h"

/* -------------------------- Peripheral libraries -------------------------- */

// PIO servo outputs
#include "Servo.h"
// Optical Flow Sensors
#include "Bitcraze_PMW3901.h"
// Laser altimeter 
#include <SparkFun_VL6180X.h>
// IMU
#include "MPU6050_light.h"
// Camera mast linear actuator
#include "Mast.h"

/* -------------------------------- Settings -------------------------------- */
#define I2C_CLK 100000
#define PRINT_FREQ 100

#define ALTIMETER_ADDR 0x29

#define MAST_FREQ 100
#define MAST_DURATION 5
#define MAST_REVERSE false


/* --------------------------------- Pinout --------------------------------- */
// Don't change these
const int servoPins[]={10,11,12,13,14,15};

const int lFlowCS=5;
const int rFlowCS=28;

const int STEP=18;
const int DIR=19;
const int EN=2;

/* -------------------------- Object instantiation -------------------------- */
Servo servos[6];
// 6-axis IMU to measure tilt and approximate heading
MPU6050 imu(Wire);
// laser altimeter in centre of vehicle, measure ride height and scale optical flow values
VL6180x altimeter(ALTIMETER_ADDR);
// Two optical flow sensors either side of the vehicle, tracks position and heading by measuring the difference
Bitcraze_PMW3901 lFlow(lFlowCS);
Bitcraze_PMW3901 rFlow(rFlowCS);
// Camera mast
Mast mast(STEP,DIR,EN,MAST_REVERSE,MAST_FREQ);

void setup() {
  /* --------------------------------- Init IO -------------------------------- */
  // Sensor I2C
  Wire.setSCL(17);
  Wire.setSDA(16);
  Wire.setClock(I2C_CLK);
  Wire.begin();

  // Secondary I2C
  Wire1.setSCL(7);
  Wire1.setSCL(6);
  Wire1.setClock(I2C_CLK);
  Wire1.begin();

  // SPI comms with optical flow sensors
  SPI.setSCK(2);
  SPI.setTX(3);
  SPI.setRX(4);

  // Serial comms with Pi
  Serial1.setPinout(0,1);
  Serial1.begin(115200);

  // Serial comms with motor
  Serial2.setTX(9);
  Serial2.setRX(9);

  /* --------------------------------- Scan IO -------------------------------- */

  // send scan over usb
  i2cscan(&Wire,Serial);
	i2cscan(&Wire1,Serial);
  // also send over uart to pi
  i2cscan(&Wire,Serial1);
	i2cscan(&Wire1,Serial1);

  /* -------------------------- Init attached devices ------------------------- */
  // Servos
  for (int i=0; i<6;i++) {
    servos[i].attach(servoPins[i]);
  }
  // 6 axis IMU
  imu.begin();
  imu.calcOffsets(true,true);
  // Laser Altimeter
  altimeter.VL6180xDefautSettings();

}

/* ------------------------- Optical flow variables ------------------------- */

// delta values from each sensor
int16_t dlX,drX,dlY,drY;
// integrated values from each sensor
int32_t lX,lY,rX,rY;
// combined position calculated from each
int32_t X,Y;

float heading=0;

/* ------------------------------ IMU variables ----------------------------- */

float temperature;
float accX,accY,accZ; // acceleration
float gyrX,gyrY,gyrZ; // gyro values
float angX,angY,angZ; // estimated angle (pitch,roll,yaw)


void loop() {
  /* -------------------------------- Read IMU -------------------------------- */

  imu.update();

  temperature=imu.getTemp();

  accX=imu.getAccX();
  accY=imu.getAccY();
  accZ=imu.getAccZ();

  gyrX=imu.getGyroX();
  gyrY=imu.getGyroY();
  gyrZ=imu.getGyroZ();
  
  angX=imu.getAngleX();
  angY=imu.getAngleY();
  angZ=imu.getAngleZ();

  /* -------------------------- Read Laser Altimeter -------------------------- */

	int16_t altitude = altimeter.getDistance();

  /* ---------------------------- Read Optical Flow --------------------------- */

	lFlow.enableFrameBuffer();
	lFlow.readMotionCount(&dlX, &dlY);
  rFlow.enableFrameBuffer();
  rFlow.readMotionCount(&drX, &drY);
  // integrate left side
  lX+=dlX;
  lY+=dlY;
  // integrate right side
  rX+=drX;
  rY+=drY;

  //! Now need to calculate X, Y and heading from these, ignoring IMU values
  // X and Y should be independent of heading

}
