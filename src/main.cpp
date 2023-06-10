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
#include "Adafruit_VL6180X.h" // #include <SparkFun_VL6180X.h>
// IMU
#include "MPU6050_light.h"
// Current and Voltage sensor
#include "INA226.h"
// Camera mast linear actuator
#include "Mast.h"

/* -------------------------------- Settings -------------------------------- */

#define I2C_CLK 100000
#define SER_BAUD 115200
#define PLOT_DELAY 20 // time between printing values in ms

#define ALTIMETER_ADDR 0x29
#define INA_ADDR 0x40

#define MAST_FREQ 100
#define MAST_DURATION 5
#define MAST_REVERSE false

HardwareSerial & debug=Serial; // set whether USB or hardware serial used for debug
HardwareSerial & plotter=Serial; // set whether USB or hardware serial used for plotting etc.

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
bool imuInit=false;
// laser altimeter in centre of vehicle, measure ride height and scale optical flow values
Adafruit_VL6180X altimeter;
bool altInit=false;
// Power sensor
INA226 INA(INA_ADDR);
bool inaInit=false;
// Two optical flow sensors either side of the vehicle, tracks position and heading by measuring the difference
Bitcraze_PMW3901 lFlow(lFlowCS);
bool lFlowInit=false;
Bitcraze_PMW3901 rFlow(rFlowCS);
bool rFlowInit=true;
// Camera mast
Mast mast(STEP,DIR,EN,MAST_REVERSE,MAST_FREQ);


/* -------------------------------- Functions ------------------------------- */

template <typename T>
void plot(String name, T val) {
  plotter.print(">");
  plotter.print(name);
  plotter.print(":");
  plotter.println(val);
}
char asciiart(int k){ //converter magic? Higher value shunts more right in char array 
  static char foo[] = "WX86*3I>!;~:,`. ";
  return foo[k>>4]; //return shunted from array value character
}

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

  // USB serial for debugging
  Serial.begin(SER_BAUD);

  // Serial comms with Pi
  Serial1.setPinout(0,1);
  Serial1.begin(SER_BAUD);

  // Serial comms with motor
  Serial2.setPinout(8,9);
  Serial1.begin(SER_BAUD);

  /* --------------------------------- Scan IO -------------------------------- */

  // output I2C scan
  i2cscan(&Wire,debug);
	i2cscan(&Wire1,debug);

  /* -------------------------- Init attached devices ------------------------- */
  
  // Servos
  for (int i=0; i<6;i++) {
    servos[i].attach(servoPins[i]);
  }

  if (lFlow.begin()) {
    lFlowInit=true;
    lFlow.enableFrameBuffer();
  } else
    debug.print("Failed to init left PMW3901 optical flow sensor");
  if (lFlow.begin()) {
    rFlowInit=true;
    rFlow.enableFrameBuffer();
  } else
    debug.print("Failed to init right PMW3901 optical flow sensor");

  // 6 axis IMU
  if (!imu.begin()) {
    imu.calcOffsets(true,true);
    imuInit=true;
  } else
    debug.print("Failed to init MPU6050 IMU");
    
  // Laser Altimeter
  if (altimeter.begin()) {
    altInit=true;
  } else
    debug.print("Failed to init VL6180x laser altimeter");

  // INA226 current / voltage sensor
  if (INA.begin()) {
    INA.setMaxCurrentShunt(20, 0.002);
    inaInit=true;
  } else
    debug.print("Failed to init INA226 current/voltage sensor");
    

}

/* ------------------------------ INA variables ----------------------------- */

float Vbus,Vshunt,current,power;

/* --------------------------- Altimeter variables -------------------------- */

uint8_t altitude; // in mm
float lux; // ambient brightness

/* ------------------------------ IMU variables ----------------------------- */

float temperature;
float accX,accY,accZ; // acceleration
float gyrX,gyrY,gyrZ; // gyro values
float angX,angY,angZ; // estimated angle (pitch,roll,yaw)

/* ------------------------- Optical flow variables ------------------------- */

// delta values from each sensor
int16_t dlX,drX,dlY,drY;
// integrated values from each sensor
int32_t lX,lY,rX,rY;
// combined coordinates
int32_t X,Y;
// calculated heading
float heading=0;
// distance between sensors
const int separation_mm = 200;
// frame buffers
char lFrame[35*35];
char rFrame[35*35];

void loop() {
  /* ---------------------- Read current, voltage, power ---------------------- */

  if (inaInit) {
    Vbus = INA.getBusVoltage();
    Vshunt = INA.getShuntVoltage_mV();
    current = INA.getCurrent_mA();
    power = INA.getPower_mW();
  }

  /* -------------------------------- Read IMU -------------------------------- */

  if (imuInit) {
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
  }
  /* -------------------------- Read Laser Altimeter -------------------------- */

  if (altInit) {
	  altitude = altimeter.readRange();
    lux=altimeter.readLux(VL6180X_ALS_GAIN_1);
  }

  /* ---------------------------- Read Optical Flow --------------------------- */

	if (lFlowInit && rFlowInit) {
    lFlow.readMotionCount(&dlX, &dlY);
    rFlow.readMotionCount(&drX, &drY);

    // integrate left side
    lX+=dlX;
    lY+=dlY;
    // integrate right side
    rX+=drX;
    rY+=drY;

    //! Now need to calculate X, Y and heading from these
    // ignore IMU values and for now, ignore altimeter. assume delta values are in mm
    // X and Y should be independent of heading
    // assume X is sideways and Y is forwards
    // as such, dX of both *should* be the same, probably just take the average? 











}

  /* ------------------------------ Print values ------------------------------ */
  
  static unsigned long prevPlot=0;
  if (millis()-prevPlot>PLOT_DELAY) {
    prevPlot=millis();
    //INA
    plot("Vbus"   ,Vbus   );
    plot("Vshunt" ,Vshunt );
    plot("Current",current);
    plot("Power"  ,power  );
    // Flow
    plot("Heading",heading);
    plot("X"      ,X      );
    plot("Y"      ,Y      );
    plot("dlX"    ,dlX    );
    plot("dlY"    ,dlY    );
    plot("drX"    ,drX    );
    plot("drY"    ,drY    );
    // IMU
    plot("yaw        ",angZ);
    plot("pitch"      ,angY);
    plot("roll"       ,angX);
    // Altimeter
    plot("altitude",altitude);
    plot("lux",lux);
  }
  // Plot the optical flow images in ascii, side by side
  static unsigned long prevFlowFrame=0;
  if (millis()-prevFlowFrame>1000) {
    if (lFlowInit && rFlowInit) {
      rFlow.readFrameBuffer(rFrame);
      lFlow.readFrameBuffer(lFrame);
      debug.println("Optical flow frames");
      int i,j,k;
      for(i=1, k=1; i<36; i++){ //i is Y pixel pos
        for(j=1; j<36; j++){  //j is X pixel pos
          debug.print(asciiart(lFrame[(i*j)-1]));
          debug.print(' ');
        }
        debug.print("   ");
        for(j=0; j<35; j++){  //j is X pixel pos
          debug.print(asciiart(rFrame[(i*j)-1]));
          debug.print(' ');
        }
        debug.println();
      }
      debug.println();
    }
  }
}
