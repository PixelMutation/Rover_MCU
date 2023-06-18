#include <Arduino.h> // using the arduino-pico core https://arduino-pico.readthedocs.io/en/latest/
// System libraries
#include "SPI.h"
#include "Wire.h"
// Utility libraries
#include "i2cscan.h"
#include "Ewma.h"

/* -------------------------- Peripheral libraries -------------------------- */

// PIO servo outputs			https://arduino-pico.readthedocs.io/en/latest/servo.html 
#include "Servo.h"
// Optical Flow Sensors			https://github.com/bitcraze/Bitcraze_PMW3901 
#include "Bitcraze_PMW3901.h"
// Laser altimeter 				https://github.com/adafruit/Adafruit_VL53L0X 
#include "Adafruit_VL53L0X.h" 
// 6 axis IMU 					https://github.com/rfetick/MPU6050_light 
#include "MPU6050_light.h" 
// Current and Voltage sensor 	https://github.com/RobTillaart/INA226 
#include "INA226.h" 
// Camera mast linear actuator 	(custom stepper driver using pico pwm)
#include "Mast.h"

/* -------------------------------- Settings -------------------------------- */

#define OPTICAL_FLOW_SEPARATION_MM 200

#define I2C_CLK 100000
#define SER_BAUD 115200
#define PLOT_DELAY 50 // time between printing values in ms

#define ALTIMETER_ADDR 0x29
#define INA_ADDR 0x40

#define MAST_FREQ 100
#define MAST_DURATION 5
#define MAST_REVERSE false

#define INA_FILTERING 0.5

#define SHUNT_RESISTANCE 0.01
#define MAX_CURRENT 8

#define BATT_CELLS 3 // number of cells in LiPo, used for finding batterj percentage
#define BATT_CAPACITY 5000 // capacity in mAH
#define BATT_ENERGY (BATT_CAPACITY / 1000 * BATT_CELLS * 3.7) // capacity in Wh

HardwareSerial & debug=Serial; // set whether USB or hardware serial used for debug
HardwareSerial & plotter=Serial; // set whether USB or hardware serial used for plotting etc.
HardwareSerial & piSer=Serial1; // serial to communicate with Pi
HardwareSerial & motorSer=Serial2; // serial to communicate with motor driver

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
Adafruit_VL53L0X altimeter;
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
// teleplot plotters https://github.com/nesnes/teleplot 
template <typename T>
void plot(String name, T val, bool autoplot=false) {
	plotter.print(">");
	plotter.print(name);
	plotter.print(":");
	plotter.print(val);
	if (!autoplot)
		plotter.print("|np");
	plotter.println();
}
template <typename T>
void plotXY(String name, T valX, T valY, bool autoplot=false) {
	plotter.print(">");
	plotter.print(name);
	plotter.print(":");
	plotter.print(valX);
	plotter.print(":");
	plotter.print(valY);
	plotter.print("|xy");
	if (!autoplot)
		plotter.print(",np");
	plotter.println();
}
void cube3D(String name, float pitch, float roll, float yaw, float z=0,float x=0, float y=0) {
	plotter.print(">3D|");
	plotter.print(name);
	plotter.print(":S:cube");
	plotter.printf(":P:%i:%i:%i",(int)x,(int)y,(int)z);
	plotter.printf(":R:%.1f:%.1f:%.1f",radians(yaw),radians(pitch),radians(roll));
	plotter.print(":W:2:H:2:D:2:C:red");
	plotter.println("");
}
// ascii image from optical flow frame
char asciiart(int k){ //converter magic? Higher value shunts more right in char array 
	static char foo[] = "WX86*3I>!;~:,`. ";
	return foo[k>>4]; //return shunted from array value character
}
// battery percent estimation
static inline uint8_t asigmoidal(uint16_t voltage, uint16_t minVoltage, uint16_t maxVoltage) {
	uint8_t result = 101 - (101 / pow(1 + pow(1.33 * (voltage - minVoltage)/(maxVoltage - minVoltage) ,4.5), 3));
	return result >= 100 ? 100 : result;
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
	Wire1.setSDA(6);
	Wire1.setClock(I2C_CLK);
	Wire1.begin();

	// SPI comms with optical flow sensors
	SPI.setSCK(2);
	SPI.setTX(3);
	SPI.setRX(4);

	// USB serial for debugging
	Serial.begin(SER_BAUD);
	delay(3000); // allow time to connect before begin

	// Serial comms with Pi
	Serial1.setPinout(0,1);
	Serial1.begin(SER_BAUD);

	// Serial comms with motor, mostlj just used as passthrough for the Pi
	Serial2.setPinout(8,9);
	Serial2.begin(SER_BAUD);

	/* --------------------------------- Scan IO -------------------------------- */

	// output I2C scan
	i2cscan(&Wire,debug);
	// i2cscan(&Wire1,debug);

	/* -------------------------- Init attached devices ------------------------- */
	
	// Servos
	for (int i=0; i<6;i++) {
		servos[i].attach(servoPins[i]);
	}
	// Optical Flow
	if (lFlow.begin()) {
		lFlowInit=true;
		// lFlow.enableFrameBuffer();
	} else
		debug.println("Failed to init left PMW3901 optical flow sensor");
	if (lFlow.begin()) {
		rFlowInit=true;
		// rFlow.enableFrameBuffer();
	} else
		debug.println("Failed to init right PMW3901 optical flow sensor");

	// 6 axis IMU
	if (!imu.begin()) {
		imu.calcOffsets(true,true);
		imuInit=true;
	} else
		debug.println("Failed to init MPU6050 IMU");
		
	// Laser Altimeter
	if (altimeter.begin()) {
		altInit=true;
	} else
		debug.println("Failed to init VL6180x laser altimeter");

	// INA226 current / voltage sensor
	if (INA.begin()) {
		INA.setMaxCurrentShunt(MAX_CURRENT, SHUNT_RESISTANCE);
		inaInit=true;
	} else
		debug.println("Failed to init INA226 current/voltage sensor");
		

}

/* ---------------------------- General variables --------------------------- */

unsigned long prevPlot=0; // time of previous plot, ms
unsigned long prevFlowFrame=0; // time of previous optical flow frame display, ms
unsigned long prevMeasure=0; // time of previous measurement, us
float dt=0; // time between measurements, s

/* ------------------------------ INA variables ----------------------------- */

float Vbus=0; // battery voltage, V
float Vshunt=0; // voltage drop across shunt, mV
Ewma current(INA_FILTERING); // current draw, A
Ewma power(INA_FILTERING); // power draw, mW
double energy=0; // estimated energy usage, Wh
uint8_t battPercent=0; // percentage estimated from bus voltage, inaccurate under load
float energyPercent=0; // percentage energy used this session

/* --------------------------- Altimeter variables -------------------------- */

uint8_t height=100; // in mm
// float lux; // ambient brightness

/* ------------------------------ IMU variables ----------------------------- */

float temperature=0;
float accX=0,accY=0,accZ=0; // linear acceleration
float gyri=0,gyrj=0,gyrZ=0; // angular acceleration
float angX=0,angY=0,angZ=0; // estimated euler angle in deg (pitch,roll,yaw)

/* ------------------------- Optical flow variables ------------------------- */

float flowScaleFactor=1;
// delta values from each sensor in mm
int16_t ldi=0,rdi=0; // sideways delta
int16_t ldj=0,rdj=0; // forwards delta
// velocity values per sensor, mm/s
float vli=0,vri=0; // sideways
float vlj=0,vrj=0; // forwards
// integrated values from each sensor
int32_t li=0,lj=0,ri=0,rj=0;
// combined coordinates
float dX=0,dY=0; // change in displacement
double X=0,Y=0; // coordinates from start point
// velocity in mm/s
float vX=0,vY=0; 
// calculated heading
float heading=0;
// scalar distance and speed
double distance=0,speed=0;
// distance between sensors
const int separation_mm = OPTICAL_FLOW_SEPARATION_MM;
// frame buffers
char lFrame[35*35];
char rFrame[35*35];

void loop() {
	dt=(micros()-prevMeasure)/1e6f;
	prevMeasure=micros();

	/* ------------------------ Motor Serial Passthrough ------------------------ */

	// Motor driver ignores irrelevant commands so just send all commands from the Pi
	if (piSer.available())
		motorSer.write(piSer.read());
	// Send all telemetrj to the Pi
	if (motorSer.available()) 
		piSer.write(motorSer.read());

	/* ---------------------- Read current, voltage, power ---------------------- */

	if (inaInit) {
		Vbus = INA.getBusVoltage();
		Vshunt = INA.getShuntVoltage_mV();
		current.filter(INA.getCurrent_mA());
		power.filter(INA.getPower_mW());
		energy+=(power.output/1e3)*(dt/3600);
		battPercent= asigmoidal(Vbus,3.27*BATT_CELLS,4.2*BATT_CELLS);
		energyPercent = (energy/BATT_ENERGY)*100;
	}

	/* -------------------------------- Read IMU -------------------------------- */

	if (imuInit) {
		imu.update();

		temperature=imu.getTemp();

		accX=imu.getAccX();
		accY=imu.getAccY();
		accZ=imu.getAccZ();

		gyri=imu.getGyroX();
		gyrj=imu.getGyroY();
		gyrZ=imu.getGyroZ();

		angX=imu.getAngleX();
		angY=imu.getAngleY();
		angZ=imu.getAngleZ();
	}
	/* -------------------------- Read Laser Altimeter -------------------------- */

	if (altInit) {
		height = altimeter.readRange();
		// lux=altimeter.readLux(VL6180X_ALS_GAIN_1);
	}

	/* ---------------------------- Read Optical Flow --------------------------- */

	// i and j are the displacements measured from each sensor, where i is sideways (right is +ve) and j is forwards
	// e.g. rj is the total forward displacement of the right hand sensor
	// whilst ldi is the change in sideways displacement of the left hand sensor

	// X and Y are the total displacement of the vehicle from its starting point within a coordinate grid where X is sideways and Y is forwards


	if (lFlowInit && rFlowInit) {
		// read raw deltas
		lFlow.readMotionCount(&ldi, &ldj);
		rFlow.readMotionCount(&rdi, &rdj);

		// scale to mm
		ldi*=(int16_t)height*flowScaleFactor;
		ldj*=(int16_t)height*flowScaleFactor;
		rdi*=(int16_t)height*flowScaleFactor;
		rdj*=(int16_t)height*flowScaleFactor;

		// integrate deltas to get total displacement per sensor
		li+=ldi;
		lj+=ldj;
		ri+=rdi;
		rj+=rdj;

		//! Now need to calculate X, Y and heading from these
		// ignore IMU values and for now, ignore altimeter. assume delta values are in mm
		// X and Y should be independent of heading
		// as such, di of both *should* be the same, probably just take the average? 

		//X=?
		//Y=?
		//heading=?
		


		// calc speed in i and j per sensor
		vli=ldi/dt;
		vlj=ldj/dt;
		vri=rdi/dt;
		vrj=rdj/dt;

		// calc speed in x and y
		vX=dX/dt;
		vY=dY/dt;
		
		// calc scalar speed and distance
		double prevDist=distance;
		distance+=sqrtf(powf(dX,2)+powf(dY,2));
		speed=(distance-prevDist)/dt;

}

	/* ------------------------------ Print values ------------------------------ */
	
	if (millis()-prevPlot>PLOT_DELAY) {
		prevPlot=millis();
		//INA
		plot("Vbus_V"   ,Vbus   );
		plot("Vshunt_mV" ,Vshunt );
		plot("current_mA",current);
		plot("power_mW"  ,power  );
		plot("battery_%"  ,battPercent);
		plot("energy_Wh"  ,energy);
		plot("energyUsed_%"  ,energyPercent);

		// Flow
		plot("distance_mm",distance);
		plot("speed_mm/s",speed);
		plot("heading_deg",heading);
		plotXY("position_mm",X,Y);
		plotXY("lDelta",ldi,ldj);
		plotXY("rDelta",rdi,rdj);
		plotXY("velocity_mm/s",vX,vY);
		plotXY("lVelocity_mm/s",vri,vrj);
		plotXY("rVelocity_mm/s",vli,vlj);
		

		// IMU
		
		float roll,pitch,yaw;
		roll=angY;
		pitch=angX;
		yaw=angZ;
		cube3D("rover",roll,pitch,yaw);
		plot("yaw        ",yaw);
		plot("pitch"      ,pitch);
		plot("roll"       ,roll);

		plot("temp_degC",temperature);

		// Altimeter
		plot("height_mm",height);
	}
	// Plot the optical flow images in ascii, side by side
	if (millis()-prevFlowFrame>1000) {
		if (false && lFlowInit && rFlowInit) {
			// rFlow.enableFrameBuffer();
			// lFlow.enableFrameBuffer();
			// rFlow.readFrameBuffer(rFrame);
			// lFlow.readFrameBuffer(lFrame);
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
