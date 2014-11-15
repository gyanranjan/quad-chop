#include <HMC5883L.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>
#include <I2Cdev.h>
#include <Wire.h>
#include <ChibiOS_ARM.h>
// Redefine AVR Flash string macro as nop for ARM
#undef F
#define F(str) str

#define SERIAL_DEBUG 1

const uint8_t LED_PIN = 13;


typedef struct mpu_6050_loc_t{
	uint8_t mpuIntStatus;
	BinarySemaphore mpu6050sem;
	uint16_t packetSize;
	uint16_t fifoCount; 
	uint8_t fifoBuffer[64];
	uint32_t overflow_count;
	uint32_t int_count; 
	boolean dmpReady;
}mpu_6050_loc_t;
mpu_6050_loc_t  mpu_6050_data;
MPU6050 mpu(0x68);
HMC5883L mag;

typedef struct sensor_data_t {
	int16_t  ax;
	int16_t  ay; 
	int16_t  az;
	int16_t  gx;
	int16_t  gy;
	int16_t  gz;
	int16_t  mx;
	int16_t  my;
	int16_t  mz;
	float  mh; //heading
}  sensor_data_t;

sensor_data_t sense;



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
void dmpDataReady() {
    chBSemReset(&mpu_6050_data.mpu6050sem, 0);
    mpu_6050_data.int_count++;
}

// ================================================================
// ===               Setup   Sensors                  
// ================================================================
static uint32_t setup_mpu_6050 () {
	uint8_t devStatus; 
#ifdef SERIAL_DEBUG
	Serial.println(F("Initializing mpu 6050 and DMP"));
#endif
	mpu.initialize();
	devStatus = mpu.dmpInitialize();

#if 1  // supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
#endif
  
	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		mpu.setDMPEnabled(true);
		attachInterrupt(2, dmpDataReady, RISING);
		//mpu_6050_data.mpuIntStatus = mpu.getIntStatus();
		mpu_6050_data.dmpReady = true;
		mpu_6050_data.packetSize = mpu.dmpGetFIFOPacketSize();
		// get expected DMP packet size for later comparison
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		return -1;
	}
	chBSemInit(&mpu_6050_data.mpu6050sem, 1);
	return 0;
}


static uint32_t setup_mag () {
	Serial.println("Initializing I2C devices...");
    mag.initialize();
	return 0;
}

static uint32_t setup_sensors () {
	
#ifdef SERIAL_DEBUG
	byte error, address;
	int nDevices;
#endif  
	//setup wire
	Wire.begin();
	Wire.setClock(400000);
 #ifdef SERIAL_DEBUG
	Serial.println("Scanning...");
	
	nDevices = 0;
  for(address = 1; address < 127; address++ )  {
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0) {
			Serial.print("I2C device found at address 0x");
			if (address<16)
				Serial.print("0");
			Serial.print(address,HEX);
			Serial.println(" !");
			nDevices++;
		} else if (error==4) {
			Serial.print("Unknow error at address 0x");
			if (address<16)
				Serial.print("0");
			Serial.println(address,HEX);
		}
	}
	if (nDevices < 2) {
		Serial.print("No devoce found \n");
		return -1;
	}
	
 #endif
	
	if (setup_mpu_6050() != 0) {
		Serial.print("unable to initialise MPU");
	}
	
	if (setup_mag() != 0) {
		Serial.print("unable to initialise MPU");
	}  

	return 0;
}




// 64 byte stack beyond task switch and interrupt needs
static WORKING_AREA(waSensors, 64);
static msg_t thSensors(void *arg) {
	// orientation/motion vars
	Quaternion q;           // [w, x, y, z]         quaternion container
	//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
	//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
	//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float euler[3];         // [psi, theta, phi]    Euler angle container
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


  pinMode(LED_PIN, OUTPUT);

  while (1) {
    // Turn LED on.
    digitalWrite(LED_PIN, HIGH);
    chBSemWait(&mpu_6050_data.mpu6050sem);
    mag.getHeading(&sense.mx, &sense.my, &sense.mz);
    
    //calculate heading
    sense.mh = atan2(sense.my, sense.mx);
    //if(sense.mh < 0) 
		//sense.mh += 2 * M_PI;
		
	mpu_6050_data.mpuIntStatus = mpu.getIntStatus();
    mpu_6050_data.fifoCount = mpu.getFIFOCount();
    	
	
	 if ((mpu_6050_data.mpuIntStatus & 0x10) || mpu_6050_data.fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
        mpu_6050_data.overflow_count++;
    } else if (mpu_6050_data.mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
        while (mpu_6050_data.fifoCount < mpu_6050_data.packetSize) 
			mpu_6050_data.fifoCount = mpu.getFIFOCount();
			// read a packet from FIFO
			mpu.getFIFOBytes(mpu_6050_data.fifoBuffer, mpu_6050_data.packetSize);
			mpu_6050_data.fifoCount -= mpu_6050_data.packetSize;	
			
			mpu.dmpGetQuaternion(&q, mpu_6050_data.fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);
            Serial.print("\t");
            Serial.println(sense.mh *  180/M_PI);
            
	}
		
    // Turn LED off.
    digitalWrite(LED_PIN, LOW);
  }
  return 0;
}


void mainThread() ;

void setup() {
  Serial.begin(115200);
  // wait for USB Serial
  
  while (!Serial) {}
  
  if (setup_sensors() !=0 ){
  } 

  chBegin(mainThread);
  // chBegin never returns, main thread continues with mainThread()
  while(1) {}
}


//------------------------------------------------------------------------------
// main thread runs at NORMALPRIO
void mainThread() {

  // start blink thread
  chThdCreateStatic(waSensors, sizeof(waSensors),
                          NORMALPRIO + 2, thSensors, NULL);

  //// start print thread
  //chThdCreateStatic(waThread2, sizeof(waThread2),
                          //NORMALPRIO + 1, Thread2, NULL);

  //// start print thread
  //chThdCreateStatic(waThread3, sizeof(waThread3),
                          //NORMALPRIO + 3, Thread3, NULL);                          

  //// start print thread
  //chThdCreateStatic(waThread3, sizeof(waThread4),
                          //NORMALPRIO + 4, Thread4, NULL);  
  //// increment counter
  //while (1) {
    //// must insure increment is atomic in case of context switch for print
    //// should use mutex for longer critical sections
    //noInterrupts();
    //count++;
    //interrupts();
  //}
}
//------------------------------------------------------------------------------
void loop() {
 // not used
}
