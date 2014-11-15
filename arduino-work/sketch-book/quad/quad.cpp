#include <HMC5883L.h>

#include <MPU6050_6Axis_MotionApps20.h>

//#include <MPU6050_9Axis_MotionApps41.h>

#include <helper_3dmath.h>


#include <I2Cdev.h>
//#include "MPU6050_6Axis_MotionApps20.h"
//#include"MPU6050_9Axis_MotionApps41.h"
#include <Wire.h>
#include <ChibiOS_ARM.h>
// Redefine AVR Flash string macro as nop for ARM
#undef F
#define F(str) str

const uint8_t LED_PIN = 13;

//----------------------------------------------
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
BinarySemaphore mpu6050sem;
uint32_t        overflow_count;
uint32_t        int_count;  


// class default I2C address is 0x1E
// specific I2C addresses may be passed as a parameter here
// this device only supports one I2C address (0x1E)
HMC5883L mag;

int16_t mx, my, mz;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

//volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  //  mpuInterrupt = true;
    chBSemReset(&mpu6050sem, 0);
    int_count++;
}





volatile uint32_t count = 0;

//------------------------------------------------------------------------------
// thread 1 - high priority for blinking LED
// 64 byte stack beyond task switch and interrupt needs
static WORKING_AREA(waThread1, 64);

static msg_t Thread1(void *arg) {
  pinMode(LED_PIN, OUTPUT);

  // Flash led every 200 ms.
  while (1) {
    // Turn LED on.
    digitalWrite(LED_PIN, HIGH);

    // Sleep for 50 milliseconds.
    chThdSleepMilliseconds(50);

    // Turn LED off.
    digitalWrite(LED_PIN, LOW);

    // Sleep for 150 milliseconds.
    chThdSleepMilliseconds(150);
  }
  return 0;
}
//------------------------------------------------------------------------------
// thread 2 - print main thread count every second
// 100 byte stack beyond task switch and interrupt needs
static WORKING_AREA(waThread2, 100);

static msg_t Thread2(void *arg) {

  // print count every second
  while (1) {
    // Sleep for one second.
    chThdSleepMilliseconds(1000);
    //Serial.println(int_count);
#if 0
    // Print count for previous second.
    Serial.print("Count: ");
    Serial.print(count);

    // Print unused stack for threads.
    Serial.print(", Unused Stack: ");
    Serial.print(chUnusedStack(waThread1, sizeof(waThread1)));
    Serial.print(' ');
    Serial.print(chUnusedStack(waThread2, sizeof(waThread2)));
    Serial.print(' ');
    Serial.println(chUnusedHeapMain());
#endif
    // Zero count.
    count = 0;
  }
}
//------------------------------------------------------------------------------

static WORKING_AREA(waThread3, 100);

static msg_t Thread3(void *arg) {
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
   
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
   
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
      Serial.print("0");
      Serial.print(address,HEX);
      Serial.println(" !");
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknow error at address 0x");
      if (address<16)
      Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
  Serial.println("No I2C devices found\n");
  else
  Serial.println("done\n");
   
  delay(5000);
}
//------------------------------------------------------------------------------

MPU6050 mpu(0x68);

int16_t ax, ay, az;
int16_t gx, gy, gz;

static WORKING_AREA(waThread4, 100);
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
static msg_t Thread4(void *arg) {
  float euler[3];         // [psi, theta, phi]    Euler angle container
  
  while(1) {
   // if programming failed, don't try to do anything
    if (!dmpReady) return 0;
    // wait for MPU interrupt or extra packet(s) available
    chBSemWait(&mpu6050sem);

      mpuIntStatus = mpu.getIntStatus();
  
      // get current FIFO count
      fifoCount = mpu.getFIFOCount();
  
      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
          // reset so we can continue cleanly
          mpu.resetFIFO();
          overflow_count++;
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mag.getHeading(&mx, &my, &mz);
#if 0
            
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);

            

    // display tab-separated gyro x/y/z values
    Serial.print("mag:\t");
    Serial.print(mx); Serial.print("\t");
    Serial.print(my); Serial.print("\t");
    Serial.print(mz); Serial.print("\t");
        
    
// To calculate heading in degrees. 0 degree indicates North
    float heading = atan2(my, mx);
    if(heading < 0)
      heading += 2 * M_PI;
    Serial.print("heading:\t");
    Serial.println(heading * 180/M_PI);
    
 #endif  
#if 0
 
 // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif

      } else {

      
      }
  }   
}
void mainThread() ;

void setup() {
  uint8_t devStatus;   
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
  
  Serial.begin(115200);
  // wait for USB Serial
  
  Wire.begin();
  Wire.setClock(400000);
  
  while (!Serial) {}
  
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
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
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(2, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
  chBSemInit(&mpu6050sem, 1);

   // initialize device
    Serial.println("Initializing I2C devices...");
    mag.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");


  chBegin(mainThread);
  // chBegin never returns, main thread continues with mainThread()
  while(1) {}
}


//------------------------------------------------------------------------------
// main thread runs at NORMALPRIO
void mainThread() {

  // start blink thread
  chThdCreateStatic(waThread1, sizeof(waThread1),
                          NORMALPRIO + 2, Thread1, NULL);

  // start print thread
  chThdCreateStatic(waThread2, sizeof(waThread2),
                          NORMALPRIO + 1, Thread2, NULL);

  // start print thread
  chThdCreateStatic(waThread3, sizeof(waThread3),
                          NORMALPRIO + 3, Thread3, NULL);                          

  // start print thread
  chThdCreateStatic(waThread3, sizeof(waThread4),
                          NORMALPRIO + 4, Thread4, NULL);  
  // increment counter
  while (1) {
    // must insure increment is atomic in case of context switch for print
    // should use mutex for longer critical sections
    noInterrupts();
    count++;
    interrupts();
  }
}
//------------------------------------------------------------------------------
void loop() {
 // not used
}
