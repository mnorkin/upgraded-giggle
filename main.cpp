/**
 * MPU9250 Basic Example Code
 * by: Kris Winer
 * date: April 1, 2014
 * license: Beerware - Use this code however you'd like. If you
 * find it useful you can buy me a beer some time.
 * 
 * Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor,
 * getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to
 * allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and
 * Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.

 * SDA and SCL should have external pull-up resistors (to 3.3V).
 * 10k resistors are on the EMSENSR-9250 breakout board.

 * Hardware setup:
 * MPU9250 Breakout --------- Arduino
 * VDD ---------------------- 3.3V
 * VDDI --------------------- 3.3V
 * SDA ----------------------- A4
 * SCL ----------------------- A5
 * GND ---------------------- GND

 * Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library.
 * Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 * We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 * We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */

#include "mbed.h"
#include "9520.h"

float sum = 0;
uint32_t sumCount = 0;
char buffer[14];
MPU9250 mpu9250;
Timer t;
Serial pc(USBTX, USBRX); // tx, rx

int main()
{
    // Set up I2C
    // use fast (400 kHz) I2C
    i2c.frequency(400000);  
    pc.printf("CPU SystemCoreClock is %d Hz\r\n", SystemCoreClock);
    t.start();

    // Read the WHO_AM_I register, this is a good test of communication
    // Read WHO_AM_I register for MPU-9250
    uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

    pc.printf("I AM 0x%x\n\r", whoami);
    pc.printf("I SHOULD BE 0x71\n\r");

    // WHO_AM_I should always be 0x68
    if (whoami == 0x71) {
        pc.printf("MPU9250 WHO_AM_I is 0x%x\n\r", whoami);
        pc.printf("MPU9250 is online...\n\r");
        sprintf(buffer, "0x%x", whoami);
        wait(1);

        // Reset registers to default in preparation for device calibration
        mpu9250.resetMPU9250();
        // Start by performing self test and reporting values
        mpu9250.MPU9250SelfTest(SelfTest);

        pc.printf("acceleration trim self test:\n");
        pc.printf("\t x: %f \t y: %f \t z: %f\n", SelfTest[0], SelfTest[1], SelfTest[2]);

        pc.printf("gyration trim self test \n");
        pc.printf("\t x: %f \t y: %f \t z: %f\n", SelfTest[3], SelfTest[4], SelfTest[5]);
        
        // Calibrate gyro and accelerometers, load biases in bias registers
        mpu9250.calibrateMPU9250(gyroBias, accelBias);

        pc.printf("gyro bias: x: %f \t y: %f z: %f \n", gyroBias[0], gyroBias[1], gyroBias[2]);
        pc.printf("accelerometer bias: x: %f \t y: %f \t z: %f \n", accelBias[0], accelBias[1], accelBias[2]);
        
        // small pause
        wait(2);
        mpu9250.initMPU9250();

        // Initialize device for active mode read of acclerometer, gyroscope, and temperature
        pc.printf("MPU9250 initialized for active data mode....\n");
        mpu9250.initAK8963(magCalibration);
        // Initialize device for active mode read of magnetometer
        pc.printf("AK8963 initialized for active data mode....\n");
        pc.printf("Accelerometer full-scale range = %f  g\n", 2.0f*(float)(1<<Ascale));
        pc.printf("Gyroscope full-scale range = %f  deg/s\n", 250.0f*(float)(1<<Gscale));

        if (Mscale == 0) {
            pc.printf("Magnetometer resolution = 14  bits\n");
        }
        if (Mscale == 1) {
            pc.printf("Magnetometer resolution = 16  bits\n");
        }
        if (Mmode == 2) {
            pc.printf("Magnetometer ODR = 8 Hz\n");
        }
        if (Mmode == 6) {
            pc.printf("Magnetometer ODR = 100 Hz\n");
        }
        wait(1);

    } else {
        pc.printf("Could not connect to MPU9250: \n\r");
        pc.printf("%#x \n",  whoami);

        sprintf(buffer, "WHO_AM_I 0x%x", whoami);

        // Loop forever if communication doesn't happen
        while(1) ; 
    }

    // Get accelerometer sensitivity
    mpu9250.getAres(); 
    // Get gyro sensitivity
    mpu9250.getGres(); 
    // Get magnetometer sensitivity
    mpu9250.getMres(); 
    
    pc.printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f / aRes);
    pc.printf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f / gRes);
    pc.printf("Magnetometer sensitivity is %f LSB/G \n\r", 1.0f / mRes);
    
    // User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss
    magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    magbias[2] = +125.;

    while(1) {

        // If intPin goes high, all data registers have new data
        if (mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
            mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values
            // Now we'll calculate the accleration value into actual g's
            ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
            ay = (float)accelCount[1]*aRes - accelBias[1];
            az = (float)accelCount[2]*aRes - accelBias[2];

            mpu9250.readGyroData(gyroCount);  // Read the x/y/z adc values
            // Calculate the gyro value into actual degrees per second
            gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
            gy = (float)gyroCount[1]*gRes - gyroBias[1];
            gz = (float)gyroCount[2]*gRes - gyroBias[2];

            mpu9250.readMagData(magCount);  // Read the x/y/z adc values
            // Calculate the magnetometer values in milliGauss
            // Include factory calibration per data sheet and user environmental corrections
            mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
            my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];
            mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];
        }

        Now = t.read_us();

        deltat = (float)((Now - lastUpdate)/1000000.0f) ; // set integration time by time elapsed since last filter update
        lastUpdate = Now;
        sum += deltat;
        sumCount++;

        // Pass gyro rate as rad/s
        mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);

        // Serial print and/or display at 0.5 s rate independent of data rates
        delt_t = t.read_ms() - count;
        if (delt_t > 500) { // update LCD once per half-second independent of read rate

            pc.printf("ax = %f", 1000 * ax);
            pc.printf(" ay = %f", 1000 * ay);
            pc.printf(" az = %f  mg\n", 1000 * az);

            pc.printf("gx = %f", gx);
            pc.printf(" gy = %f", gy);
            pc.printf(" gz = %f  deg/s\n", gz);

            pc.printf("gx = %f", mx);
            pc.printf(" gy = %f", my);
            pc.printf(" gz = %f  mG\n", mz);

            // Read the adc values
            tempCount = mpu9250.readTempData();
            // Temperature in degrees Centigrade
            temperature = ((float) tempCount) / 333.87f + 21.0f;
            pc.printf("temperature = %f  C\n", temperature);

            pc.printf("q0 = %f\n", q[0]);
            pc.printf("q1 = %f\n", q[1]);
            pc.printf("q2 = %f\n", q[2]);
            pc.printf("q3 = %f\n", q[3]);
            
            // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
            // In this coordinate system, the positive z-axis is down toward Earth.
            // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
            // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
            // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
            // These arise from the definition of the homogeneous rotation matrix constructed from quaternions
            // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
            // applied in the correct order which for this configuration is yaw, pitch, and then roll.
            // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links. 

            yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
            pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
            roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
            pitch *= 180.0f / PI;
            yaw *= 180.0f / PI;
            yaw -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
            roll *= 180.0f / PI;

            pc.printf("Yaw, Pitch, Roll: %f %f %f\n\r", yaw, pitch, roll);
            pc.printf("average rate = %f\n\r", (float) sumCount/sum);

            myled = !myled;
            count = t.read_ms();

            if (count > 1<<21) {
                t.start(); // start the timer over again if ~30 minutes has passed
                count = 0;
                deltat = 0;
                lastUpdate = t.read_us();
            }
            sum = 0;
            sumCount = 0;
        }
    }
}
