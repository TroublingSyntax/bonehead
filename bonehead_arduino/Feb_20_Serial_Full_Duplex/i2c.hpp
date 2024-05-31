/*
Simple i2c class for the bonehead i2c bus. The main functionality 
of this class is to organize and layout the functional i2c arduino code
to keep it separate from other features in the main loop().This class 
contains simple get and set methods, as well as standard data
members to hold information that is acquired and sent over i2c.
*/

#define USING_SENSORS 0 // Change this to 0 if one wants to run code without having sensors connected.
#define USING_ACTUATORS 1 // Change this to 0 if one wants to run code using only the sensors with no actuators.

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Arduino_MKRGPS.h>

class I2C
{
public:
    /************PUBLIC DATA MEMBERS FOR MAIN LOOP ACCESS*********************************/
    #if USING_SENSORS
    // sensor data members
    float euler_angles[3] = {0, 0, 0}; // holds the euler angles from the imu (roll, pitch, yaw)
    float heading = 0; // holds the calculated heading from the magnetometer data
    float latitude = 0; // holds the current latitude
    float longitude = 0; // holds the current longitude
    #endif

    #if USING_ACTUATORS
    // actuator data members
    int motor_dir[4] = {0}; // holds the motor directions from ROS
    int motor_cmds[4] = {0}; // holds the pwm motor commands from ROS
    #endif

    I2C() {} // Empty constructor

    void initI2C()
    {
    /*
    This function initialize the I2C protocol for communication with
    the sensors and the servo drivers.
    */

    #if USING_SENSORS
    /************IMU INITIALIZATION STEPS*********************************/
    Wire.begin();                      // Initialize comunication
    Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
    Wire.write(0x6B);                  // Talk to the register 6B
    Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
    Wire.endTransmission(true);        // End the transmission
    calculate_IMU_error();             // Calculate the IMU error before motion occurs

    /********************GPS INITIALIZATION STEPS********************************/
    if (!GPS.begin(GPS_MODE_I2C)) {
    Serial.println("Failed to initialize GPS!");
    while (1);
    }

    /************MAGNETOMETER INITIALIZATION STEPS********************************/
    if (! lis3mdl.begin_I2C()) {
    Serial.println("Failed to find LIS3MDL chip");
    while (1);
    }
    lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
    lis3mdl.setIntThreshold(500);
    lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!
    #endif
    }

    #if USING_SENSORS
    void calculate_IMU_error() {
    // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
    // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
    // Read accelerometer values 200 times
    while (c < 200) {
      Wire.beginTransmission(MPU);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 6, true);
      temp_val = (Wire.read() << 8 | Wire.read()); // X-axis value
      AccX = temp_val / 16384.0;
      temp_val = (Wire.read() << 8 | Wire.read()); // X-axis value
      AccY = temp_val / 16384.0;
      temp_val = (Wire.read() << 8 | Wire.read()); // X-axis value
      AccZ = temp_val / 16384.0;
      // Sum all readings
      AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
      AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
      c++;
    }
    //Divide the sum by 200 to get the error value
    AccErrorX = AccErrorX / 200;
    AccErrorY = AccErrorY / 200;
    c = 0;
    // Read gyro values 200 times
    while (c < 200) {
      Wire.beginTransmission(MPU);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 6, true);
      temp_val = (Wire.read() << 8 | Wire.read()); // X-axis value
      GyroX = temp_val;
      temp_val = (Wire.read() << 8 | Wire.read()); // X-axis value
      GyroY = temp_val;
      temp_val = (Wire.read() << 8 | Wire.read()); // X-axis value
      GyroZ = temp_val;
      // Sum all readings
      GyroErrorX = GyroErrorX + (GyroX / 131.0);
      GyroErrorY = GyroErrorY + (GyroY / 131.0);
      GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
      c++;
    }
    //Divide the sum by 200 to get the error value
    GyroErrorX = GyroErrorX / 200;
    GyroErrorY = GyroErrorY / 200;
    GyroErrorZ = GyroErrorZ / 200;
      }

    void get_orientation()
    {
      /************MAGNETOMETER READING PROCEDURE*********************************/
      // Get new sensor event with readings in uTesla
      sensors_event_t event;
      lis3mdl.getEvent(&event);
      
      // Put raw magnetometer readings into an array
      float mag_data[] = {event.magnetic.x,
                          event.magnetic.y,
                          event.magnetic.z};

      // Apply hard-iron offsets
      for (uint8_t i = 0; i < 3; i++ ) {
        hi_cal[i] = mag_data[i] - hard_iron[i];
      }

      // Apply soft-iron scaling
      for (uint8_t i = 0; i < 3; i++  ) {
        mag_data[i] = (soft_iron[i][0] * hi_cal[0]) + 
                      (soft_iron[i][1] * hi_cal[1]) +
                      (soft_iron[i][2] * hi_cal[2]);
      }

      // Calculate angle for heading, assuming board is parallel to
      // the ground and  Y points toward heading.
      heading = -1 * (atan2(mag_data[0], mag_data[1]) * 180) / M_PI;

      // Apply magnetic declination to convert magnetic heading
      // to geographic heading
      heading  += mag_decl;

      // Convert heading to 0..360 degrees
      if (heading < 0) {
        heading  += 360;
      }
      /************IMU READING PROCEDURE*********************************/
      // === Read acceleromter data === //
      Wire.beginTransmission(MPU);
      Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
      //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
      temp_val = (Wire.read() << 8 | Wire.read()); // X-axis value
      AccX = temp_val / 16384.0;
      temp_val = (Wire.read() << 8 | Wire.read()); // Y-axis value
      AccY = temp_val / 16384.0;
      temp_val = (Wire.read() << 8 | Wire.read()); // Z-axis value
      AccZ = temp_val / 16384.0;
      // Calculating Roll and Pitch from the accelerometer data
      accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
      accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY; // AccErrorY ~(-1.58)
      // === Read gyroscope data === //
      previousTime = currentTime;        // Previous time is stored before the actual time read
      currentTime = millis();            // Current time actual time read
      elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
      Wire.beginTransmission(MPU);
      Wire.write(0x43); // Gyro data first register address 0x43
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
      temp_val = (Wire.read() << 8 | Wire.read()); // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
      GyroX = temp_val / 131.0;
      temp_val = (Wire.read() << 8 | Wire.read());
      GyroY = temp_val / 131.0;
      temp_val = (Wire.read() << 8 | Wire.read());
      GyroZ = temp_val / 131.0;
      // Correct the outputs with the calculated error values
      GyroX = GyroX - GyroErrorX; // GyroErrorX ~(-0.56)
      GyroY = GyroY - GyroErrorY; // GyroErrorY ~(2)
      GyroZ = GyroZ - GyroErrorZ; // GyroErrorZ ~ (-0.8)
      // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
      gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
      gyroAngleY = gyroAngleY + GyroY * elapsedTime;
      euler_angles[2] =  euler_angles[2] + GyroZ * elapsedTime;
      // Complementary filter - combine acceleromter and gyro angle values
      euler_angles[0] = 0.96 * gyroAngleX + 0.04 * accAngleX;
      euler_angles[1] = 0.96 * gyroAngleY + 0.04 * accAngleY;
      return;
    }

    void get_GPS()
    {
      if (GPS.available()) {
        // read GPS values
        latitude  = GPS.latitude();
        longitude = GPS.longitude();
        delay(10);
        return;
      }
      else
      {
        return;
      }
    }
    #endif

private:
    #if USING_SENSORS
    // IMU members
    const int MPU = 0x68; // MPU6050 I2C address
    int16_t temp_val;
    float AccX, AccY, AccZ;
    float GyroX, GyroY, GyroZ;
    float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
    float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
    float elapsedTime, currentTime, previousTime;
    int c = 0;

    // Magnetometer members
    Adafruit_LIS3MDL lis3mdl;
    const float hard_iron[3] = { // Hard-iron calibration settings
      -15.55, 2.99, -70.15
    };
    const float soft_iron[3][3] = { // Soft-iron calibration settings
      {  1.086, 0.005, 0.037    },
      {  0.005, 1.000, -0.026   },
      {  0.037, -0.026, 0.923   }
    };
    // Magnetic declination from magnetic-declination.com
    // East is positive ( ), west is negative (-)
    // mag_decl = ( /-)(deg   min/60   sec/3600)
    // Set to 0 to get magnetic heading instead of geo heading
    const float mag_decl = -3.28333;
    float hi_cal[3];
    #endif
};