/**
 * Compass Demo
 * 
 * Print heading (in degrees) to attached I2C OLED display. Demonstrate
 * how to use magnetometer calibration data and convert magnetic heading
 * to geographic heading.
 * 
 * Author: Shawn Hymel
 * Date: May 5, 14
 * 
 * License: 0BSD (https://opensource.org/licenses/0BSD)
 */

#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

// Pins
const int pin_reset = 8;

// Hard-iron calibration settings
const float hard_iron[3] = {
  -15.55, 2.99, -70.15
};

// Soft-iron calibration settings
const float soft_iron[3][3] = {
  {  1.086, 0.005, 0.037    },
  {  0.005, 1.000, -0.026   },
  {  0.037, -0.026, 0.923   }
};

// Magnetic declination from magnetic-declination.com
// East is positive ( ), west is negative (-)
// mag_decl = ( /-)(deg   min/60   sec/3600)
// Set to 0 to get magnetic heading instead of geo heading
const float mag_decl = -3.28333;

// Globals
Adafruit_LIS3MDL lis3mdl;
 
void setup() {

  // Pour some serial
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("LIS3MDL compass test");

  // Initialize magnetometer
  if (!lis3mdl.begin_I2C()) {
    Serial.println("ERROR: Could not find magnetometer");
    while (1) {
      delay(1000);
    }
  }
}

void loop() {
  static float hi_cal[3];
  static float heading = 0;

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

  // Print calibrated results
  Serial.print("[");
  Serial.print(mag_data[0], 1);
  Serial.print("\t");
  Serial.print(mag_data[1], 1);
  Serial.print("\t");
  Serial.print(mag_data[2], 1);
  Serial.print("] Heading: ");
  Serial.println(heading, 2);
  delay(100); 
}