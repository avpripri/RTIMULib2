////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014-2015, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <Wire.h>
#include <EEPROM.h>
#include "I2Cdev.h"

#include "RTArduLinkDefs.h"
#include "RTArduLinkHAL.h"
#include "RTArduLink.h"
#include "RTArduLinkUtils.h"

#include "RTArduLinkIMUDefs.h"

#include "RTArduLinkIMU.h"
#include "RTIMUSettings.h"
#include "IMUDrivers/RTIMU.h"
#include "CalLib.h"
#include "RTFusionRTQF.h" 
#include "IMUDrivers/RTPressure.h"

#include "SimpleKalmanFilter.h"

RTIMU *imu;                                           // the IMU object
RTPressure *press;
RTIMUSettings *settings;

SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);
SimpleKalmanFilter pitotKalmanFilter(1, 1, 0.01);
SimpleKalmanFilter temperatureKalmanFilter(1, 1, 0.01);
SimpleKalmanFilter TEKalmanFilter(1, 1, 0.01);

//  DISPLAY_INTERVAL sets the rate at which results are displayed

#define DISPLAY_INTERVAL  300                         // interval between pose displays

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

//#define  SERIAL_PORT_SPEED  9600
#define  SERIAL_PORT_SPEED  115200

#define ADC_CHANNEL_0 PA1
#define CALIBRATION_PIN PA2

char* itox(uint16 val)
{
    static char buf[5] = "0000";
    const uint8 n = 4;
    char* bp = buf + n;
    do {
        auto d = val % 16; 
        *--bp = (d > 9) ? (d - 10) + 'a' : (d + '0');
        val = val / 16;
    } while (bp != buf);
    return (buf);
}

void setup()
{
    delay(5000);

    Serial.begin(SERIAL_PORT_SPEED);
    delay(1000);

    // while (!Serial) {
    //     ; // wait for serial port to connect. 
    // }
    Serial.println("IMU Starting");

    settings = new RTIMUSettings();
    imu = RTIMU::createIMU(settings);                 // create the imu object
    press = RTPressure::createPressure(settings);

    Serial.print("Device(s): "); Serial.print(imu->IMUName()); Serial.print(' '); Serial.println(press->pressureName());
    if (!imu->IMUInit()) {
        Serial.print("Failed to init IMU: ");
    }
    else {
        Serial.println("IMU Initialized");
    }

    if (!press->pressureInit())
        Serial.print("Failed to init Pressure sensor");


    pinMode(PB3, OUTPUT_OPEN_DRAIN);
    digitalWrite(PB3, LOW);

    pinMode(PB4, OUTPUT_OPEN_DRAIN);
    digitalWrite(PB4, LOW);

}

void Dump(String label, float val)
{
//    Serial.print(itox((uint16)((long)val)));
    Serial.print("," + label + ":"); Serial.print(val); 
}

void loop()
{
    if (!imu->IMURead())
        return;
                                        // get the latest data if ready yet
    RTIMU_DATA data = imu->getIMUData();
    if (press != NULL)
        press->pressureRead(data);

    Serial.print("F:");
    Serial.print(imu->IMUGyroBiasValid() | 2*imu->getAccelCalibrationValid() | 4*imu->getRuntimeCompassCalibrationValid());        
    Dump("R", data.fusionPose.x());
    Dump("P", data.fusionPose.y());
    Dump("H", (int)(540.0 + data.fusionPose.z() * RTMATH_RAD_TO_DEGREE) % 360);
    Dump("G", data.accel.z());
    Dump("Y", data.accel.y());
    Dump("A", pitotKalmanFilter.updateEstimate(analogRead(ADC_CHANNEL_0)));
    Dump("B", pressureKalmanFilter.updateEstimate(data.pressure));
    Dump("T", temperatureKalmanFilter.updateEstimate(data.temperature));
    Dump("E", TEKalmanFilter.updateEstimate(imu->getTotalEnergy(data.accel)));
    Serial.println();
}

/* 	   1   2   3 4   5 6
	   |   |   | |   | |
$--HDG,x.x,x.x,a,x.x,a*hh
1) Magnetic Sensor heading in degrees
2) Magnetic Deviation, degrees
3) Magnetic Deviation direction, E = Easterly, W = Westerly
4) Magnetic Variation degrees
5) Magnetic Variation directio
6) Checksum
*/

#define NMEA_END_CHAR_1 '\n'
#define NMEA_MAX_LENGTH 70

uint8_t nmea_get_checksum(char *sentence)
{
	const char *n = sentence + 1;
	uint8_t chk = 0;
    uint8_t count = NMEA_MAX_LENGTH - 1;

	/* While current char isn't '*' or sentence ending (newline) */
	while ('*' != *n && NMEA_END_CHAR_1 != *n && '\0' != *n && --count > 0) {
		chk ^= (uint8_t) *n;
		n++;
	}

	return chk;
}