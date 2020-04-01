////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
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

#include "RTPressureMS5611.h"

#if defined(MS5611)

RTPressureMS5611::RTPressureMS5611(RTIMUSettings *settings) : RTPressure(settings)
{
    m_validReadings = false;
}

RTPressureMS5611::~RTPressureMS5611()
{
}

bool RTPressureMS5611::pressureInit()
{
    unsigned char cmd = MS5611_CMD_PROM + 2;
    unsigned char data[2];

    m_pressureAddr = m_settings->m_I2CPressureAddress;

    for (uint8_t offset = 0; offset < 6; offset++)
    {
        if (!m_settings->HALRead(m_pressureAddr, cmd + (offset * 2), 2, data, "Failed to read MS5611 calibration data"))
            return false;
        fc[offset] = (((uint16_t)data[0]) << 8) + (uint16_t)data[1];    
        delay(10);
    }

    return true;
}

#define MS5611_ADDRESS m_pressureAddr

uint32_t RTPressureMS5611::readRaw(unsigned char command)
{
    uint8_t data[3];

    if (!m_settings->HALWrite (MS5611_ADDRESS, command, 0, 0, "Failed to start MS5611 pressure conversion")) {
        return 0;    
    }

    delay(10);

    if (!m_settings->HALRead(MS5611_ADDRESS, MS5611_CMD_ADC, 3, data, "Failed to read MS5611 pressure")) {
        return 0;
    }
    
    return (((uint32_t)data[0]) << 16) | (((uint32_t)data[1]) << 8) | (uint32_t)data[2];
}

bool RTPressureMS5611::pressureRead(RTIMU_DATA& data)
{
    uint32_t D1 = readRaw(MS5611_CMD_CONV_D1);
    uint32_t D2 = readRaw(MS5611_CMD_CONV_D2);
    int32_t dT = D2 - (uint32_t)fc[4] * 256;

    if (D1 == 0 || D2 == 0)
    {
        data.temperature = data.pressure = 0;
        data.pressureValid = data.temperatureValid = false;
        return false;
    }

    int64_t OFF = (int64_t)fc[1] * 65536 + (int64_t)fc[3] * dT / 128;
    int64_t SENS = (int64_t)fc[0] * 32768 + (int64_t)fc[2] * dT / 256;
    
    uint32_t P = (D1 * SENS / 2097152 - OFF) / 32768;

    data.pressure = (float)P/100.0;
    data.pressureValid = true;
    data.temperature = tempFromDT(dT);
    data.temperatureValid = true;

    return true;
}

float RTPressureMS5611::tempFromDT(int32_t dT)
{
    int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;
    return ((float)TEMP/100);
}
#endif