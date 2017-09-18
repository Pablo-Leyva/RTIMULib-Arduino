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


#include "RTIMUGY85.h"
#include "RTIMUSettings.h"

//  this sets the learning rate for compass running average calculation
#if defined(GY85_53) || defined(GY85_1d)
#define COMPASS_ALPHA 0.2f

RTIMUGY85::RTIMUGY85(RTIMUSettings *settings) : RTIMU(settings)
{
    m_sampleRate = 10;
}

RTIMUGY85::~RTIMUGY85()
{
}

int RTIMUGY85::IMUInit()
{
    unsigned char gyroData[6];
    unsigned char result;

    m_accelSlaveAddr = m_settings->m_I2CSlaveAddress;
    m_gyroSlaveAddr = ITG3205_ADDRESS0;
    m_compassSlaveAddr = HMC5883L_ADDRESS0;

    accelInit();
    gyroInit();
    compassInit();

    gyroBiasInit();
    return true;
}

void RTIMUGY85::accelInit() {
    I2CWrite(m_accelSlaveAddr, ADXL345_PWR_CTL, 0x00);
    I2CWrite(m_accelSlaveAddr, ADXL345_PWR_CTL, ADXL345_OPER_MODE_MEASURE);
    setAccelRange();
    setAccelSampleRate();
}

void RTIMUGY85::setAccelSampleRate() {    
    I2CWrite(m_accelSlaveAddr, ADXL345_BW_RATE, m_settings->m_GY85AccelSampleRate);
}

void RTIMUGY85::setAccelRange() {
    bool valid_config = true;
    switch (m_settings->m_GY85AccelFsr) {
    case ADXL345_FSR_2:
        m_accelScale = (RTFLOAT)0.0039;
        break;
    case ADXL345_FSR_4:
        m_accelScale = (RTFLOAT)0.0078;
        break;

    case ADXL345_FSR_8:
        m_accelScale = (RTFLOAT)0.0156;
        break;

    case ADXL345_FSR_16:
        m_accelScale = (RTFLOAT)0.0312;
        break;

    case ADXL345_FSR_FULL:
        m_accelScale = (RTFLOAT)0.0039;
        break;
    
    default:
        valid_config = false;
    }
    if (valid_config){
        switch (m_settings->m_GY85AccelFsr) {
        case ADXL345_FSR_FULL:
            I2CWrite(m_accelSlaveAddr, ADXL345_DATA_FORMAT, 1<<3);
        break;

        default:
            I2CWrite(m_accelSlaveAddr, ADXL345_DATA_FORMAT, m_settings->m_GY85AccelFsr);
        }
    }
}


void RTIMUGY85::gyroInit() {
    I2CWrite(m_gyroSlaveAddr, ITG3205_DLPF_FS, (0x03<<3));
    setGyroBW();
    setGyroSampleRate();
    m_gyroScale = (RTFLOAT)69.56521739*1e-3 * RTMATH_DEGREE_TO_RAD;
}

void RTIMUGY85::setGyroBW() {
    uint8_t reg;
    I2CRead(m_gyroSlaveAddr, ITG3205_DLPF_FS, 1, &reg);
    I2CWrite(m_gyroSlaveAddr, ITG3205_DLPF_FS,  (uint8_t)(reg | m_settings->m_GY85GyroBW));
}

void RTIMUGY85::setGyroSampleRate() {
    I2CWrite(m_gyroSlaveAddr, ITG3205_SMPLRT_DIV,  m_settings->m_GY85GyroSampleRate);
}


void RTIMUGY85::compassInit() {    
    setCompassRange();
    setCompassSampleRate();
    I2CWrite(m_compassSlaveAddr, HMC5883L_MODE,  0);
}

void RTIMUGY85::setCompassSampleRate() {
    uint8_t reg;
    I2CRead(m_compassSlaveAddr, HMC5883L_CONFIG_A, 1, &reg);
    reg = reg & 0xE3;
    I2CWrite(m_compassSlaveAddr, HMC5883L_CONFIG_A,  (uint8_t)(reg | (m_settings->m_GY85CompassSampleRate<<2)));
}

void RTIMUGY85::setCompassRange() {
    bool valid_config = true;
    switch (m_settings->m_GY85CompassFsr) {
    case HMC5883L_FSR_0P88:
        m_compassScale = (RTFLOAT)0.073;
        break;
    case HMC5883L_FSR_1P3:
        m_compassScale = (RTFLOAT)0.092;
        break;
    case HMC5883L_FSR_1P9:
        m_compassScale = (RTFLOAT)0.122;
        break;
    case HMC5883L_FSR_2P5:
        m_compassScale = (RTFLOAT)0.152;
        break;
    case HMC5883L_FSR_4:
        m_compassScale = (RTFLOAT)0.227;
        break;
    case HMC5883L_FSR_4P7:
        m_compassScale = (RTFLOAT)0.256;
        break;
    case HMC5883L_FSR_5P6:
        m_compassScale = (RTFLOAT)0.303;
        break;
    case HMC5883L_FSR_8P1:
        m_compassScale = (RTFLOAT)0.435;
    default:
        valid_config = false;
    }
    if (valid_config){
        I2CWrite(m_compassSlaveAddr, HMC5883L_CONFIG_B,  (uint8_t)(m_settings->m_GY85CompassFsr << 5));
    }    
}

int RTIMUGY85::IMUGetPollInterval()
{
    return (1000 / m_sampleRate);
}

bool RTIMUGY85::IMURead()
{
    unsigned char accelData[6];
    unsigned char gyroData[6];
    unsigned char compassData[6];
    
    I2CRead(m_accelSlaveAddr,   ADXL345_DATA_X_LSB,     6, accelData);
    I2CRead(m_gyroSlaveAddr,    ITG3205_GYRO_XOUT_H,    6, gyroData);
    I2CRead(m_compassSlaveAddr, HMC5883L_DATAX_H,       6, compassData);

    RTMath::convertToVector(accelData,   m_accel,   m_accelScale,   false);
    RTMath::convertToVector(gyroData,    m_gyro,    m_gyroScale,    true);    
    RTMath::convertToVector(compassData, m_compass, m_compassScale, true);

    //  sort out compass axes
   
    //  now do standard processing
    
    handleGyroBias();
    calibrateAverageCompass();

    return true;
}
#endif