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


#ifndef _RTIMUGY85_H
#define	_RTIMUGY85_H

#include "RTIMU.h"

//----------------------------------------------------------
//
//  GY-85


//  ADXL345
//  I2C Slave Addresses
#define ADXL345_ADDRESS0            0x53
#define ADXL345_ADDRESS1            0x1D
#define ADXL345_ID                  0xE5

//  Register map
#define ADXL345_WHO_AM_I            0x00
#define ADXL345_BW_RATE             0x2C
#define ADXL345_PWR_CTL             0x2D
#define ADXL345_DATA_FORMAT         0x31

#define ADXL345_DATA_X_LSB          0x32
#define ADXL345_DATA_X_MSB          0x33
#define ADXL345_DATA_Y_LSB          0x34
#define ADXL345_DATA_Y_MSB          0x35
#define ADXL345_DATA_Z_LSB          0x36
#define ADXL345_DATA_Z_MSB          0x37

#define ADXL345_FIFO_CTL            0x38
#define ADXL345_FIFO_STATUS         0x39

#define ADXL345_SAMPLERATE_6        0X06
#define ADXL345_SAMPLERATE_12       0X07
#define ADXL345_SAMPLERATE_25       0X08
#define ADXL345_SAMPLERATE_50       0X09
#define ADXL345_SAMPLERATE_100      0x0A
#define ADXL345_SAMPLERATE_200      0x0B
#define ADXL345_SAMPLERATE_400      0X0C
#define ADXL345_SAMPLERATE_800      0X0D
#define ADXL345_SAMPLERATE_1600     0X0E
#define ADXL345_SAMPLERATE_3200     0X0F

#define ADXL345_OPER_MODE_MEASURE   0x08

#define ADXL345_FSR_2               0x00
#define ADXL345_FSR_4               0x01
#define ADXL345_FSR_8               0x02
#define ADXL345_FSR_16              0x03
#define ADXL345_FSR_FULL            0x08

#define ADXL345_FIFO_BYPASS         0x00
#define ADXL345_FIFO_FIFO           0x01
#define ADXL345_FIFO_STREAM         0x02
#define ADXL345_FIFO_TRIGGER        0x03

//  ITG3205
//  I2C Slave Addresses
#define ITG3205_ADDRESS0            0x68
#define ITG3205_ADDRESS1            0x69
#define ITG3205_ID                  0x68

//  Register map
#define ITG3205_WHO_AM_I            0x00
#define ITG3205_SMPLRT_DIV          0x15
#define ITG3205_DLPF_FS             0x16
#define ITG3205_INT_CFG             0x17
#define ITG3205_INT_STATUS          0x1A
#define ITG3205_TEMP_OUT_H          0x1B
#define ITG3205_TEMP_OUT_L          0x1C
#define ITG3205_GYRO_XOUT_H         0x1D
#define ITG3205_GYRO_XOUT_L         0x1E
#define ITG3205_GYRO_YOUT_H         0x1F
#define ITG3205_GYRO_YOUT_L         0x20
#define ITG3205_GYRO_ZOUT_H         0x21
#define ITG3205_GYRO_ZOUT_L         0x22
#define ITG3205_PWR_MGM             0x3E

#define ITG3205_FULLSCALE_2000      0x03

//#define ITG3205_BW_256                0x00
#define ITG3205_BW_188              0x01
#define ITG3205_BW_98               0x02
#define ITG3205_BW_42               0x03
#define ITG3205_BW_20               0x04
#define ITG3205_BW_10               0x05
#define ITG3205_BW_5                0x06

#define ITG3205_SAMPLERATE_5        0Xc8
#define ITG3205_SAMPLERATE_10       0X64
#define ITG3205_SAMPLERATE_25       0x28
#define ITG3205_SAMPLERATE_50       0x14
#define ITG3205_SAMPLERATE_100      0X0A
#define ITG3205_SAMPLERATE_250      0X04
#define ITG3205_SAMPLERATE_500      0X02
#define ITG3205_SAMPLERATE_1000     0X01

//  HMC5883L
//  I2C Slave Addresses
#define HMC5883L_ADDRESS0           0x1E
#define HMC5883L_ID                 0x48

//  Register map
#define HMC5883L_WHO_AM_I           0x0A
#define HMC5883L_CONFIG_A           0x00
#define HMC5883L_CONFIG_B           0x01
#define HMC5883L_MODE               0x02
#define HMC5883L_DATAX_H            0x03
#define HMC5883L_DATAX_L            0x04
#define HMC5883L_DATAZ_H            0x05
#define HMC5883L_DATAZ_L            0x06
#define HMC5883L_DATAY_H            0x07
#define HMC5883L_DATAY_L            0x08
#define HMC5883L_STATUS             0x09
#define HMC5883L_ID_A               0x0A
#define HMC5883L_ID_B               0x0B
#define HMC5883L_ID_C               0x0C

#define HMC5883L_AVERAGING_1        0x00
#define HMC5883L_AVERAGING_2        0x01
#define HMC5883L_AVERAGING_4        0x02
#define HMC5883L_AVERAGING_8        0x03

#define HMC5883L_SAMPLERATE_0P75    0x00
#define HMC5883L_SAMPLERATE_1P5     0x01
#define HMC5883L_SAMPLERATE_3       0x02
#define HMC5883L_SAMPLERATE_7P5     0x03
#define HMC5883L_SAMPLERATE_15      0x04
#define HMC5883L_SAMPLERATE_30      0x05
#define HMC5883L_SAMPLERATE_75      0x06

#define HMC5883L_BIAS_NORMAL        0x00
#define HMC5883L_BIAS_POSITIVE      0x01
#define HMC5883L_BIAS_NEGATIVE      0x02

#define HMC5883L_FSR_0P88           0x00
#define HMC5883L_FSR_1P3            0x01
#define HMC5883L_FSR_1P9            0x02
#define HMC5883L_FSR_2P5            0x03
#define HMC5883L_FSR_4              0x04
#define HMC5883L_FSR_4P7            0x05
#define HMC5883L_FSR_5P6            0x06
#define HMC5883L_FSR_8P1            0x07

class RTIMUGY85 : public RTIMU
{

public:
    RTIMUGY85(RTIMUSettings *settings);
    ~RTIMUGY85();

    virtual const char *IMUName() { return "ADXL345 + ITG3205 + HMC5883L";}
    virtual int IMUType() { return RTIMU_TYPE_GY85; }
    virtual int IMUInit();
    virtual int IMUGetPollInterval();
    virtual bool IMURead();

private:

    // ADXL345
    void    accelInit();
    void    setAccelSampleRate();
    void    setAccelRange();

    void    gyroInit();
    void    setGyroBW();
    void    setGyroSampleRate();
    void    setGyroRange();

    void    compassInit();
    void    setCompassSampleRate();
    void    setCompassRange();

    unsigned char m_accelSlaveAddr;                         // I2C address of gyro
    unsigned char m_gyroSlaveAddr;                          // I2C address of gyro
    unsigned char m_compassSlaveAddr;                       // I2C address of gyro

    int m_GY85AccelSampleRate;                             // the accel sample rate
    int m_GY85AccelFsr;                                    // the accel full scale range

    int m_GY85GyroBW;                                      // the gyro bandwidth code
    int m_GY85GyroFsr;                                     // the gyro full scale range

    int m_GY85CompassSampleRate;                           // the compass sample rate
    int m_GY85CompassFsr;                                  // the compass full scale range

    int m_gyroOffsetX, m_gyroOffsetY, m_gyroOffsetZ;
    int m_gyroX, m_gyroY, m_gyroZ;
    int m_compassX, m_compassY, m_compassZ;


    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;
    RTFLOAT m_compassScale;
};

#endif // _RTIMUGY85_H
