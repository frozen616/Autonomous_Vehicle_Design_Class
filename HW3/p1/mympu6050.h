#ifndef MYMPU6050_H
#define MYMPU6050_H

// MPU 6050 macro definitions from 
// MPU-6000 and MPU-6050, Register Map and Descriptions, Revision 4.2, 8/19/13
// MPU-6000/MPU-6050 Product Specification, Revision 3.4, 8/19/13

// MPU 6050 I2C address
#define MPU6050_ADDRESS_AD0_LOW     0x68   // i2c address pin low (GND), default
#define MPU6050_ADDRESS_AD0_HIGH    0x69



// MPU 6050 register addresses
#define MPU6050_RA_ACCEL_XG_OFFS_USRH 0x06 // accel x-axis offset cancellation register high byte
#define MPU6050_RA_ACCEL_XG_OFFS_USRL 0x07
#define MPU6050_RA_ACCEL_YG_OFFS_USRH 0x08 // accel y-axis offset cancellation register high byte
#define MPU6050_RA_ACCEL_YG_OFFS_USRL 0x09
#define MPU6050_RA_ACCEL_ZG_OFFS_USRH 0x0A // accel z-axis offset cancellation register high byte
#define MPU6050_RA_ACCEL_ZG_OFFS_USRL 0x0B

#define MPU6050_RA_GYRO_XG_OFFS_USRH 0x13      // gyro x-axis offset cancellation register high byte
#define MPU6050_RA_GYRO_XG_OFFS_USRL 0x14      // gyro x-axis offset cancellation register low byte
#define MPU6050_RA_GYRO_YG_OFFS_USRH 0x15      // gyro x-axis offset cancellation register high byte
#define MPU6050_RA_GYRO_YG_OFFS_USRL 0x16      // gyro x-axis offset cancellation register low byte
#define MPU6050_RA_GYRO_ZG_OFFS_USRH 0x17      // gyro x-axis offset cancellation register high byte
#define MPU6050_RA_GYRO_ZG_OFFS_USRL 0x18      // gyro x-axis offset cancellation register low byte

#define MPU6050_RA_PWR_MGMT_1   0x6B
#define MPU6050_RA_CONFIG       0x1A
#define MPU6050_RA_GYRO_CONFIG  0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C

#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_ACCEL_XOUT_L 0x3C
#define MPU6050_RA_ACCEL_YOUT_H 0x3D
#define MPU6050_RA_ACCEL_YOUT_L 0x3E
#define MPU6050_RA_ACCEL_ZOUT_H 0x3F
#define MPU6050_RA_ACCEL_ZOUT_L 0x40

#define MPU6050_RA_TEMP_OUT_H   0x41
#define MPU6050_RA_TEMP_OUT_L   0x42

#define MPU6050_RA_GYRO_XOUT_H  0x43
#define MPU6050_RA_GYRO_XOUT_L  0x44
#define MPU6050_RA_GYRO_YOUT_H  0x45
#define MPU6050_RA_GYRO_YOUT_L  0x46
#define MPU6050_RA_GYRO_ZOUT_H  0x47
#define MPU6050_RA_GYRO_ZOUT_L  0x48

#define MPU6050_RA_WHO_AM_I     0x75     


// MPU 6050 Clock Settings
#define MPU6050_CLOCK_PLL_XGYRO 0x01

// MPU gyro full scale range settings 
#define MPU6050_GYRO_FS_250  0x00
#define MPU6050_GYRO_FS_500  0x01
#define MPU6050_GYRO_FS_1000 0x02
#define MPU6050_GYRO_FS_2000 0x03

// MPU accel full scale settings 
#define MPU6050_ACCEL_FS_2G  0x00
#define MPU6050_ACCEL_FS_4G  0x01
#define MPU6050_ACCEL_FS_8G  0x02
#define MPU6050_ACCEL_FS_16G 0x03

// Sizes
#define MPU6050_ALL_MEASUREMENT_BYTES 14

struct SensorData
{
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t temperature;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
};

struct mean_values {
 
    int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
};

struct offset_values {
  
    int ax_offset = 0, ay_offset = 0, az_offset = 0, gx_offset = 0, gy_offset = 0, gz_offset = 0;
};


class MyMPU6050{
  public:
    MyMPU6050(uint8_t i2cAddr = MPU6050_ADDRESS_AD0_LOW );
    
    void initialize(void);
    void reset(void);
    uint8_t testConnection(void);

    void setClockSource(uint8_t source);
    void setFullScaleAccelRange(uint8_t scale);
    void setFullScaleGyroRange(uint8_t scale);
    void setSleepEnabled(bool state);


    uint8_t getDeviceAddress(void)const;
    uint8_t getConfigState(void)const;
    uint8_t getPowerManagement1State(void)const;
    uint8_t getAccelConfigState(void)const;
    uint8_t getGyroConfigState(void)const;

    int buffersize=1100; //amojnt of readings used to calibrate
    int acel_deadzone=4; //Accelerometer error allowed lower=moreprecise but may not converge
    int giro_deadzone=1; //Giro error allowed lower=more precise but my not converge
   
    uint8_t readAllData(struct SensorData* sd);
    void mean_calc(struct SensorData* sd, struct mean_values* mv, struct offset_values* ov);
    void calibration(struct SensorData* sd, struct offset_values* ov, struct mean_values* mv);

    
    
  private:
    uint8_t devAddr;

    void writeBits(uint8_t regAddr, uint8_t leftBit, uint8_t numBits, uint8_t data);
    void writeByte(uint8_t regAddr, uint8_t data);
    uint8_t readByte(uint8_t regAddr)const;
    uint8_t readBytes(uint8_t regAddr, uint8_t *buf, uint8_t count);
    
};



#endif 
