#include <Arduino.h>    // Serial
#include <Wire.h>
#include "mympu6050.h"
#include <stdio.h>

//#define DEBUG




#define SERIAL_DEBUG




MyMPU6050::MyMPU6050(uint8_t i2cAddr)
{
  devAddr = i2cAddr;
}


uint8_t MyMPU6050::getDeviceAddress()const
{
  return devAddr;
}


uint8_t MyMPU6050::getConfigState(void)const
{
  return readByte((uint8_t)MPU6050_RA_CONFIG); 
}


uint8_t MyMPU6050::getAccelConfigState(void)const
{
  return readByte((uint8_t)MPU6050_RA_ACCEL_CONFIG); 
}


uint8_t MyMPU6050::getGyroConfigState(void)const
{
  return readByte((uint8_t)MPU6050_RA_GYRO_CONFIG); 
}


uint8_t MyMPU6050::getPowerManagement1State(void)const
{
  return readByte(MPU6050_RA_PWR_MGMT_1);
}

/*
 * Wakes from sleep mode
 */
void MyMPU6050::initialize(void)
{
  setSleepEnabled(false); // wake up device
}

/*
 * Sets DEVICE_RESET bit to 1
 * All internal registers are reset to their default
 * values.
 * 
 * DEVICE_RESET bit automatically clears to 0 once 
 * the reset is complete.
 */
void MyMPU6050::reset(void)
{
  writeByte(MPU6050_RA_PWR_MGMT_1, 0x80);
}

/* Sets clock source
 *  Parameter source
 *  0   Internal 8MHz oscillator
 *  1   PLL with X axis gyroscope reference
 *  2   PLL with Y axis gyroscope reference
 *  3   PLL with Z axis gyroscope reference
 *  4   PLL with external 32.768 kHz reference
 *  5   PLL with external 19.2 MHz reference
 *  6   reserved
 *  7   stops the clock and keeps the timing generator 
 *      in reset
 */
void MyMPU6050::setClockSource(uint8_t source)
{
  // clock select is bits 2:0
  writeBits(MPU6050_RA_PWR_MGMT_1, 2, 3, source);
  
  #ifdef SERIAL_DEBUG
    uint8_t b;
    b = readByte(MPU6050_RA_PWR_MGMT_1);

    Serial.println(F("\nfunction: setClockSource"));
    
    if ( (b & 0x07) == source)
    {
      Serial.println(F("success updating source"));
    }
    else
    {
      Serial.print(F("error setting clock source, expected: "));
      Serial.print(source);
      Serial.print(F(", regVal: "));
      Serial.println(b,HEX);
    }

  #endif
}


/* Sets accelerometer full scale range
 *  
 * Parameter scale
 *    0     +- 2g
 *    1     +- 4g
 *    2     +- 8g
 *    3     +- 16g
 * 
 */
void MyMPU6050::setFullScaleAccelRange(uint8_t scale)
{
  // full scale is bits 4:3
  writeBits(MPU6050_RA_ACCEL_CONFIG, 4, 2, scale);
  
  #ifdef SERIAL_DEBUG
    uint8_t b;
    b = readByte(MPU6050_RA_ACCEL_CONFIG);

    // full scale is bits 4:3
    uint8_t mask = 0x18;   // b0001 1000
    
    Serial.println(F("\nfunction:setFullScaleAccelRange"));
   
    if( (b & mask)>>3 == scale)
    {
      Serial.print(F("Success setting accel scale to "));
      Serial.println(scale, HEX); 
    }
    else
    {
      Serial.print(F("error setting accel scale, expected: "));
      Serial.print(scale);
      Serial.print(F(", regVal: "));
      Serial.println(b,HEX);
    }
  #endif 

}


/* Sets gyroscope full scale range
 *  
 * Parameter scale
 *    0     +- 250 deg/sec
 *    1     +- 500 deg/sec
 *    2     +- 1000 deg/sec
 *    3     +- 2000 deg/sec
 * 
 */
void MyMPU6050::setFullScaleGyroRange(uint8_t scale)
{
  // full scale is bits 4:3
  writeBits(MPU6050_RA_GYRO_CONFIG, 4, 2, scale);
  
  #ifdef SERIAL_DEBUG
    uint8_t b;
    b = readByte(MPU6050_RA_GYRO_CONFIG);

    // full scale is bits 4:3
    uint8_t mask = 0x18;   // b0001 1000
    
    Serial.println(F("\nfunction setFullScaleGyroRange"));
   
    if( (b & mask)>>3 == scale)
    {
      Serial.print(F("Success setting gyro scale to "));
      Serial.println(scale, HEX); 
    }
    else
    {
      Serial.print(F("error setting gyro scale, expected: "));
      Serial.print(scale);
      Serial.print(F(", regVal: "));
      Serial.println(b,HEX);
    }
  #endif 

}


/* Parameter
 *  enabled - false wakes from sleep mode
 *          - true puts device into sleep mode
 *          
 * Register: Power Management 1
 * Bit 6: sleep bit 
 *         1 - device is in low power sleep mode
 *         0 - device is not in sleep mode
 */
void MyMPU6050::setSleepEnabled(bool enabled)
{
  // sleep bit is 6
  writeBits(MPU6050_RA_PWR_MGMT_1, 6, 1, (uint8_t)enabled);

  #ifdef SERIAL_DEBUG
    uint8_t b;
    b = readByte(MPU6050_RA_PWR_MGMT_1);

    // sleep bit is 6
    uint8_t mask = 0x40;   // b0100 0000

    //Serial.println(F("\nfunction: setSleepEnabled"));
    
    if( (b & mask)>>6 == enabled)
    {
     // Serial.print(F("Success setting sleep to "));
     // Serial.println(enabled, HEX); 
    }
    else
    {
      Serial.print(F("error setting sleep bit, expected: "));
      Serial.print(enabled);
      Serial.print(F(", regVal: "));
      Serial.println(b,HEX);
    }
  #endif 
}


/* WHO_AM_I register stores the upper 6 bits of MPU-60X0's
 *  7-bit I2C address. The least significant bit is determined
 *  by the vaue of the AD0 pin, which is not reflected in this
 *  register.
 *  
 * Returns address stored in WHO_AM_I register, 0x68
 * bits 0 and 7 are hard coded to 0 
 */
uint8_t MyMPU6050::testConnection(void)
{
  return readByte((uint8_t)MPU6050_RA_WHO_AM_I);
}


/* Reads accel, temperature and gyro measurement registers.
 *  
 * Parameter
 *  output: sd 
 *    If all measurement bytes are read, the sd is populated with
 *    measurement data.
 *    If not all bytes are read, then no data is stored in sd.
 *  
 * Returns number of bytes read
 */
uint8_t MyMPU6050::readAllData(struct SensorData* sd)
{
  uint8_t buf[MPU6050_ALL_MEASUREMENT_BYTES];
  uint8_t bytesRead;
  bytesRead = readBytes(MPU6050_RA_ACCEL_XOUT_H, buf, (uint8_t)MPU6050_ALL_MEASUREMENT_BYTES);
  if(bytesRead == (uint8_t)MPU6050_ALL_MEASUREMENT_BYTES)
  {
    // first byte read from high order register, second byte from low
    // form 16 bit value from 8 bit values
    sd->accelX = (((int16_t)buf[0]) << 8) | buf[1];
    sd->accelY = (((int16_t)buf[2]) << 8) | buf[3];
    sd->accelZ = (((int16_t)buf[4]) << 8) | buf[5];
    sd->temperature = (((int16_t)buf[6]) << 8) | buf[7];
    sd->gyroX = (((int16_t)buf[8]) << 8) | buf[9];
    sd->gyroY = (((int16_t)buf[10]) << 8) | buf[11];
    sd->gyroZ = (((int16_t)buf[12]) << 8) | buf[13];
  }

  return bytesRead;
}


/* Reads one byte from parameter-specified register address
 * 
 * regAddr - register address to read 
 *
 * Returns byte read
 */
uint8_t MyMPU6050::readByte(uint8_t regAddr)const
{
  uint8_t data;
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  Wire.endTransmission(false);            // send TX buffer, send restart to keep connection alive
  Wire.requestFrom(devAddr, (uint8_t)1);  // request 1 byte, default true sends stop message after request,
                                          // releasing i2c bus
  data = Wire.read();
  return data;
}


/* Reads bytes from register and stores them in buf.
 * 
 * Parameters
 *  regAddr - register address to read
 *  buf - storage for bytes read
 *  count - number of bytes to read
 * 
 * Returns number of bytes read
 */
uint8_t MyMPU6050::readBytes(uint8_t regAddr, uint8_t *buf, uint8_t count)
{
  uint8_t i = 0;
  
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  Wire.endTransmission(false);      // send TX buffer, send restart to keep connection alive
  Wire.requestFrom(devAddr, count);
 
  while(Wire.available() && i < count)
  {
    buf[i] = Wire.read();
    ++i;
  }
  
  return i;
}

/* Write multiple bits to an 8-bit device register
*
* regAddr - register address
* leftBit - leftmost bit to write (7-0)
* numBits - number of bits to write
* data - right-aligned value to write
*/
void MyMPU6050::writeBits(uint8_t regAddr, uint8_t leftBit, uint8_t numBits, uint8_t data)
{
  // 
  // 76543210 bit positions
  //      xx  arguments: leftBit = 4, length = 2
  // 00000110 bit mask 

  uint8_t regVal, mask;

  // get current register values
  regVal = readByte(regAddr);
  
  
  // shifts 1 to bit position length value
  // subtracting 1 ensures all bits to the right are 1's
  // and all other bits are 0's 
  // Example: length = 4
  // 1 << numBits produces 0001 0000 
  // (1 << numBits) - 1 produces 0000 1111 
  mask =  (1 << numBits) - 1;
  
  // shift the 1's to the left and zero fill
  // Example: leftBit is 5
  // 5 - 4 + 1 = 2, 0000 1111 << 2 becomes 0011 1100
  // Now have a bit mask of length 4 with leftmost bit position 5 
  mask = mask << (leftBit - numBits + 1);

  // shift data to correct position
  // example: data is 3, 0000 0011
  // shifting 5-4+1 = 2 to the left produces 0000 1100
  data = data << (leftBit - numBits + 1);

  // zero bits of interest in existing register value 
  regVal &= ~(mask); 
  // update register bits with data value 
  regVal |= data;

  writeByte(regAddr, regVal);
}



/*
 * Writes one byte to specified register address.
 * 
 * regAddr - register address
 * data    - byte to write
 */
void MyMPU6050::writeByte(uint8_t regAddr, uint8_t data)
{
  Wire.beginTransmission(devAddr);     
  Wire.write(regAddr);              
  Wire.write(data);
  Wire.endTransmission(true);       // transmits bytes that were queued by write
                                    // sends stop message after transmission, releases i2c bus
}




/*
 * finds the mean value for each sensor data output. 
 * 
 * 
 */
void MyMPU6050::mean_calc(struct SensorData* sd, struct mean_values* mv, struct offset_values* ov){
 
  int i = 0;
  long buff_ax=0, buff_ay=0, buff_az=0, buff_gx=0, buff_gy=0, buff_gz=0;




  while (i<(buffersize+101)){
      

    if((i>100) && (i<=(buffersize+100))){

      
      buff_ax = buff_ax + (sd->accelX) + (ov->ax_offset);
      buff_ay = buff_ay + (sd->accelY) + (ov->ay_offset);
      buff_az = buff_az + (sd->accelZ) + (ov->az_offset);
      buff_gx = buff_gx + (sd->gyroX) + (ov->gx_offset);
      buff_gy = buff_gy + (sd->gyroY) + (ov->gy_offset);
      buff_gz = buff_gz + (sd->gyroZ) + (ov->gz_offset);
      
       
    }

    //int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz
    if(i==(buffersize+100)){
      mv->mean_ax = (buff_ax/buffersize);
      mv->mean_ay = (buff_ay/buffersize);
      mv->mean_az = buff_az/buffersize;
      mv->mean_gx = buff_gx/buffersize;
      mv->mean_gy = buff_gy/buffersize;
      mv->mean_gz = buff_gz/buffersize;

     

      
    }
    i++;
    delay(2); //needed to avoid repeated values
  }
  
}


void MyMPU6050::calibration(struct SensorData* sd, struct offset_values* ov, struct mean_values* mv){
  ov->ax_offset = -(mv->mean_ax)/8;
  ov->ay_offset = -(mv->mean_ay)/8;
  ov->az_offset = (16384 -(mv->mean_az))/8;

  ov->gx_offset = -(mv->mean_gx)/4;
  ov->gy_offset = -(mv->mean_gy)/4;
  ov->gz_offset = -(mv->mean_gz)/4;



  int loop_iteration = 0;
  while(1){
    
    int ready=0;
    mean_calc(sd, mv, ov);

    if (abs(mv->mean_ax)<=acel_deadzone) { ready++;}
    else ov->ax_offset = ov->ax_offset - mv->mean_ax/acel_deadzone; //ax offset = ax offset - 

    
    if (abs(mv->mean_ay)<=acel_deadzone){  ready++;}
    else ov->ay_offset = ov->ay_offset - mv->mean_ay/acel_deadzone;
    
    
    if (abs(16384-mv->mean_az)<=acel_deadzone) {ready++;}
    else ov->az_offset = ov->az_offset + (16384-mv->mean_az)/acel_deadzone;


    if (abs(mv->mean_gx)<=giro_deadzone) { ready++;}
    else ov->gx_offset = ov->gx_offset - mv->mean_gx/(giro_deadzone+1);


    if (abs(mv->mean_gy)<=giro_deadzone){  ready++;}
    else ov->gy_offset = ov->gy_offset - mv->mean_gy/(giro_deadzone+1);


    if (abs(mv->mean_gz)<=giro_deadzone){ ready++;}
    else ov->gz_offset = ov->gz_offset - mv->mean_gz/(giro_deadzone+1);

    #ifdef DEBUG 
    Serial.print("Mean Values:   ");
    Serial.print(" \tax = ");
    Serial.print(mv->mean_ax);
    Serial.print(" \tay = ");
    Serial.print(mv->mean_ay);
    Serial.print(" \taz = ");
    Serial.print(mv->mean_az);
    Serial.print(" \tgx = ");
    Serial.print(mv->mean_gx);
    Serial.print(" \tgy = ");
    Serial.print(mv->mean_gy);
    Serial.print(" \t\tgz = ");
    Serial.println(mv->mean_gz);

    Serial.print("Offset Values: ");
    Serial.print(" \tax = ");
    Serial.print(ov->ax_offset);
    Serial.print(" \tay = ");
    Serial.print(ov->ay_offset);
    Serial.print(" \taz = ");
    Serial.print(ov->az_offset);
    Serial.print(" \tgx = ");
    Serial.print(ov->gx_offset);
    Serial.print(" \tgy = ");
    Serial.print(ov->gy_offset);
    Serial.print(" \tgz = ");
    Serial.println(ov->gz_offset);

    loop_iteration+=1 ;
    Serial.print("loop # = ");
    Serial.println(loop_iteration);
    Serial.println(" ");
    Serial.println(" ");
    #endif 
    

    
    if (ready==6) break;
  }

}
