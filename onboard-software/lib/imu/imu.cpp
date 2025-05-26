#include "imu.h"
#include "util.h"
#include <EEPROM.h>
#define EEPROM_SIZE 32
#define ADDR_CALIB_FLAG 0
#define ADDR_X_GYRO_OFFSET 1 // each int16_t takes 2 bytes
#define ADDR_Y_GYRO_OFFSET 3
#define ADDR_Z_GYRO_OFFSET 5
#define ADDR_X_ACCEL_OFFSET 7
#define ADDR_Y_ACCEL_OFFSET 9
#define ADDR_Z_ACCEL_OFFSET 11
IMU::IMU()
{
    dmpReady = false;
    dataReady = false;
    oldYPR[0] = 0.0;
    oldYPR[1] = 0.0;
    oldYPR[2] = 0.0;
}

IMU::~IMU()
{
}

void IMU::init()
{
    EEPROM.begin(EEPROM_SIZE);

    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0)
    {
        // only calibrate once & store the results in EEPROM
        if (EEPROM.read(ADDR_CALIB_FLAG) != 1)
        {
            Serial.println("Calibrating...");
            // very first boot: run the slow routines
            calibrate();

            // grab offsets out of the MPU registers
            int16_t xgo = mpu.getXGyroOffset();
            int16_t ygo = mpu.getYGyroOffset();
            int16_t zgo = mpu.getZGyroOffset();
            int16_t xao = mpu.getXAccelOffset();
            int16_t yao = mpu.getYAccelOffset();
            int16_t zao = mpu.getZAccelOffset();

            // write them to EEPROM
            EEPROM.put(ADDR_X_GYRO_OFFSET, xgo);
            EEPROM.put(ADDR_Y_GYRO_OFFSET, ygo);
            EEPROM.put(ADDR_Z_GYRO_OFFSET, zgo);
            EEPROM.put(ADDR_X_ACCEL_OFFSET, xao);
            EEPROM.put(ADDR_Y_ACCEL_OFFSET, yao);
            EEPROM.put(ADDR_Z_ACCEL_OFFSET, zao);
            EEPROM.write(ADDR_CALIB_FLAG, 1);
            EEPROM.commit();
        }
        else
        {
            Serial.println("Loading offsets from memory");
            // subsequent boots: load & apply the stored offsets
            int16_t xgo, ygo, zgo, xao, yao, zao;
            EEPROM.get(ADDR_X_GYRO_OFFSET, xgo);
            EEPROM.get(ADDR_Y_GYRO_OFFSET, ygo);
            EEPROM.get(ADDR_Z_GYRO_OFFSET, zgo);
            EEPROM.get(ADDR_X_ACCEL_OFFSET, xao);
            EEPROM.get(ADDR_Y_ACCEL_OFFSET, yao);
            EEPROM.get(ADDR_Z_ACCEL_OFFSET, zao);

            mpu.setXGyroOffset(xgo);
            mpu.setYGyroOffset(ygo);
            mpu.setZGyroOffset(zgo);
            mpu.setXAccelOffset(xao);
            mpu.setYAccelOffset(yao);
            mpu.setZAccelOffset(zao);

            mpu.setDMPEnabled(true);
            dmpReady = true;
            packetSize = mpu.dmpGetFIFOPacketSize();
        }
    }
}

bool IMU::is_ready()
{
    return dmpReady;
}

void IMU::calibrate()
{
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
}

void IMU::calculateAngles()
{
    dataReady = mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
    if (dataReady && is_ready())
    { // Get the Latest packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
}

float IMU::get_angle(OrientationDimension dimension)
{
    if (dataReady && is_ready())
    {
        oldYPR[dimension] = degToRad(ypr[dimension]);
    }
    return oldYPR[dimension];
}

float IMU::get_yaw()
{
    return get_angle(YAW);
}

float IMU::get_pitch()
{
    return get_angle(PITCH);
}

float IMU::get_roll()
{
    return get_angle(ROLL);
}