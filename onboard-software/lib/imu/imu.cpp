#include "imu.h"
#include "util.h"

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
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    // Initialize offsets (might have to tweak these)
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    if (devStatus == 0)
    {
        calibrate();
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