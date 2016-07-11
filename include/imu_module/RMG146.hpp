//For acquiring RMG146 data
//Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#pragma once
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <math.h>
#include <limits>
#define ACC_ADDRESS (0x30 >> 1)
#define MAG_ADDRESS (0x3C >> 1)
#define GYR_ADDRESS (0xD0 >> 1)

using namespace std;
struct Vector3D
{
	double x,y,z;
};
class RMG146
{
private:
	int fd;
	char fName[20];
	int errorCounter;
	Vector3D m_min,m_max;
	Vector3D gyro_bias;
	bool inited;
	bool read1b(uint8_t add,uint8_t reg,uint8_t &data);
	bool readNb(uint8_t add,uint8_t reg,uint8_t *data,uint8_t size);
	bool write1b(uint8_t add,uint8_t reg,uint8_t val);
	bool initRMG146();
	void vector_cross(const Vector3D *a,const Vector3D *b, Vector3D *out);
	float vector_dot(const Vector3D *a,const Vector3D *b);
	void vector_normalize(Vector3D *a);
public:
	RMG146(int devNumber=2);
	void MagnetCalibXY(Vector3D magnet);
	void MagnetCalibZ(Vector3D magnet);
	bool ReadRawTemp(int &res);
	bool ReadTemp(int &res);
	bool ReadRawAccel(Vector3D &res);
	bool ReadAccel(Vector3D &res);
	bool ReadRawGyro(Vector3D &res);
	bool ReadRawMagnet(Vector3D &res);
	bool ReadGyro(Vector3D &res);
	bool ReadMagnet(Vector3D &res);
	int  GetHeading(Vector3D from,Vector3D accel,Vector3D magnet);
	void PrintMagnetCalib();
	void GyroCalib(Vector3D gyro);
	virtual ~RMG146();
};
