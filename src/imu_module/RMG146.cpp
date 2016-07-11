//For acquiring data from RMG146 
//Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#include <imu_module/RMG146.hpp>
RMG146::RMG146(int devNum)
{
	errorCounter=0;
        inited=false;
	fd=0;
        m_min.x=m_min.y=m_min.z=std::numeric_limits<int16_t>::max();
        m_max.x=m_max.y=m_max.z=std::numeric_limits<int16_t>::min();
        sprintf(fName,"/dev/i2c-%d",devNum);
	if(!initRMG146())
	{
		perror("Error Initializing IMU");
	}
}

RMG146::~RMG146()
{
	if(inited)
	{
		close(fd);
	}
}

bool RMG146::initRMG146()
{
	if(fd>0)
	{
		close(fd);
	}
        fd=open(fName,O_RDWR);
        errorCounter=0;
        inited=false;
        if(fd<=0)
        {
		return false;
        }
        else
        {
                inited=true;
        }
        bool res = true;
        res&= (write1b(GYR_ADDRESS,0x16,0x18) == 1); // 8kHz, ±2000°/sec
        res&= (write1b(ACC_ADDRESS,0x20,0x37) == 1); // 
        res&= (write1b(MAG_ADDRESS,0x00,0x18) == 1); // 30Hz
        res&= (write1b(MAG_ADDRESS,0x01,0x60) == 1); // gain = +-2.5
        res&= (write1b(MAG_ADDRESS,0x02,0x00) == 1); // continue-measureture mode: 0x00  
        return res;
}

void RMG146::vector_cross(const Vector3D *a,const Vector3D *b, Vector3D *out)
{
	out->x = a->y*b->z - a->z*b->y;
	out->y = a->z*b->x - a->x*b->z;
	out->z = a->x*b->y - a->y*b->x;
}

float RMG146::vector_dot(const Vector3D *a,const Vector3D *b)
{
	return a->x*b->x+a->y*b->y+a->z*b->z;
}

void RMG146::vector_normalize(Vector3D *a)
{
	float coef = sqrt(vector_dot(a,a));
	a->x /= coef;
	a->y /= coef;
	a->z /= coef;
}


void RMG146::MagnetCalibXY(Vector3D magnet)
{
	m_max.x=max(m_max.x,magnet.x);
        m_max.y=max(m_max.y,magnet.y);
        m_min.x=min(m_min.x,magnet.x);
        m_min.y=min(m_min.y,magnet.y);

}

void RMG146::MagnetCalibZ(Vector3D magnet)
{
	
	m_max.z=max(m_max.z,magnet.z);
	m_min.z=min(m_min.z,magnet.z);
}

void RMG146::GyroCalib(Vector3D gyro)
{
	gyro_bias.x=(0.99*gyro_bias.x)+(0.01*gyro.x);
	gyro_bias.y=(0.99*gyro_bias.y)+(0.01*gyro.y);
	gyro_bias.z=(0.99*gyro_bias.z)+(0.01*gyro.z);
}

void RMG146::PrintMagnetCalib()
{
	printf("Min {%lf,%lf,%lf} \r\n",m_min.x,m_min.y,m_min.z);
	printf("Max {%lf,%lf,%lf} \r\n",m_max.x,m_max.y,m_max.z);
}

int RMG146::GetHeading(Vector3D from,Vector3D accel,Vector3D magnet)
{
    // shift and scale
    magnet.x = (magnet.x - m_min.x) / (m_max.x - m_min.x) * 2 - 1.0;
    magnet.y = (magnet.y - m_min.y) / (m_max.y - m_min.y) * 2 - 1.0;
    magnet.z = (magnet.z - m_min.z) / (m_max.z - m_min.z) * 2 - 1.0;

    // normalize
    vector_normalize(&accel);

    // compute E and N
    Vector3D E;
    Vector3D N;
    vector_cross(&magnet, &accel, &E);
    vector_normalize(&E);
    vector_cross(&accel, &E, &N);
	
    // compute heading
    int heading = round(atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / M_PI);
    if (heading < 0) heading += 360;
	return heading;
}

bool RMG146::write1b(uint8_t add,uint8_t reg,uint8_t val)
{
        unsigned char buff[2];
        int retVal = -1;
        struct i2c_rdwr_ioctl_data packets;
        struct i2c_msg messages[1];

        buff[0] = reg;
        buff[1] = val;

        messages[0].addr = add;
        messages[0].flags = 0;
        messages[0].len = sizeof(buff);
        messages[0].buf = buff;

        packets.msgs = messages;
        packets.nmsgs = 1;

        retVal = ioctl(fd, I2C_RDWR, &packets);
        if(retVal < 0)
        {
                perror("Write to I2C Device failed");
                return false;
        }
        return (retVal==1);
}

bool RMG146::read1b(uint8_t add,uint8_t reg,uint8_t &data)
{
        unsigned char *inbuff, outbuff;
        int retVal = -1;
        struct i2c_rdwr_ioctl_data packets;
        struct i2c_msg messages[2];

        outbuff = reg;
        messages[0].addr = add;
        messages[0].flags= 0;
        messages[0].len = sizeof(outbuff);
        messages[0].buf = &outbuff;

        inbuff = &data;
        messages[1].addr = add;
        messages[1].flags = I2C_M_RD;
        messages[1].len = 1;
        messages[1].buf = inbuff;

        packets.msgs = messages;
        packets.nmsgs = 2;

        retVal = ioctl(fd, I2C_RDWR, &packets);
        if(retVal < 0)
        {
		errorCounter++;
                if(errorCounter>10)
                {
                        errorCounter=0;
                        initRMG146();
                }

                perror("Read from I2C Device failed");
                return false;
        }
        return true;
}


bool RMG146::readNb(uint8_t add,uint8_t reg,uint8_t *data,uint8_t size)
{
        uint8_t outbuff;
        int retVal = -1;
        struct i2c_rdwr_ioctl_data packets;
        struct i2c_msg messages[2];

        outbuff = reg;
        messages[0].addr = add;
        messages[0].flags= 0;
        messages[0].len = 1;
        messages[0].buf = &outbuff;

       
        messages[1].addr = add;
        messages[1].flags = I2C_M_RD;
        messages[1].len = size;
        messages[1].buf = data;

        packets.msgs = messages;
        packets.nmsgs = 2;

        retVal = ioctl(fd, I2C_RDWR, &packets);
        if(retVal < 0)
        {	
		errorCounter++;
		if(errorCounter>10)
		{
			errorCounter=0;
			initRMG146();
		}
                perror("Read from I2C Device failed");
                return false;
        }
        return true;
}

bool RMG146::ReadTemp(int &res)
{
        int temp_r;
        bool ret=ReadRawTemp(temp_r);
        if(ret)
	{
		int16_t tmp;
        	// convert it to degrees
        	// subtract room temperature offset
        	tmp = temp_r + 13200;
        	// convert to degrees
        	res = (double)tmp / 280;
        	// add room offset
        	res = res + 35;
	}
        return ret;
}

bool RMG146::ReadRawTemp(int &res)
{
        uint8_t d[2];
        bool ret=readNb(GYR_ADDRESS,0x1b,d,2);
	if(ret)
	{
        	res=(d[0]<<8) | (d[1] & 0xff);
	}
        return ret;
}

bool RMG146::ReadRawAccel(Vector3D &res)
{
     	uint8_t d[6];//Atthention for the byte order!
        bool ret=readNb(ACC_ADDRESS,0xa8,d,6);
	if(ret)
	{
        	res.x=  (float)((int16_t)(((int16_t) d[1]) << 8) | ((int16_t) d[0])>>4);
       		res.y=  (float)((int16_t)(((int16_t) d[3]) << 8) | ((int16_t) d[2])>>4);
        	res.z=  (float)((int16_t)(((int16_t) d[5]) << 8) | ((int16_t) d[4])>>4);
	}
        return ret;
}
bool RMG146::ReadAccel(Vector3D &res)
{
	return ReadRawAccel(res);
}

//The reslut is in Radian
bool RMG146::ReadRawGyro(Vector3D &res)
{
        double divisor=16.4;
	uint8_t d[6];
        bool ret=readNb(GYR_ADDRESS,0x1d,d,6);
	if(ret)
	{
        	res.x=  (float)((int16_t)(((int16_t) d[0]) << 8) | ((int16_t) d[1]));
        	res.y=  (float)((int16_t)(((int16_t) d[2]) << 8) | ((int16_t) d[3]));
        	res.z=  (float)((int16_t)(((int16_t) d[4]) << 8) | ((int16_t) d[5]));
        
        	res.x=(res.x/divisor) * M_PI / 180.;
        	res.y=(res.y/divisor) * M_PI / 180.;
        	res.z=(res.z/divisor) * M_PI / 180.;
	}
        return ret;
}

bool RMG146::ReadGyro(Vector3D &res)
{
	bool ret=ReadRawGyro(res);
	if(ret)
	{
		res.x-=gyro_bias.x;
		res.y-=gyro_bias.y;
		res.z-=gyro_bias.z;
	}
	return ret;
}

bool RMG146::ReadRawMagnet(Vector3D &res)
{
        uint8_t d[6];
        bool ret=readNb(MAG_ADDRESS,0x03,d,6);
        if(ret)
	{
		res.x=  (float)((int16_t)(((int16_t) d[0]) << 8) | ((int16_t) d[1]));
        	res.y=  (float)((int16_t)(((int16_t) d[2]) << 8) | ((int16_t) d[3]));
        	res.z=  (float)((int16_t)(((int16_t) d[4]) << 8) | ((int16_t) d[5]));
	}
        return ret;
}

bool RMG146::ReadMagnet(Vector3D &res)
{
	return ReadRawMagnet(res);
}
