//For read RMG146
//Author : Hafez Farazi <Farazi@ais.uni-bonn.de>

#include <imu_module/main.hpp>
using namespace boost::timer;
ros::NodeHandle *nodeHandle;
bool calibXY,calibZ,calibGyro;
ros::Timer timer;
RMG146 rmg(1);
void QuaternionToEuler(float q0,float q1,float q2,float q3, Vector3D &euler)
{
    float w,x,y,z;

    w = q0;
    x = q1;
    y = q2;
    z = q3;

    double sqw = w*w;    
    double sqx = x*x;    
    double sqy = y*y;    
    double sqz = z*z; 

    euler.z = (float) (atan2(2.0 * (x*y + z*w),(sqx - sqy - sqz + sqw)) * (180.0f/M_PI));
    euler.x = (float) (atan2(2.0 * (y*z + x*w),(-sqx - sqy + sqz + sqw)) * (180.0f/M_PI));          
    euler.y = (float) (asin(-2.0 * (x*z - y*w)) * (180.0f/M_PI));

}
void finishCalib(const ros::TimerEvent&)
{
	calibZ=calibXY=false;
	timer=ros::Timer();
	rmg.PrintMagnetCalib();	
	ROS_INFO("finishCalib triggered");
}

bool CalibrateXY(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if(calibZ|| calibGyro)
	{
		return false;
	}
	cout<<"CalibXY Started " <<endl;
	timer = nodeHandle->createTimer(ros::Duration(30),finishCalib);
	calibXY=true;
	return true;
}

bool CalibrateGyro(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if(calibXY || calibZ)
	{
		return false;
	}
	cout<<"CalibGyro Started" <<endl;
	timer = nodeHandle->createTimer(ros::Duration(5),finishCalib);
	calibGyro=true;
	return true;
}

bool CalibrateZ(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if(calibXY||calibGyro)
	{
		return false;
	}
	cout<<"CalibZ Started " <<endl;
	timer = nodeHandle->createTimer(ros::Duration(30),finishCalib);
	calibZ=true;
	return true;
}



int main(int argc, char **argv)
{
	const int LOOPRATE=sampleFreq;
	const int TEMPRATE =LOOPRATE/1;
	const int ACCRATE=LOOPRATE/400;
	const int GYRORATE=LOOPRATE/LOOPRATE;
	const int MAGRATE=LOOPRATE/30;
  	
	ros::init(argc, argv, "imu_module");
	nodeHandle=new ros::NodeHandle();
	ros::Rate loop_rate(LOOPRATE);
	ros::Publisher IMU_Temp_pub = nodeHandle->advertise<std_msgs::Int32>("IMU/temperture", 10);
	ros::Publisher IMU_Data_pub = nodeHandle->advertise<sensor_msgs::Imu>("IMU/data", 10);
	ros::Publisher IMU_Mag_pub = nodeHandle->advertise<geometry_msgs::Vector3Stamped>("IMU/magnet", 10);
	ros::Publisher IMU_AHRS_pub = nodeHandle->advertise<geometry_msgs::PoseStamped>("IMU/AHRS", 10);
	ros::Publisher IMU_Euler_pub = nodeHandle->advertise<geometry_msgs::Vector3Stamped>("IMU/euler", 10);
	ros::Publisher IMU_Cube_pub = nodeHandle->advertise<visualization_msgs::Marker>("IMU/cube", 10);
	ros::ServiceServer calibXY_srv = nodeHandle->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("IMU/calibrateXY", CalibrateXY);
	ros::ServiceServer calibZ_srv = nodeHandle->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("IMU/calibrateZ", CalibrateZ);
	ros::ServiceServer calibGyro_srv = nodeHandle->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("IMU/calibrateGyro", CalibrateGyro);
	cout<<"imu_module Started!"<<endl;
	double fpsData=LOOPRATE;
	int lCounter=0;
	sensor_msgs::Imu msg = sensor_msgs::Imu();
	geometry_msgs::Vector3Stamped msgMag=  geometry_msgs::Vector3Stamped();
	geometry_msgs::PoseStamped msgAHRS=geometry_msgs::PoseStamped();
	geometry_msgs::Vector3Stamped msgEuler=geometry_msgs::Vector3Stamped();
	visualization_msgs::Marker msgMarker;
	msgMarker.id=0;
	msgMarker.type=1;//CUBE
	msgMarker.action=0;//ADD
	msgMarker.scale.x=3;
	msgMarker.scale.y=2;
	msgMarker.scale.z=1;
	msgMarker.color.r=1;
	msgMarker.color.b=0;
	msgMarker.color.g=0;
	msgMarker.color.a=1;
	msgMarker.lifetime=ros::Duration(0.2);
	msgMarker.frame_locked=false; 
	msgMarker.header.frame_id="map";
	msgEuler.header.frame_id="map";
	msgAHRS.header.frame_id="map";
	msgMag.header.frame_id="map";
	msg.header.frame_id="map";
	Vector3D accRes,magRes,gyroRes;
	while(ros::ok())
	{	 
		cpu_timer timer;
		int tempRes;
		if(((lCounter%TEMPRATE)==0)&&rmg.ReadTemp(tempRes))
		{
			std_msgs::Int32 msg;
			msg.data =tempRes;
			IMU_Temp_pub.publish(msg);
			ROS_INFO_THROTTLE(1, "Temp ={%d}",tempRes);
		}
		msg.header.stamp=ros::Time::now();
		msgMag.header.stamp=ros::Time::now();
		msgAHRS.header.stamp=ros::Time::now();
		msgEuler.header.stamp=ros::Time::now();
		msgMarker.header.stamp=ros::Time::now();
		if(((lCounter%ACCRATE)==0)&&rmg.ReadAccel(accRes))
		{	
			msg.linear_acceleration.x=accRes.x;
			msg.linear_acceleration.y=accRes.y;
			msg.linear_acceleration.z=accRes.z;
			ROS_INFO_THROTTLE(1, "Accel ={%.1lf, %.1lf, %.1lf}",accRes.x,accRes.y,accRes.z);
		}
		if(((lCounter%GYRORATE)==0)&&rmg.ReadGyro(gyroRes))
                {
			if(calibGyro)
			{
				rmg.GyroCalib(gyroRes);
			}
			msg.angular_velocity.x=gyroRes.x;
			msg.angular_velocity.y=gyroRes.y;
			msg.angular_velocity.z=gyroRes.z;
			ROS_INFO_THROTTLE(1, "Gyro ={%.1lf, %.1lf, %.1lf}",gyroRes.x,gyroRes.y,gyroRes.z);
                }
		if(((lCounter%MAGRATE)==0)&&rmg.ReadMagnet(magRes))
                {		
			if(calibXY)
			{
				rmg.MagnetCalibXY(magRes);
			}
			if(calibZ)
			{
				rmg.MagnetCalibZ(magRes);
			}
			msgMag.vector.x=magRes.x;
			msgMag.vector.y=magRes.y;
			msgMag.vector.z=magRes.z;
			ROS_INFO_THROTTLE(1, "Magnet ={%.1lf, %.1lf, %.1lf}",magRes.x,magRes.y,magRes.z);
                }
		MadgwickAHRSupdate(gyroRes.x,gyroRes.y,gyroRes.z, accRes.x,accRes.y,accRes.z,magRes.x,magRes.y,magRes.z);
		msgAHRS.pose.orientation.x=q1;
		msgAHRS.pose.orientation.y=q2;
		msgAHRS.pose.orientation.z=q3;
		msgAHRS.pose.orientation.w=q0;
		msgAHRS.pose.position.x=0;
		msgAHRS.pose.position.y=0;
		msgAHRS.pose.position.z=0;
		msgMarker.pose.orientation.x=q1;
		msgMarker.pose.orientation.y=q2;
		msgMarker.pose.orientation.z=q3;
		msgMarker.pose.orientation.w=q0;
		msgMarker.pose.position.x=0;
		msgMarker.pose.position.y=0;
		msgMarker.pose.position.z=0;
		msg.orientation=msgAHRS.pose.orientation;
		Vector3D euler;
		QuaternionToEuler(q0,q1,q2,q3, euler);
		msgEuler.vector.x=euler.x;
		msgEuler.vector.y=euler.y;
		msgEuler.vector.z=euler.z;
		IMU_Data_pub.publish(msg);
		IMU_Mag_pub.publish(msgMag);
		IMU_AHRS_pub.publish(msgAHRS);
		IMU_Euler_pub.publish(msgEuler);
		IMU_Cube_pub.publish(msgMarker);
		ros::spinOnce();
		loop_rate.sleep();
		fpsData=(0.99*fpsData)+(0.01*(1000000000l/timer.elapsed().wall));
		ROS_INFO_THROTTLE(1, "loop rate =  %.2lf Hz",fpsData);
		lCounter++;
  	}
	cout<<"IMU Finished"<<endl; 
  	return 0;
}
