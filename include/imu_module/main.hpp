#pragma once
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <sstream>
#include <boost/timer/timer.hpp>
#include <inttypes.h>
#include <time.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <imu_module/RMG146.hpp>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <imu_module/MadgwickAHRS.hpp>
using namespace std;
int main(int argc, char **argv);
bool CalibrateXY(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
bool CalibrateZ(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
