//  Copyright (c) 2014 Justin Eskesen
//
//  This file is part of i2c_imu
//
//  i2c_imu is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  i2c_imu is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with i2c_imu.  If not, see <http://www.gnu.org/licenses/>.
//

#include <exception>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <angles/angles.h>

#include "RTIMULib.h"
#include "RTIMUSettings.h"

#define G_2_MPSS 9.80665
#define uT_2_T 1000000

class I2cImu
{
public:
	I2cImu();

	void update();

private:
	//ROS Stuff
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	// sensor msg topic output
	sensor_msgs::Imu imu_msg;

	tf::TransformBroadcaster tf_broadcaster_;

	ros::Publisher imu_pub_;
	ros::Publisher magnetometer_pub_;
	ros::Publisher euler_pub_;

	std::string imu_frame_id_;

	double rate_;
	ros::Time last_update_;
	double declination_radians_;

	//RTUIMULib stuff
	RTIMU *imu_;

	RTIMUSettings imu_settings_;
};

I2cImu::I2cImu() :
		nh_(), private_nh_("~")
{
	if(private_nh_.hasParam("settings_directory")) {
		std::string settings_directory;
		private_nh_.getParam("settings_directory", settings_directory);
		imu_settings_ = RTIMUSettings(settings_directory.c_str(), "RTIMULib");
	} else {
		throw std::runtime_error("parameter settings_directory not set");
	}


	// do all the ros parameter reading & pulbishing
	private_nh_.param<std::string>("frame_id", imu_frame_id_, "imu_link");

	imu_pub_ = nh_.advertise<sensor_msgs::Imu>("data",10);

	bool magnetometer;
	private_nh_.param("publish_magnetometer", magnetometer, false);
	if (magnetometer)
	{
		magnetometer_pub_ = nh_.advertise<sensor_msgs::MagneticField>("mag", 10, false);
	}

	bool euler;
	private_nh_.param("publish_euler", euler, false);
	if (euler)
	{
		euler_pub_ = nh_.advertise<geometry_msgs::Vector3>("euler", 10, false);
	}

	std::vector<double> orientation_covariance, angular_velocity_covariance, linear_acceleration_covariance;
	if (private_nh_.getParam("orientation_covariance", orientation_covariance) && orientation_covariance.size() == 9)
	{
		for(int i=0; i<9; i++)
		{
			imu_msg.orientation_covariance[i]=orientation_covariance[i];
		}
	}

	if (private_nh_.getParam("angular_velocity_covariance", angular_velocity_covariance) && angular_velocity_covariance.size() == 9)
	{
		for(int i=0; i<9; i++)
		{
			imu_msg.angular_velocity_covariance[i]=angular_velocity_covariance[i];
		}
	}

	if (private_nh_.getParam("linear_acceleration_covariance", linear_acceleration_covariance) && linear_acceleration_covariance.size() == 9)
	{
		for(int i=0; i<9; i++)
		{
			imu_msg.linear_acceleration_covariance[i]=linear_acceleration_covariance[i];
		}
	}

	imu_settings_.loadSettings();

	private_nh_.param("magnetic_declination", declination_radians_, 0.0);

	// now set up the IMU

	imu_ = RTIMU::createIMU(&imu_settings_);
	if (imu_ == NULL)
	{
		ROS_FATAL("I2cImu - %s - Failed to open the i2c device", __FUNCTION__);
		ROS_BREAK();
	}

	if (!imu_->IMUInit())
	{
		ROS_FATAL("I2cImu - %s - Failed to init the IMU", __FUNCTION__);
		ROS_BREAK();
	}

	imu_->setSlerpPower(0.02);
	imu_->setGyroEnable(true);
	imu_->setAccelEnable(true);
	imu_->setCompassEnable(true);

	private_nh_.param<double>("rate_hz", rate_, 1.0 / (imu_->IMUGetPollInterval() / 1000.0));
}

void I2cImu::update()
{
	ros::Rate r(rate_);

	while (imu_->IMURead() && ros::ok())
	{
		RTIMU_DATA imuData = imu_->getIMUData();

		ros::Time current_time = ros::Time::now();


		imu_msg.header.stamp = current_time;
		imu_msg.header.frame_id = imu_frame_id_;
		// NED --> ENU coordinate frame conversion: Swap X->Y, invert Z
		imu_msg.orientation.x = imuData.fusionQPose.y();
		imu_msg.orientation.y = imuData.fusionQPose.x();
		imu_msg.orientation.z = -imuData.fusionQPose.z();
		imu_msg.orientation.w = imuData.fusionQPose.scalar();

		imu_msg.angular_velocity.x = imuData.gyro.x();
		imu_msg.angular_velocity.y = imuData.gyro.y();
		imu_msg.angular_velocity.z = imuData.gyro.z();

		imu_msg.linear_acceleration.x = imuData.accel.x() * G_2_MPSS;
		imu_msg.linear_acceleration.y = imuData.accel.y() * G_2_MPSS;
		imu_msg.linear_acceleration.z = imuData.accel.z() * G_2_MPSS;

		imu_pub_.publish(imu_msg);

		if (magnetometer_pub_ != NULL && imuData.compassValid)
		{
			sensor_msgs::MagneticField msg;

			msg.header.frame_id=imu_frame_id_;
			msg.header.stamp=ros::Time::now();

			msg.magnetic_field.x = imuData.compass.x()/uT_2_T;
			msg.magnetic_field.y = imuData.compass.y()/uT_2_T;
			msg.magnetic_field.z = imuData.compass.z()/uT_2_T;

			magnetometer_pub_.publish(msg);
		}

		if (euler_pub_ != NULL)
		{
			geometry_msgs::Vector3 msg;
			msg.x = imuData.fusionPose.x();
			msg.y = imuData.fusionPose.y();
			msg.z = -imuData.fusionPose.z();
			msg.z = (-imuData.fusionPose.z()) - declination_radians_;
			euler_pub_.publish(msg);
		}
		ros::spinOnce();
		r.sleep();
	}

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "i2c_imu_node");

	ROS_INFO("RTIMU Node for ROS");

	I2cImu i2c_imu;
	i2c_imu.update();

	return (0);
}
// EOF
