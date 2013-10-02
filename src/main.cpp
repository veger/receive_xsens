/* Copyright (c) Xsens Technologies B.V., 2006-2012. All rights reserved.

	  This source code is provided under the MT SDK Software License Agreement
and is intended for use only by Xsens Technologies BV and
	   those that have explicit written permission to use it from
	   Xsens Technologies BV.

	  THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
	   KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
	   IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
	   PARTICULAR PURPOSE.
 */

//--------------------------------------------------------------------------------
// ROS driver for Xsens MTi-10 and MTi-100 series
//
//--------------------------------------------------------------------------------
#include <xsensdeviceapi.h> // The Xsens device API header
#include "serialkey.h"

#include <iostream>
#include <list>
#include <iomanip>
#include <stdexcept>

#include <xsens/xstime.h>

#include "conio.h" // for non ANSI _kbhit() and _getch()

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class CallbackHandler : public XsCallback
{
public:
	CallbackHandler(size_t maxBufferSize = 5) : m_maxNumberOfPacketsInBuffer(maxBufferSize), m_numberOfPacketsInBuffer(0)
#ifdef _MSC_VER
	{InitializeCriticalSection(&m_CriticalSection);}
	virtual ~CallbackHandler() throw() {DeleteCriticalSection(&m_CriticalSection);}
#else
	{
	  //create mutex attribute variable
	  pthread_mutexattr_t mAttr;

	  // setup recursive mutex for mutex attribute
	  pthread_mutexattr_settype(&mAttr, PTHREAD_MUTEX_RECURSIVE_NP);

	  // Use the mutex attribute to create the mutex
	  pthread_mutex_init(&m_CriticalSection, &mAttr);

	  // Mutex attribute can be destroy after initializing the mutex variable
	  pthread_mutexattr_destroy(&mAttr);

	}
	virtual ~CallbackHandler() throw() {pthread_mutex_destroy(&m_CriticalSection);}
#endif

	bool packetAvailable() const {Locker lock(*this); return m_numberOfPacketsInBuffer > 0;}
	XsDataPacket getNextPacket()
	{
		assert(packetAvailable());
		Locker lock(*this);
		XsDataPacket oldestPacket(m_packetBuffer.front());
		m_packetBuffer.pop_front();
		--m_numberOfPacketsInBuffer;
		return oldestPacket;
	}

protected:
	virtual void onDataAvailable(XsDevice*, const XsDataPacket* packet)
	{
		Locker lock(*this);
		assert(packet != 0);
		while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
		{
			(void)getNextPacket();
		}
		m_packetBuffer.push_back(*packet);
		++m_numberOfPacketsInBuffer;
		assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
	}
private:
#ifdef _MSC_VER
	mutable CRITICAL_SECTION m_CriticalSection;
#else
	mutable pthread_mutex_t m_CriticalSection;
#endif
	struct Locker
	{
#ifdef _MSC_VER
		Locker(CallbackHandler const & self) : m_self(self) {EnterCriticalSection(&m_self.m_CriticalSection);}
		~Locker() throw() {LeaveCriticalSection(&m_self.m_CriticalSection);}
#else
		Locker(CallbackHandler const & self) : m_self(self) {pthread_mutex_lock(&m_self.m_CriticalSection);}
		~Locker() throw() {pthread_mutex_unlock(&m_self.m_CriticalSection);}
#endif
		CallbackHandler const & m_self;
	};
	size_t m_maxNumberOfPacketsInBuffer;
	size_t m_numberOfPacketsInBuffer;
	std::list<XsDataPacket> m_packetBuffer;
};

//--------------------------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "receive_xsens");
	ros::NodeHandle nh;

  	ros::Publisher publisher = nh.advertise<sensor_msgs::Imu> ("/imu/data", 1);

	if (!setSerialKey())
	{
		std::cout << "Invalid serial key." << std::endl;
		std::cout << "Press [ENTER] to continue." << std::endl; std::cin.get();
		return 1;
	}

	// Create XsControl object
	XsControl* control = XsControl::construct();
	assert(control != 0);

	try
	{
		// Scan for connected devices
		XsPortInfoArray portInfoArray = XsScanner::scanPorts();

		// Find an MTi / MTx / MTmk4 device
		XsPortInfoArray::const_iterator mtPort = portInfoArray.begin();
		while (mtPort != portInfoArray.end() && !mtPort->deviceId().isMtix() && !mtPort->deviceId().isMtMk4()) {++mtPort;}
		if (mtPort == portInfoArray.end())
		{
			throw std::runtime_error("No MTi / MTx / MTmk4 device found. Aborting.");
		}
		std::cout << "Found a device with id: " << mtPort->deviceId().toString().toStdString() << " @ port: " << mtPort->portName().toStdString() << ", baudrate: " << mtPort->baudrate() << std::endl;

		// Open the port with the detected device
		if (!control->openPort(mtPort->portName().toStdString(), mtPort->baudrate()))
		{
			throw std::runtime_error("Could not open port. Aborting.");
		}

		try
		{
			// Get the device object
			XsDevice* device = control->device(mtPort->deviceId());
			assert(device != 0);

			// Print information about detected MTi / MTx / MTmk4 device
			std::cout << "Device: " << device->productCode().toStdString() << " opened." << std::endl;

			// Create and attach callback handler to device
			CallbackHandler callback;
			device->addCallbackHandler(&callback);

			// Put the device in measurement mode
			if (!device->gotoMeasurement())
			{
				throw std::runtime_error("Could not put device into measurement mode. Aborting.");
			}

			//std::cout << "\nMain loop (press Ctrl+C to quit)" << std::endl;
			//std::cout << std::string(79, '-') << std::endl;
			while (ros::ok())
			{
				if (callback.packetAvailable())
				{
					// Create ROS message
					sensor_msgs::Imu imuData;
					imuData.header.frame_id = "/imu";
					imuData.header.stamp = ros::Time::now();

					// Retrieve a packet
					XsDataPacket packet = callback.getNextPacket();

					// Get the orientation data
					if (packet.containsOrientation()) {
						/*XsEuler euler = packet.orientationEuler();
						std::cout << "\r"
						  	  << "Roll:" << std::setw(5) << std::fixed 
<< std::setprecision(2) << euler.m_roll
						  	  << ", Pitch:" << std::setw(5) << std::fixed << std::setprecision(2) << euler.m_pitch
						  	  << ", Yaw:" << std::setw(5) << std::fixed << std::setprecision(2) << euler.m_yaw;
						std::cout << std::flush;*/

						XsQuaternion quaternion = packet.orientationQuaternion();

						imuData.orientation.x = quaternion.m_x;
						imuData.orientation.y = quaternion.m_y;
						imuData.orientation.z = quaternion.m_z;
						imuData.orientation.w = quaternion.m_w;
					}

					// Get the gyroscope data
					if (packet.containsCalibratedGyroscopeData()) {
						XsVector gyroscope = packet.calibratedGyroscopeData();
					
						imuData.angular_velocity.x = gyroscope.at(0);
						imuData.angular_velocity.y = gyroscope.at(1);
						imuData.angular_velocity.z = gyroscope.at(2);
					}

					// Get the acceleration data
					if (packet.containsCalibratedAcceleration()) {
						XsVector acceleration = packet.calibratedAcceleration();

						imuData.linear_acceleration.x = acceleration.at(0);
						imuData.linear_acceleration.y = acceleration.at(1);
						imuData.linear_acceleration.z = acceleration.at(2);
					}

					// Get the magnetic field data
                                        /*if (packet.containsCalibratedMagneticField()) {
						XsVector magneticField = packet.calibratedMagneticField();
					}*/

					// Publish ROS message
					publisher.publish(imuData);
				}

    				ros::spinOnce();
				XsTime::msleep(1.0);
			}
			//std::cout << "\r" << std::string(79, '-') << "\n";
			//std::cout << std::endl;
		}
		catch (std::runtime_error const & error)
		{
			std::cout << error.what() << std::endl;
		}
		catch (...)
		{
			std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
		}

		// Close port
		control->closePort(mtPort->portName().toStdString());
	}
	catch (std::runtime_error const & error)
	{
		std::cout << error.what() << std::endl;
	}
	catch (...)
	{
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
	}

	// Free XsControl object
	control->destruct();

	std::cout << "Successful exit." << std::endl;

	return 0;
}
