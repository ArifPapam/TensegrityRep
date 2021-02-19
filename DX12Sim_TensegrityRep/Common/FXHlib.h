///////////////////////////////////
// Arif Pramudwiatmoko           //
// Konagaya Laboratory           //
// Tokyo Institute fo Technology //
///////////////////////////////////

/*This file serves as the driver for the haptic rendering device.
The haptic rendering device hardware was developed by Mr. Satoru Tsutoh from Fuji Xerox Co., Ltd.*/

#pragma once

#include <Windows.h>
#include <iostream>

namespace FXH {
	/*Finger's (or palm hand's) ID.*/
	enum FingerID : unsigned char {
		RIGHT_THUMB = 1,
		RIGHT_INDEX = 2,
		RIGHT_MIDDLE = 3,
		RIGHT_RING = 4,
		RIGHT_PINKY = 5,
		RIGHT_PALM = 6,
		LEFT_THUMB = 11,
		LEFT_INDEX = 12,
		LEFT_MIDDLE = 13,
		LEFT_RING = 14,
		LEFT_PINKY = 15,
		LEFT_PALM = 16,
		ALL = 120
	};

	/*Handler for Fuji-Xerox Haptic Touching Sensor device*/
	class FXHdevice {
	private:
		HANDLE serialPortHandle;
		int sendArray(unsigned char *, int);
		void clear();

	public:
		FXHdevice();
		~FXHdevice();

		/*To connect to the device through a serial port
		where the dongle attached.
		Parameter:
		# device = serial port name
		Return:
		#  0 = no error.
		# -1 = CreateFile() failed. Maybe the USB dongle isn't attached.
		# -2 = SetCommState() failed.*/
		int connect(LPCWSTR device);

		void disconnect(void);

		/*To change the device vibration.
		Parameter:
		# id = finger's (or palm hand's) ID.
		# frequency = frequency of vibration. 10 upto 1024. 0 for stop.
		# amplitude = amplitude of vibration. 1 upto 1024.
		# maxAmpl = maximum amplitude of vibration. 0 upto 1024.
		# pressure = pressure (reaction force). 41 (maximum) upto 112 (minimum).*/
		int setHaptic(FingerID id, int frequency, int amplitude, int maxAmpl, int pressure);
	};
}