///////////////////////////////////
// Arif Pramudwiatmoko           //
// Konagaya Laboratory           //
// Tokyo Institute fo Technology //
///////////////////////////////////

/*This file serves as the driver for the haptic rendering device.
The haptic rendering device hardware was developed by Mr. Satoru Tsutoh from Fuji Xerox Co., Ltd.*/

#include "FXHlib.h"

FXH::FXHdevice::FXHdevice() {
	serialPortHandle = INVALID_HANDLE_VALUE;
}

FXH::FXHdevice::~FXHdevice() {
	if (serialPortHandle != INVALID_HANDLE_VALUE)
		CloseHandle(serialPortHandle);

	serialPortHandle = INVALID_HANDLE_VALUE;
}

int FXH::FXHdevice::connect(LPCWSTR device = L"COM3") {
	int error = 0;
	DCB dcb;

	memset(&dcb, 0, sizeof(dcb));

	dcb.DCBlength = sizeof(dcb);

	dcb.BaudRate = 115200;
	dcb.Parity = NOPARITY;
	dcb.fParity = 0;
	dcb.StopBits = ONESTOPBIT;
	dcb.ByteSize = 8;

	serialPortHandle = CreateFile(device, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, NULL, NULL);

	if (serialPortHandle != INVALID_HANDLE_VALUE) {
		if (!SetCommState(serialPortHandle, &dcb))
			error = -2;
	}
	else {
		error = -1;
	}

	if (error != 0) {
		disconnect();
	}
	else {
		clear();
	}

	return error;
}

void FXH::FXHdevice::disconnect(void) {
	CloseHandle(serialPortHandle);
	serialPortHandle = INVALID_HANDLE_VALUE;
}

int FXH::FXHdevice::sendArray(unsigned char *buffer, int len) {
	unsigned long result = 0;

	if (serialPortHandle != INVALID_HANDLE_VALUE)
		WriteFile(serialPortHandle, buffer, len, &result, NULL);

	return result;
}

void FXH::FXHdevice::clear() {
	PurgeComm(serialPortHandle, PURGE_RXCLEAR | PURGE_TXCLEAR);
}

int FXH::FXHdevice::setHaptic(FingerID id, int frequency, int amplitude, int maxAmpl, int pressure) {
	// Filter to get the allowed haptic values.
	if (frequency < 10) frequency = 0;
	else if (frequency > 1024) frequency = 1024;
	if (amplitude < 1) amplitude = 1025;
	else if (amplitude > 1024) amplitude = 1024;
	if (maxAmpl < 0) maxAmpl = 0;
	else if (maxAmpl > 1024) maxAmpl = 1024;
	if (pressure < 41) pressure = 41;
	else if (pressure > 112) pressure = 112;

	unsigned char *buffer = new unsigned char[29];
	unsigned char temp;
	buffer[0] = ':';
	temp = (id >> 4);
	buffer[1] = temp < 10 ? temp + 48 : temp + 55;
	temp = (id & 15);
	buffer[2] = temp < 10 ? temp + 48 : temp + 55;
	buffer[3] = '8';
	buffer[4] = '0';
	buffer[5] = '0';
	buffer[6] = '1';
	buffer[7] = '0';
	buffer[8] = '0';
	buffer[9] = '0';
	buffer[10] = '0';
	temp = (frequency >> 12);
	buffer[11] = temp < 10 ? temp + 48 : temp + 55;
	temp = ((frequency >> 8) & 15);
	buffer[12] = temp < 10 ? temp + 48 : temp + 55;
	temp = ((frequency >> 4) & 15);
	buffer[13] = temp < 10 ? temp + 48 : temp + 55;
	temp = (frequency & 15);
	buffer[14] = temp < 10 ? temp + 48 : temp + 55;
	temp = (amplitude >> 12);
	buffer[15] = temp < 10 ? temp + 48 : temp + 55;
	temp = ((amplitude >> 8) & 15);
	buffer[16] = temp < 10 ? temp + 48 : temp + 55;
	temp = ((amplitude >> 4) & 15);
	buffer[17] = temp < 10 ? temp + 48 : temp + 55;
	temp = (amplitude & 15);
	buffer[18] = temp < 10 ? temp + 48 : temp + 55;
	temp = (maxAmpl >> 12);
	buffer[19] = temp < 10 ? temp + 48 : temp + 55;
	temp = ((maxAmpl >> 8) & 15);
	buffer[20] = temp < 10 ? temp + 48 : temp + 55;
	temp = ((maxAmpl >> 4) & 15);
	buffer[21] = temp < 10 ? temp + 48 : temp + 55;
	temp = (maxAmpl & 15);
	buffer[22] = temp < 10 ? temp + 48 : temp + 55;
	temp = (pressure >> 12);
	buffer[23] = temp < 10 ? temp + 48 : temp + 55;
	temp = ((pressure >> 8) & 15);
	buffer[24] = temp < 10 ? temp + 48 : temp + 55;
	temp = ((pressure >> 4) & 15);
	buffer[25] = temp < 10 ? temp + 48 : temp + 55;
	temp = (pressure & 15);
	buffer[26] = temp < 10 ? temp + 48 : temp + 55;
	buffer[27] = 'X';
	buffer[28] = '\n';

	return sendArray(buffer, 29);
}