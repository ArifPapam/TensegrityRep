#ifndef UNICODE
#define UNICODE
#endif

#pragma once
/*#include <winsock2.h>
#include <Ws2tcpip.h>
#include <stdio.h>
#include <conio.h>
#include <iostream>
#include <pthread.h>

// Link with ws2_32.lib
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "pthreadVC2.lib")*/

//#include "vector_types.h"
//#include "InstCullingHeader.h"
//#include "RunTimeVariables.h"
#include "DirectXMath.h"
#include "networkConfig.h"
#include <windows.h>
#include <iomanip>
#include <iostream>
#include "Render_DX12.h"
#define HAVE_STRUCT_TIMESPEC
#include "Common/pthread.h"
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "pthreadVC2.lib")

#define MSG_WAITALL 0x8
#define DUEL_LAN 0
#define MULTITHREAD_NET 0

using namespace DirectX;
typedef unsigned int UINT;
typedef uint64_t UINT64;



//Render_DX.h Ln51
struct statusVariables{
	int running;				// 1
	bool paused;				// 1 - 2
	bool mAppPaused;			// 1 - 3
	int minutes;				///1 - 4
	double seconds;				// 2 - 6
	int screenXZ[2];			///2 - 8
	XMFLOAT3 camXYZ;			// 3 - 11
	int useRegion;				///1 - 12
	float dataMon[2];			// 2 - 14
	unsigned int inputLag[2]; //x Send, y Recv ///2 - 16
	float4x4 controllerMats[2];
	XMFLOAT3 controllerLoc[2];
}; 

const char * IP_B = _IP_B;
#if 0
const char * IP_A = _IP_A;
#else
const char * IP_A = _IP_A;
#endif

const UINT initVarCount = 11;
const UINT initDataSize = sizeof(UINT64)*initVarCount;
UINT64 initSettings[initVarCount];

timer_ms timerPart;

char * RecvBuf_init = NULL;
char * RecvBuf_MTlengths = NULL;
//int * MTlengths = NULL;
char * RecvBuf_LocColor = NULL;
char * RecvBuf_QuatLocXZ = NULL;
char * RecvBuf_RegionSizes = NULL;
char * RecvBuf_MotorLoc = NULL;
char * RecvBuf_MotorQuat = NULL;
//----------
char * RecvBuf_Loc = NULL;
char * RecvBuf_Color = NULL;
//-----------
WSADATA RecvWsaData;

WSADATA SendWsaData;
SOCKET connectSocket_A = INVALID_SOCKET;
SOCKET connectSocket_B = INVALID_SOCKET;

sockaddr_in address_A;
sockaddr_in address_B;
char statusBuffer[sizeof(statusVariables)]; 

int loopCnt; 
UINT64 dataAccRecv;
UINT64 adjustedLocColorSendTotal;
UINT64 adjustedQuatLocXZSendTotal;
//UINT segment_count;
UINT floatCount;
UINT sizeSend_LocColor;
UINT sizeSend_QuatAndLoc;
UINT sendSize_MotorLoc;
UINT sendSize_MotorQuat;
UINT sendSize_Loc;
UINT sendSize_Color;
UINT microtubuleCount;

statusVariablesPoint *varsPoint_ptr;
statusVariables varsLocal;

double memCpy_Timer;
double recv_Timer;
double pause_Timer;
double totalRecv;

errno_t err; 
FILE * clientPrint;
int * NetworkLive_ptr;
