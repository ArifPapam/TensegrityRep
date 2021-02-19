#ifndef UNICODE
#define UNICODE
#endif

#pragma once

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
struct statusVariables {
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


class NetworkStatus
{
public:
	void NetworkStatus::StartCounterClient_ms(timer_ms &t);
	double NetworkStatus::GetCounterClient_ms(timer_ms &t);
	void copy16mat(float4x4 *out, float4x4 in);

	void NetworkStatus::ErrorReport(ULONG dd);
	void NetworkStatus::cleanUpSockets();
	void NetworkStatus::checkError_MaybeExit_basic(int iResult, wchar_t * stringMsg);
	void NetworkStatus::checkError_MaybeExit_SndRcV(int iResult, wchar_t * stringMsg, unsigned int sendSize);
	void NetworkStatus::sock_checkErrorMaybeExit(SOCKET * sock);
	void NetworkStatus::initServer(int addSize);
	void NetworkStatus::changeBuffer(int siz, SOCKET * sock);

	void NetworkStatus::updateLocalVarsToSend();
	void NetworkStatus::updateVarsPointToRecv();

	void NetworkStatus::statusSend();
	void NetworkStatus::waitOnOther();

	void NetworkStatus::networkStatus(void * arg);

	const char * IP_B = _IP_B;
#if 0
	const char * IP_A = _IP_A;
#else
	const char * IP_A = _IP_A;
#endif

	timer_ms timerStat;
	timer_ms timerWait;

	WSADATA RecvWsaData;
	WSADATA SendWsaData;
	SOCKET connectSocket_A = INVALID_SOCKET;

	sockaddr_in address_A;
	char statusBuffer[sizeof(statusVariables)];

	int loopCnt;

	statusVariablesPoint *varsPoint_ptr;
	statusVariables varsLocal;
	int statusSize = sizeof(statusVariables);

	double statusTimer;
	double waitTimer;
	double totalRecv;
	UINT64 dataAccRecv;
	double totalSend;
	UINT64 dataAccSend;

	errno_t err;
	FILE * clientPrintStat;
	int * NetworkLive_ptr;

};

NetworkStatus netStat;