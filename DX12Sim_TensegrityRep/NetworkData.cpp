#include "NetworkData.h"

void NetworkData::StartCounterClient_ms(timer_ms &t)
{
	LARGE_INTEGER li;
	if(!QueryPerformanceFrequency(&li))
		std::wcout << "QueryPerformanceFrequency failed!\n";

	t.PCFreq = double(li.QuadPart)/1000.0;

	QueryPerformanceCounter(&li);
	t.CounterStart = li.QuadPart;
}
double NetworkData::GetCounterClient_ms(timer_ms &t)
{
	LARGE_INTEGER li;
	QueryPerformanceCounter(&li);
	return double(li.QuadPart-t.CounterStart)/t.PCFreq;
}

void NetworkData::ErrorReport(ULONG dd)
{ 
	// Retrieve the system error message for the last-error code

	LPTSTR lpMsgBuf = NULL;
	DWORD dw = dd; 

	int length = FormatMessage(
		FORMAT_MESSAGE_ALLOCATE_BUFFER | 
		FORMAT_MESSAGE_FROM_SYSTEM |
		FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL,
		dw,
		MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
		(LPTSTR)&lpMsgBuf,
		0, NULL );


	if(dw == 0)
		fwprintf(clientPrint, L"%s\n", lpMsgBuf);
	else
		fwprintf(clientPrint, L"Error: %s\n", lpMsgBuf);

	LocalFree(lpMsgBuf);
	//ExitProcess(dw); 
}
void NetworkData::cleanUpSockets(){
	fprintf(clientPrint, "\nExiting.\n");
	if(closesocket(connectSocket_A) != 0) { /* Error or already closed */ }
	connectSocket_A = INVALID_SOCKET;

	if(DUEL_LAN){
		if(closesocket(connectSocket_B) != 0) { /* Error or already closed */ }
		connectSocket_B = INVALID_SOCKET;
	}

	if(RecvBuf_MTlengths != NULL){ free(RecvBuf_MTlengths); RecvBuf_MTlengths = NULL; }
	if(RecvBuf_init != NULL){ free(RecvBuf_init); RecvBuf_init = NULL; }
	if(RecvBuf_LocColor != NULL){ free(RecvBuf_LocColor); RecvBuf_LocColor = NULL; }
	if(RecvBuf_QuatLocXZ != NULL){ free(RecvBuf_QuatLocXZ); RecvBuf_QuatLocXZ = NULL; }
	if(RecvBuf_RegionSizes != NULL){ free(RecvBuf_RegionSizes); RecvBuf_RegionSizes = NULL; }
	if(RecvBuf_MotorLoc != NULL){ free(RecvBuf_MotorLoc); RecvBuf_MotorLoc = NULL; }
	if(RecvBuf_MotorQuat != NULL){ free(RecvBuf_MotorQuat); RecvBuf_MotorQuat = NULL; }
	if(RecvBuf_Loc != NULL){ free(RecvBuf_Loc); RecvBuf_Loc = NULL; }
	if(RecvBuf_Color != NULL){ free(RecvBuf_Color); RecvBuf_Color = NULL; }
	
	//WSACleanup();

	UINT64 full_MotorQuat = (UINT64)loopCnt*sendSize_MotorQuat;
	UINT64 full_LocColorSize = (UINT64)loopCnt*sizeSend_LocColor;
	double triedToRecv = (adjustedQuatLocXZSendTotal+adjustedLocColorSendTotal) / 1000000.0;
	double fullRecv = (full_MotorQuat+full_LocColorSize) / 1000000.0;
	double dataRecv = (dataAccRecv/1000000.0f);
	fprintf(clientPrint, "Bytes Recv: %5.2f MB Out of %5.2f MB\n", dataRecv, triedToRecv);
	fprintf(clientPrint, "Recv Rate: %5.2f MB/s\n", (dataRecv/(totalRecv*0.001)));
	fprintf(clientPrint, "Region reduced total data from (%5.2f) to (%5.2f)\n", fullRecv, triedToRecv);
	fprintf(clientPrint, "A reduction of (%5.2f) or (%5.2f%%)\n", fullRecv-triedToRecv, (fullRecv/triedToRecv)-1.0f);

	fclose(clientPrint);
	*NetworkLive_ptr = 0;
	//_getch();
}
void NetworkData::checkError_MaybeExit_basic(int iResult, wchar_t * stringMsg){
	if (iResult == SOCKET_ERROR) {
		int error = WSAGetLastError();
		fwprintf(clientPrint, L"Socket Error: %s failed with error: %d\n", stringMsg, error);
		ErrorReport(error);
		cleanUpSockets();
		pthread_exit((void*)EXIT_FAILURE);
	}
}
void NetworkData::checkError_MaybeExit_SndRcV(int iResult, wchar_t * stringMsg, unsigned int sendSize){
	if (iResult == SOCKET_ERROR) {
		int error = WSAGetLastError();
		fwprintf(clientPrint, L"Socket Error: %s failed with error: %d\n", stringMsg, error);
		ErrorReport(error);
		cleanUpSockets();
		pthread_exit((void*)EXIT_FAILURE);
	}else if(iResult != sendSize){
		int error = WSAGetLastError();
		fwprintf(clientPrint, L"Not all data moved: %s failed with error: %d (ln101 ish)\n", stringMsg, error);
		ErrorReport(error);
	}
}
void NetworkData::sock_checkErrorMaybeExit(SOCKET * sock){
	if (*sock == INVALID_SOCKET) {
		int error = WSAGetLastError();
		fprintf(clientPrint, "socket failed with error: %ld\n", WSAGetLastError());
		ErrorReport(error);
		cleanUpSockets();
		pthread_exit((void*)EXIT_FAILURE);
	}
}
void NetworkData::checkInitSettings(){
	if ((initSettings[0]*3 != initSettings[1]) || (initSettings[0]*sizeof(XMFLOAT4) != initSettings[2])) {
		fprintf(clientPrint, "Corrupted Parameters \n");
		cleanUpSockets();
		pthread_exit((void*)EXIT_FAILURE);
	}
}
void NetworkData::initServer(int addSize){
	int iResult;
	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &SendWsaData);
	checkError_MaybeExit_basic(iResult, L"WSAStartup");
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++

	// Create a socket for sending data
	connectSocket_A = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	sock_checkErrorMaybeExit(&connectSocket_A);

	address_A.sin_family = AF_INET;
	address_A.sin_port = htons(dataPort);
	address_A.sin_addr.s_addr = inet_addr(IP_A);

	iResult = connect( connectSocket_A, (SOCKADDR*) &address_A, addSize );
	checkError_MaybeExit_basic(iResult, L"connect");

	if(DUEL_LAN){
		// Create a socket for sending data
		connectSocket_B = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		sock_checkErrorMaybeExit(&connectSocket_B);

		address_B.sin_family = AF_INET;
		address_B.sin_port = htons(dataPort);
		address_B.sin_addr.s_addr = inet_addr(IP_B);

		iResult = connect( connectSocket_B, (SOCKADDR*) &address_B, addSize );
		checkError_MaybeExit_basic(iResult, L"connect");
	}
}
void NetworkData::changeBuffer(int siz, SOCKET * sock){
	int iResult = 0;
	int optVal;
	int optLen = sizeof(int);
	int optValNew = siz;
	int optLenNew = sizeof(int);

	if (getsockopt(*sock, SOL_SOCKET, SO_RCVBUF, (char*)&optVal, &optLen) != SOCKET_ERROR)
		fprintf(clientPrint, "Checking SockOpt SO_RCVBUF Value: %ld", optVal);

	iResult = setsockopt(*sock, SOL_SOCKET, SO_RCVBUF, (char *) &optValNew, optLenNew);
	if (iResult == SOCKET_ERROR) {
		fprintf(clientPrint, "\tsetsockopt for failed with error: %u\n", WSAGetLastError());
	} else
		fprintf(clientPrint, "\tSO_RCVBUF succeeded\n");

	if (getsockopt(*sock, SOL_SOCKET, SO_RCVBUF, (char*)&optVal, &optLen) != SOCKET_ERROR)
		fprintf(clientPrint, "Checking SockOpt SO_RCVBUF Value: %ld\n", optVal);

	////////////////////////////////////////////////////////////////////////////////////////
	if (getsockopt(*sock, SOL_SOCKET, SO_SNDBUF, (char*)&optVal, &optLen) != SOCKET_ERROR)
		fprintf(clientPrint, "Checking SockOpt SO_SNDBUF Value: %ld", optVal);

	iResult = setsockopt(*sock, SOL_SOCKET, SO_SNDBUF, (char *) &optValNew, optLenNew);
	if (iResult == SOCKET_ERROR) {
		fprintf(clientPrint, "\tsetsockopt for failed with error: %u\n", WSAGetLastError());
	} else
		fprintf(clientPrint, "\tSO_SNDBUF succeeded\n");

	if (getsockopt(*sock, SOL_SOCKET, SO_SNDBUF, (char*)&optVal, &optLen) != SOCKET_ERROR)
		fprintf(clientPrint, "Checking SockOpt SO_SNDBUF Value: %ld\n", optVal);
}

void NetworkData::indicateReady(){
	//fwprintf(clientPrint, L"Indicating ready\t"); 
	int statusSize = sizeof(waitBuffer);
	int status = 1;
	memset(waitBuffer, 0, statusSize);
	memcpy(waitBuffer, &status, statusSize);
	int iResult = send(connectSocket_A, waitBuffer, statusSize, 0);
	checkError_MaybeExit_SndRcV(iResult, L"indicateReady", statusSize);
	//fwprintf(clientPrint, L"Status sent\n");

	if(*varsPointData->running){ // TODO may not be the correct exit point
		Sleep(200); //Give time for the client to recv
		cleanUpSockets();
		pthread_exit((void*)0);
	}
}
void NetworkData::waitOnOther(){
	//fwprintf(clientPrint, L"Wating on other\t");
	int statusSize = sizeof(waitBuffer);
	int status = 0;
	memset(waitBuffer, 0, statusSize);
	int iResult = recv(connectSocket_A, waitBuffer, statusSize, 0);
	checkError_MaybeExit_SndRcV(iResult, L"waitOnOther", statusSize);
	memcpy(&status, waitBuffer, statusSize);
	//fprintf("Other Connection is ready Continuing\n");

	if (status) {
		cleanUpSockets();
		pthread_exit((void*)0);
	}
}
void NetworkData::checkRunning() {
	if (*varsPointData->running) {
		cleanUpSockets();
		pthread_exit((void*)0);
	}
}

XMUINT2 NetworkData::recvRegionData(){

	int iResult = recv(connectSocket_A, RecvBuf_RegionSizes, sizeof(XMUINT2), MSG_WAITALL);
	checkError_MaybeExit_SndRcV(iResult, L"recvfrom RegionSizes", sizeof(XMUINT2));
	dataAccRecv += iResult; 

	XMUINT2 RENDER_B_DATASIZE; RENDER_B_DATASIZE.x = 0; RENDER_B_DATASIZE.y = 0;
	memcpy(&RENDER_B_DATASIZE, RecvBuf_RegionSizes, sizeof(XMUINT2));
	adjustedLocColorSendTotal += RENDER_B_DATASIZE.x;
	adjustedQuatLocXZSendTotal += RENDER_B_DATASIZE.y;

	pthread_t segThread;
	void *retSeg;
	pthread_t motThread;
	void *retMot;

	////////////////////  Normal Logic  ////////////////////////////////////////////////////////////////////////////////////
	if (RENDER_B_DATASIZE.x > 0) {
		iResult = recv(connectSocket_A, RecvBuf_LocColor, RENDER_B_DATASIZE.x, MSG_WAITALL);///////////////////////////////
		checkError_MaybeExit_SndRcV(iResult, L"recvfrom locations", RENDER_B_DATASIZE.x);/////////////////////////////////
		dataAccRecv += iResult;
	}

	if (DUEL_LAN) {
		if (RENDER_B_DATASIZE.y > 0) {
			iResult = recv(connectSocket_B, RecvBuf_QuatLocXZ, RENDER_B_DATASIZE.y, MSG_WAITALL);
			checkError_MaybeExit_SndRcV(iResult, L"recvfrom MotorQuatLoc", RENDER_B_DATASIZE.y);
			dataAccRecv += iResult;
		}
	}
	else {
		if (RENDER_B_DATASIZE.y > 0) {
			iResult = recv(connectSocket_A, RecvBuf_QuatLocXZ, RENDER_B_DATASIZE.y, MSG_WAITALL);///////////////////////////////
			checkError_MaybeExit_SndRcV(iResult, L"recvfrom MotorQuatLoc", RENDER_B_DATASIZE.y);/////////////////////////////////
			dataAccRecv += iResult;
		}
	}

	if (loopCnt == 1) {
		iResult = recv(connectSocket_A, RecvBuf_MotorLoc, sendSize_MotorLoc, MSG_WAITALL);
		checkError_MaybeExit_SndRcV(iResult, L"recvfrom motorLoc", sendSize_MotorLoc);
		//dataAccRecv += iResult;
		iResult = recv(connectSocket_A, RecvBuf_MotorQuat, sendSize_MotorQuat, MSG_WAITALL);
		checkError_MaybeExit_SndRcV(iResult, L"recvfrom motorQuat", sendSize_MotorQuat);
		//dataAccRecv += iResult; 
	}
	return RENDER_B_DATASIZE;
}

void NetworkData::recvAll(){

	int iResult = recv(connectSocket_A, RecvBuf_Loc, sendSize_Loc, MSG_WAITALL);
	checkError_MaybeExit_SndRcV(iResult, L"recvfrom locations", sendSize_Loc);
	dataAccRecv += iResult; 


	iResult = recv(connectSocket_A, RecvBuf_Color, sendSize_Color, MSG_WAITALL);
	checkError_MaybeExit_SndRcV(iResult, L"recvfrom color", sendSize_Color);
	dataAccRecv += iResult;

	if(loopCnt == 1){
		iResult = recv(connectSocket_A, RecvBuf_MotorLoc, sendSize_MotorLoc, MSG_WAITALL);
		checkError_MaybeExit_SndRcV(iResult, L"recvfrom motorLoc", sendSize_MotorLoc);
		//dataAccRecv += iResult; 
	}

	if(DUEL_LAN){
		iResult = recv(connectSocket_B, RecvBuf_MotorQuat, sendSize_MotorQuat, MSG_WAITALL);
		checkError_MaybeExit_SndRcV(iResult, L"recvfrom motorQuat", sendSize_MotorQuat);
		dataAccRecv += iResult;
	}else{
		iResult = recv(connectSocket_A, RecvBuf_MotorQuat, sendSize_MotorQuat, MSG_WAITALL);
		checkError_MaybeExit_SndRcV(iResult, L"recvfrom motorQuat", sendSize_MotorQuat);
		dataAccRecv += iResult; 
	}
}

void NetworkData::networkData(void * arg)
{
	void ** inputs = (void**)arg;
	varsPointData = (statusVariablesPoint*)(inputs[0]);
	float ** host_segLocColor_ptr = (float**)(inputs[1]);
	int ** MTlengths_ptr  = (int**)(inputs[2]);
	int * MT_COUNT  = (int*)(inputs[3]);
	bool* initWait = (bool*)(inputs[4]);
	UINT * segRenderCount_ptr = (UINT*)(inputs[5]);/////
	int * MOTOR_CNT = (int*)(inputs[6]);
	float ** host_motorLoc_ptr = (float**)(inputs[7]);
	XMFLOAT4 ** host_motorQuat_ptr = (XMFLOAT4**)(inputs[8]);
	float ** host_segLoc_ptr = (float**)(inputs[9]);
	float ** host_segColor_ptr = (float**)(inputs[10]);
	UINT * motorRenderCount_ptr = (UINT*)(inputs[11]);//////
	quatLocXZ ** host_MotorQuatLoc_ptr = (quatLocXZ**)(inputs[12]);
	int * MT_SEGCOUNT = (int*)(inputs[13]);
	NetworkLive_ptr = (int*)(inputs[14]);
	float * surfaceX = (float*)(inputs[15]);
	float * surfaceY = (float*)(inputs[16]);
	float * surfaceZ = (float*)(inputs[17]);
	err  = fopen_s( &clientPrint, "clientPrint.txt","w");

	//############# Start Step 1 init ######################
	XMUINT2 RENDER_B_DATASIZE;
	RENDER_B_DATASIZE.x = 0; RENDER_B_DATASIZE.y = 0;
	double t21 = 0.0;
	memCpy_Timer = 0.0;
	recv_Timer = 0.0;
	pause_Timer = 0.0;
	int iResult = 0;
	loopCnt = 0;
	dataAccRecv = 0;
	adjustedLocColorSendTotal = 0;
	adjustedQuatLocXZSendTotal = 0;
	totalRecv = 0.0;
	int satusBuff_size = sizeof(int);
	int addressSize = sizeof (address_A);
	RecvBuf_init = (char*)malloc(initDataSize);

	initServer(addressSize);

	// Call the recvfrom function to receive datagrams
	// on the bound socket.
	fprintf(clientPrint, "Receiving init message...\n");
	iResult = recv(connectSocket_A, RecvBuf_init, initDataSize, 0); // 0
	checkError_MaybeExit_SndRcV(iResult, L"recv init", initDataSize);

	memcpy(initSettings, RecvBuf_init, initDataSize);
	std::cout << "cordanateCount: " << initSettings[0] << "  floatCount: " << initSettings[1] << "\nsizeSend_Loc: " << initSettings[2] << "  messageCount: " << initSettings[3] << "\n"; 

	checkInitSettings();
	*MT_SEGCOUNT = initSettings[0];
	floatCount = initSettings[1];
	sizeSend_LocColor = initSettings[2];
	microtubuleCount = initSettings[3];
	*MT_COUNT = microtubuleCount;
	*MOTOR_CNT = initSettings[4];

	UINT wtf = sizeof(quatLocXZ);
	UINT wtf2 = sizeof(statusVariables);
	sizeSend_QuatAndLoc = sizeof(quatLocXZ)*(*MOTOR_CNT);
	sendSize_MotorLoc = initSettings[5];
	sendSize_MotorQuat = sizeof(XMFLOAT4)*(*MOTOR_CNT);
	sendSize_Loc = *MT_SEGCOUNT *sizeof(XMFLOAT3);
	sendSize_Color = *MT_SEGCOUNT *sizeof(float);
	UINT TCPbufferSize = initSettings[6];
	*surfaceX = initSettings[7] * 2.0f;
	*surfaceY = initSettings[8];
	*surfaceZ = initSettings[9] * 2.0f;
	*varsPointData->useRegion = initSettings[10];

	fprintf(clientPrint, "Adjusting connectSocket_A...\n");
	changeBuffer((int)TCPbufferSize+28, &connectSocket_A);
	if(DUEL_LAN){
		fprintf(clientPrint, "Adjusting connectSocket_B...\n");
		changeBuffer((int)TCPbufferSize+28, &connectSocket_B);
	}
	//====================  Allocate  =================================

	RecvBuf_LocColor = (char*)malloc(sizeSend_LocColor);

	RecvBuf_RegionSizes = (char*)malloc(sizeof(XMUINT2));
	RecvBuf_QuatLocXZ = (char*)malloc(sizeSend_QuatAndLoc);
	RecvBuf_MotorLoc = (char*)malloc(sendSize_MotorLoc);
	RecvBuf_MotorQuat = (char*)malloc(sendSize_MotorQuat);
	RecvBuf_Loc = (char*)malloc(sendSize_Loc);
	RecvBuf_Color = (char*)malloc(sendSize_Color);

	UINT64 MTlength_buffSize = sizeof(int)*microtubuleCount;
	*MTlengths_ptr = (int*)malloc(MTlength_buffSize);
	RecvBuf_MTlengths = (char*)malloc(MTlength_buffSize);

	indicateReady(); // 1

	fwprintf(clientPrint, L"Receiving the segmentCount per MT...\n");
	iResult = recv(connectSocket_A, RecvBuf_MTlengths, (int)MTlength_buffSize, 0);
	checkError_MaybeExit_SndRcV(iResult, L"recv init", MTlength_buffSize);

	memcpy(*MTlengths_ptr, RecvBuf_MTlengths, MTlength_buffSize);

	*initWait = false;
	//Send MT_COUNT and MT_LENGTHS to main area for init. !!!!!!!!

	//############# End Step 1 init ########################

	//############# Start Step 2 receive and translate ############

	for(;;){

		//>>>>>>>>>>>>>>>>>>>>Send>>>>>>>>>>>>>>>>>>
		StartCounterClient_ms(timerPart);
		while(*varsPointData->paused) {
			Sleep(1); 
			checkRunning();
		}
		pause_Timer += GetCounterClient_ms(timerPart);
		//indicateReady();//Real wait // 2

		loopCnt++;
		if (loopCnt % PERF_AVG_CNT == 0)
			fwprintf(clientPrint, L"<<< Data Frame %d >>>\n", loopCnt);

		StartCounterClient_ms(timerPart);
		bool USE_REGION = *varsPointData->useRegion;
		if(USE_REGION){
			RENDER_B_DATASIZE = recvRegionData();
			*segRenderCount_ptr = RENDER_B_DATASIZE.x/sizeof(XMFLOAT4);
			*motorRenderCount_ptr = RENDER_B_DATASIZE.y/sizeof(quatLocXZ);
		}else{
			recvAll();
			*segRenderCount_ptr = *MT_SEGCOUNT;
			*motorRenderCount_ptr = *MOTOR_CNT;
		}
		recv_Timer += GetCounterClient_ms(timerPart);

		//======================  Translate Data  ====================

		StartCounterClient_ms(timerPart);
		if(USE_REGION){
			memcpy(*host_segLocColor_ptr, RecvBuf_LocColor, RENDER_B_DATASIZE.x);////////////////////////////////////////////////////
			memcpy(*host_MotorQuatLoc_ptr, RecvBuf_QuatLocXZ, RENDER_B_DATASIZE.y);////////////////////////////////////////////////////
			if(loopCnt == 1){
				memcpy(*host_motorLoc_ptr, RecvBuf_MotorLoc, sendSize_MotorLoc);
				memcpy(*host_motorQuat_ptr, RecvBuf_MotorQuat, sendSize_MotorQuat);
			}
		}else{
			memcpy(*host_segLoc_ptr, RecvBuf_Loc, sendSize_Loc);
			memcpy(*host_segColor_ptr, RecvBuf_Color, sendSize_Color);
			if(loopCnt == 1)
				memcpy(*host_motorLoc_ptr, RecvBuf_MotorLoc, sendSize_MotorLoc);
			memcpy(*host_motorQuat_ptr, RecvBuf_MotorQuat, sendSize_MotorQuat);
		}
		memCpy_Timer += GetCounterClient_ms(timerPart);

		if (loopCnt % PERF_AVG_CNT == 0){
			fprintf(clientPrint, ">>MemTime:    %5.2f\n", (float)memCpy_Timer / (float)PERF_AVG_CNT);
			fprintf(clientPrint, ">>RecvTime:   %5.2f\n", (float)recv_Timer / (float)PERF_AVG_CNT);
			fprintf(clientPrint, ">>pauseTimer: %5.2f\n", (float)pause_Timer / (float)PERF_AVG_CNT);
			totalRecv += recv_Timer;
			memCpy_Timer = 0.0;
			recv_Timer = 0.0;
			pause_Timer = 0.0;
		}

		//<<<<<<<<<<<<<<<<<<<<Recv<<<<<<<<<<<<<<<<<<
		waitOnOther(); // Status recive
	}
	//float locCheck[300];
	//memcpy(locCheck, locations, 300*sizeof(float));

	//############# End Step 2 receive and translate ############

	// Close the socket when finished receiving datagrams
	fwprintf(clientPrint, L"Finished receiving. Closing socket out of loop, a issue.\n");
	iResult = closesocket(connectSocket_A);
	checkError_MaybeExit_basic(iResult, L"closesocket_A");
	if(DUEL_LAN)
		iResult = closesocket(connectSocket_B);
	checkError_MaybeExit_basic(iResult, L"closesocket_B");

	// Clean up and exit.
	cleanUpSockets();
}

extern "C" void * networkDataThread(void * arg)
{
	netData.networkData(arg);
	return 0;
}