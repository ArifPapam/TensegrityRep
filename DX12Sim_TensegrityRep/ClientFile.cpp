#include "ClientFileHeader.h"

void StartCounterClient_ms(timer_ms &t)
{
	LARGE_INTEGER li;
	if(!QueryPerformanceFrequency(&li))
		std::wcout << "QueryPerformanceFrequency failed!\n";

	t.PCFreq = double(li.QuadPart)/1000.0;

	QueryPerformanceCounter(&li);
	t.CounterStart = li.QuadPart;
}
double GetCounterClient_ms(timer_ms &t)
{
	LARGE_INTEGER li;
	QueryPerformanceCounter(&li);
	return double(li.QuadPart-t.CounterStart)/t.PCFreq;
}

void copy16matClient(float4x4 *out, float4x4 in) {
	out->m00 = in.m00;
	out->m01 = in.m01;
	out->m02 = in.m02;
	out->m03 = in.m03;

	out->m10 = in.m10;
	out->m11 = in.m11;
	out->m12 = in.m12;
	out->m13 = in.m13;

	out->m20 = in.m20;
	out->m21 = in.m21;
	out->m22 = in.m22;
	out->m23 = in.m23;

	out->m30 = in.m30;
	out->m31 = in.m31;
	out->m32 = in.m32;
	out->m33 = in.m33;
}

void ErrorReport(ULONG dd) 
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
void cleanUpSockets(){
	fprintf(clientPrint, "\nExiting.\n");
	if(closesocket(connectSocket_A) != 0) { /* Error or already closed */ }
	connectSocket_A = INVALID_SOCKET;

	if(DUEL_LAN || MULTITHREAD_NET){
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
	WSACleanup();

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
void checkError_MaybeExit_basic(int iResult, wchar_t * stringMsg){
	if (iResult == SOCKET_ERROR) {
		int error = WSAGetLastError();
		fwprintf(clientPrint, L"Socket Error: %s failed with error: %d\n", stringMsg, error);
		ErrorReport(error);
		cleanUpSockets();
		pthread_exit((void*)EXIT_FAILURE);
	}
}
void checkError_MaybeExit_SndRcV(int iResult, wchar_t * stringMsg, unsigned int sendSize){
	if (iResult == SOCKET_ERROR) {
		int error = WSAGetLastError();
		fwprintf(clientPrint, L"Socket Error: %s failed with error: %d\n", stringMsg, error);
		ErrorReport(error);
		cleanUpSockets();
		pthread_exit((void*)EXIT_FAILURE);
	}else if(iResult != sendSize){
		int error = WSAGetLastError();
		fwprintf(clientPrint, L"Not all data moved: %s failed with error: %d\n", stringMsg, error);
		ErrorReport(error);
	}
}
void sock_checkErrorMaybeExit(SOCKET * sock){
	if (*sock == INVALID_SOCKET) {
		int error = WSAGetLastError();
		fprintf(clientPrint, "socket failed with error: %ld\n", WSAGetLastError());
		ErrorReport(error);
		cleanUpSockets();
		pthread_exit((void*)EXIT_FAILURE);
	}
}
void checkInitSettings(){
	if ((initSettings[0]*3 != initSettings[1]) || (initSettings[0]*sizeof(XMFLOAT4) != initSettings[2])) {
		fprintf(clientPrint, "Corrupted Parameters \n");
		cleanUpSockets();
		pthread_exit((void*)EXIT_FAILURE);
	}
}
void initServer(int addSize){
	int iResult;
	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &SendWsaData);
	checkError_MaybeExit_basic(iResult, L"WSAStartup");
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++

	// Create a socket for sending data
	connectSocket_A = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	sock_checkErrorMaybeExit(&connectSocket_A);

	address_A.sin_family = AF_INET;
	address_A.sin_port = htons(clientPort);
	address_A.sin_addr.s_addr = inet_addr(IP_A);

	iResult = connect( connectSocket_A, (SOCKADDR*) &address_A, addSize );
	checkError_MaybeExit_basic(iResult, L"connect");

	if(DUEL_LAN){
		// Create a socket for sending data
		connectSocket_B = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		sock_checkErrorMaybeExit(&connectSocket_B);

		address_B.sin_family = AF_INET;
		address_B.sin_port = htons(clientPort);
		address_B.sin_addr.s_addr = inet_addr(IP_B);

		iResult = connect( connectSocket_B, (SOCKADDR*) &address_B, addSize );
		checkError_MaybeExit_basic(iResult, L"connect");
	}else if(MULTITHREAD_NET){
		// Create a socket On the same IP but different port
		connectSocket_B = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		sock_checkErrorMaybeExit(&connectSocket_B);

		address_B.sin_family = AF_INET;
		address_B.sin_port = htons(clientPortB);
		address_B.sin_addr.s_addr = inet_addr(IP_A);

		iResult = connect( connectSocket_B, (SOCKADDR*) &address_B, addSize );
		checkError_MaybeExit_basic(iResult, L"connect");
	}
}
void changeBuffer(int siz, SOCKET * sock){
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

void updateLocalVarsToSend() {
	varsLocal.paused = *varsPoint_ptr->paused || *varsPoint_ptr->mAppPaused;
	varsLocal.running = *varsPoint_ptr->running;
	varsLocal.camXYZ = *varsPoint_ptr->camXYZ;
	/*Hack*/varsLocal.camXYZ.y = CAM_HEIGHT_HACK + varsLocal.camXYZ.y;
	varsLocal.useRegion = *varsPoint_ptr->useRegion;
	//varsLocal.screenXZ[0] = *varsPoint_ptr->screenXZ[0];
	//varsLocal.screenXZ[1] = *varsPoint_ptr->screenXZ[1]; //Used in overhead non VR
	/*Hack*/varsLocal.screenXZ[0] = 2000;
	/*Hack*/varsLocal.screenXZ[1] = 2000;
	varsLocal.inputLag[0] = *varsPoint_ptr->inputLag[0];
	copy16matClient(&varsLocal.controllerMats[0], *varsPoint_ptr->controllerMats[0]);
	copy16matClient(&varsLocal.controllerMats[1], *varsPoint_ptr->controllerMats[1]);
	varsLocal.controllerLoc[0] = *varsPoint_ptr->controllerLoc[0];
	varsLocal.controllerLoc[1] = *varsPoint_ptr->controllerLoc[1];
}
void updateVarsPointToRecv() {
	*varsPoint_ptr->minutes = varsLocal.minutes;
	*varsPoint_ptr->seconds = varsLocal.seconds;
	*varsPoint_ptr->dataMon[0] = varsLocal.dataMon[0];
	*varsPoint_ptr->dataMon[1] = varsLocal.dataMon[1];
	*varsPoint_ptr->inputLag[1] = varsLocal.inputLag[1];
}

void statusSend(){
	//fwprintf(clientPrint, L"Indicating ready\t"); 
	int statusSize = sizeof(statusBuffer);
	memset(statusBuffer, 0, statusSize);
	memcpy(statusBuffer, &varsLocal, statusSize);
	int iResult = send(connectSocket_A, statusBuffer, statusSize, 0);
	checkError_MaybeExit_SndRcV(iResult, L"indicateReady", statusSize);
	//fwprintf(clientPrint, L"Status sent\n");

	if(varsLocal.running){
		Sleep(200); //Give time for the client to recv
		cleanUpSockets();
		pthread_exit((void*)0);
	}
}
void waitOnOther(){
	//fwprintf(clientPrint, L"Wating on other\t");
	int statusSize = sizeof(statusBuffer);
	memset(statusBuffer, 0, statusSize);
	int iResult = recv(connectSocket_A, statusBuffer, statusSize, 0);
	checkError_MaybeExit_SndRcV(iResult, L"waitOnOther", statusSize);

	memcpy(&varsLocal, statusBuffer, statusSize);
	if (varsLocal.minutes > 2000 || varsLocal.running > 1) { //why 2000
		varsLocal.minutes = 0;
	}
	else {
		//Recv Data
		updateVarsPointToRecv();
	}
	//fprintf("Other Connection is ready Continuing\n");
}

void * secondSocketA(void * args){
	int sizex = *((int*)args);
	int iResult = recv(connectSocket_A, RecvBuf_LocColor, sizex, MSG_WAITALL);///////////////////////////////
	checkError_MaybeExit_SndRcV(iResult, L"recvfrom locations", sizex);/////////////////////////////////
	dataAccRecv += iResult; 
	return 0;
}
void * secondSocketB(void * args){
	int sizey = *((int*)args);
	int iResult = recv(connectSocket_B, RecvBuf_QuatLocXZ, sizey, MSG_WAITALL);
	checkError_MaybeExit_SndRcV(iResult, L"recvfrom MotorQuatLoc", sizey);
	dataAccRecv += iResult;
	return 0;
}

XMUINT2 recvRegionData(){

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
	if(MULTITHREAD_NET){
	////////////////////  Two Thread Logic  //////////////////////////////////////////////////////////////////////////////////
		if(RENDER_B_DATASIZE.x > 0){
			int erro = pthread_create(&segThread, NULL, secondSocketA, &RENDER_B_DATASIZE.x);
			if (erro != 0)
				wprintf(L"can't create thread error code %d\n", erro);
		}
		if(RENDER_B_DATASIZE.y > 0){
			int erro = pthread_create(&motThread, NULL, secondSocketB, &RENDER_B_DATASIZE.y);
			if (erro != 0)
				wprintf(L"can't create thread error code %d\n", erro);
		}
		//--------------Sync Threads-----------------------------------------------------------------
		if(RENDER_B_DATASIZE.x > 0){
			int erro = pthread_join(segThread, &retSeg);
			if (erro != 0)
				wprintf(L"Thread exit code: %d, thread returned: %d\n", erro, (int)retSeg);
		}

		if(RENDER_B_DATASIZE.y > 0){
			int erro = pthread_join(motThread, &retMot);
			if (erro != 0)
				wprintf(L"Thread exit code: %d, thread returned: %d\n", erro, (int)retMot);
		}
	}else{
	////////////////////  Normal Logic  ////////////////////////////////////////////////////////////////////////////////////
		if(RENDER_B_DATASIZE.x > 0){
			iResult = recv(connectSocket_A, RecvBuf_LocColor, RENDER_B_DATASIZE.x, MSG_WAITALL);///////////////////////////////
			checkError_MaybeExit_SndRcV(iResult, L"recvfrom locations", RENDER_B_DATASIZE.x);/////////////////////////////////
			dataAccRecv += iResult; 
		}

		if(DUEL_LAN){
			if(RENDER_B_DATASIZE.y > 0){
				iResult = recv(connectSocket_B, RecvBuf_QuatLocXZ, RENDER_B_DATASIZE.y, MSG_WAITALL);
				checkError_MaybeExit_SndRcV(iResult, L"recvfrom MotorQuatLoc", RENDER_B_DATASIZE.y);
				dataAccRecv += iResult;
			}
		}else{
			if(RENDER_B_DATASIZE.y > 0){
				iResult = recv(connectSocket_A, RecvBuf_QuatLocXZ, RENDER_B_DATASIZE.y, MSG_WAITALL);///////////////////////////////
				checkError_MaybeExit_SndRcV(iResult, L"recvfrom MotorQuatLoc", RENDER_B_DATASIZE.y);/////////////////////////////////
				dataAccRecv += iResult; 
			}
		}	
	}

	if(loopCnt == 1){
		iResult = recv(connectSocket_A, RecvBuf_MotorLoc, sendSize_MotorLoc, MSG_WAITALL);
		checkError_MaybeExit_SndRcV(iResult, L"recvfrom motorLoc", sendSize_MotorLoc);
		//dataAccRecv += iResult;
		iResult = recv(connectSocket_A, RecvBuf_MotorQuat, sendSize_MotorQuat, MSG_WAITALL);
		checkError_MaybeExit_SndRcV(iResult, L"recvfrom motorQuat", sendSize_MotorQuat);
		//dataAccRecv += iResult; 
	}
	return RENDER_B_DATASIZE;
}

void recvAll() {

	int iResult = recv(connectSocket_A, RecvBuf_Loc, sendSize_Loc, MSG_WAITALL);
	checkError_MaybeExit_SndRcV(iResult, L"recvfrom locations", sendSize_Loc);
	dataAccRecv += iResult;


	iResult = recv(connectSocket_A, RecvBuf_Color, sendSize_Color, MSG_WAITALL);
	checkError_MaybeExit_SndRcV(iResult, L"recvfrom color", sendSize_Color);
	dataAccRecv += iResult;

	if (loopCnt == 1) {
		iResult = recv(connectSocket_A, RecvBuf_MotorLoc, sendSize_MotorLoc, MSG_WAITALL);
		checkError_MaybeExit_SndRcV(iResult, L"recvfrom motorLoc", sendSize_MotorLoc);
		//dataAccRecv += iResult; 
	}

	if (DUEL_LAN) {
		iResult = recv(connectSocket_B, RecvBuf_MotorQuat, sendSize_MotorQuat, MSG_WAITALL);
		checkError_MaybeExit_SndRcV(iResult, L"recvfrom motorQuat", sendSize_MotorQuat);
		dataAccRecv += iResult;
	}
	else {
		iResult = recv(connectSocket_A, RecvBuf_MotorQuat, sendSize_MotorQuat, MSG_WAITALL);
		checkError_MaybeExit_SndRcV(iResult, L"recvfrom motorQuat", sendSize_MotorQuat);
		dataAccRecv += iResult;
	}
}

extern "C" void * serverThread(void * arg)
{
	void ** inputs = (void**)arg;
	varsPoint_ptr = (statusVariablesPoint*)(inputs[0]);
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

	updateLocalVarsToSend();
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
	*MT_SEGCOUNT = (int)initSettings[0];
	floatCount = (UINT)initSettings[1];
	sizeSend_LocColor = (UINT)initSettings[2];
	microtubuleCount = (UINT)initSettings[3];
	*MT_COUNT = (int)microtubuleCount;
	*MOTOR_CNT = (int)initSettings[4];

	UINT wtf = sizeof(quatLocXZ);
	UINT wtf2 = sizeof(statusVariables);
	sizeSend_QuatAndLoc = sizeof(quatLocXZ)*(*MOTOR_CNT);
	sendSize_MotorLoc = initSettings[5];
	sendSize_MotorQuat = sizeof(XMFLOAT4)*(*MOTOR_CNT);
	sendSize_Loc = *MT_SEGCOUNT *sizeof(XMFLOAT3);
	sendSize_Color = *MT_SEGCOUNT *sizeof(float);
	UINT TCPbufferSize = initSettings[6];
	*surfaceX = initSettings[7] * 2.0f;
	*surfaceY = initSettings[8] * 2.0f;
	*surfaceZ = initSettings[9] * 2.0f;
	*varsPoint_ptr->useRegion = initSettings[10];

	fprintf(clientPrint, "Adjusting connectSocket_A...\n");
	changeBuffer((int)TCPbufferSize+28, &connectSocket_A);
	if(DUEL_LAN || MULTITHREAD_NET){
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

	statusSend(); // 1

	if (microtubuleCount > 0) {
		fwprintf(clientPrint, L"Receiving the segmentCount per MT...\n");
		iResult = recv(connectSocket_A, RecvBuf_MTlengths, (int)MTlength_buffSize, 0);
		checkError_MaybeExit_SndRcV(iResult, L"recv init", MTlength_buffSize);
		memcpy(*MTlengths_ptr, RecvBuf_MTlengths, MTlength_buffSize);

	}

	*initWait = false;
	//Send MT_COUNT and MT_LENGTHS to main area for init. !!!!!!!!

	//############# End Step 1 init ########################

	//############# Start Step 2 receive and translate ############

	for(;;){

		//>>>>>>>>>>>>>>>>>>>>Send>>>>>>>>>>>>>>>>>>
		updateLocalVarsToSend();

		int justEntered = 1;
		StartCounterClient_ms(timerPart);
		while(varsLocal.paused) { 
			if(justEntered){
				statusSend(); //Tell the other to pause
				justEntered = 0;
			}
			Sleep(1); 
			updateLocalVarsToSend();
		}
		pause_Timer += GetCounterClient_ms(timerPart);
		statusSend();//Real wait // 2

		loopCnt++;
		if (loopCnt % PERF_AVG_CNT == 0)
			fwprintf(clientPrint, L"<<< Frame %d >>>\n", loopCnt);

		StartCounterClient_ms(timerPart);
		bool USE_REGION = varsLocal.useRegion;
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

		//t21 = GetCounterClient_ms(timerPart);
		//fwprintf(clientPrint, L"Copy to location array: %f\n", float(t21));

		//indicateReady();
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
	if(DUEL_LAN || MULTITHREAD_NET)
		iResult = closesocket(connectSocket_B);
	checkError_MaybeExit_basic(iResult, L"closesocket_B");

	// Clean up and exit.
	cleanUpSockets();

	return 0;
}

void ddd(){
	XMUINT2 RENDER_B_DATASIZE; int loopCount = 0;

	for(;;){
		//*Update local status variables from client application here*
		int justEntered = 1;
		while(varsLocal.paused) { 
			if(justEntered){
				statusSend(); //Tell the other to pause
				justEntered = 0;
			}
			//*Update local status variables from client application here*
		}

		statusSend();//Status send used as a wait command

		//Increment loop count and print current iteration every 30
		loopCount ++;
		if (loopCount % 30 == 0)
			fwprintf(clientPrint, L"<<< Frame %d >>>\n", loopCount);

		//Check if the user is using a render region, save it to insure 
		//---consistency in each iteration
		bool USE_REGION = varsLocal.useRegion;
		if(USE_REGION){
			//Receives the buffer sizes then calls receive commands
			//Also sets the object counts for DirectX instanced draw calls
			RENDER_B_DATASIZE = recvRegionData();
		}else{
			//Calls receive commands for full data sets
			//Also sets the object counts for DirectX instanced draw calls
			recvAll();
		}

		if(USE_REGION){
			//Copies only the needed data for the selected region
			//---segmment locations, segment colors, and motor quaternions
		}else{
			//Copies full data sets for segment locations, segment colors
			//---and motor quaternions
		}

		if (loopCnt % 30 == 0){
			//Timers above were excluded for simplification
			fprintf(clientPrint, ">>MemTime:    %5.2f\n", (float)memCpy_Timer / 30.0f);
			fprintf(clientPrint, ">>RecvTime:   %5.2f\n", (float)recv_Timer / 30.0f);
			fprintf(clientPrint, ">>pauseTimer: %5.2f\n", (float)pause_Timer / 30.0f);
			memCpy_Timer = 0.0;
			recv_Timer = 0.0;
			pause_Timer = 0.0;
		}

		waitOnOther(); // Status receive
	}

}

