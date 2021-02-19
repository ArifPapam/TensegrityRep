#include "NetworkStatus.h"

void NetworkStatus::StartCounterClient_ms(timer_ms &t)
{
	LARGE_INTEGER li;
	if(!QueryPerformanceFrequency(&li))
		std::wcout << "QueryPerformanceFrequency failed!\n";

	t.PCFreq = double(li.QuadPart)/1000.0;

	QueryPerformanceCounter(&li);
	t.CounterStart = li.QuadPart;
}
double NetworkStatus::GetCounterClient_ms(timer_ms &t)
{
	LARGE_INTEGER li;
	QueryPerformanceCounter(&li);
	return double(li.QuadPart-t.CounterStart)/t.PCFreq;
}

void NetworkStatus::copy16mat(float4x4 *out, float4x4 in) {
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

void NetworkStatus::ErrorReport(ULONG dd)
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
		fwprintf(clientPrintStat, L"%s\n", lpMsgBuf);
	else
		fwprintf(clientPrintStat, L"Error: %s\n", lpMsgBuf);

	LocalFree(lpMsgBuf);
	//ExitProcess(dw); 
}
void NetworkStatus::cleanUpSockets(){
	fprintf(clientPrintStat, "\nExiting.\n");
	if(closesocket(connectSocket_A) != 0) { /* Error or already closed */ }
	connectSocket_A = INVALID_SOCKET;

	//WSACleanup();

	
	double dataRecv = (dataAccRecv/1000000.0f);
	fprintf(clientPrintStat, "Bytes Recv: %5.2f MB\n", dataRecv);
	//fprintf(clientPrintStat, "Recv Rate: %5.2f MB/s\n", (dataRecv/(totalRecv*0.001)));

	double dataSend = (dataAccSend / 1000000.0f);
	fprintf(clientPrintStat, "Bytes Send: %5.2f MB\n", dataSend);
	//fprintf(clientPrintStat, "Send Rate: %5.2f MB/s\n", (dataSend / (totalSend*0.001)));

	fclose(clientPrintStat);
	*NetworkLive_ptr = 0;
	//_getch();
}
void NetworkStatus::checkError_MaybeExit_basic(int iResult, wchar_t * stringMsg){
	if (iResult == SOCKET_ERROR) {
		int error = WSAGetLastError();
		fwprintf(clientPrintStat, L"Socket Error: %s failed with error: %d\n", stringMsg, error);
		ErrorReport(error);
		cleanUpSockets();
		pthread_exit((void*)EXIT_FAILURE);
	}
}
void NetworkStatus::checkError_MaybeExit_SndRcV(int iResult, wchar_t * stringMsg, unsigned int sendSize){
	if (iResult == SOCKET_ERROR) {
		int error = WSAGetLastError();
		fwprintf(clientPrintStat, L"Socket Error: %s failed with error: %d\n", stringMsg, error);
		ErrorReport(error);
		cleanUpSockets();
		pthread_exit((void*)EXIT_FAILURE);
	}else if(iResult != sendSize){
		int error = WSAGetLastError();
		fwprintf(clientPrintStat, L"Not all data moved: %s failed with error: %d (ln85 ish)\n", stringMsg, error);
		ErrorReport(error);
	}
}
void NetworkStatus::sock_checkErrorMaybeExit(SOCKET * sock){
	if (*sock == INVALID_SOCKET) {
		int error = WSAGetLastError();
		fprintf(clientPrintStat, "socket failed with error: %ld\n", WSAGetLastError());
		ErrorReport(error);
		cleanUpSockets();
		pthread_exit((void*)EXIT_FAILURE);
	}
}

void NetworkStatus::initServer(int addSize){
	int iResult;
	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &SendWsaData);
	checkError_MaybeExit_basic(iResult, L"WSAStartup");
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++

	// Create a socket for sending data
	connectSocket_A = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	sock_checkErrorMaybeExit(&connectSocket_A);

	address_A.sin_family = AF_INET;
	address_A.sin_port = htons(statPort);
	address_A.sin_addr.s_addr = inet_addr(IP_A);

	iResult = connect( connectSocket_A, (SOCKADDR*) &address_A, addSize );
	checkError_MaybeExit_basic(iResult, L"connect");
}
void NetworkStatus::changeBuffer(int siz, SOCKET * sock){
	int iResult = 0;
	int optVal;
	int optLen = sizeof(int);
	int optValNew = siz;
	int optLenNew = sizeof(int);

	if (getsockopt(*sock, SOL_SOCKET, SO_RCVBUF, (char*)&optVal, &optLen) != SOCKET_ERROR)
		fprintf(clientPrintStat, "Checking SockOpt SO_RCVBUF Value: %ld", optVal);

	iResult = setsockopt(*sock, SOL_SOCKET, SO_RCVBUF, (char *) &optValNew, optLenNew);
	if (iResult == SOCKET_ERROR) {
		fprintf(clientPrintStat, "\tsetsockopt for failed with error: %u\n", WSAGetLastError());
	} else
		fprintf(clientPrintStat, "\tSO_RCVBUF succeeded\n");

	if (getsockopt(*sock, SOL_SOCKET, SO_RCVBUF, (char*)&optVal, &optLen) != SOCKET_ERROR)
		fprintf(clientPrintStat, "Checking SockOpt SO_RCVBUF Value: %ld\n", optVal);

	////////////////////////////////////////////////////////////////////////////////////////
	if (getsockopt(*sock, SOL_SOCKET, SO_SNDBUF, (char*)&optVal, &optLen) != SOCKET_ERROR)
		fprintf(clientPrintStat, "Checking SockOpt SO_SNDBUF Value: %ld", optVal);

	iResult = setsockopt(*sock, SOL_SOCKET, SO_SNDBUF, (char *) &optValNew, optLenNew);
	if (iResult == SOCKET_ERROR) {
		fprintf(clientPrintStat, "\tsetsockopt for failed with error: %u\n", WSAGetLastError());
	} else
		fprintf(clientPrintStat, "\tSO_SNDBUF succeeded\n");

	if (getsockopt(*sock, SOL_SOCKET, SO_SNDBUF, (char*)&optVal, &optLen) != SOCKET_ERROR)
		fprintf(clientPrintStat, "Checking SockOpt SO_SNDBUF Value: %ld\n", optVal);
}

void NetworkStatus::updateLocalVarsToSend() {
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
	copy16mat(&varsLocal.controllerMats[0], *varsPoint_ptr->controllerMats[0]);
	copy16mat(&varsLocal.controllerMats[1], *varsPoint_ptr->controllerMats[1]);
	varsLocal.controllerLoc[0] = *varsPoint_ptr->controllerLoc[0];
	varsLocal.controllerLoc[1] = *varsPoint_ptr->controllerLoc[1];
}
void NetworkStatus::updateVarsPointToRecv() {
	*varsPoint_ptr->minutes = varsLocal.minutes;
	*varsPoint_ptr->seconds = varsLocal.seconds;
	*varsPoint_ptr->dataMon[0] = varsLocal.dataMon[0];
	*varsPoint_ptr->dataMon[1] = varsLocal.dataMon[1];
	*varsPoint_ptr->inputLag[1] = varsLocal.inputLag[1];
}

void NetworkStatus::statusSend(){
	//fwprintf(clientPrintStat, L"Indicating ready\t"); 
	memset(statusBuffer, 0, statusSize);
	memcpy(statusBuffer, &varsLocal, statusSize);
	int iResult = send(connectSocket_A, statusBuffer, statusSize, 0);
	checkError_MaybeExit_SndRcV(iResult, L"indicateReady", statusSize);
	dataAccSend += iResult;
	//fwprintf(clientPrintStat, L"Status sent\n");

	if(varsLocal.running){ // TODO may not be the correct exit point
		Sleep(200); //Give time for the client to recv
		cleanUpSockets();
		pthread_exit((void*)0);
	}
}
void NetworkStatus::waitOnOther(){
	//fwprintf(clientPrintStat, L"Wating on other\t");
	memset(statusBuffer, 0, statusSize);
	int iResult = recv(connectSocket_A, statusBuffer, statusSize, 0);
	checkError_MaybeExit_SndRcV(iResult, L"waitOnOther", statusSize);
	dataAccRecv += iResult;

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

void NetworkStatus::networkStatus(void * arg)
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
	float * surfaceZ = (float*)(inputs[16]);
	err  = fopen_s( &clientPrintStat, "clientPrintStat.txt","w");

	updateLocalVarsToSend();
	//############# Start Step 1 init ######################
	XMUINT2 RENDER_B_DATASIZE;
	RENDER_B_DATASIZE.x = 0; RENDER_B_DATASIZE.y = 0;
	double t21 = 0.0;
	statusTimer = 0.0;
	waitTimer = 0.0;
	int iResult = 0;
	loopCnt = 0;
	dataAccRecv = 0;
	totalRecv = 0.0;
	timerStat.CounterStart = 0;
	timerStat.PCFreq = 0;
	timerWait.CounterStart = 0;
	timerWait.PCFreq = 0;
	int satusBuff_size = sizeof(int);
	int addressSize = sizeof (address_A);

	initServer(addressSize);

	UINT wtf = sizeof(quatLocXZ);
	UINT wtf2 = sizeof(statusVariables);

	//##### Start Step 2 receive and translate <doesn't wait for anything to start> #####

	StartCounterClient_ms(timerStat);

	for(;;){

		//>>>>>>>>>>>>>>>>>>>>Send>>>>>>>>>>>>>>>>>>
		updateLocalVarsToSend();
		
		statusSend();

		loopCnt++;			

		if (loopCnt % PERF_AVG_CNT == 0){
			statusTimer = GetCounterClient_ms(timerStat);
			fwprintf(clientPrintStat, L"<<< Status Frame %d >>>\n", loopCnt);
			fprintf(clientPrintStat, "--statusTimer: %5.2f\n", (float)statusTimer / (float)PERF_AVG_CNT);
			fprintf(clientPrintStat, "--waitTimer:   %5.2f\n", (float)waitTimer / (float)PERF_AVG_CNT);
			statusTimer = 0.0;
			waitTimer = 0.0;
			StartCounterClient_ms(timerStat);
		}

		//<<<<<<<<<<<<<<<<<<<<Recv<<<<<<<<<<<<<<<<<<
		StartCounterClient_ms(timerWait);
		waitOnOther(); // Status recive
		waitTimer = GetCounterClient_ms(timerWait);
	}

	//############# End Step 2 receive and translate ############

	// Close the socket when finished receiving datagrams
	fwprintf(clientPrintStat, L"Finished receiving. Closing socket out of loop, a issue.\n");
	iResult = closesocket(connectSocket_A);
	checkError_MaybeExit_basic(iResult, L"closesocket_A");

	// Clean up and exit.
	cleanUpSockets();
}

extern "C" void * networkStatusThread(void * arg)
{
	netStat.networkStatus(arg);
	return 0;
}