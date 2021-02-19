#pragma once

#include "Render_DX12.h"
#include "ObjectUpdateLogic.h"
#include "PolynomialRegression.h"

UpdateClass *UCP;

extern XMFLOAT3 multiply(float f, XMFLOAT3 a);
extern XMFLOAT3 subtract(XMFLOAT3 a, XMFLOAT3 b);
extern "C" void * serverThread(void *arg);
extern "C" void * networkStatusThread(void * arg);
extern "C" void * networkDataThread(void * arg);
extern "C" void * controllerWork(void * arg);

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE prevInstance,
    PSTR cmdLine, int showCmd)
{
    // Enable run-time memory check for debug builds.
#if defined(DEBUG) | defined(_DEBUG)
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif

    try
    {
		UpdateClass theApp(hInstance);
		
		UCP = &theApp;

        if(!theApp.Initialize())
            return 0;

		return theApp.Run();
    }
    catch(DxException& e)
    {
        MessageBox(nullptr, e.ToString().c_str(), L"HR Failed", MB_OK);
        return 0;
    }
}

RenderDX12::RenderDX12(HINSTANCE hInstance)
    : D3DApp(hInstance)
{
	LEAP_SEG_CNT = hd1.getNumSeg();
	LS = new LeapSegments(LEAP_SEG_CNT); /////////////////////////////For hands segments buffer data/////
}

RenderDX12::~RenderDX12()
{

	end_CurrentTest = true;

	while (NetworkLive && useNetworkCompute) {
		Sleep(2);
	}

	if (useNetworkCompute){
		if (USEstatData) {
			int err = pthread_join(netDataThread, &returnValDT);
			if (err != 0) { wprintf(L"Data Socket: Thread exit code: %d, thread returned: %d\n", err, (int)returnValDT); }
			else { wprintf(L"Socket: Thread exited and returned: %d\n", (int)returnValDT); }

			err = pthread_join(netStatusThread, &returnValST);
			if (err != 0) { wprintf(L"Status Socket: Thread exit code: %d, thread returned: %d\n", err, (int)returnValST); }
			else { wprintf(L"Socket: Thread exited and returned: %d\n", (int)returnValST); }

			WSACleanup();
		}
		else {
			int err = pthread_join(netDataThread, &returnValDT);
			if (err != 0) { wprintf(L"Data Socket: Thread exit code: %d, thread returned: %d\n", err, (int)returnValDT); }
			else { wprintf(L"Socket: Thread exited and returned: %d\n", (int)returnValDT); }
		}
	}
	
	if (clientPrint != NULL)
		fclose(clientPrint);
	if (md3dDevice != nullptr) {
		FlushCommandQueue();
		FlushTextData();
	}
	if(!useNetworkCompute)
		UCP->closeUpdateLogic();

	if (ComputeShaderActive) {
		UCP->CloseComputeBuffers();
	}

	saferDelete(host_segLocColor);
	saferDelete(host_segLoc);
	saferDelete(host_segColor);
	saferDelete(host_motorLocations);
	saferDelete(host_motorQuatLocXZ);
	saferDelete(host_motorQuat);
	saferDelete(MTCountsMain);

	delete LS;
}
void RenderDX12::saferDelete(void * dataToDelete) {
	if (dataToDelete != NULL) {
		delete[] dataToDelete;
		dataToDelete = NULL;
	}
}

void RenderDX12::StartCounter_ms(timer_ms &t) {
	LARGE_INTEGER li;
	if (!QueryPerformanceFrequency(&li))
		MessageBox(0, L"QueryPerformanceFrequency failed!", 0, 0);

	t.PCFreq = double(li.QuadPart) / 1000.0;

	QueryPerformanceCounter(&li);
	t.CounterStart = li.QuadPart;
}
double RenderDX12::GetCounter_ms(timer_ms &t)
{
	LARGE_INTEGER li;
	QueryPerformanceCounter(&li);
	return double(li.QuadPart - t.CounterStart) / t.PCFreq;
}

bool RenderDX12::InitControllerThread() {

	///void * cInputs[9]; //Cannot be here becuse this function is short lived and cInputs would be destoyed before the thread has a chance to read it. 
	cInputs[0] = &vrActivate;
	cInputs[1] = &mCamera;
	cInputs[2] = &end_CurrentTest; // Used when exiting APP with VR enabled
	cInputs[3] = m_pOpenVRSystem; // Can be used when Exiting VR but keeping APP running
	cInputs[4] = &vc;
	cInputs[5] = mOpaqueRitems[getInstIdx("controller")];
	cInputs[6] = mOpaqueRitems[getInstIdx("line")];
	cInputs[7] = m_poseOpenVR;
	cInputs[8] = m_pOpenVRCompositor;
	
	int err = pthread_create(&controllerThread, NULL, controllerWork, cInputs);
	if (err != 0) { wprintf(L"Can't Create Controller thread error code %d\n", err); }
	else { wprintf(L"Created Controller thread\n"); }

	return 0;
}
bool RenderDX12::CloseControllerThread() {

	int err = pthread_join(controllerThread, &returnValCT);
	if (err != 0) { wprintf(L"Controller: Thread exit code: %d, thread returned: %d\n", err, (int)returnValCT); }
	else { wprintf(L"Controller: Thread exited and returned: %d\n", (int)returnValCT); }

	return 0;
}

void RenderDX12::OnResize()
{
	D3DApp::OnResize();

	if (STEREO_RENDER) {
		mCamera.SetLens(0.25f*MathHelper::Pi, AspectRatio()/2.0f, 1.0f, CAMERA_ZFAR);
	}
	else {
		mCamera.SetLens(0.25f*MathHelper::Pi, AspectRatio(), 1.0f, CAMERA_ZFAR);
	}

	BoundingFrustum::CreateFromMatrix(mCamFrustum, mCamera.GetProj());
}

void RenderDX12::Update(const GameTimer& gt)
{
	if (simInputLag.y > simInputLag.x) {
		timeCumulativeIpLag += GetCounter_ms(timerSimLag);
		simLagStart = true;
		simInputLag.x = simInputLag.y;

		inputLagCount++;
		if (inputLagCount > 100) {
			timeModIpLag = timeCumulativeIpLag / 100.0;
			timeCumulativeIpLag = 0;
			inputLagCount = 0;
		}
	}
	
	if (simLagStart) {
		simLagStart = false;
		StartCounter_ms(timerSimLag);
	}
	
	OnKeyboardInput(gt);

	if (0) {
		cameraTeta += gt.DeltaTime() / 50.f;
		float cx = (spaceDimZ_sg + 300) * cos(cameraTeta);
		float cz = (spaceDimZ_sg + 300) * sin(cameraTeta);
		mCamera.LookAt(XMFLOAT3(cx, spaceDimY_sg / 2, cz), XMFLOAT3(0, spaceDimY_sg / 2, 0), XMFLOAT3(0, 1, 0));
		Sleep(3);
	}

    // Cycle through the circular frame resource array.
    mCurrFrameResourceIndex = (mCurrFrameResourceIndex + 1) % gNumFrameResources;
    mCurrFrameResource = mFrameResources[mCurrFrameResourceIndex].get();

    // Has the GPU finished processing the commands of the current frame resource?
    // If not, wait until the GPU has completed commands up to this fence point.
    if(mCurrFrameResource->Fence != 0 && mFence->GetCompletedValue() < mCurrFrameResource->Fence)
    {
        HANDLE eventHandle = CreateEventEx(nullptr, false, false, EVENT_ALL_ACCESS);
        ThrowIfFailed(mFence->SetEventOnCompletion(mCurrFrameResource->Fence, eventHandle));
        WaitForSingleObject(eventHandle, INFINITE);
        CloseHandle(eventHandle);
    }

	//UCP->FlushCommandQueueCS();


	AnimateMaterials(gt);
	UINT rc = UpdateInstanceData(gt);
	UpdateMaterialBuffer(gt);

	if (STEREO_RENDER) {
		UpdateMainPassCBStereo(gt);
	}
	else {
		UpdateMainPassCB(gt);
	}

	//----------------------------------------------------------------------------------------------------------
	// Get the timestamp values from the result buffers. Calculate the GPU execution time in microseconds.
	D3D12_RANGE readRange = {};
	const D3D12_RANGE emptyRange = {};

	readRange.Begin = 0;
	readRange.End = readRange.Begin + 2 * sizeof(UINT64);

	void* pData = nullptr;
	ThrowIfFailed(mCurrFrameResource->mTimestampResultBuffers->Map(0, &readRange, &pData));
	const UINT64* pTimestamps = reinterpret_cast<UINT64*>(static_cast<UINT8*>(pData) + readRange.Begin);
	const UINT64 timeStampDelta = pTimestamps[1] - pTimestamps[0];
	mCurrFrameResource->mTimestampResultBuffers->Unmap(0, &emptyRange);
	const UINT64 gpuTimeUS = (timeStampDelta * 1000000) / mDirectCommandQueueTimestampFrequencies;
	accumDXRender += gpuTimeUS;

	if (ComputeShaderActive) {
		void* pDataCS = nullptr;
		ThrowIfFailed(UCP->mTimestampResultBuffers_Sim->Map(0, &readRange, &pDataCS));
		const UINT64* pTimestampsCS = reinterpret_cast<UINT64*>(static_cast<UINT8*>(pDataCS) + readRange.Begin);
		const UINT64 timeStampDeltaCS = pTimestampsCS[1] - pTimestampsCS[0];
		UCP->mTimestampResultBuffers_Sim->Unmap(0, &emptyRange);
		const UINT64 CSTimeUS = (timeStampDeltaCS * 1000000) / mDirectCommandQueueTimestampFrequencies;
		accumDXCSsim += CSTimeUS;
	}

	if (useCSInstanceCopy) {
		void* pDataCS = nullptr;
		ThrowIfFailed(UCP->mTimestampResultBuffers_Cpy->Map(0, &readRange, &pDataCS));
		const UINT64* pTimestampsCS = reinterpret_cast<UINT64*>(static_cast<UINT8*>(pDataCS) + readRange.Begin);
		const UINT64 timeStampDeltaCS = pTimestampsCS[1] - pTimestampsCS[0];
		UCP->mTimestampResultBuffers_Cpy->Unmap(0, &emptyRange);
		const UINT64 CSTimeUS = (timeStampDeltaCS * 1000000) / mDirectCommandQueueTimestampFrequencies;
		accumDXCScpy += CSTimeUS;
	}

	if (frameCount % 40 == 0) {
		timerDXRender = ( accumDXRender / 40.0 ) / 1000.0; //Use Constantly increasing average ?
		accumDXRender = 0;
		timerDXCSsim = (accumDXCSsim / 40.0) / 1000.0; 
		accumDXCSsim = 0;
		timerDXCScpy = (accumDXCScpy / 40.0) / 1000.0;
		accumDXCScpy = 0;
	}
	//----------------------------------------------------------------------------------------------------------

	std::wostringstream outs;
	outs.precision(4);
	

	if (1) {
		outs << L"(Gutmann)" <<
			L"  " << rc <<
			L" objects: " <<
			L" Picking: " << timeModPick <<
			L" Frame: " << frameCount <<
			L" VR Sync: " << timeModVRSync;
	}
	else {
		outs << L"(Gutmann)";
	}
	mMainWndCaption = outs.str();

	frameCount++;
}

void RenderDX12::Draw(const GameTimer& gt)
{
	StartCounter_ms(timerDraw);
	if ((lastCapturedSec != (int)time_simulatedSeconds) && ((int)time_simulatedSeconds % CAPTURE_INTERVAL_SEC == 0)) {
		if(saveImage)
			DrawUpscale(gt);
	}

    auto cmdListAlloc = mCurrFrameResource->CmdListAlloc;

    // Reuse the memory associated with command recording.
    // We can only reset when the associated command lists have finished execution on the GPU.
    ThrowIfFailed(cmdListAlloc->Reset());

	PIXBeginEvent(mCommandQueue.Get(), 0, L"Render 3D");
    // A command list can be reset after it has been added to the command queue via ExecuteCommandList.
    // Reusing the command list reuses memory.
    
	ThrowIfFailed(mCommandList->Reset(cmdListAlloc.Get(), mPSOs["texColor"].Get()));

    mCommandList->RSSetViewports(1, &mScreenViewport);
    mCommandList->RSSetScissorRects(1, &mScissorRect);

	XMVECTORF32 backGroundColor = Colors::Black;
    // Indicate a state transition on the resource usage.
	if (!m4xMsaaState) {
		mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(CurrentBackBuffer(), D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET));
		// Clear the back buffer and depth buffer.
		mCommandList->ClearRenderTargetView(CurrentBackBufferView(), backGroundColor, 0, nullptr);
		mCommandList->ClearDepthStencilView(DepthStencilView(), D3D12_CLEAR_FLAG_DEPTH | D3D12_CLEAR_FLAG_STENCIL, 1.0f, 0, 0, nullptr);
		// Specify the buffers we are going to render to.
		mCommandList->OMSetRenderTargets(1, &CurrentBackBufferView(), true, &DepthStencilView());
	}
	else {
		mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(msaaResource.Get(), D3D12_RESOURCE_STATE_RESOLVE_SOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET));
		// Clear the back buffer and depth buffer.
		CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle = (const CD3DX12_CPU_DESCRIPTOR_HANDLE)mRtvMSAAHeap->GetCPUDescriptorHandleForHeapStart();
		mCommandList->ClearRenderTargetView(rtvHandle, backGroundColor, 0, nullptr);
		mCommandList->ClearDepthStencilView(DepthStencilView(), D3D12_CLEAR_FLAG_DEPTH | D3D12_CLEAR_FLAG_STENCIL, 1.0f, 0, 0, nullptr);
		// Specify the buffers we are going to render to.
		mCommandList->OMSetRenderTargets(1, &rtvHandle, true, &DepthStencilView());
	}

	ID3D12DescriptorHeap* descriptorHeaps[] = { mSrvDescriptorHeap.Get() };
	mCommandList->SetDescriptorHeaps(_countof(descriptorHeaps), descriptorHeaps);

	mCommandList->SetGraphicsRootSignature(mRootSignature.Get());

	// Bind all the materials used in this scene.  For structured buffers, we can bypass the heap and 
	// set as a root descriptor.
	auto matBuffer = mCurrFrameResource->MaterialBuffer->Resource();
	mCommandList->SetGraphicsRootShaderResourceView(1, matBuffer->GetGPUVirtualAddress());

	auto passCB = mCurrFrameResource->PassCBL->Resource();
	mCommandList->SetGraphicsRootConstantBufferView(2, passCB->GetGPUVirtualAddress());

	// Bind all the textures used in this scene.
	mCommandList->SetGraphicsRootDescriptorTable(3, mSrvDescriptorHeap->GetGPUDescriptorHandleForHeapStart());

	// Get a timestamp at the start of the command list.
	const UINT timestampHeapIndex = 0;
	mCommandList->EndQuery(mCurrFrameResource->mTimestampQueryHeaps.Get(), D3D12_QUERY_TYPE_TIMESTAMP, timestampHeapIndex);

    DrawRenderItems(mCommandList.Get(), mOpaqueRitems);
	
	// Get a timestamp at the end of the command list and resolve the query data.
	mCommandList->EndQuery(mCurrFrameResource->mTimestampQueryHeaps.Get(), D3D12_QUERY_TYPE_TIMESTAMP, timestampHeapIndex + 1);
	mCommandList->ResolveQueryData(mCurrFrameResource->mTimestampQueryHeaps.Get(), D3D12_QUERY_TYPE_TIMESTAMP, timestampHeapIndex, 2, mCurrFrameResource->mTimestampResultBuffers.Get(), timestampHeapIndex * sizeof(UINT64));

    // Indicate a state transition on the resource usage.
	if (!m4xMsaaState) {
		mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(CurrentBackBuffer(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT));
	}
	else {
		mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(msaaResource.Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_RESOLVE_SOURCE));
		mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(CurrentBackBuffer(), D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RESOLVE_DEST));
		UINT num = D3D12CalcSubresource(0, 0, 0, 1, 1);
		mCommandList->ResolveSubresource(CurrentBackBuffer(), num, msaaResource.Get(), num, DXGI_FORMAT_R8G8B8A8_UNORM);
		mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(CurrentBackBuffer(), D3D12_RESOURCE_STATE_RESOLVE_DEST, D3D12_RESOURCE_STATE_PRESENT));
	}

    // Done recording commands.
    ThrowIfFailed(mCommandList->Close());

#if VR_EXPLICIT_TIMING
	if (m_pOpenVRSystem)
		m_pOpenVRCompositor->SubmitExplicitTimingData();
#endif


    // Add the command list to the queue for execution.
    ID3D12CommandList* cmdsLists[] = { mCommandList.Get() };
    mCommandQueue->ExecuteCommandLists(_countof(cmdsLists), cmdsLists);
	PIXEndEvent(mCommandQueue.Get());
	
	if (m_pOpenVRSystem) { SubmitToHMD(); }

	//ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo

	PIXBeginEvent(mCommandQueue.Get(), 0, L"Render UI");
	StartCounter_ms(timer_a);
	
	if (mTextToScreen)
		RenderUI();

	timeCumulativeText += GetCounter_ms(timer_a);
	if (frameCount % 40 == 0) {
		timeModText = timeCumulativeText / 40.f;
		timeCumulativeText = 0;
	}
	PIXEndEvent(mCommandQueue.Get());

    // Swap the back and front buffers
    ThrowIfFailed(mSwapChain->Present(0, 0));

#if VR_EXPLICIT_TIMING
	if (m_pOpenVRSystem)
		m_pOpenVRCompositor->PostPresentHandoff();
#endif

	if ((lastCapturedSec != (int)time_simulatedSeconds) && ((int)time_simulatedSeconds % CAPTURE_INTERVAL_SEC == 0)) {
		lastCapturedSec = (int)time_simulatedSeconds;
		//PrintScreenImage(0);
	}

	mCurrBackBuffer = (mCurrBackBuffer + 1) % SwapChainBufferCount;

    // Advance the fence value to mark commands up to this fence point.
    mCurrFrameResource->Fence = ++mCurrentFence;

    // Add an instruction to the command queue to set a new fence point. 
    // Because we are on the GPU timeline, the new fence point won't be 
    // set until the GPU finishes processing all the commands prior to this Signal().
    mCommandQueue->Signal(mFence.Get(), mCurrentFence);

	timeCumulativeDraw += GetCounter_ms(timerDraw);
	if (frameCount % 40 == 0) {
		timeModDraw = timeCumulativeDraw / 40.f;
		timeCumulativeDraw = 0;
	}
}

void RenderDX12::DrawStereo(const GameTimer& gt)
{
	StartCounter_ms(timerDraw);
	if ((lastCapturedSec != (int)time_simulatedSeconds) && ((int)time_simulatedSeconds % CAPTURE_INTERVAL_SEC == 0)) {
		if (saveImage)
			DrawUpscale(gt);
	}

	auto cmdListAlloc = mCurrFrameResource->CmdListAlloc;

	// Reuse the memory associated with command recording.
	// We can only reset when the associated command lists have finished execution on the GPU.
	ThrowIfFailed(cmdListAlloc->Reset());

	PIXBeginEvent(mCommandQueue.Get(), 0, L"Render 3D");
	// A command list can be reset after it has been added to the command queue via ExecuteCommandList.
	// Reusing the command list reuses memory.

	ThrowIfFailed(mCommandList->Reset(cmdListAlloc.Get(), mPSOs["texColor"].Get()));

	mCommandList->RSSetViewports(1, &m_viewportA[0]);
	mCommandList->RSSetScissorRects(1, &m_scissorRectA[0]);

	XMVECTORF32 backGroundColor = Colors::Black;
	// Indicate a state transition on the resource usage.
	if (!m4xMsaaState) {
		mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(CurrentBackBuffer(), D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET));
		// Clear the back buffer and depth buffer.
		mCommandList->ClearRenderTargetView(CurrentBackBufferView(), backGroundColor, 1, &m_scissorRectA[0]);
		mCommandList->ClearDepthStencilView(DepthStencilView(), D3D12_CLEAR_FLAG_DEPTH | D3D12_CLEAR_FLAG_STENCIL, 1.0f, 0, 1, &m_scissorRectA[0]);
		// Specify the buffers we are going to render to.
		mCommandList->OMSetRenderTargets(1, &CurrentBackBufferView(), true, &DepthStencilView());
	}
	else {
		mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(msaaResource.Get(), D3D12_RESOURCE_STATE_RESOLVE_SOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET));
		// Clear the back buffer and depth buffer.
		CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle = (const CD3DX12_CPU_DESCRIPTOR_HANDLE)mRtvMSAAHeap->GetCPUDescriptorHandleForHeapStart();
		mCommandList->ClearRenderTargetView(rtvHandle, backGroundColor, 1, &m_scissorRectA[0]);
		mCommandList->ClearDepthStencilView(DepthStencilView(), D3D12_CLEAR_FLAG_DEPTH | D3D12_CLEAR_FLAG_STENCIL, 1.0f, 0, 1, &m_scissorRectA[0]);
		// Specify the buffers we are going to render to.
		mCommandList->OMSetRenderTargets(1, &rtvHandle, true, &DepthStencilView());
	}

	ID3D12DescriptorHeap* descriptorHeaps[] = { mSrvDescriptorHeap.Get() };
	mCommandList->SetDescriptorHeaps(_countof(descriptorHeaps), descriptorHeaps);

	mCommandList->SetGraphicsRootSignature(mRootSignature.Get());

	// Bind all the materials used in this scene.  For structured buffers, we can bypass the heap and 
	// set as a root descriptor.
	auto matBuffer = mCurrFrameResource->MaterialBuffer->Resource();
	mCommandList->SetGraphicsRootShaderResourceView(1, matBuffer->GetGPUVirtualAddress());

	auto passCBL = mCurrFrameResource->PassCBL->Resource();
	mCommandList->SetGraphicsRootConstantBufferView(2, passCBL->GetGPUVirtualAddress());

	// Bind all the textures used in this scene.
	mCommandList->SetGraphicsRootDescriptorTable(3, mSrvDescriptorHeap->GetGPUDescriptorHandleForHeapStart());

	// Get a timestamp at the start of the command list.
	const UINT timestampHeapIndex = 0;
	mCommandList->EndQuery(mCurrFrameResource->mTimestampQueryHeaps.Get(), D3D12_QUERY_TYPE_TIMESTAMP, timestampHeapIndex);

	DrawRenderItems(mCommandList.Get(), mOpaqueRitems);

	///===================================================================================
	auto passCBR = mCurrFrameResource->PassCBR->Resource();
	mCommandList->SetGraphicsRootConstantBufferView(2, passCBR->GetGPUVirtualAddress());

	//backGroundColor = Colors::Silver;
	if (!m4xMsaaState) {
		// Clear the back buffer and depth buffer.
		mCommandList->ClearRenderTargetView(CurrentBackBufferView(), backGroundColor, 1, &m_scissorRectA[1]);
		mCommandList->ClearDepthStencilView(DepthStencilView(), D3D12_CLEAR_FLAG_DEPTH | D3D12_CLEAR_FLAG_STENCIL, 1.0f, 0, 1, &m_scissorRectA[1]);
		// Specify the buffers we are going to render to.
	}
	else {
		// Clear the back buffer and depth buffer.
		CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle = (const CD3DX12_CPU_DESCRIPTOR_HANDLE)mRtvMSAAHeap->GetCPUDescriptorHandleForHeapStart();
		mCommandList->ClearRenderTargetView(rtvHandle, backGroundColor, 1, &m_scissorRectA[1]);
		mCommandList->ClearDepthStencilView(DepthStencilView(), D3D12_CLEAR_FLAG_DEPTH | D3D12_CLEAR_FLAG_STENCIL, 1.0f, 0, 1, &m_scissorRectA[1]);
		// Specify the buffers we are going to render to.
	}

	mCommandList->RSSetViewports(1, &m_viewportA[1]);
	mCommandList->RSSetScissorRects(1, &m_scissorRectA[1]);
	DrawRenderItems(mCommandList.Get(), mOpaqueRitems);
	///===================================================================================

	// Get a timestamp at the end of the command list and resolve the query data.
	mCommandList->EndQuery(mCurrFrameResource->mTimestampQueryHeaps.Get(), D3D12_QUERY_TYPE_TIMESTAMP, timestampHeapIndex + 1);
	mCommandList->ResolveQueryData(mCurrFrameResource->mTimestampQueryHeaps.Get(), D3D12_QUERY_TYPE_TIMESTAMP, timestampHeapIndex, 2, mCurrFrameResource->mTimestampResultBuffers.Get(), timestampHeapIndex * sizeof(UINT64));

	// Indicate a state transition on the resource usage.
	if (!m4xMsaaState) {
		mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(CurrentBackBuffer(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT));
	}
	else {
		mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(msaaResource.Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_RESOLVE_SOURCE));
		mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(CurrentBackBuffer(), D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RESOLVE_DEST));
		UINT num = D3D12CalcSubresource(0, 0, 0, 1, 1);
		mCommandList->ResolveSubresource(CurrentBackBuffer(), num, msaaResource.Get(), num, DXGI_FORMAT_R8G8B8A8_UNORM);
		mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(CurrentBackBuffer(), D3D12_RESOURCE_STATE_RESOLVE_DEST, D3D12_RESOURCE_STATE_PRESENT));
	}

	// Done recording commands.
	ThrowIfFailed(mCommandList->Close());

#if VR_EXPLICIT_TIMING
	if (m_pOpenVRSystem)
		m_pOpenVRCompositor->SubmitExplicitTimingData();
#endif

	// Add the command list to the queue for execution.
	ID3D12CommandList* cmdsLists[] = { mCommandList.Get() };
	mCommandQueue->ExecuteCommandLists(_countof(cmdsLists), cmdsLists);
	PIXEndEvent(mCommandQueue.Get());

	if (m_pOpenVRSystem) { SubmitToHMD(); } // If this is after Present then the text will appear on the VR headset

	//ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo

	PIXBeginEvent(mCommandQueue.Get(), 0, L"Render UI");
	StartCounter_ms(timer_a);

	if (mTextToScreen)
		RenderUI();

	timeCumulativeText += GetCounter_ms(timer_a);
	if (frameCount % 40 == 0) {
		timeModText = timeCumulativeText / 40.f;
		timeCumulativeText = 0;
	}
	PIXEndEvent(mCommandQueue.Get());

	// Swap the back and front buffers
	ThrowIfFailed(mSwapChain->Present(0, 0));

#if VR_EXPLICIT_TIMING
	if (m_pOpenVRSystem)
		m_pOpenVRCompositor->PostPresentHandoff();
#endif

	if ((lastCapturedSec != (int)time_simulatedSeconds) && ((int)time_simulatedSeconds % CAPTURE_INTERVAL_SEC == 0)) {
		lastCapturedSec = (int)time_simulatedSeconds;
		//PrintScreenImage(0);
	}

	mCurrBackBuffer = (mCurrBackBuffer + 1) % SwapChainBufferCount;

	// Advance the fence value to mark commands up to this fence point.
	mCurrFrameResource->Fence = ++mCurrentFence;

	// Add an instruction to the command queue to set a new fence point. 
	// Because we are on the GPU timeline, the new fence point won't be 
	// set until the GPU finishes processing all the commands prior to this Signal().
	mCommandQueue->Signal(mFence.Get(), mCurrentFence);

	timeCumulativeDraw += GetCounter_ms(timerDraw);
	if (frameCount % 40 == 0) {
		timeModDraw = timeCumulativeDraw / 40.f;
		timeCumulativeDraw = 0;
	}
}

void RenderDX12::DrawUpscale(const GameTimer& gt)
{
	int saveWidth = mClientWidth;
	int saveHeight = mClientHeight;
	mClientWidth = 1920 * 4;
	mClientHeight = 1080 * 4;
	UpdateMainPassCB(gt);
	OnResize();

	auto cmdListAlloc = mCurrFrameResource->CmdListAlloc;

	// Reuse the memory associated with command recording.
	// We can only reset when the associated command lists have finished execution on the GPU.
	ThrowIfFailed(cmdListAlloc->Reset());

	PIXBeginEvent(mCommandQueue.Get(), 0, L"Render 3D");
	// A command list can be reset after it has been added to the command queue via ExecuteCommandList.
	// Reusing the command list reuses memory.

	ThrowIfFailed(mCommandList->Reset(cmdListAlloc.Get(), mPSOs["texColor"].Get()));

	mCommandList->RSSetViewports(1, &mScreenViewport);
	mCommandList->RSSetScissorRects(1, &mScissorRect);

	// Indicate a state transition on the resource usage.
	if (!m4xMsaaState) {
		mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(CurrentBackBuffer(), D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET));
		// Clear the back buffer and depth buffer.
		mCommandList->ClearRenderTargetView(CurrentBackBufferView(), Colors::LightSteelBlue, 0, nullptr);
		mCommandList->ClearDepthStencilView(DepthStencilView(), D3D12_CLEAR_FLAG_DEPTH | D3D12_CLEAR_FLAG_STENCIL, 1.0f, 0, 0, nullptr);
		// Specify the buffers we are going to render to.
		mCommandList->OMSetRenderTargets(1, &CurrentBackBufferView(), true, &DepthStencilView());
	}
	else {
		mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(msaaResource.Get(), D3D12_RESOURCE_STATE_RESOLVE_SOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET));
		// Clear the back buffer and depth buffer.
		CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle = (const CD3DX12_CPU_DESCRIPTOR_HANDLE)mRtvMSAAHeap->GetCPUDescriptorHandleForHeapStart();
		mCommandList->ClearRenderTargetView(rtvHandle, Colors::LightSteelBlue, 0, nullptr);
		mCommandList->ClearDepthStencilView(DepthStencilView(), D3D12_CLEAR_FLAG_DEPTH | D3D12_CLEAR_FLAG_STENCIL, 1.0f, 0, 0, nullptr);
		// Specify the buffers we are going to render to.
		mCommandList->OMSetRenderTargets(1, &rtvHandle, true, &DepthStencilView());
	}

	ID3D12DescriptorHeap* descriptorHeaps[] = { mSrvDescriptorHeap.Get() };
	mCommandList->SetDescriptorHeaps(_countof(descriptorHeaps), descriptorHeaps);

	mCommandList->SetGraphicsRootSignature(mRootSignature.Get());

	// Bind all the materials used in this scene.  For structured buffers, we can bypass the heap and 
	// set as a root descriptor.
	auto matBuffer = mCurrFrameResource->MaterialBuffer->Resource();
	mCommandList->SetGraphicsRootShaderResourceView(1, matBuffer->GetGPUVirtualAddress());

	auto passCBL = mCurrFrameResource->PassCBL->Resource();
	mCommandList->SetGraphicsRootConstantBufferView(2, passCBL->GetGPUVirtualAddress());

	// Bind all the textures used in this scene.
	mCommandList->SetGraphicsRootDescriptorTable(3, mSrvDescriptorHeap->GetGPUDescriptorHandleForHeapStart());

	// Get a timestamp at the start of the command list.
	const UINT timestampHeapIndex = 0;
	mCommandList->EndQuery(mCurrFrameResource->mTimestampQueryHeaps.Get(), D3D12_QUERY_TYPE_TIMESTAMP, timestampHeapIndex);

	DrawRenderItems(mCommandList.Get(), mOpaqueRitems);

	// Get a timestamp at the end of the command list and resolve the query data.
	mCommandList->EndQuery(mCurrFrameResource->mTimestampQueryHeaps.Get(), D3D12_QUERY_TYPE_TIMESTAMP, timestampHeapIndex + 1);
	mCommandList->ResolveQueryData(mCurrFrameResource->mTimestampQueryHeaps.Get(), D3D12_QUERY_TYPE_TIMESTAMP, timestampHeapIndex, 2, mCurrFrameResource->mTimestampResultBuffers.Get(), timestampHeapIndex * sizeof(UINT64));

	// Indicate a state transition on the resource usage.
	if (!m4xMsaaState) {
		mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(CurrentBackBuffer(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT));
	}
	else {
		mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(msaaResource.Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_RESOLVE_SOURCE));
		mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(CurrentBackBuffer(), D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RESOLVE_DEST));
		UINT num = D3D12CalcSubresource(0, 0, 0, 1, 1);
		mCommandList->ResolveSubresource(CurrentBackBuffer(), num, msaaResource.Get(), num, DXGI_FORMAT_R8G8B8A8_UNORM);
		mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(CurrentBackBuffer(), D3D12_RESOURCE_STATE_RESOLVE_DEST, D3D12_RESOURCE_STATE_PRESENT));
	}

	// Done recording commands.
	ThrowIfFailed(mCommandList->Close());

	// Add the command list to the queue for execution.
	ID3D12CommandList* cmdsLists[] = { mCommandList.Get() };
	mCommandQueue->ExecuteCommandLists(_countof(cmdsLists), cmdsLists);
	PIXEndEvent(mCommandQueue.Get());

	//ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo

	PIXBeginEvent(mCommandQueue.Get(), 0, L"Render UI");
	StartCounter_ms(timer_a);

	//if (mTextToScreen)
	//	RenderUI();

	timeCumulativeText += GetCounter_ms(timer_a);
	if (frameCount % 40 == 0) {
		timeModText = timeCumulativeText / 40.f;
		timeCumulativeText = 0;
	}
	PIXEndEvent(mCommandQueue.Get());

	// Swap the back and front buffers
	ThrowIfFailed(mSwapChain->Present(0, 0));

	//if (m_pOpenVRSystem) { SubmitToHMD(); }

	if ((lastCapturedSec != (int)time_simulatedSeconds) && ((int)time_simulatedSeconds % CAPTURE_INTERVAL_SEC == 0)) {
		lastCapturedSec = (int)time_simulatedSeconds;
		PrintScreenImage(1000);
	}

	
	mCurrBackBuffer = (mCurrBackBuffer + 1) % SwapChainBufferCount;

	// Advance the fence value to mark commands up to this fence point.
	mCurrFrameResource->Fence = ++mCurrentFence;

	// Add an instruction to the command queue to set a new fence point. 
	// Because we are on the GPU timeline, the new fence point won't be 
	// set until the GPU finishes processing all the commands prior to this Signal().
	mCommandQueue->Signal(mFence.Get(), mCurrentFence);/**/

	// Cycle through the circular frame resource array.
	//mCurrFrameResourceIndex = (mCurrFrameResourceIndex + 1) % gNumFrameResources;
	mCurrFrameResource = mFrameResources[mCurrFrameResourceIndex].get();

	// Has the GPU finished processing the commands of the current frame resource?
	// If not, wait until the GPU has completed commands up to this fence point.
	int jjj = mFence->GetCompletedValue();
	bool fff = mFence->GetCompletedValue() < mCurrFrameResource->Fence;
	bool ffg = mCurrFrameResource->Fence;
	if (mCurrFrameResource->Fence != 0 && mFence->GetCompletedValue() < mCurrFrameResource->Fence)
	{
		int de = 0;
		HANDLE eventHandle = CreateEventEx(nullptr, false, false, EVENT_ALL_ACCESS);
		ThrowIfFailed(mFence->SetEventOnCompletion(mCurrFrameResource->Fence, eventHandle));
		WaitForSingleObject(eventHandle, INFINITE);
		CloseHandle(eventHandle);
	}

	mClientWidth = saveWidth;
	mClientHeight = saveHeight;
	UpdateMainPassCB(gt);
	OnResize();
}

void RenderDX12::OnMouseDown(WPARAM btnState, int x, int y)
{
	if ((btnState & MK_LBUTTON) != 0)
	{
		mLastMousePos.x = x;
		mLastMousePos.y = y;

		SetCapture(mhMainWnd);
	}
	else if ((btnState & MK_RBUTTON) != 0)
	{
		MouseRightSearching = true;
		MouseRightPressed = true;
	}
}

void RenderDX12::OnMouseUp(WPARAM btnState, int x, int y)
{
    ReleaseCapture();

	if (MouseRightPressed) {
		MouseRightSearching = false;
		MouseRightPressed = false;
	}
}

void RenderDX12::OnMouseMove(WPARAM btnState, int x, int y)
{
    if((btnState & MK_LBUTTON) != 0)
    {
		// Make each pixel correspond to a quarter of a degree.
		float dx = XMConvertToRadians(0.25f*static_cast<float>(x - mLastMousePos.x));
		float dy = XMConvertToRadians(0.25f*static_cast<float>(y - mLastMousePos.y));

		mCamera.Pitch(dy);
		mCamera.RotateY(dx);
    }

    mLastMousePos.x = x;
    mLastMousePos.y = y;
}
 
void RenderDX12::DirectionalMovement(const GameTimer& gt) {
	const float dt = gt.DeltaTime();
	const float speed = 300.0f;

	if (!m_pOpenVRSystem) { 
		if (GetAsyncKeyState('W') & 0x8000)
			mCamera.Walk(speed*dt);

		if (GetAsyncKeyState('S') & 0x8000)
			mCamera.Walk(-speed*dt);

		if (GetAsyncKeyState('A') & 0x8000)
			mCamera.Strafe(-speed*dt);

		if (GetAsyncKeyState('D') & 0x8000)
			mCamera.Strafe(speed*dt);

		if (GetAsyncKeyState('Q') & 0x8000)
			mCamera.Rise(-speed*dt);

		if (GetAsyncKeyState('E') & 0x8000)
			mCamera.Rise(speed*dt);
	}
	else {
		if (GetAsyncKeyState('W') & 0x8000) {
			vc.VR_MoveOffset.x += mCamera.mLook.x * speed * dt;
			vc.VR_MoveOffset.y += mCamera.mLook.y * speed * dt;
			vc.VR_MoveOffset.z += mCamera.mLook.z * speed * dt;
		}

		if (GetAsyncKeyState('S') & 0x8000) {
			vc.VR_MoveOffset.x -= mCamera.mLook.x * speed * dt;
			vc.VR_MoveOffset.y -= mCamera.mLook.y * speed * dt;
			vc.VR_MoveOffset.z -= mCamera.mLook.z * speed * dt;
		}

		if (GetAsyncKeyState('A') & 0x8000) {
			vc.VR_MoveOffset.x -= mCamera.mRight.x * speed * dt;
			vc.VR_MoveOffset.y -= mCamera.mRight.y * speed * dt;
			vc.VR_MoveOffset.z -= mCamera.mRight.z * speed * dt;
		}

		if (GetAsyncKeyState('D') & 0x8000) {
			vc.VR_MoveOffset.x += mCamera.mRight.x * speed * dt;
			vc.VR_MoveOffset.y += mCamera.mRight.y * speed * dt;
			vc.VR_MoveOffset.z += mCamera.mRight.z * speed * dt;
		}

		if (GetAsyncKeyState('E') & 0x8000) {
			vc.VR_MoveOffset.x += mCamera.mUp.x * speed * dt;
			vc.VR_MoveOffset.y += mCamera.mUp.y * speed * dt;
			vc.VR_MoveOffset.z += mCamera.mUp.z * speed * dt;
		}

		if (GetAsyncKeyState('Q') & 0x8000) {
			vc.VR_MoveOffset.x -= mCamera.mUp.x * speed * dt;
			vc.VR_MoveOffset.y -= mCamera.mUp.y * speed * dt;
			vc.VR_MoveOffset.z -= mCamera.mUp.z * speed * dt;
		}
	}
}

void RenderDX12::OnKeyboardInput(const GameTimer& gt)
{
	const float dt = 5.0f*gt.DeltaTime();
	const float speed = 40.0f;

	DirectionalMovement(gt);

	if (GetAsyncKeyState('Z') & 0x8000)
		mCamera.RotateY(-dt);

	if (GetAsyncKeyState('X') & 0x8000)
		mCamera.RotateY(dt);

	if (GetAsyncKeyState('1') & 0x0001) {
		TryActivateVR();
		STEREO_RENDER = 1;
	}

	if (GetAsyncKeyState('2') & 0x0001) {
		DeactivateVR();
		STEREO_RENDER = 0;
	}

	if (GetAsyncKeyState('3') & 0x8000) {
		if (STEREO_RENDER) {
			VRoffsetStereo += 0.01;
			if (VRoffsetStereo > 2.2f)
				VRoffsetStereo = 2.2f;
		}
		else {
			VRoffsetFlat += 0.0001;
			if (VRoffsetFlat > 0.5f)
				VRoffsetFlat = 0.5f;
		}
		
	}
	if (GetAsyncKeyState('4') & 0x8000) {
		if (STEREO_RENDER) {
			VRoffsetStereo -= 0.01f;
			if (VRoffsetStereo < -0.0f)
				VRoffsetStereo = -0.0f;
		}
		else {
			VRoffsetFlat -= 0.0001;
			if (VRoffsetFlat < 0.0f)
				VRoffsetFlat = 0.0f;
		}
		
	}
	if (GetAsyncKeyState('5') & 0x0001) {
		fprintf(clientPrint, "%8.5f \n", (STEREO_RENDER) ? VRoffsetStereo : VRoffsetFlat);
	}

	if (GetAsyncKeyState('6') & 0x0001) {
		mCamera.Pitch(0.0);
		mCamera.LookAt(XMFLOAT3(0, 0, 0), XMFLOAT3(0, 0, 1), XMFLOAT3(0, 1, 0));
		mCamera.Rise(0.0);
	}

	if (GetAsyncKeyState('7') & 0x0001) {
		//mCamera.Pitch(0.0);
		//mCamera.LookAt(XMFLOAT3(0, 800, 0), XMFLOAT3(0, -1, 0), XMFLOAT3(0, 0, 1));
		//mCamera.Rise(0.0);
		
		//mCamera.LookAt(XMFLOAT3(0, 120, -90), XMFLOAT3(0, 0, 10), XMFLOAT3(0, 0, 1));

		mCamera.LookAt(XMFLOAT3(0, spaceDimY_sg / 2, -spaceDimZ_sg), XMFLOAT3(0, spaceDimY_sg / 2, 0), XMFLOAT3(0, 1, 0));
		//mCamera.LookAt(XMFLOAT3(2600, spaceDimY_sg / 2, 0), XMFLOAT3(0, spaceDimY_sg / 2, 0), XMFLOAT3(0, 1, 0));
	}

	if (GetAsyncKeyState(VK_F4) & 0x0001) {
		STEREO_RENDER = STEREO_RENDER ? 0 : 1;
	}

	if (GetAsyncKeyState(VK_F3) & 0x0001) {
		transparentPS = !transparentPS;
	}
	if (GetAsyncKeyState(VK_F2) & 0x0001) {
		//Not the cleanest way to change MSAA when using VR
		bool VRWasActive = false;
		if (m_pOpenVRSystem) { DeactivateVR(); VRWasActive = true; }

		Set4xMsaaState(!m4xMsaaState);

		if (VRWasActive) { TryActivateOpenVR(); }
	}

	if (GetAsyncKeyState('P') & 0x0001) {
		PAUSED = !PAUSED;
	}

	if (GetAsyncKeyState('O') & 0x0001) {
		holdWall = !holdWall;
		wallLeft = true;
	}
	if (GetAsyncKeyState('K') & 0x0001) {
		sendWall = !sendWall;
		wallLeft = true;
	}
	if (GetAsyncKeyState('L') & 0x0001) {
		sendWall = !sendWall;
		wallLeft = false;
	}
	if (GetAsyncKeyState('M') & 0x0001) {
		UpdateMotorColors();
	}
	if (GetAsyncKeyState('R') & 0x0001) {
		USE_REGION = !USE_REGION;
	}

	if (GetAsyncKeyState('G') & 0x0001) {
		param.GRAVITY_ON = !param.GRAVITY_ON;
	}
	if (GetAsyncKeyState('V') & 0x8000) {
		param.NEG_F_MULTIPLY += 0.001f;
		if (param.NEG_F_MULTIPLY > 2.0f)
			param.NEG_F_MULTIPLY = 2.0f;
	}
	if (GetAsyncKeyState('B') & 0x8000) {
		param.NEG_F_MULTIPLY -= 0.001f;
		if (param.NEG_F_MULTIPLY < 0.0f)
			param.NEG_F_MULTIPLY = 0.0f;
	}

	/*if (GetAsyncKeyState('H') & 0x8000) {
		VIVEtoWorldScaling += 1.0f;
		if (VIVEtoWorldScaling > 300.0f)
			VIVEtoWorldScaling = 300.0f;
	}
	if (GetAsyncKeyState('J') & 0x8000) {
		VIVEtoWorldScaling -= 1.0;
		if (VIVEtoWorldScaling < 20.0f)
			VIVEtoWorldScaling = 20.0f;
	}*/

	if (GetAsyncKeyState('N') & 0x0001) {
		NPRESS = !NPRESS;
	}

	if (GetAsyncKeyState('T') & 0x0001) {
		isFolded = !isFolded;
		isFoldChanged = true;
	}
	
	if (GetAsyncKeyState('Y') & 0x0001) {
		isForceApplied = !isForceApplied;
		isForceChanged = !isForceChanged;
	}

	if (GetAsyncKeyState('U') & 0x0001) {
		isRecording = !isRecording;
		isForceChanged = !isForceChanged;
	}
	
	mCamera.UpdateViewMatrix();
}
 
void RenderDX12::AnimateMaterials(const GameTimer& gt)
{
	

	//mAllRitems[mAllRitems.size() - 1]->Instances[0].MaterialIndex = (frameCount/100) % 2 + 9;

	if (!PAUSED) {
		//If this increments when paused visuals look like a short GIF loop
		currentInstanceBuffer = (currentInstanceBuffer + 1) % heapCount; // (must be 2 or 3)
	}
///##########################################################################################################
	StartCounter_ms(timerAnimate);
	if (!useNetworkCompute) {
		if (!PAUSED) {
			float bounTheta = 0.4f;
			int loopN = 1;
			for (int itr = 0; itr < loopN; ++itr)
			{
				INC_TEMP.x += (0.001f/(float)loopN) * INC_TEMP.y;
				if (abs(INC_TEMP.x) > bounTheta) {
					INC_TEMP.x = bounTheta * INC_TEMP.y;
					INC_TEMP.y *= -1.0f;
				}
				UCP->MainUpdateMethod(gt.DeltaTime(), LS); // LJP Simulation
			}

			if (holdWall) {
				UCP->holdWall(); // Extra
			}
			else if (sendWall) {
				UCP->sendWall(wallLeft); // Extra
				sendWall = false;
			}
			//---------User Interactions---------------------------------
			if (!m_pOpenVRSystem) {
				UCP->MouseInteractions();
			}
			else {
				UCP->VRControllerInteractions();
			}
			
			UCP->SimDataToInstData(getInstIdx("segments"));
		}
	}
///#########################################################################################################
	else {
		if (!PAUSED) {
			//Updating data that will be used to render
			if (USE_REGION) {
				LoadNetworkDataRegion();
			}
			else {
				LoadNetworkDataAll();
			}

			//Controller Overriding Code
			if (0) {
				TubeWandOverrideDebug(vc.controllerMatrixSave, vc.lastControllerLoc, 0, (int)tubeRadius*tubeLength * 2);
				TubeWandOverrideDebug(vc.controllerMatrixSave, vc.lastControllerLoc, 1, (int)tubeRadius*tubeLength * 3);
			}
			else if(0) {
				TubeWandOverride(vc.controllerMatrixSave, vc.lastControllerLoc, 0, (int)tubeRadius*tubeLength * 0);
				TubeWandOverride(vc.controllerMatrixSave, vc.lastControllerLoc, 1, (int)tubeRadius*tubeLength * 1);
			}
		}
	}
///#########################################################################################################
	timeCumulativeAnima += GetCounter_ms(timerAnimate);
	if (frameCount % 40 == 0) {
		timeModAnima = timeCumulativeAnima / 40.f;
		timeCumulativeAnima = 0;
	}
}

void RenderDX12::LoadNetworkDataRegion() {
	int up = 0;

		if (SEG_RenderCnt > inst1Max) {
			sphereRenderCnt1 = SEG_RenderCnt / 2;
			sphereRenderCnt2 = SEG_RenderCnt - sphereRenderCnt1;
		}
		else {
			sphereRenderCnt1 = SEG_RenderCnt;
			sphereRenderCnt2 = 0;
		}

		auto riSeg = mOpaqueRitems[getInstIdx("segments")];
		riSeg->InstanceCount = sphereRenderCnt1;
		auto riSeg2 = mOpaqueRitems[getInstIdx("segments") + 1];
		riSeg2->InstanceCount = sphereRenderCnt2;

#pragma omp parallel for private (up)
		for (up = 0; up < (int)sphereRenderCnt1; ++up)
		{
			//Updating data that will be used to render
			riSeg->Instances[up].World = XMFLOAT4X4(
				1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				host_segLocColor[up].x, host_segLocColor[up].y + 0.5f, host_segLocColor[up].z, 1.0f);

			/*//Debugging issue that when using a single Compute GPU the region algorithm as issues
			XMFLOAT4 TempColor = XMFLOAT4(0.0f, 0.9f, 0.3f, 1.0f);
			if (errLoc < 500 && (
				host_segLocColor[up].x > xHig || host_segLocColor[up].x < xLow ||
				host_segLocColor[up].z > zHig || host_segLocColor[up].z < zLow)) {
				errorLoc[errLoc].x = host_segLocColor[up].x;
				errorLoc[errLoc].y = host_segLocColor[up].y;
				errorLoc[errLoc].z = host_segLocColor[up].z;
				errorLoc[errLoc].w = up;
				errLoc++;
				TempColor = XMFLOAT4(0.9f, 0.0f, 0.3f, 1.0f);
			}*/

			unsigned char RGBA[4] = { 0, 0, 0, 0 };
			unsigned long int xx = (unsigned long int)host_segLocColor[up].w;
			packi32b(RGBA, xx);
			XMFLOAT4 TempColor = XMFLOAT4(RGBA[0] / 128.0f, RGBA[1] / 128.0f, RGBA[2] / 128.0f, 1.0f);
			//XMFLOAT4 TempColor = XMFLOAT4(1.0, 0, 0, 1.0f);

			riSeg->Instances[up].Color.x = TempColor.x;
			riSeg->Instances[up].Color.y = TempColor.y;
			riSeg->Instances[up].Color.z = TempColor.z;
		}

		sphereRenderCnt2 = 0;
		if (sphereRenderCnt2 != 0) {

			int upOld = up;
#pragma omp parallel for private (upOld)
			for (up = 0; up < (int)sphereRenderCnt2; ++up)
			{
				//Updating data that will be used to render
				riSeg2->Instances[up].World = XMFLOAT4X4(
					1.0f, 0.0f, 0.0f, 0.0f,
					0.0f, 1.0f, 0.0f, 0.0f,
					0.0f, 0.0f, 1.0f, 0.0f,
					host_segLocColor[up + upOld].x, host_segLocColor[up + upOld].y + 0.5f, host_segLocColor[up + upOld].z, 1.0f);

				unsigned char RGBA[4] = { 0, 0, 0, 0 };
				unsigned long int xx = (unsigned long int)host_segLocColor[up + upOld].w;
				packi32b(RGBA, xx);
				XMFLOAT4 TempColor = XMFLOAT4(RGBA[0] / 128.0f, RGBA[1] / 128.0f, RGBA[2] / 128.0f, 1.0f);
				//XMFLOAT4 TempColor = XMFLOAT4(0, 0, 1, 1.0f);

				riSeg2->Instances[up].Color.x = TempColor.x;
				riSeg2->Instances[up].Color.y = TempColor.y;
				riSeg2->Instances[up].Color.z = TempColor.z;
			}
		}

		///??????????????????????????????????????????????????????????????????????????????
		auto riMotor = mOpaqueRitems[getInstIdx("motor")];
		riMotor->InstanceCount = MOTOR_RenderCnt;
		// Data used: host_motorQuatLocXZ
#pragma omp parallel for private (up)
		for (up = 0; up < (int)MOTOR_RenderCnt; ++up)
		{
			XMFLOAT4X4 worldMat;
			XMMATRIX rotateMat = XMMATRIX(
				cosf(host_motorQuatLocXZ[up].q.w), 0.0f, sinf(host_motorQuatLocXZ[up].q.w), 0.0f,
				0.0f, 2.5f - (2.5f*sinf(host_motorQuatLocXZ[up].q.y)), 0.0f, 0.0f,
				-sinf(host_motorQuatLocXZ[up].q.w), 0.0f, cosf(host_motorQuatLocXZ[up].q.w), 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f);

			XMFLOAT4X4 toSegMat4x4 = quatToMatrix_r4(XMFLOAT4(host_motorQuatLocXZ[up].q.x, host_motorQuatLocXZ[up].q.y, host_motorQuatLocXZ[up].q.z, 0.0f));
			XMMATRIX toSegMatXM = XMLoadFloat4x4(&toSegMat4x4);
			XMMATRIX tanslate = XMMatrixMultiply(rotateMat, toSegMatXM);
			XMStoreFloat4x4(&worldMat, tanslate);

			float xLoc = host_motorQuatLocXZ[up].pos.x;
			float zLoc = host_motorQuatLocXZ[up].pos.y; //Actually Z
			worldMat._41 = xLoc;
			worldMat._42 = 0.0f;
			worldMat._43 = zLoc;
			worldMat._44 = 1.0f;
			riMotor->Instances[up].World = worldMat;

			float bright = 0.3f;
			if (host_motorQuatLocXZ[up].q.w > 0.0001f || host_motorQuatLocXZ[up].q.w < -0.0001f)
				bright = 3.0f;

			int zCube = 0, xCube = 0;
			xCube = (int)((xLoc + spaceDimXD2_sg) / 8);
			zCube = (int)((zLoc + spaceDimZD2_sg) / 8);
			if ((xCube % 2 == 0 && zCube % 2 == 0) || (xCube % 2 == 1 && zCube % 2 == 1)) {
				riMotor->Instances[up].Color = XMFLOAT3(0.08f * bright, 0.7f * bright, 0.15f * bright);
			}
			else {
				riMotor->Instances[up].Color = XMFLOAT3(0.5f * bright, 0.07f * bright, 0.5f * bright);
			}
		}
}

void RenderDX12::LoadNetworkDataAll() {
	int up;
		auto riSeg = mOpaqueRitems[getInstIdx("segments")];
		riSeg->InstanceCount = SEG_RenderCnt;
		// Data used: host_segLoc, host_segColor
#pragma omp parallel for private (up)
		for (up = 0; up < (int)SEG_RenderCnt; ++up)
		{
			riSeg->Instances[up].World = XMFLOAT4X4(
				1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				host_segLoc[up].x, host_segLoc[up].y + 0.5f, host_segLoc[up].z, 1.0f);

			unsigned char RGBA[4] = { 0, 0, 0, 0 };
			unsigned long int xx = (unsigned long int)host_segColor[up];
			packi32b(RGBA, xx);
			XMFLOAT4 TempColor = XMFLOAT4(RGBA[0] / 128.0f, RGBA[1] / 128.0f, RGBA[2] / 128.0f, 1.0f);

			riSeg->Instances[up].Color.x = TempColor.x;
			riSeg->Instances[up].Color.y = TempColor.y;
			riSeg->Instances[up].Color.z = TempColor.z;
		}
		auto riMotor = mOpaqueRitems[getInstIdx("motor")];
		riMotor->InstanceCount = MOTOR_RenderCnt;
		// Data used: host_motorQuat, host_motorLocations
#pragma omp parallel for private (up)
		for (up = 0; up < (int)MOTOR_RenderCnt; ++up)
		{
			XMFLOAT4X4 worldMat;
			XMMATRIX rotateMat = XMMATRIX(
				cosf(host_motorQuat[up].w), 0.0f, sinf(host_motorQuat[up].w), 0.0f,
				0.0f, 2.5f - (2.5f*sinf(host_motorQuat[up].y)), 0.0f, 0.0f,
				-sinf(host_motorQuat[up].w), 0.0f, cosf(host_motorQuat[up].w), 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f);
			//XMMATRIX toSegMat = quatToMatrix_r(XMFLOAT4(host_motorQuat[up].x, host_motorQuat[up].y, host_motorQuat[up].z, 0.0f));
			//XMMATRIX tanslate = XMMatrixMultiply(rotateMat, toSegMat);
			//XMStoreFloat4x4(&worldMat, tanslate);

			XMFLOAT4X4 toSegMat4x4 = quatToMatrix_r4(XMFLOAT4(host_motorQuat[up].x, host_motorQuat[up].y, host_motorQuat[up].z, 0.0f));
			XMMATRIX toSegMatXM = XMLoadFloat4x4(&toSegMat4x4);
			XMMATRIX tanslate = XMMatrixMultiply(rotateMat, toSegMatXM);
			XMStoreFloat4x4(&worldMat, tanslate);

			worldMat._41 = host_motorLocations[up].x;
			worldMat._42 = host_motorLocations[up].y;
			worldMat._43 = host_motorLocations[up].z;
			worldMat._44 = 1.0f;
			riMotor->Instances[up].World = worldMat;
			float bright = 0.3f;
			if (host_motorQuat[up].w > 0.0001f || host_motorQuat[up].w < -0.0001f)
				bright = 3.0f;
			riMotor->Instances[up].Color = XMFLOAT3(1.0f * bright, 0.0f * bright, 0.0f * bright);
		}
}

UINT RenderDX12::UpdateInstanceData(const GameTimer& gt)
{
	//100ms with Culling -> 8ms with no culling -> 3.5ms with 8 threads -> 3.6ms with 16 threads
	XMMATRIX view = mCamera.GetView();
	XMMATRIX invView = XMMatrixInverse(&XMMatrixDeterminant(view), view);

	float fub = gt.TotalTime();

	int renderCount = 0;

	StartCounter_ms(timer_a);

	if (1 || !PAUSED) { // TODO: Keep or not keep?

		for (size_t itm = 0; itm < mAllRitems.size(); ++itm)
		{
			auto& e = mAllRitems[itm];
			const auto& instanceData = e->Instances;
			auto currInstanceBuffer = mCurrFrameResource->InstanceBuffer[itm].get();


			int i; int loopSize = (int)e->InstanceCount;

			if (useCSInstanceCopy && 
				((strcmp(instancingCounts.name[itm], "segments") == 0) ||
				((strcmp(instancingCounts.name[itm], "motor") == 0) && useNetworkCompute))) {
				// Nothing needed
			}
			else {
#pragma omp parallel for private(i)
				for (i = 0; i < loopSize; i++)
				{
					XMMATRIX world = XMLoadFloat4x4(&instanceData[i].World);
					XMMATRIX texTransform = XMLoadFloat4x4(&instanceData[i].TexTransform);

					InstanceData data;
					//UINT32 ss = sizeof(data); //144
					XMStoreFloat4x4(&data.World, XMMatrixTranspose(world));
					XMStoreFloat4x4(&data.TexTransform, XMMatrixTranspose(texTransform));
					if (NPRESS) {
						XMFLOAT3 fcolor = { 1, 0, 0 };
						if (frameCount % 3 == 0)
							fcolor = XMFLOAT3(0, 1, 0);
						else if (frameCount % 2 == 0)
							fcolor = XMFLOAT3(0, 0, 1);
						data.Color = fcolor;
					}
					else {
						data.Color = instanceData[i].Color;
					}
					
					data.MaterialIndex = instanceData[i].MaterialIndex;

					currInstanceBuffer->CopyData(i, data);
					//totCopy += sizeof(data);

					//currInstanceBuffer->streamToBuffer(i, &data);
					//currInstanceBuffer->streamBuffer(i, &data);
					//memcpy(&mapPoint[i*instDataSize], &data, instDataSize);
				}
			}

			e->InstanceCount = loopSize;
			renderCount += loopSize;

		}

		//--------------------------------------------------------------------------------
		if (flexiGrid) {
			// Update the wave vertex buffer with the new solution.
			auto currDynamicVB = mCurrFrameResource->DynamicVB.get();

			// i start = offset; i max = offset + size
			auto geo = mGeometries["shapeGeoDy"].get();
			INT offset = geo->DrawArgs["gridDy"].BaseVertexLocation;
			INT vertexCount = geo->DrawArgs["gridDy"].VertexCount;
			int i;
			float time = gt.TotalTime();
			float separation = 20.0f;
			float multi = 200.0f;
			float inverse = 1.0f / multi;
			int mod = (int)(multi * separation);

#pragma omp parallel for private(i)
			for (i = 1; i < offset + vertexCount; ++i)
			{
				float f = ((2.0f*3.1415f) / separation);
				float a = ((mDynamicVBCPU[i].Pos.x + (size1D_fg*0.1f)) / (float)vertDivisor_fg) + (((int)(time*multi) % mod)*inverse);
				a = (a > separation) ? (a - separation) : a;
				float b = ((mDynamicVBCPU[i].Pos.z + (size1D_fg*0.1f)) / (float)vertDivisor_fg) + (((int)(time*multi) % mod)*inverse);
				b = (b > separation) ? (b - separation) : b;
				mDynamicVBCPU[i].Pos.y = 5.0f*sin(f * b) + 5.0f*sin(f * a);

				unsigned int red = (unsigned int)((mDynamicVBCPU[i].Pos.y + 10.0f) * 12.0f);
				unsigned int gre = 50;
				unsigned int blu = 150;
				unsigned int alf = 255;
				float rg = (float)((red << 8) | gre);
				float ba = (float)((blu << 8) | alf);
				mDynamicVBCPU[i].TexC = { rg, ba };
			}

			Vertex *offsetPoint = &mDynamicVBCPU[offset];
			int gridxz = (int)sqrt(vertexCount);
#pragma omp parallel for private(i)
			for (i = 1; i < gridxz - 1; ++i)
			{
				for (int j = 1; j < gridxz - 1; ++j)
				{
					float l = offsetPoint[i*gridxz + j - 1].Pos.y;
					float r = offsetPoint[i*gridxz + j + 1].Pos.y;
					float t = offsetPoint[(i - 1)*gridxz + j].Pos.y;
					float b = offsetPoint[(i + 1)*gridxz + j].Pos.y;
					offsetPoint[i*gridxz + j].Normal.x = -r + l;
					offsetPoint[i*gridxz + j].Normal.y = 2.0f;
					offsetPoint[i*gridxz + j].Normal.z = b - t;

					XMVECTOR n = XMVector3Normalize(XMLoadFloat3(&offsetPoint[i*gridxz + j].Normal));
					XMStoreFloat3(&offsetPoint[i*gridxz + j].Normal, n);

					//mTangentX[i*mNumCols + j] = XMFLOAT3(2.0f*mSpatialStep, r - l, 0.0f);
					//XMVECTOR T = XMVector3Normalize(XMLoadFloat3(&mTangentX[i*mNumCols + j]));
					//XMStoreFloat3(&mTangentX[i*mNumCols + j], T);
				}
			}
#pragma omp parallel for private(i)
			for (i = offset; i < offset + vertexCount; ++i)
			{
				currDynamicVB->CopyData(i, mDynamicVBCPU[i]);
			}
			// Set the dynamic VB of the wave renderitem to the current frame VB.
			mDynamicRitem->Geo->VertexBufferGPU = currDynamicVB->Resource();
		}
		//--------------------------------------------------------------------------------
	}

	timeCumulativeInst += GetCounter_ms(timer_a);
	if (frameCount % 400 == 0) {
		timeModInst = timeCumulativeInst / 400.f;
		timeCumulativeInst = 0;
	}

	return renderCount;
}

void RenderDX12::UpdateMotorColors() {
	//Doesnt do anything
	int up;
	auto riMotor = mOpaqueRitems[getInstIdx("motor")];
	riMotor->InstanceCount = MOTOR_RenderCnt;
#pragma omp parallel for private (up)
	for (up = 0; up < (int)MOTOR_RenderCnt; ++up)
	{
		float xLoc = host_motorQuatLocXZ[up].pos.x;
		float zLoc = host_motorQuatLocXZ[up].pos.y; //Actually Z
		int zCube = 0, xCube = 0;
		xCube = (int)((xLoc + spaceDimXD2_sg) / 40);
		zCube = (int)((zLoc + spaceDimZD2_sg) / 40);
		if ((xCube % 2 == 0 && zCube % 2 == 0) || (xCube % 2 == 1 && zCube % 2 == 1)) {
			riMotor->Instances[up].Color = XMFLOAT3(0.5f, 0.5f, 0.1f);
		}
		else {
			riMotor->Instances[up].Color = XMFLOAT3(0.08f, 0.3f, 0.6f);
		}
		/*if ((xCube % 2 == 0 && zCube % 2 == 0) || (xCube % 2 == 1 && zCube % 2 == 1)) {
		riMotor->Instances[up].Color = XMFLOAT3(0.5f * bright, 0.5f * bright, 0.1f * bright);
		}
		else {
		riMotor->Instances[up].Color = XMFLOAT3(0.08f * bright, 0.3f * bright, 0.6f * bright);
		}*/
	}

	
}

void RenderDX12::UpdateMaterialBuffer(const GameTimer& gt)
{
	auto currMaterialBuffer = mCurrFrameResource->MaterialBuffer.get();
	for(auto& e : mMaterials)
	{
		// Only update the cbuffer data if the constants have changed.  If the cbuffer
		// data changes, it needs to be updated for each FrameResource.
		Material* mat = e.second.get();
		if(mat->NumFramesDirty > 0)
		{
			XMMATRIX matTransform = XMLoadFloat4x4(&mat->MatTransform);

			MaterialData matData;
			matData.DiffuseAlbedo = mat->DiffuseAlbedo;
			matData.FresnelR0 = mat->FresnelR0;
			matData.Roughness = mat->Roughness;
			XMStoreFloat4x4(&matData.MatTransform, XMMatrixTranspose(matTransform));
			matData.DiffuseMapIndex = mat->DiffuseSrvHeapIndex;

			currMaterialBuffer->CopyData(mat->MatCBIndex, matData);

			// Next FrameResource need to be updated too.
			mat->NumFramesDirty--;
		}
	}
}

void RenderDX12::UpdateMainPassCB(const GameTimer& gt)
{
	if (m_pOpenVRSystem) { 
		//UpdateEye(0); 
		UpdateEyeTwo(0);
		UpdateController(gt);
		mCamera.UpdateViewMatrix();
		//////////////////////////////////////////LEAP MOTION//////////////////////////////////////////
		updateLeapMotionHandsVR();
		///////////////////////////////////////////////////////////////////////////////////////////////
	}
	else {
		//////////////////////////////////////////LEAP MOTION//////////////////////////////////////////
		updateLeapMotionHands();
		///////////////////////////////////////////////////////////////////////////////////////////////
	}

	XMMATRIX view = mCamera.GetView();
	XMMATRIX proj = mCamera.GetProj();

	XMMATRIX viewProj = XMMatrixMultiply(view, proj);
	XMMATRIX invView = XMMatrixInverse(&XMMatrixDeterminant(view), view);
	XMMATRIX invProj = XMMatrixInverse(&XMMatrixDeterminant(proj), proj);
	XMMATRIX invViewProj = XMMatrixInverse(&XMMatrixDeterminant(viewProj), viewProj);

	XMStoreFloat4x4(&mMainPassCBL.View, XMMatrixTranspose(view));
	XMStoreFloat4x4(&mMainPassCBL.InvView, XMMatrixTranspose(invView));
	XMStoreFloat4x4(&mMainPassCBL.Proj, XMMatrixTranspose(proj));
	XMStoreFloat4x4(&mMainPassCBL.InvProj, XMMatrixTranspose(invProj));
	XMStoreFloat4x4(&mMainPassCBL.ViewProj, XMMatrixTranspose(viewProj));
	XMStoreFloat4x4(&mMainPassCBL.InvViewProj, XMMatrixTranspose(invViewProj));
	mMainPassCBL.EyePosW = mCamera.GetPosition3f();
	mMainPassCBL.RenderTargetSize = XMFLOAT2((float)mClientWidth, (float)mClientHeight);
	mMainPassCBL.InvRenderTargetSize = XMFLOAT2(1.0f / mClientWidth, 1.0f / mClientHeight);
	mMainPassCBL.NearZ = 1.0f;
	mMainPassCBL.FarZ = CAMERA_ZFAR;
	mMainPassCBL.TotalTime = gt.TotalTime();
	mMainPassCBL.DeltaTime = gt.DeltaTime();
	mMainPassCBL.AmbientLight = { 0.25f, 0.25f, 0.35f, 1.0f };
	mMainPassCBL.Lights[0].Direction = { 0.57735f, -0.57735f, 0.57735f };
	mMainPassCBL.Lights[0].Strength = { 0.8f, 0.8f, 0.8f };
	mMainPassCBL.Lights[1].Direction = { -0.57735f, -0.57735f, 0.57735f };
	mMainPassCBL.Lights[1].Strength = { 0.7f, 0.7f, 0.7f };
	mMainPassCBL.Lights[2].Direction = { 0.0f, -0.707f, -0.707f };
	mMainPassCBL.Lights[2].Strength = { 0.6f, 0.6f, 0.6f };

	auto currPassCB = mCurrFrameResource->PassCBL.get();
	currPassCB->CopyData(0, mMainPassCBL);
}

void RenderDX12::UpdateMainPassCBStereo(const GameTimer& gt)
{
	if (m_pOpenVRSystem) {
		//UpdateEye(0); 
		UpdateEyeTwo(0);
		//UpdateController(gt);
		mCamera.UpdateViewMatrix();
		//////////////////////////////////////////LEAP MOTION//////////////////////////////////////////
		updateLeapMotionHandsVR();
		///////////////////////////////////////////////////////////////////////////////////////////////
	}
	else {
		//////////////////////////////////////////LEAP MOTION//////////////////////////////////////////
		updateLeapMotionHands();
		///////////////////////////////////////////////////////////////////////////////////////////////
	}

	Camera CameraL = mCamera;
	CameraL.Strafe(-VRoffsetStereo);
	CameraL.mViewDirty = true;
	CameraL.UpdateViewMatrix();

	XMMATRIX view = CameraL.GetView();
	XMMATRIX proj = (m_pOpenVRSystem) ? mProjectionLeft : CameraL.GetProj();

	XMMATRIX viewProj = XMMatrixMultiply(view, proj);
	XMMATRIX invView = XMMatrixInverse(&XMMatrixDeterminant(view), view);
	XMMATRIX invProj = XMMatrixInverse(&XMMatrixDeterminant(proj), proj);
	XMMATRIX invViewProj = XMMatrixInverse(&XMMatrixDeterminant(viewProj), viewProj);

	XMStoreFloat4x4(&mMainPassCBL.View, XMMatrixTranspose(view));
	XMStoreFloat4x4(&mMainPassCBL.InvView, XMMatrixTranspose(invView));
	XMStoreFloat4x4(&mMainPassCBL.Proj, XMMatrixTranspose(proj));
	XMStoreFloat4x4(&mMainPassCBL.InvProj, XMMatrixTranspose(invProj));
	XMStoreFloat4x4(&mMainPassCBL.ViewProj, XMMatrixTranspose(viewProj));
	XMStoreFloat4x4(&mMainPassCBL.InvViewProj, XMMatrixTranspose(invViewProj));
	mMainPassCBL.EyePosW = CameraL.GetPosition3f();
	mMainPassCBL.RenderTargetSize = XMFLOAT2((float)mClientWidth, (float)mClientHeight);
	mMainPassCBL.InvRenderTargetSize = XMFLOAT2(1.0f / mClientWidth, 1.0f / mClientHeight);
	mMainPassCBL.NearZ = 1.0f;
	mMainPassCBL.FarZ = CAMERA_ZFAR;
	mMainPassCBL.TotalTime = gt.TotalTime();
	mMainPassCBL.DeltaTime = gt.DeltaTime();
	mMainPassCBL.AmbientLight = { 0.25f, 0.25f, 0.35f, 1.0f };
	mMainPassCBL.Lights[0].Direction = { 0.57735f, -0.57735f, 0.57735f };
	mMainPassCBL.Lights[0].Strength = { 0.8f, 0.8f, 0.8f };
	mMainPassCBL.Lights[1].Direction = { -0.57735f, -0.57735f, 0.57735f };
	mMainPassCBL.Lights[1].Strength = { 0.7f, 0.7f, 0.7f };
	mMainPassCBL.Lights[2].Direction = { 0.0f, -0.707f, -0.707f };
	mMainPassCBL.Lights[2].Strength = { 0.6f, 0.6f, 0.6f };

	auto currPassCBL = mCurrFrameResource->PassCBL.get();
	currPassCBL->CopyData(0, mMainPassCBL);

	///=======================================================================

	Camera CameraR = mCamera;
	CameraR.Strafe(VRoffsetStereo);
	CameraR.mViewDirty = true;
	CameraR.UpdateViewMatrix();

	XMMATRIX viewR = CameraR.GetView();
	XMMATRIX projR = (m_pOpenVRSystem) ? mProjectionRight : CameraR.GetProj();

	XMMATRIX viewProjR = XMMatrixMultiply(viewR, projR);
	XMMATRIX invViewR = XMMatrixInverse(&XMMatrixDeterminant(viewR), viewR);
	XMMATRIX invProjR = XMMatrixInverse(&XMMatrixDeterminant(projR), projR);
	XMMATRIX invViewProjR = XMMatrixInverse(&XMMatrixDeterminant(viewProjR), viewProjR);

	XMStoreFloat4x4(&mMainPassCBR.View, XMMatrixTranspose(viewR));
	XMStoreFloat4x4(&mMainPassCBR.InvView, XMMatrixTranspose(invViewR));
	XMStoreFloat4x4(&mMainPassCBR.Proj, XMMatrixTranspose(projR));
	XMStoreFloat4x4(&mMainPassCBR.InvProj, XMMatrixTranspose(invProjR));
	XMStoreFloat4x4(&mMainPassCBR.ViewProj, XMMatrixTranspose(viewProjR));
	XMStoreFloat4x4(&mMainPassCBR.InvViewProj, XMMatrixTranspose(invViewProjR));
	mMainPassCBR.EyePosW = CameraR.GetPosition3f();
	mMainPassCBR.RenderTargetSize = XMFLOAT2((float)mClientWidth, (float)mClientHeight);
	mMainPassCBR.InvRenderTargetSize = XMFLOAT2(1.0f / mClientWidth, 1.0f / mClientHeight);
	mMainPassCBR.NearZ = 1.0f;
	mMainPassCBR.FarZ = CAMERA_ZFAR;
	mMainPassCBR.TotalTime = gt.TotalTime();
	mMainPassCBR.DeltaTime = gt.DeltaTime();
	mMainPassCBR.AmbientLight = { 0.25f, 0.25f, 0.35f, 1.0f };
	mMainPassCBR.Lights[0].Direction = { 0.57735f, -0.57735f, 0.57735f };
	mMainPassCBR.Lights[0].Strength = { 0.8f, 0.8f, 0.8f };
	mMainPassCBR.Lights[1].Direction = { -0.57735f, -0.57735f, 0.57735f };
	mMainPassCBR.Lights[1].Strength = { 0.7f, 0.7f, 0.7f };
	mMainPassCBR.Lights[2].Direction = { 0.0f, -0.707f, -0.707f };
	mMainPassCBR.Lights[2].Strength = { 0.6f, 0.6f, 0.6f };

	auto currPassCBR = mCurrFrameResource->PassCBR.get();
	currPassCBR->CopyData(0, mMainPassCBR);
}

void RenderDX12::DrawRenderItems(ID3D12GraphicsCommandList* cmdList, const std::vector<RenderItem*>& ritems)
{
    // For each render item...
    for(size_t i = 0; i < ritems.size(); ++i)
    {
		if ((strcmp(instancingCounts.name[i], "boxMenue") == 0) || transparentPS) {
			mCommandList->SetPipelineState(mPSOs["transparent"].Get());
		}
		else if(strcmp(instancingCounts.name[i], "line") == 0){
			mCommandList->SetPipelineState(mPSOs["line"].Get()); // Line can actually use any PS for some reason
			//mCommandList->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
		}

		auto ri = ritems[i];

        cmdList->IASetVertexBuffers(0, 1, &ri->Geo->VertexBufferView());
        cmdList->IASetIndexBuffer(&ri->Geo->IndexBufferView());
        cmdList->IASetPrimitiveTopology(ri->PrimitiveType);

		// Set the instance buffer to use for this render-item.  For structured buffers, we can bypass the heap and set as a root descriptor.
		/*if (useCSInstanceCopy && (i == sphereRenderNumber || (i == sphereRenderNumber + 1 && useNetworkCompute) )) {
			mCommandList->SetGraphicsRootShaderResourceView(0, OR_InstancedData[i - sphereRenderNumber][currentInstanceBuffer]->GetGPUVirtualAddress()); // TODO find cleaner way?
		}*/
		if (useCSInstanceCopy && ((strcmp(instancingCounts.name[i], "segments") == 0) || ((strcmp(instancingCounts.name[i], "motor") == 0) && useNetworkCompute))) {
			if (strcmp(instancingCounts.name[i], "segments") == 0)
				mCommandList->SetGraphicsRootShaderResourceView(0, OR_InstancedData[0][currentInstanceBuffer]->GetGPUVirtualAddress()); // TODO find cleaner way?
			else
				mCommandList->SetGraphicsRootShaderResourceView(0, OR_InstancedData[1][currentInstanceBuffer]->GetGPUVirtualAddress());
		}
		else {
			auto instanceBuffer = mCurrFrameResource->InstanceBuffer[i]->Resource();
			mCommandList->SetGraphicsRootShaderResourceView(0, instanceBuffer->GetGPUVirtualAddress());
		}

		//auto zx = mCurrFrameResource->InstanceBuffer.get();
		//InstanceData *data = (InstanceData *)zx->mMappedData[ri->ObjCBIndex*instDataSize];
		//XMFLOAT3 dd = ri->Instances[0].Color;

		//if ((strcmp(instancingCounts.name[i], "line") != 0) && (strcmp(instancingCounts.name[i], "grid") != 0)) {
			cmdList->DrawIndexedInstanced(ri->IndexCount, ri->InstanceCount, ri->StartIndexLocation, ri->BaseVertexLocation, 0);
		//}

		//TODO: Edit draw order to not need the reset below
		if ((strcmp(instancingCounts.name[i], "forceParticle") == 0)) {
			mCommandList->SetPipelineState(mPSOs["texColor"].Get());
		}
    }
}

std::array<const CD3DX12_STATIC_SAMPLER_DESC, 6> RenderDX12::GetStaticSamplers()
{
	// Applications usually only need a handful of samplers.  So just define them all up front
	// and keep them available as part of the root signature.  

	const CD3DX12_STATIC_SAMPLER_DESC pointWrap(
		0, // shaderRegister
		D3D12_FILTER_MIN_MAG_MIP_POINT, // filter
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressU
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressV
		D3D12_TEXTURE_ADDRESS_MODE_WRAP); // addressW

	const CD3DX12_STATIC_SAMPLER_DESC pointClamp(
		1, // shaderRegister
		D3D12_FILTER_MIN_MAG_MIP_POINT, // filter
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressU
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressV
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP); // addressW

	const CD3DX12_STATIC_SAMPLER_DESC linearWrap(
		2, // shaderRegister
		D3D12_FILTER_MIN_MAG_MIP_LINEAR, // filter
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressU
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressV
		D3D12_TEXTURE_ADDRESS_MODE_WRAP); // addressW

	const CD3DX12_STATIC_SAMPLER_DESC linearClamp(
		3, // shaderRegister
		D3D12_FILTER_MIN_MAG_MIP_LINEAR, // filter
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressU
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressV
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP); // addressW

	const CD3DX12_STATIC_SAMPLER_DESC anisotropicWrap(
		4, // shaderRegister
		D3D12_FILTER_ANISOTROPIC, // filter
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressU
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressV
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressW
		0.0f,                             // mipLODBias
		8);                               // maxAnisotropy

	const CD3DX12_STATIC_SAMPLER_DESC anisotropicClamp(
		5, // shaderRegister
		D3D12_FILTER_ANISOTROPIC, // filter
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressU
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressV
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressW
		0.0f,                              // mipLODBias
		8);                                // maxAnisotropy

	return { 
		pointWrap, pointClamp,
		linearWrap, linearClamp, 
		anisotropicWrap, anisotropicClamp };
}

void RenderDX12::PrintScreenImage(int offset) {
	if (saveImage) {
		PrintScreen(mSwapChain.Get(), mCommandQueue.Get(), CurrentBackBuffer(), time_simulatedMinutes, (int)time_simulatedSeconds + offset);
	}
}

void RenderDX12::RenderUI()
{
	D2D1_RECT_F textRect = D2D1::RectF(0, 0, mMainPassCBL.RenderTargetSize.x, mMainPassCBL.RenderTargetSize.y);
	static const WCHAR text[] = L"こんばんは\nThis is line two\nライン三番";

	buildString = "";

#if defined(DEBUG) | defined(_DEBUG)
	if (m_pOpenVRSystem) {
		auto ri = mOpaqueRitems[getInstIdx("controller")];
		std::ostringstream ss;
		buildString.append("Head Position\n");
		for (int l = 0; l < 3; l++) {
			ss << m_poseOpenVR[0].mDeviceToAbsoluteTracking.m[l][3];
			buildString.append(ss.str()+ "\n");
			ss.str(std::string());
		}
		for (int cntr = 0; cntr < 2; cntr++) {
			buildString.append((cntr == 0) ? "\nController 1 Vive Matrix\n" : "\nController 2 Vive Matrix\n");
			for (int x = 0; x < 3; x++) {
				for (int l = 0; l < 4; l++) {
					ss << m_poseOpenVR[cntr + 1].mDeviceToAbsoluteTracking.m[x][l];
					buildString.append(ss.str() + "\t");
					ss.str(std::string());
				}
				buildString.append(ss.str() + "\n");
			}
			buildString.append((cntr == 0) ? "\nController 1 Instances Matrix\n" : "\nController 2 Instances Matrix\n");
			for (int x = 0; x < 4; x++) {
				for (int l = 0; l < 4; l++) {
					ss << ri->Instances[cntr].World.m[x][l];
					buildString.append(ss.str() + "\t");
					ss.str(std::string());
				}
				buildString.append(ss.str() + "\n");
			}
			//if (cntr == 0) { fprintf(clientPrint, "%8.5f\t%8.5f\t%8.5f\t", ri->Instances[cntr].World._41, ri->Instances[cntr].World._42, ri->Instances[cntr].World._43); }

			buildString.append((cntr == 0) ? "\nController 1 Delta\nx: " : "\nController 2 Delta\nx: ");
			{
				ss << vc.controllerDeltaLoc[cntr].x;
				buildString.append(ss.str() + "\ny: ");
				ss.str(std::string());
				ss << vc.controllerDeltaLoc[cntr].y;
				buildString.append(ss.str() + "\nz: ");
				ss.str(std::string());
				ss << vc.controllerDeltaLoc[cntr].z;
				buildString.append(ss.str() + "\n");
				ss.str(std::string());
				//if (cntr == 0) { fprintf(clientPrint, "%8.5f\t%8.5f\t%8.5f\n", controllerDeltaLoc[cntr].x, controllerDeltaLoc[cntr].y, controllerDeltaLoc[cntr].z); }
			}
		}
		buildString.append("\nHolding\n0: ");
		{
			ss << holdingIdx[0];
			buildString.append(ss.str() + "\n1: ");
			ss.str(std::string());
			ss << holdingIdx[1];
			buildString.append(ss.str() + "\n");
			ss.str(std::string());
		}
	}
	else {
		std::ostringstream ss;
		ss.precision(3);
		buildString.append("Average Segments Found:        ");

		ss << std::fixed << averageSegmentsFoundLJP;
		buildString.append(ss.str() + "\n");
		ss.str(std::string());

		buildString.append("Average Segments Computed:  ");
		ss << std::fixed << averageSegmentsComputedLJP;
		buildString.append(ss.str() + "\n");
		ss.str(std::string());

		buildString.append("\nMouse Loc\nx: ");
		{
			ss << lastMousePos.x;
			buildString.append(ss.str() + "\ny: ");
			ss.str(std::string());
			ss << lastMousePos.y;
			buildString.append(ss.str() + "\n: ");
			ss.str(std::string());
		}
		buildString.append("Mouse Delta\nx: ");
		{
			ss << mouseDeltaLoc.x;
			buildString.append(ss.str() + "\ny: ");
			ss.str(std::string());
			ss << mouseDeltaLoc.y;
			buildString.append(ss.str() + "\nz: ");
			ss.str(std::string());
			ss << mouseDeltaLoc.z;
			buildString.append(ss.str() + "\n");
			ss.str(std::string());
		}
		buildString.append("\nHolding: ");
		{
			ss << holdingIdx[0];
			buildString.append(ss.str() + "\n");
			ss.str(std::string());
		}
		buildString.append("MousePressed: ");
		{
			ss << MouseRightPressed;
			buildString.append(ss.str() + "\n");
			ss.str(std::string());
		}
		////////////////////////////////////////////////
		buildString.append("Sort:\t ");
		{
			ss << timeModSort;
			buildString.append(ss.str() + "\n");
			ss.str(std::string());
		}
		buildString.append("LJP:\t ");
		{
			ss << timeModLJP;
			buildString.append(ss.str() + "\n");
			ss.str(std::string());
		}
		buildString.append("Move:\t ");
		{
			ss << timeModMove;
			buildString.append(ss.str() + "\n");
			ss.str(std::string());
		}
		buildString.append("SimInputLag: ( ");
		{
			ss << simInputLag.x;
			buildString.append(ss.str() + ", ");
			ss.str(std::string());
			ss << simInputLag.y;
			buildString.append(ss.str() + " )\t");
			ss.str(std::string());
			ss << timeModIpLag;
			buildString.append(ss.str() + "\n");
			ss.str(std::string());
		}
		buildString.append("totalEnergy:\t ");
		{
			ss << totalEnergy;
			buildString.append(ss.str() + "\n");
			ss.str(std::string());
		}
	}
#else

	std::ostringstream ss;
	ss.precision(3);
	buildString.append("Average Segments Found:        ");

	ss << std::fixed << averageSegmentsFoundLJP;
	buildString.append(ss.str() + "\n");
	ss.str(std::string());

	buildString.append("Average Segments Computed:  ");
	ss << std::fixed << averageSegmentsComputedLJP;
	buildString.append(ss.str() + "\n");
	ss.str(std::string());

	buildString.append("Head Position\n");
	for (int l = 0; l < 3; l++) {
		ss << m_poseOpenVR[0].mDeviceToAbsoluteTracking.m[l][3];
		buildString.append(ss.str() + "\n");
		ss.str(std::string());
	}
	buildString.append("Neg F M:\t ");
	{
		ss << param.NEG_F_MULTIPLY;
		buildString.append(ss.str() + "\n");
		ss.str(std::string());
	}
	buildString.append("totalEnergy:\t ");
	{
		ss << totalEnergy;
		buildString.append(ss.str() + "\n");
		ss.str(std::string());
	}
	buildString.append("Simulation time:\t ");
	{
		ss << time_simulatedMinutes;
		buildString.append(ss.str() + " min : ");
		ss.str(std::string());
		ss << time_simulatedSeconds;
		buildString.append(ss.str() + " sec\n");
		ss.str(std::string());
	}
	ss.precision(5);
	if (useNetworkCompute) {
		buildString.append("Average Speed: ");
		{
			ss << dataMontitor[0];
			buildString.append(ss.str() + "\n");
			ss.str(std::string());
		}
		buildString.append("Average Direction D: ");
		{
			ss << dataMontitor[1];
			buildString.append(ss.str() + "\n");
			ss.str(std::string());
		}
		buildString.append("SimInputLag: ( ");
		{
			ss << simInputLag.x;
			buildString.append(ss.str() + ", ");
			ss.str(std::string());
			ss << simInputLag.y;
			buildString.append(ss.str() + " )\t");
			ss.str(std::string());
			ss << timeModIpLag;
			buildString.append(ss.str() + "\n");
			ss.str(std::string());
		}
	}
	buildString.append("Sort:\t\t\t");
	{
		ss << timeModSort;
		buildString.append(ss.str() + "\n");
		ss.str(std::string());
	}
	buildString.append("LJP:\t\t\t");
	{
		ss << timeModLJP;
		buildString.append(ss.str() + "\n");
		ss.str(std::string());
	}
	buildString.append("Move:\t\t\t");
	{
		ss << timeModMove;
		buildString.append(ss.str() + "\n");
		ss.str(std::string());
	}
	buildString.append("CS Sim:\t\t");
	{
		ss << timerDXCSsim;
		buildString.append(ss.str() + "\n");
		ss.str(std::string());
	}
	buildString.append("CS Cpy:\t\t");
	{
		ss << timerDXCScpy;
		buildString.append(ss.str() + "\n");
		ss.str(std::string());
	}
	buildString.append("UpdateInstanceData:\t");
	{
		ss << timeModInst;
		buildString.append(ss.str() + "\n");
		ss.str(std::string());
	}
	buildString.append("AnimateMaterials:\t");
	{
		ss << timeModAnima;
		buildString.append(ss.str() + "\n");
		ss.str(std::string());
	}
	buildString.append("Render Text:\t\t");
	{
		ss << timeModText;
		buildString.append(ss.str() + "\n");
		ss.str(std::string());
	}
	buildString.append("Render DX Timer:\t");
	{
		ss << timerDXRender;
		buildString.append(ss.str() + "\n");
		ss.str(std::string());
	}
	buildString.append("Build Draw List:\t");
	{
		ss << timeModDraw;
		buildString.append(ss.str() + "\n");
		ss.str(std::string());
	}
	buildString.append("VR Sync:\t\t");
	{
		ss << timeModVRSync;
		buildString.append(ss.str() + "\n");
		ss.str(std::string());
	}
	//Add///////////////////////////////////////////////////////////////////
	buildString.append("\nThis is a test field:\n");
	{
		ss << testString;
		buildString.append(ss.str() + "\n");
		ss.str(std::string());
	}
	////////////////////////////////////////////////////////////////////////
#endif

	const size_t newsizew = strlen(buildString.c_str()) + 1;
	size_t convertedChars = 0;
	wchar_t *wcstring = new wchar_t[newsizew];
	mbstowcs_s(&convertedChars, wcstring, newsizew, buildString.c_str(), _TRUNCATE);

	// Acquire our wrapped render target resource for the current back buffer.
	m_d3d11On12Device->AcquireWrappedResources(m_wrappedBackBuffers[mCurrBackBuffer].GetAddressOf(), 1);

	// Render text directly to the back buffer.
	m_d2dDeviceContext->SetTarget(m_d2dRenderTargets[mCurrBackBuffer].Get());
	m_d2dDeviceContext->BeginDraw();
	m_d2dDeviceContext->SetTransform(D2D1::Matrix3x2F::Identity());
	m_d2dDeviceContext->DrawTextW(
		wcstring,
		(UINT32)newsizew, //_countof(text) - 1,
		m_textFormat.Get(),
		&textRect,
		m_textBrush.Get()
	);
	ThrowIfFailed(m_d2dDeviceContext->EndDraw());

	free(wcstring);

	// Release our wrapped render target resource. Releasing 
	// transitions the back buffer resource to the state specified
	// as the OutState when the wrapped resource was created.
	m_d3d11On12Device->ReleaseWrappedResources(m_wrappedBackBuffers[mCurrBackBuffer].GetAddressOf(), 1);

	// Flush to submit the 11 command list to the shared command queue.
	m_d3d11DeviceContext->Flush();
}

UINT RenderDX12::getInstIdx(const char * name)
{
	//Using this will decrease performance a tiny bit but less dangerous code
	for (UINT i = 0; i < instancingCounts.instanceGroupCount; i++) {
		if ((strcmp(instancingCounts.name[i], name) == 0)) {
			return i;
		}
	}
	return 30; // Cause an Error
}
void RenderDX12::packi32b(unsigned char *buf, unsigned long int i)
{
	buf[0] = (unsigned char)(i >> 24);
	buf[1] = (unsigned char)(i >> 16);
	buf[2] = (unsigned char)(i >> 8);
	buf[3] = (unsigned char)(i);
}
XMMATRIX RenderDX12::quatToMatrix_r(XMFLOAT4 q) {
	float sqw = q.w*q.w;
	float sqx = q.x*q.x;
	float sqy = q.y*q.y;
	float sqz = q.z*q.z;

	// invs (inverse square length) is only required if quaternion is not already normalised
	float invs = 1.0f / (sqx + sqy + sqz + sqw);
	XMMATRIX m = XMMATRIX(
		0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f);
	m.r[1].m128_f32[1] = (sqx - sqy - sqz + sqw)*invs; // since sqw + sqx + sqy + sqz =1/invs*invs
	m.r[2].m128_f32[2] = (-sqx + sqy - sqz + sqw)*invs;
	m.r[3].m128_f32[3] = (-sqx - sqy + sqz + sqw)*invs;
	float tmp1 = q.x*q.y;
	float tmp2 = q.z*q.w;
	m.r[1].m128_f32[2] = 2.0f * (tmp1 + tmp2)*invs;
	m.r[2].m128_f32[1] = 2.0f * (tmp1 - tmp2)*invs;

	tmp1 = q.x*q.z;
	tmp2 = q.y*q.w;
	m.r[1].m128_f32[3] = 2.0f * (tmp1 - tmp2)*invs;
	m.r[3].m128_f32[1] = 2.0f * (tmp1 + tmp2)*invs;
	tmp1 = q.y*q.z;
	tmp2 = q.x*q.w;
	m.r[2].m128_f32[3] = 2.0f * (tmp1 + tmp2)*invs;
	m.r[3].m128_f32[2] = 2.0f * (tmp1 - tmp2)*invs;
	return m;
}
XMFLOAT4X4 RenderDX12::quatToMatrix_r4(XMFLOAT4 q) {
	float sqw = q.w*q.w;
	float sqx = q.x*q.x;
	float sqy = q.y*q.y;
	float sqz = q.z*q.z;

	// invs (inverse square length) is only required if quaternion is not already normalised
	float invs = 1.0f / (sqx + sqy + sqz + sqw);
	XMFLOAT4X4 m = XMFLOAT4X4(
		0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f);
	m._11 = (sqx - sqy - sqz + sqw)*invs; // since sqw + sqx + sqy + sqz =1/invs*invs
	m._22 = (-sqx + sqy - sqz + sqw)*invs;
	m._33 = (-sqx - sqy + sqz + sqw)*invs;
	float tmp1 = q.x*q.y;
	float tmp2 = q.z*q.w;
	m._21 = 2.0f * (tmp1 + tmp2)*invs;
	m._12 = 2.0f * (tmp1 - tmp2)*invs;

	tmp1 = q.x*q.z;
	tmp2 = q.y*q.w;
	m._31 = 2.0f * (tmp1 - tmp2)*invs;
	m._13 = 2.0f * (tmp1 + tmp2)*invs;
	tmp1 = q.y*q.z;
	tmp2 = q.x*q.w;
	m._32 = 2.0f * (tmp1 + tmp2)*invs;
	m._23 = 2.0f * (tmp1 - tmp2)*invs;
	return m;
}
void RenderDX12::make16mat(float * out, XMMATRIX in) {
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			out[c * 4 + r] = in.r[c].m128_f32[r];
		}
	}
}
void RenderDX12::make16mat(float4x4 *out, XMMATRIX in) {
	out->m00 = in.r[0].m128_f32[0];
	out->m01 = in.r[0].m128_f32[1];
	out->m02 = in.r[0].m128_f32[2];
	out->m03 = in.r[0].m128_f32[3];

	out->m10 = in.r[1].m128_f32[0];
	out->m11 = in.r[1].m128_f32[1];
	out->m12 = in.r[1].m128_f32[2];
	out->m13 = in.r[1].m128_f32[3];

	out->m20 = in.r[2].m128_f32[0];
	out->m21 = in.r[2].m128_f32[1];
	out->m22 = in.r[2].m128_f32[2];
	out->m23 = in.r[2].m128_f32[3];

	out->m30 = in.r[3].m128_f32[0];
	out->m31 = in.r[3].m128_f32[1];
	out->m32 = in.r[3].m128_f32[2];
	out->m33 = in.r[3].m128_f32[3];
}
void RenderDX12::copy16mat(float4x4 *out, float4x4 in) {
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

//-----------------------COMPUTE SHADERS------------------------

void RenderDX12::InitGPUMemOut(ComPtr<ID3D12Resource> * memory, UINT64 byteSize, std::wstring name) {

	std::wstring name0 = name + L"[DEFAULT]";
	ThrowIfFailed(md3dDevice->CreateCommittedResource(
		&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
		D3D12_HEAP_FLAG_NONE,
		&CD3DX12_RESOURCE_DESC::Buffer(byteSize, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS),
		D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
		nullptr,
		IID_PPV_ARGS(&memory[0])));
	memory[0]->SetName(const_cast<LPWSTR>(name0.c_str()));

	// memory[1] is not used

	std::wstring name2 = name + L"[READBACK]";
	ThrowIfFailed(md3dDevice->CreateCommittedResource(
		&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_READBACK),
		D3D12_HEAP_FLAG_NONE,
		&CD3DX12_RESOURCE_DESC::Buffer(byteSize),
		D3D12_RESOURCE_STATE_COPY_DEST,
		nullptr,
		IID_PPV_ARGS(&memory[2])));
	memory[2]->SetName(const_cast<LPWSTR>(name2.c_str()));

}
void RenderDX12::InitGPUMemIn(ComPtr<ID3D12Resource> * memory, UINT64 byteSize, std::wstring name) {

	std::wstring name0 = name + L"[DEFAULT]";
	ThrowIfFailed(md3dDevice->CreateCommittedResource(
		&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
		D3D12_HEAP_FLAG_NONE,
		&CD3DX12_RESOURCE_DESC::Buffer(byteSize, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS),
		D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
		nullptr,
		IID_PPV_ARGS(&memory[0])));
	memory[0]->SetName(const_cast<LPWSTR>(name0.c_str()));

	std::wstring name1 = name + L"[UPLOAD]";
	ThrowIfFailed(md3dDevice->CreateCommittedResource(
		&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
		D3D12_HEAP_FLAG_NONE,
		&CD3DX12_RESOURCE_DESC::Buffer(byteSize),
		D3D12_RESOURCE_STATE_GENERIC_READ,
		nullptr,
		IID_PPV_ARGS(&memory[1])));
	memory[1]->SetName(const_cast<LPWSTR>(name1.c_str()));

	// memory[2] is not used
}
void RenderDX12::InitGPUMemInOut(ComPtr<ID3D12Resource> * memory, UINT64 byteSize, std::wstring name) {

	std::wstring name0 = name + L"[DEFAULT]";
	ThrowIfFailed(md3dDevice->CreateCommittedResource(
		&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
		D3D12_HEAP_FLAG_NONE,
		&CD3DX12_RESOURCE_DESC::Buffer(byteSize, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS),
		D3D12_RESOURCE_STATE_UNORDERED_ACCESS,  //D3D12_RESOURCE_STATE_COMMON
		nullptr,
		IID_PPV_ARGS(&memory[0])));
	memory[0]->SetName(const_cast<LPWSTR>(name0.c_str()));

	std::wstring name1 = name + L"[UPLOAD]";
	ThrowIfFailed(md3dDevice->CreateCommittedResource(
		&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
		D3D12_HEAP_FLAG_NONE,
		&CD3DX12_RESOURCE_DESC::Buffer(byteSize),
		D3D12_RESOURCE_STATE_GENERIC_READ, //D3D12_RESOURCE_STATE_COPY_SOURCE
		nullptr,
		IID_PPV_ARGS(&memory[1])));
	memory[1]->SetName(const_cast<LPWSTR>(name1.c_str()));

	std::wstring name2 = name + L"[READBACK]";
	ThrowIfFailed(md3dDevice->CreateCommittedResource(
		&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_READBACK),
		D3D12_HEAP_FLAG_NONE,
		&CD3DX12_RESOURCE_DESC::Buffer(byteSize),
		D3D12_RESOURCE_STATE_COPY_DEST,
		nullptr,
		IID_PPV_ARGS(&memory[2])));
	memory[2]->SetName(const_cast<LPWSTR>(name2.c_str()));
}
void RenderDX12::InitGPUMemBuffered(ComPtr<ID3D12Resource> * memory, UINT64 byteSize, std::wstring name) {

	std::wstring name0 = name + L"[DEFAULT0]";
	ThrowIfFailed(md3dDevice->CreateCommittedResource(
		&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
		D3D12_HEAP_FLAG_NONE,
		&CD3DX12_RESOURCE_DESC::Buffer(byteSize, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS),
		D3D12_RESOURCE_STATE_UNORDERED_ACCESS,  //D3D12_RESOURCE_STATE_COMMON
		nullptr,
		IID_PPV_ARGS(&memory[0])));
	memory[0]->SetName(const_cast<LPWSTR>(name0.c_str()));

	std::wstring name1 = name + L"[DEFAULT1]";
	ThrowIfFailed(md3dDevice->CreateCommittedResource(
		&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
		D3D12_HEAP_FLAG_NONE,
		&CD3DX12_RESOURCE_DESC::Buffer(byteSize, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS),
		D3D12_RESOURCE_STATE_UNORDERED_ACCESS,  //D3D12_RESOURCE_STATE_COMMON
		nullptr,
		IID_PPV_ARGS(&memory[1])));
	memory[1]->SetName(const_cast<LPWSTR>(name1.c_str()));

	std::wstring name2 = name + L"[DEFAULT2]";
	ThrowIfFailed(md3dDevice->CreateCommittedResource(
		&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
		D3D12_HEAP_FLAG_NONE,
		&CD3DX12_RESOURCE_DESC::Buffer(byteSize, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS),
		D3D12_RESOURCE_STATE_UNORDERED_ACCESS,  //D3D12_RESOURCE_STATE_COMMON
		nullptr,
		IID_PPV_ARGS(&memory[2])));
	memory[2]->SetName(const_cast<LPWSTR>(name2.c_str()));
}

void RenderDX12::BarriorCopyFromGPU(ComPtr<ID3D12GraphicsCommandList> &commandList, ComPtr<ID3D12Resource> * memory) {
	// Schedule to copy the data to the default buffer to the readback buffer.
	commandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(memory[0].Get(),
		D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COPY_SOURCE));

	commandList->CopyResource(memory[2].Get(), memory[0].Get());

	commandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(memory[0].Get(),
		D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS));

}
void RenderDX12::BarriorCopyToGPU(ComPtr<ID3D12GraphicsCommandList> &commandList, ComPtr<ID3D12Resource> * memory) {
	// Schedule to copy the data to the default buffer to the readback buffer.
	commandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(memory[0].Get(),
		D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COPY_DEST));

	commandList->CopyResource(memory[0].Get(), memory[1].Get());

	commandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(memory[0].Get(),
		D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_UNORDERED_ACCESS));
}

void RenderDX12::BuildRootSignatureCS()
{
	//-----------LJP CS----------------------------------------------------------------------
	{
		// Root parameter can be a table, root descriptor or root constants.
		const int rootCScount = 14;		//Addition
		CD3DX12_ROOT_PARAMETER slotRootParameterCS[rootCScount];		//Addition (Change)

		// Perfomance TIP: Order from most frequent to least frequent.
		slotRootParameterCS[0].InitAsConstants(16, 0);		//Addition (Change)
		slotRootParameterCS[1].InitAsUnorderedAccessView(0);
		slotRootParameterCS[2].InitAsUnorderedAccessView(1);
		slotRootParameterCS[3].InitAsUnorderedAccessView(2);
		slotRootParameterCS[4].InitAsUnorderedAccessView(3);
		slotRootParameterCS[5].InitAsUnorderedAccessView(4);
		slotRootParameterCS[6].InitAsUnorderedAccessView(5);
		slotRootParameterCS[7].InitAsUnorderedAccessView(6);
		slotRootParameterCS[8].InitAsUnorderedAccessView(7);
		slotRootParameterCS[9].InitAsUnorderedAccessView(8);
		slotRootParameterCS[10].InitAsUnorderedAccessView(9);
		slotRootParameterCS[11].InitAsUnorderedAccessView(10);
		slotRootParameterCS[12].InitAsUnorderedAccessView(11);
		slotRootParameterCS[13].InitAsUnorderedAccessView(12);		//Addition

		// A root signature is an array of root parameters.
		CD3DX12_ROOT_SIGNATURE_DESC rootSigDescCS(rootCScount, slotRootParameterCS,
			0, nullptr,
			D3D12_ROOT_SIGNATURE_FLAG_NONE);// or D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT ?

											// create a root signature with a single slot which points to a descriptor range consisting of a single constant buffer
		ComPtr<ID3DBlob> serializedRootSigCS = nullptr;
		ComPtr<ID3DBlob> errorBlobCS = nullptr;
		HRESULT hrCS = D3D12SerializeRootSignature(&rootSigDescCS, D3D_ROOT_SIGNATURE_VERSION_1,
			serializedRootSigCS.GetAddressOf(), errorBlobCS.GetAddressOf());

		if (errorBlobCS != nullptr)
		{
			::OutputDebugStringA((char*)errorBlobCS->GetBufferPointer());
		}
		ThrowIfFailed(hrCS);

		ThrowIfFailed(md3dDevice->CreateRootSignature(
			0,
			serializedRootSigCS->GetBufferPointer(),
			serializedRootSigCS->GetBufferSize(),
			IID_PPV_ARGS(mRootSignatureCS.GetAddressOf())));
	}

	//-----------Spring CS----------------------------------------------------------------------
	{
		// Root parameter can be a table, root descriptor or root constants.
		const int rootSpringCScount = 13;		//Addition
		CD3DX12_ROOT_PARAMETER slotRootParameterSpringCS[rootSpringCScount];		//Addition (Change)

																					// Perfomance TIP: Order from most frequent to least frequent.
		slotRootParameterSpringCS[0].InitAsConstants(5, 0);		//Addition (Change)
		slotRootParameterSpringCS[1].InitAsUnorderedAccessView(0);
		slotRootParameterSpringCS[2].InitAsUnorderedAccessView(1);
		slotRootParameterSpringCS[3].InitAsUnorderedAccessView(2);
		slotRootParameterSpringCS[4].InitAsUnorderedAccessView(3);
		slotRootParameterSpringCS[5].InitAsUnorderedAccessView(4);
		slotRootParameterSpringCS[6].InitAsUnorderedAccessView(5);
		slotRootParameterSpringCS[7].InitAsUnorderedAccessView(6);
		slotRootParameterSpringCS[8].InitAsUnorderedAccessView(7);
		slotRootParameterSpringCS[9].InitAsUnorderedAccessView(8);
		slotRootParameterSpringCS[10].InitAsUnorderedAccessView(9);
		slotRootParameterSpringCS[11].InitAsUnorderedAccessView(10);
		slotRootParameterSpringCS[12].InitAsUnorderedAccessView(11);

		// A root signature is an array of root parameters.
		CD3DX12_ROOT_SIGNATURE_DESC rootSigDescSpringCS(rootSpringCScount, slotRootParameterSpringCS,
			0, nullptr,
			D3D12_ROOT_SIGNATURE_FLAG_NONE);// or D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT ?

											// create a root signature with a single slot which points to a descriptor range consisting of a single constant buffer
		ComPtr<ID3DBlob> serializedRootSigSpringCS = nullptr;
		ComPtr<ID3DBlob> errorBlobSpringCS = nullptr;
		HRESULT hrSpringCS = D3D12SerializeRootSignature(&rootSigDescSpringCS, D3D_ROOT_SIGNATURE_VERSION_1,
			serializedRootSigSpringCS.GetAddressOf(), errorBlobSpringCS.GetAddressOf());

		if (errorBlobSpringCS != nullptr)
		{
			::OutputDebugStringA((char*)errorBlobSpringCS->GetBufferPointer());
		}
		ThrowIfFailed(hrSpringCS);

		ThrowIfFailed(md3dDevice->CreateRootSignature(
			0,
			serializedRootSigSpringCS->GetBufferPointer(),
			serializedRootSigSpringCS->GetBufferSize(),
			IID_PPV_ARGS(mRootSignatureSpringCS.GetAddressOf())));
	}
	
	//-----------Anchor2Segment CS----------------------------------------------------------------------
	{
		// Root parameter can be a table, root descriptor or root constants.
		const int rootAnch2SegCScount = 18;		//Addition
		CD3DX12_ROOT_PARAMETER slotRootParameterAnch2SegCS[rootAnch2SegCScount];		//Addition (Change)

																					// Perfomance TIP: Order from most frequent to least frequent.
		slotRootParameterAnch2SegCS[0].InitAsConstants(2, 0);		//Addition (Change)
		slotRootParameterAnch2SegCS[1].InitAsUnorderedAccessView(0);
		slotRootParameterAnch2SegCS[2].InitAsUnorderedAccessView(1);
		slotRootParameterAnch2SegCS[3].InitAsUnorderedAccessView(2);
		slotRootParameterAnch2SegCS[4].InitAsUnorderedAccessView(3);
		slotRootParameterAnch2SegCS[5].InitAsUnorderedAccessView(4);
		slotRootParameterAnch2SegCS[6].InitAsUnorderedAccessView(5);
		slotRootParameterAnch2SegCS[7].InitAsUnorderedAccessView(6);
		slotRootParameterAnch2SegCS[8].InitAsUnorderedAccessView(7);
		slotRootParameterAnch2SegCS[9].InitAsUnorderedAccessView(8);
		slotRootParameterAnch2SegCS[10].InitAsUnorderedAccessView(9);
		slotRootParameterAnch2SegCS[11].InitAsUnorderedAccessView(10);
		slotRootParameterAnch2SegCS[12].InitAsUnorderedAccessView(11);
		slotRootParameterAnch2SegCS[13].InitAsUnorderedAccessView(12);
		slotRootParameterAnch2SegCS[14].InitAsUnorderedAccessView(13);
		slotRootParameterAnch2SegCS[15].InitAsUnorderedAccessView(14);
		slotRootParameterAnch2SegCS[16].InitAsUnorderedAccessView(15);
		slotRootParameterAnch2SegCS[17].InitAsUnorderedAccessView(16);

		// A root signature is an array of root parameters.
		CD3DX12_ROOT_SIGNATURE_DESC rootSigDescAnch2SegCS(rootAnch2SegCScount, slotRootParameterAnch2SegCS,
			0, nullptr,
			D3D12_ROOT_SIGNATURE_FLAG_NONE);// or D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT ?

											// create a root signature with a single slot which points to a descriptor range consisting of a single constant buffer
		ComPtr<ID3DBlob> serializedRootSigAnch2SegCS = nullptr;
		ComPtr<ID3DBlob> errorBlobAnch2SegCS = nullptr;
		HRESULT hrAnch2SegCS = D3D12SerializeRootSignature(&rootSigDescAnch2SegCS, D3D_ROOT_SIGNATURE_VERSION_1,
			serializedRootSigAnch2SegCS.GetAddressOf(), errorBlobAnch2SegCS.GetAddressOf());

		if (errorBlobAnch2SegCS != nullptr)
		{
			::OutputDebugStringA((char*)errorBlobAnch2SegCS->GetBufferPointer());
		}
		ThrowIfFailed(hrAnch2SegCS);

		ThrowIfFailed(md3dDevice->CreateRootSignature(
			0,
			serializedRootSigAnch2SegCS->GetBufferPointer(),
			serializedRootSigAnch2SegCS->GetBufferSize(),
			IID_PPV_ARGS(mRootSignatureAnch2SegCS.GetAddressOf())));
	}

	//-----------Move Bound Drag CS----------------------------------------------------------------------
	{
		// Root parameter can be a table, root descriptor or root constants.
		CD3DX12_ROOT_PARAMETER slotRootParameterMoveCS[7];

		// Perfomance TIP: Order from most frequent to least frequent.
		slotRootParameterMoveCS[0].InitAsConstants(11, 0);
		slotRootParameterMoveCS[1].InitAsUnorderedAccessView(0);
		slotRootParameterMoveCS[2].InitAsUnorderedAccessView(1);
		slotRootParameterMoveCS[3].InitAsUnorderedAccessView(2);
		slotRootParameterMoveCS[4].InitAsUnorderedAccessView(3);
		slotRootParameterMoveCS[5].InitAsUnorderedAccessView(4);
		slotRootParameterMoveCS[6].InitAsUnorderedAccessView(5);

		// A root signature is an array of root parameters.
		CD3DX12_ROOT_SIGNATURE_DESC rootSigDescMoveCS(7, slotRootParameterMoveCS,
			0, nullptr,
			D3D12_ROOT_SIGNATURE_FLAG_NONE);// or D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT ?

											// create a root signature with a single slot which points to a descriptor range consisting of a single constant buffer
		ComPtr<ID3DBlob> serializedRootSigMoveCS = nullptr;
		ComPtr<ID3DBlob> errorBlobMoveCS = nullptr;
		HRESULT hrMoveCS = D3D12SerializeRootSignature(&rootSigDescMoveCS, D3D_ROOT_SIGNATURE_VERSION_1,
			serializedRootSigMoveCS.GetAddressOf(), errorBlobMoveCS.GetAddressOf());

		if (errorBlobMoveCS != nullptr)
		{
			::OutputDebugStringA((char*)errorBlobMoveCS->GetBufferPointer());
		}
		ThrowIfFailed(hrMoveCS);

		ThrowIfFailed(md3dDevice->CreateRootSignature(
			0,
			serializedRootSigMoveCS->GetBufferPointer(),
			serializedRootSigMoveCS->GetBufferSize(),
			IID_PPV_ARGS(mRootSignatureMoveCS.GetAddressOf())));
	}
}
void RenderDX12::BuildShadersAndInputLayoutCS()
{
	mShaders["forceSoACS"] = d3dUtil::CompileShader(L"Shaders\\ForceSoA.hlsl", nullptr, "CS2", "cs_5_0");
	mShaders["springSegCS"] = d3dUtil::CompileShader(L"Shaders\\SpringSeg.hlsl", nullptr, "computeSpringSeg", "cs_5_0");
	mShaders["anch2SegCS"] = d3dUtil::CompileShader(L"Shaders\\Anch2Seg.hlsl", nullptr, "computeAnchorSegment", "cs_5_0");
	mShaders["moveSegCS"] = d3dUtil::CompileShader(L"Shaders\\MoveSeg.hlsl", nullptr, "CS", "cs_5_0");
}
void RenderDX12::BuildPSO_CS()
{
	//For LJP - simulation compute
	D3D12_COMPUTE_PIPELINE_STATE_DESC computePsoDesc2 = {};
	computePsoDesc2.pRootSignature = mRootSignatureCS.Get();
	computePsoDesc2.CS =
	{
		reinterpret_cast<BYTE*>(mShaders["forceSoACS"]->GetBufferPointer()),
		mShaders["forceSoACS"]->GetBufferSize()
	};
	computePsoDesc2.Flags = D3D12_PIPELINE_STATE_FLAG_NONE;
	ThrowIfFailed(md3dDevice->CreateComputePipelineState(&computePsoDesc2, IID_PPV_ARGS(&mPSOs["forceSoA"])));

	//For Spring between particles/segments - simulation compute
	D3D12_COMPUTE_PIPELINE_STATE_DESC computePsoDescSpring = {};
	computePsoDescSpring.pRootSignature = mRootSignatureSpringCS.Get();
	computePsoDescSpring.CS =
	{
		reinterpret_cast<BYTE*>(mShaders["springSegCS"]->GetBufferPointer()),
		mShaders["springSegCS"]->GetBufferSize()
	};
	computePsoDescSpring.Flags = D3D12_PIPELINE_STATE_FLAG_NONE;
	ThrowIfFailed(md3dDevice->CreateComputePipelineState(&computePsoDescSpring, IID_PPV_ARGS(&mPSOs["springSeg"])));

	//For anchor-to-particle/segment springs - simulation compute
	D3D12_COMPUTE_PIPELINE_STATE_DESC computePsoDescAnch2Seg = {};
	computePsoDescAnch2Seg.pRootSignature = mRootSignatureAnch2SegCS.Get();
	computePsoDescAnch2Seg.CS =
	{
		reinterpret_cast<BYTE*>(mShaders["anch2SegCS"]->GetBufferPointer()),
		mShaders["anch2SegCS"]->GetBufferSize()
	};
	computePsoDescAnch2Seg.Flags = D3D12_PIPELINE_STATE_FLAG_NONE;
	ThrowIfFailed(md3dDevice->CreateComputePipelineState(&computePsoDescAnch2Seg, IID_PPV_ARGS(&mPSOs["anch2Seg"])));

	//Move Segments - simulation compute
	D3D12_COMPUTE_PIPELINE_STATE_DESC computePsoDescMove = {};
	computePsoDescMove.pRootSignature = mRootSignatureMoveCS.Get();
	computePsoDescMove.CS =
	{
		reinterpret_cast<BYTE*>(mShaders["moveSegCS"]->GetBufferPointer()),
		mShaders["moveSegCS"]->GetBufferSize()
	};
	computePsoDescMove.Flags = D3D12_PIPELINE_STATE_FLAG_NONE;
	ThrowIfFailed(md3dDevice->CreateComputePipelineState(&computePsoDescMove, IID_PPV_ARGS(&mPSOs["moveSeg"])));
}

//---------------------------Open VR----------------------------

void RenderDX12::UpdateEyeTwo(int eye) {

	//Wait condition before starting next update work
	StartCounter_ms(timerVRSync);
	vr::EVRCompositorError err0 = m_pOpenVRCompositor->WaitGetPoses(m_poseOpenVR, vr::k_unMaxTrackedDeviceCount, nullptr, 0);
	//m_pOpenVRCompositor->PostPresentHandoff();
	timeCumulativeVRSync += GetCounter_ms(timerVRSync);
	if (frameCount % 40 == 0) {
		timeModVRSync = timeCumulativeVRSync / 40.f;
		timeCumulativeVRSync = 0;
	}

	vr::HmdMatrix34_t hmdPose = m_poseOpenVR[0].mDeviceToAbsoluteTracking;
	mCamera.mRight = XMFLOAT3(hmdPose.m[0][0], hmdPose.m[1][0], -hmdPose.m[2][0]);
	mCamera.mUp = XMFLOAT3(hmdPose.m[0][1], hmdPose.m[1][1], hmdPose.m[2][1]);
	mCamera.mLook = XMFLOAT3(-hmdPose.m[0][2], -hmdPose.m[1][2], hmdPose.m[2][2]);

	/*hLocSave[hMemIter % hMemIterCnt] = XMFLOAT3(hmdPose.m[0][3], hmdPose.m[1][3], hmdPose.m[2][3]);

	XMFLOAT3 hAverage = XMFLOAT3(0.0f, 0.0f, 0.0f);
	for (int i = 0; i < hMemIterCnt; i++) {
		hAverage.x += hLocSave[i].x;
		hAverage.y += hLocSave[i].y;
		hAverage.z += hLocSave[i].z;
	}
	hAverage.x /= (float)hMemIterCnt;
	hAverage.y /= (float)hMemIterCnt;
	hAverage.z /= (float)hMemIterCnt;

	mCamera.mPosition.x = hAverage.x * vc.VIVEtoWorldScaling + vc.VR_MoveOffset.x;
	mCamera.mPosition.y = hAverage.y * vc.VIVEtoWorldScaling + vc.VR_MoveOffset.y;
	mCamera.mPosition.z = -hAverage.z * vc.VIVEtoWorldScaling + vc.VR_MoveOffset.z;*/
	
	mCamera.mPosition.x = hmdPose.m[0][3] * vc.VIVEtoWorldScaling + vc.VR_MoveOffset.x;
	mCamera.mPosition.y = hmdPose.m[1][3] * vc.VIVEtoWorldScaling + vc.VR_MoveOffset.y;
	mCamera.mPosition.z = -hmdPose.m[2][3] * vc.VIVEtoWorldScaling + vc.VR_MoveOffset.z;

	/*mCamera.mRight = XMFLOAT3(0.107311428, -0.0175852757, -0.994070172);
	mCamera.mUp = XMFLOAT3(-0.116210237, 0.992768228, 0.0301073194);
	mCamera.mLook = XMFLOAT3(0.987410665, 0.118751951, 0.104491770);
	mCamera.mPosition.x = -89;
	mCamera.mPosition.y = 61;
	mCamera.mPosition.z = -27;*/

	mCamera.mViewDirty = true;
	//mCamera.UpdateViewMatrix();
	hMemIter++;
}

void RenderDX12::ControllerButtonPress(vr::VRControllerState_t *state, RenderItem * contRI, const GameTimer& gt, int controllerID) {
	if ((state->ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Touchpad)) != 0) {
		const float threshold = 0.7f;
		const float dt = gt.DeltaTime();
		const float speed = 40.0f;
		if (controllerID == 1) {
			if (state->rAxis[0].y > threshold) {
				vc.VR_MoveOffset.x += mCamera.mLook.x * speed * dt;
				vc.VR_MoveOffset.y += mCamera.mLook.y * speed * dt;
				vc.VR_MoveOffset.z += mCamera.mLook.z * speed * dt;
			}
			else if (state->rAxis[0].y < -threshold) {
				vc.VR_MoveOffset.x -= mCamera.mLook.x * speed * dt;
				vc.VR_MoveOffset.y -= mCamera.mLook.y * speed * dt;
				vc.VR_MoveOffset.z -= mCamera.mLook.z * speed * dt;
			}
			else if (state->rAxis[0].x > threshold) {
				vc.VR_MoveOffset.x += mCamera.mRight.x * speed * dt;
				vc.VR_MoveOffset.y += mCamera.mRight.y * speed * dt;
				vc.VR_MoveOffset.z += mCamera.mRight.z * speed * dt;
			}
			else if (state->rAxis[0].x < -threshold) {
				vc.VR_MoveOffset.x -= mCamera.mRight.x * speed * dt;
				vc.VR_MoveOffset.y -= mCamera.mRight.y * speed * dt;
				vc.VR_MoveOffset.z -= mCamera.mRight.z * speed * dt;
			}
		}
		else if (controllerID == 2) {
			if (state->rAxis[0].y > threshold) {
				vc.grabForOffset += 0.1f;
				if (vc.grabForOffset > 200.0f) {
					vc.grabForOffset = 200.0f;
				}
			}
			else if (state->rAxis[0].y < -threshold) {
				vc.grabForOffset -= 0.1f;
				if (vc.grabForOffset < 5.0f) {
					vc.grabForOffset = 5.0f;
				}
			}
			else if (state->rAxis[0].x > threshold) {
				vc.VR_MoveOffset.x += mCamera.mUp.x * speed * dt;
				vc.VR_MoveOffset.y += mCamera.mUp.y * speed * dt;
				vc.VR_MoveOffset.z -= mCamera.mUp.z * speed * dt;
			}
			else if (state->rAxis[0].x < -threshold) {
				vc.VR_MoveOffset.x -= mCamera.mUp.x * speed * dt;
				vc.VR_MoveOffset.y -= mCamera.mUp.y * speed * dt;
				vc.VR_MoveOffset.z += mCamera.mUp.z * speed * dt;
			}
		}
	}

	if ((state->ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_Grip)) != 0) {
		vc.VR_MoveOffset.x = 0.0f - mCamera.mPosition.x; // Moves To 0 location in Sim Space ( it negates where you are in VR space )
		vc.VR_MoveOffset.y = 50.0f - mCamera.mPosition.y;
		vc.VR_MoveOffset.z = 0.0f - mCamera.mPosition.z;
	}

	//Trigger Actions - if(pressed) else(not pressed)
	if ((state->ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger)) != 0) {
		if (!vc.VRTriggerPressed[controllerID - 1])
			vc.VRTriggerSearching[controllerID - 1] = true;
		vc.VRTriggerPressed[controllerID - 1] = true;
		contRI->Instances[controllerID - 1].Color.x = 0.0f;
		contRI->Instances[controllerID - 1].Color.y = 1.0f * (1 / controllerID) + 0.3f * (controllerID - 1);
		contRI->Instances[controllerID - 1].Color.z = 1.0f * (controllerID - 1);
	}
	else {
		vc.VRTriggerPressed[controllerID - 1] = false;
		vc.VRTriggerSearching[controllerID - 1] = false;
		contRI->Instances[controllerID - 1].Color.x = 0.0f;
		contRI->Instances[controllerID - 1].Color.y = 0.6f * (1 / controllerID);
		contRI->Instances[controllerID - 1].Color.z = 0.6f * (controllerID - 1);
	}
}
void RenderDX12::TakeControllerInputTwo(const GameTimer& gt, int controllerID) {
	auto contRI = mOpaqueRitems[getInstIdx("controller")];
	auto lineRI = mOpaqueRitems[getInstIdx("line")];
	vr::VRControllerState_t state;
	if (m_pOpenVRSystem->GetControllerState(controllerID, &state, sizeof(state)))
	{
		ControllerButtonPress(&state, contRI, gt, controllerID);

		//------------------Orientation Work-------------------------------------------------------------------

		//Tracking controller
		vr::HmdMatrix34_t controllerPos = m_poseOpenVR[controllerID].mDeviceToAbsoluteTracking;

		//Translate VIVE Space to Sim Space
		XMFLOAT3 locationAcc;
		locationAcc.x = controllerPos.m[0][3] * vc.VIVEtoWorldScaling + vc.VR_MoveOffset.x;
		locationAcc.y = controllerPos.m[1][3] * vc.VIVEtoWorldScaling + vc.VR_MoveOffset.y;
		locationAcc.z = controllerPos.m[2][3] * -vc.VIVEtoWorldScaling + vc.VR_MoveOffset.z;

		//Read in VIVE Matrix - Carefull row vs colum major order
		XMMATRIX ViveMatrix = XMMATRIX(
			controllerPos.m[0][0], controllerPos.m[1][0], controllerPos.m[2][0], 0.0f,
			controllerPos.m[0][1], controllerPos.m[1][1], controllerPos.m[2][1], 0.0f,
			controllerPos.m[0][2], controllerPos.m[1][2], controllerPos.m[2][2], 0.0f, 
			0.0f, 0.0f, 0.0f, 1.0f);

		//Adjust Object Rotation - import pre translated model? TODO
		XMMATRIX ZfixRotation = XMMATRIX(
			cos(3.1416f), -sin(3.1416f), 0.0f, 0.0f,
			sin(3.1416f), cos(3.1416f), 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f);

		//Create Final Controller Matrix
		XMMATRIX controllerMatrix = XMMatrixMultiply(ViveMatrix, ZfixRotation);

		//Storing Matrix and location for DX Render
		XMStoreFloat4x4(&contRI->Instances[controllerID - 1].World, controllerMatrix);
		contRI->Instances[controllerID - 1].World._41 = locationAcc.x;
		contRI->Instances[controllerID - 1].World._42 = locationAcc.y;
		contRI->Instances[controllerID - 1].World._43 = locationAcc.z;

		//Matrix to scale line to interaction point
		XMMATRIX scaleLine = XMMATRIX(
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, vc.grabForOffset, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f);

		//Rotation Adjustment Matrix
		XMMATRIX XfixRotation = XMMATRIX(
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, cos(1.570796f), sin(1.570796f), 0.0f,
			0.0f, -sin(1.570796f), cos(1.570796f), 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f);

		//Create Final Line Matrix
		XMMATRIX lineMatrix = XMMatrixMultiply(scaleLine, XMMatrixMultiply(XfixRotation, controllerMatrix));

		//Storing Matrix and location for DX Render
		XMStoreFloat4x4(&lineRI->Instances[controllerID + 7].World, lineMatrix);
		lineRI->Instances[controllerID + 7].World._41 = locationAcc.x;
		lineRI->Instances[controllerID + 7].World._42 = locationAcc.y;
		lineRI->Instances[controllerID + 7].World._43 = locationAcc.z;

		//Creating Data for Controller Force Particle Interactions
		vc.controllerLook[controllerID - 1] = XMFLOAT3(contRI->Instances[controllerID - 1].World._31, contRI->Instances[controllerID - 1].World._32, contRI->Instances[controllerID - 1].World._33);
		XMFLOAT3 grabLoc = XMFLOAT3(locationAcc.x + vc.controllerLook[controllerID - 1].x*2.0f, locationAcc.y + vc.controllerLook[controllerID - 1].y*2.0f, locationAcc.z + vc.controllerLook[controllerID - 1].z*2.0f);
		vc.grabDeltaLoc[controllerID - 1] = UCP->subtract(grabLoc, vc.lastGrabLoc[controllerID - 1]);
		vc.lastGrabLoc[controllerID - 1] = grabLoc;

		//Used to draw a tubeWand with Live positioning to compare to the CUDA positioning
		//TubeWandOverride(controllerMatrix, locationAcc); not here it will be over written

		

		///Saving Data to Globals <locationAcc & controllerMatrix>
		{
			///controllerDeltaLoc Currently used only to print to the screen
			vc.controllerDeltaLoc[controllerID - 1] = UCP->subtract(locationAcc, vc.lastControllerLoc[controllerID - 1]);
			//Used for DXSim and CUDA
			vc.controllerMatrixSave[controllerID - 1] = controllerMatrix;
			vc.lastControllerLoc[controllerID - 1] = locationAcc;
			//if (controllerID == 1) { std::wstring text = L" locationAcc: x" + std::to_wstring(locationAcc.x) + L" y" + std::to_wstring(locationAcc.y) + L" z" + std::to_wstring(locationAcc.z) + L"\n"; ::OutputDebugString(text.c_str()); }

			make16mat(&vc.controllerMats16[controllerID - 1], controllerMatrix);
		}

		
		
	}
}
void RenderDX12::UpdateController(const GameTimer& gt) {
	if (!USE_ControllerThread) {
		TakeControllerInputTwo(gt, 1);
		TakeControllerInputTwo(gt, 2);
		//TakeControllerInputPredict(gt, 1);
	}
}
void RenderDX12::SubmitToHMD() {

	D3D12TextureData_t d3d12Texture;
	d3d12Texture.m_pResource = CurrentBackBuffer();
	d3d12Texture.m_pCommandQueue = mCommandQueue.Get();
	d3d12Texture.m_nNodeMask = 0;
	vr::Texture_t leftEyeTexture = { (void *)&d3d12Texture, vr::TextureType_DirectX12, vr::ColorSpace_Gamma };
	vr::VRTextureBounds_t bounds;

	if (STEREO_RENDER) {
		bounds = { 0.0f, 0.0f, 0.5f, 1.0f };
		vr::EVRCompositorError err1 = m_pOpenVRCompositor->Submit(vr::Eye_Left, &leftEyeTexture, &bounds, vr::Submit_Default);

		bounds = { 0.5f, 0.0f, 1.0f, 1.0f };
		vr::EVRCompositorError err2 = m_pOpenVRCompositor->Submit(vr::Eye_Right, &leftEyeTexture, &bounds, vr::Submit_Default);
	}
	else
	{
		bounds = { 0.125f - (float)VRoffsetFlat, 0.0f, 0.875f - (float)VRoffsetFlat, 1.0f };
		vr::EVRCompositorError err1 = m_pOpenVRCompositor->Submit(vr::Eye_Left, &leftEyeTexture, &bounds, vr::Submit_Default);

		bounds = { 0.125f + (float)VRoffsetFlat, 0.0f, 0.875f + (float)VRoffsetFlat, 1.0f };
		vr::EVRCompositorError err2 = m_pOpenVRCompositor->Submit(vr::Eye_Right, &leftEyeTexture, &bounds, vr::Submit_Default);
	}
}

bool RenderDX12::TryActivateVR()
{
	if (IsVRActive())
		return true;

	// Detect OpenVR headset
	if (vr::VR_IsHmdPresent())
	{
		if (TryActivateOpenVR())
		{
			return true;
		}
		else
		{
			DeactivateVR();
			return false;
		}
	}

	fprintf(clientPrint, "No VR headset detected");
	return false;
}

bool RenderDX12::TryActivateOpenVR()
{
	/*mClientWidth = 1890;
	mClientHeight = 1680;*/
	vrActivate = true;
	OnResize();

	// Loading the SteamVR Runtime
	vr::EVRInitError initError;
	m_pOpenVRSystem = vr::VR_Init(&initError, vr::VRApplication_Scene);
	if (initError != vr::VRInitError_None)
	{
		vrActivate = false;
		return false;
	}

	// Init compositor.
	m_pOpenVRCompositor = (vr::IVRCompositor *)vr::VR_GetGenericInterface(vr::IVRCompositor_Version, &initError);
	if (initError != vr::VRInitError_None)
	{
		fprintf(clientPrint, "VR_GetGenericInterface failed with error code: %d\nError message: %s", initError, vr::VR_GetVRInitErrorAsEnglishDescription(initError));
		vrActivate = false;
		return false;
	}

	SetupCameras();

	// Set new render target size for side-by-side stereo
	uint32_t eyeWidth, eyeHeight;
	m_pOpenVRSystem->GetRecommendedRenderTargetSize(&eyeWidth, &eyeHeight);

#if VR_EXPLICIT_TIMING
	vr::EVRCompositorTimingMode hmm = vr::VRCompositorTimingMode_Explicit_ApplicationPerformsPostPresentHandoff;//vr::VRCompositorTimingMode_Explicit_RuntimePerformsPostPresentHandoff;//
	m_pOpenVRCompositor->SetExplicitTimingMode(hmm);
#endif

	if (USE_ControllerThread) {
		InitControllerThread();
	}

	return true;
}
void RenderDX12::DeactivateVR()
{
	if (m_pOpenVRSystem)
	{
		vr::VR_Shutdown();
		m_pOpenVRSystem = nullptr;
		m_pOpenVRCompositor = nullptr;

		/*mClientWidth = 1920;
		mClientHeight = 1080;*/
		vrActivate = false;
		OnResize();

		if (USE_ControllerThread) {
			CloseControllerThread();
		}
	}
}

void RenderDX12::TubeWand(int c) {
	//make16mat(&vc.controllerMats16[c], vc.controllerMatrixSave[c]); // this is in controller method
}
void RenderDX12::TubeWandOverride(XMMATRIX controllerMatrix, XMFLOAT3 locationAcc) {
	int up;
	float spee = 0.7f;
	float separation = 1.6f;
	auto sphereRI = mOpaqueRitems[getInstIdx("segments")];
	if (m_pOpenVRSystem) { //Stationary Tubes
		float tau = 6.2831853f;
		float ttau = 12.56637f;
		up = 0;
		//Color CUDA side particles - if using 1 tube of equal size
		for (int itr = 0; itr < tubeRadius*tubeLength * 2; ++itr, ++up)
		{
			sphereRI->Instances[up].Color.x = 1.0f;
			sphereRI->Instances[up].Color.y = 0.0f;
			sphereRI->Instances[up].Color.z = 0.0f;
		}
		//Reposition and color a second set of particles
		unsigned short hapticRed = 0;
		for (int itr = 0; itr < tubeRadius*tubeLength * 2; ++itr, ++up)
		{
			XMFLOAT4 tmpQuat = { 1, 0, 0, 0 };
			XMFLOAT4 rotQuat = UCP->CreateFromYawPitchRoll((ttau / tubeRadius)*(float)(itr % (int)tubeRadius), 0, 0);
			XMFLOAT4 fQuat = UCP->Multiply(tmpQuat, rotQuat);
			float ci = sqrt(separation*separation * 2) / 2.0f;
			float ra = (separation*tubeRadius) / tau;
			XMFLOAT4 f2Quat = UCP->Multiply(fQuat, ra);

			XMFLOAT3 particleLoc;
			particleLoc.x = f2Quat.x;
			particleLoc.y = f2Quat.y + ((itr / tubeRadius)*separation);
			particleLoc.z = f2Quat.z;

			XMMATRIX scaleLine = XMMATRIX(
				1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				particleLoc.x, particleLoc.y, particleLoc.z, 1.0f);
			XMMATRIX XfixRotation = XMMATRIX(
				1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, cos(1.570796f), sin(1.570796f), 0.0f,
				0.0f, -sin(1.570796f), cos(1.570796f), 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f);

			XMMATRIX lineMatrix = XMMatrixMultiply(scaleLine, XMMatrixMultiply(XfixRotation, controllerMatrix));

			sphereRI->Instances[up].World._41 = lineMatrix.r[3].m128_f32[0] + locationAcc.x;
			sphereRI->Instances[up].World._42 = lineMatrix.r[3].m128_f32[1] + locationAcc.y;
			sphereRI->Instances[up].World._43 = lineMatrix.r[3].m128_f32[2] + locationAcc.z;
			sphereRI->Instances[up].Color.x = 0.0f;
			sphereRI->Instances[up].Color.y = 1.0f;
			sphereRI->Instances[up].Color.z = 0.0f;
		}
	}
}
void RenderDX12::TubeWandOverrideDebug(XMMATRIX *controllerMatrix, XMFLOAT3 *locationAcc, int cID, int start) {
	int up;
	float spee = 0.7f;
	float separation = 1.6f;
	auto sphereRI = mOpaqueRitems[getInstIdx("segments")];
	float tau = 6.2831853f;
	float ttau = 12.56637f;
	up = start;
	//Color CUDA side particles - if using 1 tube of equal size
	/*for (int itr = 0; itr < tubeRadius*tubeLength; ++itr, ++up)
	{
		sphereRI->Instances[up].Color.x = 1.0f;
		sphereRI->Instances[up].Color.y = 0.0f;
		sphereRI->Instances[up].Color.z = 0.0f;
	}*/
	//Reposition and color a second set of particles
	unsigned short hapticRed = 0;
	for (int itr = 0; itr < tubeRadius*tubeLength; ++itr, ++up)
	{
		XMFLOAT4 tmpQuat = { 1, 0, 0, 0 };
		XMFLOAT4 rotQuat = UCP->CreateFromYawPitchRoll((ttau / tubeRadius)*(float)(itr % (int)tubeRadius), 0, 0);
		XMFLOAT4 fQuat = UCP->Multiply(tmpQuat, rotQuat);
		float ci = sqrt(separation*separation * 2) / 2.0f;
		float ra = (separation*tubeRadius) / tau;
		XMFLOAT4 f2Quat = UCP->Multiply(fQuat, ra);

		XMFLOAT3 particleLoc;
		particleLoc.x = f2Quat.x;
		particleLoc.y = f2Quat.y + ((itr / tubeRadius)*separation);
		particleLoc.z = f2Quat.z;

		XMMATRIX scaleLine = XMMATRIX(
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			particleLoc.x, particleLoc.y, particleLoc.z, 1.0f);
		XMMATRIX XfixRotation = XMMATRIX(
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, cos(1.570796f), sin(1.570796f), 0.0f,
			0.0f, -sin(1.570796f), cos(1.570796f), 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f);

		XMMATRIX lineMatrix = XMMatrixMultiply(scaleLine, XMMatrixMultiply(XfixRotation, controllerMatrix[cID]));

		sphereRI->Instances[up].World._41 = lineMatrix.r[3].m128_f32[0] + locationAcc[cID].x;
		sphereRI->Instances[up].World._42 = lineMatrix.r[3].m128_f32[1] + locationAcc[cID].y;
		sphereRI->Instances[up].World._43 = lineMatrix.r[3].m128_f32[2] + locationAcc[cID].z;
		sphereRI->Instances[up].Color.x = 0.0f;
		sphereRI->Instances[up].Color.y = 1.0f;
		sphereRI->Instances[up].Color.z = 0.0f;
	}
}

void RenderDX12::TubeWandOverride(XMMATRIX *controllerMatrix, XMFLOAT3 *locationAcc, int cID, int start) {
	int up;
	float spee = 0.7f;
	float separation = 1.6f;
	auto sphereRI = mOpaqueRitems[getInstIdx("segments")];
	float tau = 6.2831853f;
	float ttau = 12.56637f;
	up = start;
	//Reposition particles
	unsigned short hapticRed = 0;
	for (int itr = 0; itr < tubeRadius*tubeLength; ++itr, ++up)
	{
		XMFLOAT4 tmpQuat = { 1, 0, 0, 0 };
		XMFLOAT4 rotQuat = UCP->CreateFromYawPitchRoll((ttau / tubeRadius)*(float)(itr % (int)tubeRadius), 0, 0);
		XMFLOAT4 fQuat = UCP->Multiply(tmpQuat, rotQuat);
		float ci = sqrt(separation*separation * 2) / 2.0f;
		float ra = (separation*tubeRadius) / tau;
		XMFLOAT4 f2Quat = UCP->Multiply(fQuat, ra);

		XMFLOAT3 particleLoc;
		particleLoc.x = f2Quat.x;
		particleLoc.y = f2Quat.y + ((itr / tubeRadius)*separation);
		particleLoc.z = f2Quat.z;

		XMMATRIX scaleLine = XMMATRIX(
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			particleLoc.x, particleLoc.y, particleLoc.z, 1.0f);
		XMMATRIX XfixRotation = XMMATRIX(
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, cos(1.570796f), sin(1.570796f), 0.0f,
			0.0f, -sin(1.570796f), cos(1.570796f), 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f);

		XMMATRIX lineMatrix = XMMatrixMultiply(scaleLine, XMMatrixMultiply(XfixRotation, controllerMatrix[cID]));

		sphereRI->Instances[up].World._41 = lineMatrix.r[3].m128_f32[0] + locationAcc[cID].x;
		sphereRI->Instances[up].World._42 = lineMatrix.r[3].m128_f32[1] + locationAcc[cID].y;
		sphereRI->Instances[up].World._43 = lineMatrix.r[3].m128_f32[2] + locationAcc[cID].z;
	}
}

//-----------------------------------------------------------------------------------------------------------
/////////////////////////////////////////LEAP MOTION//////////////////////////////////////////
//-----------------------------------------------------------------------------------------------------------
struct LeapHandTemp {
	float boneLength[5][4];
	float boneWidth[5][4];
	Leap::Vector boneCenter[5][4];
	Leap::Vector boneUp[5][4];
	Leap::Vector boneForward[5][4];
} leapTemp;

XMMATRIX leapToDXTransform(Leap::Vector pos, Leap::Matrix basMatx, float posScale, XMMATRIX cam) {
	Leap::Vector toRight = basMatx.yBasis.cross(basMatx.zBasis);
	toRight = toRight.normalized();
	XMMATRIX transform = XMMATRIX(toRight.x, toRight.y, -toRight.z, 0.0f,
		basMatx.yBasis.x, basMatx.yBasis.y, -basMatx.yBasis.z, 0.0f,
		-basMatx.zBasis.x, -basMatx.zBasis.y, basMatx.zBasis.z, 0.0f,
		pos.x / posScale, pos.y / posScale, -pos.z / posScale, 1.0f);
	return transform * cam;
}

XMMATRIX leapToDXTransform(Leap::Vector &startPos, float length, Leap::Vector toUp, Leap::Vector toRight, float posScale, XMMATRIX cam) {
	Leap::Vector toForward = toUp.cross(toRight);
	toForward = toForward.normalized();
	Leap::Vector halfPos = toForward * 0.5f * length;
	startPos += halfPos;

	XMMATRIX transform = XMMATRIX(toRight.x, toRight.y, -toRight.z, 0.0f,
		toUp.x, toUp.y, -toUp.z, 0.0f,
		toForward.x, toForward.y, -toForward.z, 0.0f,
		startPos.x / posScale, startPos.y / posScale, -startPos.z / posScale, 1.0f);

	startPos += halfPos;
	return transform * cam;
}

void leapTwistDownRight(Leap::Vector &targetVector, Leap::Vector upVector, Leap::Vector rightVector, float downCos, float rightCos) {
	targetVector = targetVector - downCos * upVector;
	targetVector += rightCos * rightVector;
	targetVector.normalized();
}

Leap::Vector projectPoint(Leap::Vector point, Leap::Vector planePoint, Leap::Vector planeNormal) {
	Leap::Vector temp = point - planePoint;
	float height = temp.dot(planeNormal);
	return point - height * planeNormal;
}

XMMATRIX leapToDX_HMD_Transform(Leap::Vector pos, Leap::Matrix basMatx, float posScale, XMMATRIX cam) {
	Leap::Vector toRight = basMatx.yBasis.cross(basMatx.zBasis);
	toRight = toRight.normalized();
	XMMATRIX transform = XMMATRIX(-toRight.x, -toRight.z, toRight.y, 0.0f,
		-basMatx.zBasis.x, -basMatx.zBasis.z, basMatx.zBasis.y, 0.0f,
		-basMatx.yBasis.x, -basMatx.yBasis.z, basMatx.yBasis.y, 0.0f,
		(-pos.x / posScale), (-pos.z / posScale), (pos.y / posScale), 1.0f);
	return transform * cam;
}

LeapSegments::LeapSegments(int ns) {
	numSeg = ns;
	x = new float[numSeg];
	y = new float[numSeg];
	z = new float[numSeg];
	radius = new float[numSeg];

	for (UINT i = 0; i < handData.getStartSegRight(); i++) {
		radius[i] = handData.getHandSegRad(i);
		radius[i + handData.getStartSegRight()] = radius[i];
	}
};

LeapSegments::~LeapSegments() {
	delete[] x;
	delete[] y;
	delete[] z;
	delete[] radius;
}

void LeapSegments::setX(int idx, float value) {
	x[idx] = value;
}

void LeapSegments::setY(int idx, float value) {
	y[idx] = value;
}

void LeapSegments::setZ(int idx, float value) {
	z[idx] = value;
}

float LeapSegments::getX(int idx) {
	return x[idx];
}

float LeapSegments::getY(int idx) {
	return y[idx];
}

float LeapSegments::getZ(int idx) {
	return z[idx];
}

float LeapSegments::getRad(int idx) {
	return radius[idx];
}

int LeapSegments::getNumSeg() {
	return numSeg;
}

void LeapSegments::setNumHands(int n) {
	numHands = n;
}

int LeapSegments::getNumHands() {
	return numHands;
}

void LeapSegments::segmentsPlacing(bool isLeft, Leap::Finger::Type finger, Leap::Bone::Type bone, XMMATRIX pos) {
	XMMATRIX segMX;
	int segID = 0;
	if (isLeft) {
		for (int i = handData.getStartSegBone(finger, bone); i <= handData.getEndSegBone(finger, bone); i++) {
			segMX = XMMatrixTranslation(handData.getHandSegPosX(i), handData.getHandSegPosY(i), handData.getHandSegPosZ(i)) * pos;
			setX(i, XMVectorGetX(segMX.r[3]));
			setY(i, XMVectorGetY(segMX.r[3]));
			setZ(i, XMVectorGetZ(segMX.r[3]));
		}
	}
	else {
		for (int i = handData.getStartSegBone(finger, bone); i <= handData.getEndSegBone(finger, bone); i++) {
			segMX = XMMatrixTranslation(-handData.getHandSegPosX(i), handData.getHandSegPosY(i), handData.getHandSegPosZ(i)) * pos;
			setX(i + handData.getStartSegRight(), XMVectorGetX(segMX.r[3]));
			setY(i + handData.getStartSegRight(), XMVectorGetY(segMX.r[3]));
			setZ(i + handData.getStartSegRight(), XMVectorGetZ(segMX.r[3]));
		}
	}
}

void LeapSegments::segments_HMD_Placing(bool isLeft, Leap::Finger::Type finger, Leap::Bone::Type bone, XMMATRIX pos) {
	XMMATRIX segMX;
	int segID = 0;
	if (isLeft) {
		for (int i = handData.getStartSegBone(finger, bone); i <= handData.getEndSegBone(finger, bone); i++) {
			segMX = XMMatrixTranslation(handData.getHandSegPosX(i), -handData.getHandSegPosY(i), handData.getHandSegPosZ(i)) * pos;
			setX(i, XMVectorGetX(segMX.r[3]));
			setY(i, XMVectorGetY(segMX.r[3]));
			setZ(i, XMVectorGetZ(segMX.r[3]));
		}
	}
	else {
		for (int i = handData.getStartSegBone(finger, bone); i <= handData.getEndSegBone(finger, bone); i++) {
			segMX = XMMatrixTranslation(-handData.getHandSegPosX(i), -handData.getHandSegPosY(i), handData.getHandSegPosZ(i)) * pos;
			setX(i + handData.getStartSegRight(), XMVectorGetX(segMX.r[3]));
			setY(i + handData.getStartSegRight(), XMVectorGetY(segMX.r[3]));
			setZ(i + handData.getStartSegRight(), XMVectorGetZ(segMX.r[3]));
		}
	}
}

void RenderDX12::updateLeapMotionHands() {
	//auto handItem = mOpaqueRitems[sphereRenderNumber + 2];

	leapFrame = leapController.frame();
	leapHandList = leapFrame.hands();
	LS->setNumHands(leapHandList.count());

	Leap::Frame prevFrame = leapController.frame(1);
	Leap::Vector tipVelo;
	int32_t fingerID;

	for (Leap::HandList::const_iterator hl = leapHandList.begin(); hl != leapHandList.end(); hl++) {
		Leap::FingerList fingers = (*hl).fingers();
		Leap::Bone fingerBone; //Needed????
		int instanceLeft = 0;
		int instanceRight = 20;
		int fingerType;
		int bone;
		Leap::Matrix boneBasis;
		Leap::Vector boneRight;
		Leap::Vector boneUp;
		Leap::Vector boneEnd;
		Leap::Vector projectedPoint;
		XMMATRIX transform;

		XMFLOAT4X3 camera;
		XMFLOAT3 temp = mCamera.GetRight3f();
		camera._11 = temp.x;
		camera._12 = temp.y;
		camera._13 = temp.z;
		temp = mCamera.GetUp3f();
		camera._21 = temp.x;
		camera._22 = temp.y;
		camera._23 = temp.z;
		temp = mCamera.GetLook3f();
		camera._31 = temp.x;
		camera._32 = temp.y;
		camera._33 = temp.z;
		temp = mCamera.GetPosition3f();
		camera._41 = temp.x;
		camera._42 = temp.y;
		camera._43 = temp.z;

		if ((*hl).isLeft()) {
			for (Leap::FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); fl++)
				for (int fingerType = 0; fingerType < 5; fingerType++) {
					if ((*fl).type() == fingerType) {
						/////This part is for calculating fingertips velocities/////
						fingerID = (*fl).id();
						tipVelo = (*fl).tipPosition() - prevFrame.finger(fingerID).tipPosition();
						hd1.fingerVelocity[0][fingerType] = tipVelo.magnitude();
						////////////////////////////////////////////////////////////
						for (int i = 0; i < 4; i++) {
							fingerBone = (*fl).bone((Leap::Bone::Type)i);
							transform = XMMatrixScaling(fingerBone.width() / leapVRscale, fingerBone.width() / leapVRscale, fingerBone.length() / leapVRscale)
								* leapToDXTransform(fingerBone.center(), fingerBone.basis(), leapVRscale, XMMatrixTranslation(0.0f, -250.0f/*(-250.0f)(-250.0f)(-550.0f)*/ / leapVRscale, 350.0f/*(250.0f)(350.0f)(1000.0f)*/ / leapVRscale) * XMLoadFloat4x3(&camera));
							LS->segmentsPlacing(true, (Leap::Finger::Type)fingerType, (Leap::Bone::Type)i, transform);
							instanceLeft++;
						}
					}
				}
		} else {
			for (Leap::FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); fl++)
				for (int fingerType = 0; fingerType < 5; fingerType++) {
					if ((*fl).type() == fingerType) {
						/////This part is for calculating fingertips velocities/////
						fingerID = (*fl).id();
						tipVelo = (*fl).tipPosition() - prevFrame.finger(fingerID).tipPosition();
						hd1.fingerVelocity[1][fingerType] = tipVelo.magnitude();
						////////////////////////////////////////////////////////////

						for (int i = 0; i < 4; i++) {
							fingerBone = (*fl).bone((Leap::Bone::Type)i);
							transform = XMMatrixScaling(fingerBone.width() / leapVRscale, fingerBone.width() / leapVRscale, fingerBone.length() / leapVRscale)
								* leapToDXTransform(fingerBone.center(), fingerBone.basis(), leapVRscale, XMMatrixTranslation(0.0f, -250.0f/*(-250.0f)(-250.0f)(-550.0f)*/ / leapVRscale, 350.0f/*(250.0f)(350.0f)(1000.0f)*/ / leapVRscale) * XMLoadFloat4x3(&camera));
							LS->segmentsPlacing(false, (Leap::Finger::Type)fingerType, (Leap::Bone::Type)i, transform);
							instanceRight++;
						}
					}
				}
		}
	}
}

void RenderDX12::updateLeapMotionHandsVR() {
	//auto handItem = mOpaqueRitems[sphereRenderNumber + 2];

	leapController.setPolicy(Leap::Controller::POLICY_OPTIMIZE_HMD);
	leapFrame = leapController.frame();
	leapHandList = leapFrame.hands();
	LS->setNumHands(leapHandList.count());

	for (Leap::HandList::const_iterator hl = leapHandList.begin(); hl != leapHandList.end(); hl++) {
		Leap::FingerList fingers = (*hl).fingers();
		Leap::Bone fingerBone;
		int instanceLeft = 0;
		int instanceRight = 20;

		XMFLOAT4X3 camera;
		XMFLOAT3 temp = mCamera.GetRight3f();
		camera._11 = temp.x;
		camera._12 = temp.y;
		camera._13 = temp.z;
		temp = mCamera.GetUp3f();
		camera._21 = temp.x;
		camera._22 = temp.y;
		camera._23 = temp.z;
		temp = mCamera.GetLook3f();
		camera._31 = temp.x;
		camera._32 = temp.y;
		camera._33 = temp.z;
		temp = mCamera.GetPosition3f();
		camera._41 = temp.x;
		camera._42 = temp.y;
		camera._43 = temp.z;

		if ((*hl).isLeft()) {
			for (Leap::FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); fl++)
				for (int fingerType = 0; fingerType < 5; fingerType++)
					if ((*fl).type() == fingerType) {
						for (int i = 0; i < 4; i++) {
							fingerBone = (*fl).bone((Leap::Bone::Type)i);
							XMMATRIX transform = XMMatrixScaling(fingerBone.width() / leapVRscale, fingerBone.length() / leapVRscale, fingerBone.width() / leapVRscale)
								* leapToDX_HMD_Transform(fingerBone.center(), fingerBone.basis(), leapVRscale, XMLoadFloat4x3(&camera));
							LS->segments_HMD_Placing(true, (Leap::Finger::Type)fingerType, (Leap::Bone::Type)i, transform);
							instanceLeft++;
						}
					}
		}
		else {
			for (Leap::FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); fl++)
				for (int fingerType = 0; fingerType < 5; fingerType++)
					if ((*fl).type() == fingerType) {
						for (int i = 0; i < 4; i++) {
							fingerBone = (*fl).bone((Leap::Bone::Type)i);
							XMMATRIX transform = XMMatrixScaling(fingerBone.width() / leapVRscale, fingerBone.length() / leapVRscale, fingerBone.width() / leapVRscale)
								* leapToDX_HMD_Transform(fingerBone.center(), fingerBone.basis(), leapVRscale, XMLoadFloat4x3(&camera));
							LS->segments_HMD_Placing(false, (Leap::Finger::Type)fingerType, (Leap::Bone::Type)i, transform);
							instanceRight++;
						}
					}
		}
	}
}
//-----------------------------------------------------------------------------------------------------------

/*
XMMATRIX Xro = XMMATRIX(
	1.0f, 0.0f, 0.0f, 0.0f,
	0.0f, cos(1.570796f), -sin(1.570796f), 0.0f,
	0.0f, sin(1.570796f), cos(1.570796f), 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f);

XMMATRIX Zro = XMMATRIX(
	cos(3.1416f), -sin(3.1416f), 0.0f, 0.0f,
	sin(3.1416f), cos(3.1416f), 0.0f, 0.0f,
	0.0f, 0.0f, 1.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f);

XMMATRIX Yro = XMMATRIX(
	cos(3.1416f), 0.0f, sin(3.1416f), 0.0f,
	0.0f, 1.0f, 0.0f, 0.0f,
	-sin(3.1416f), 0.0f, cos(3.1416f), 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f);

XMMATRIX ide = XMMATRIX(
	1.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 1.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 1.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f);
*/