#include "Render_DX12.h"
#include "ObjectUpdateLogic.h"

extern UpdateClass *UCP;
extern "C" void * serverThread(void *arg);
extern "C" void * networkStatusThread(void * arg);
extern "C" void * networkDataThread(void * arg);
extern "C" void * controllerWork(void * arg);

bool RenderDX12::Initialize()
{
	///////////// INPUT FILE READ IN ////////////////////
	int count = 0;
	for (int g = 0; g < 32; g++)
		inputLines[g] = "";		// Only 32 Entries
	std::string line;
	std::string inputFileName = "inputFile.txt";
	std::ifstream myfile(inputFileName);
	if (myfile.is_open())
	{
		while (getline(myfile, line))
		{
			if (line[0] != '#') {
				inputLines[count] = line;
				count++;
			}
		}
		myfile.close();
	}
	else {
		std::cout << "Error Could not find input file, check name or number.\n";
	}

	SEGMENTCOUNT = atoi(inputLines[0].c_str());
	spaceDimX_sg = (float)atoi(inputLines[1].c_str());
	spaceDimY_sg = (float)atoi(inputLines[2].c_str());
	spaceDimZ_sg = (float)atoi(inputLines[3].c_str());

	flexiGrid = (bool)atoi(inputLines[4].c_str());

	param.INTER_E = atoi(inputLines[5].c_str()) / 1000.0f;
	param.INTER_S = atoi(inputLines[6].c_str()) / 1000.0f;
	param.NEG_FORCES_ON = (unsigned int)atoi(inputLines[7].c_str());
	param.NEG_F_MULTIPLY = atoi(inputLines[8].c_str()) / 1000.0f;
	param.GRAVITY_ON = (unsigned int)atoi(inputLines[9].c_str());
	param.FORCECOLOR_ON = (unsigned int)atoi(inputLines[10].c_str());
	param.BOUNDED = (unsigned int)atoi(inputLines[11].c_str());
	int cubeLength = atoi(inputLines[12].c_str());

	saveImage = inputLines[13][0] == 't';
	CAPTURE_INTERVAL_SEC = atoi(inputLines[14].c_str());
	mTextToScreen = inputLines[15][0] == 't';
	useNetworkCompute = inputLines[16][0] == 't';
	ComputeShaderActive = inputLines[17][0] == 't';
	useCSInstanceCopy = inputLines[18][0] == 't';
	mClientWidth = atoi(inputLines[19].c_str());
	mClientHeight = atoi(inputLines[20].c_str());

	///////////////INIT D3D12 WINDOW/////////////////

	if (!D3DApp::Initialize())
		return false;

	// Reset the command list to prep for initialization commands.
	ThrowIfFailed(mCommandList->Reset(mDirectCmdListAlloc.Get(), nullptr));

	// Get the increment size of a descriptor in this heap type.  This is hardware specific, 
	// so we have to query this information.
	mCbvSrvDescriptorSize = md3dDevice->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
	/////////////////////////////////////////////////
	int threadCount = 8; omp_get_max_threads() / 2;
	omp_set_num_threads(threadCount);
	increment = 0;
	frameCount = 0;
	timer_a.CounterStart = 0;
	timer_a.PCFreq = 0;
	timerAnimate.CounterStart = 0;
	timerAnimate.PCFreq = 0;
	timerSimLag.CounterStart = 0;
	timerSimLag.PCFreq = 0;
	/////////////////////////////////////////////////

	//Must be early as the networked system sends initilization parameters
	if (useNetworkCompute) {
		InitNetwork();
		SEG_RenderCnt = SEGMENTCOUNT;
	}

	spaceDimXD2_sg = spaceDimX_sg / 2.0f;
	spaceDimZD2_sg = spaceDimZ_sg / 2.0f;
	spaceDimYD2_sg = spaceDimY_sg / 2.0f;

	//Comment Here
	geoGridx = (int)(spaceDimX_sg / coarseGraining) * coarseGraining;
	geoGridz = (int)(spaceDimZ_sg / coarseGraining) * coarseGraining;
	gridDivisorx = (int)geoGridx / 10;
	gridDivisorz = (int)geoGridz / 10;

	if (!useNetworkCompute) {
		UCP->initUpdateClass(spaceDimXD2_sg, spaceDimZD2_sg, &SEGMENTCOUNT, cubeLength, &averageSegmentsFoundLJP, &averageSegmentsComputedLJP);
		SEG_RenderCnt = SEGMENTCOUNT;
	}

	/////////////////////////////////////////////////
	if (useNetworkCompute) {
		mCamera.Pitch(1.57f);
		mCamera.SetPosition(0.0f, 70.0f, 0.0f);
		vc.VR_MoveOffset = XMFLOAT3(0.0f, 50.0f, 0.0f);
	}
	else {
		mCamera.Pitch(0.0f);
		mCamera.SetPosition(0.0f, spaceDimYD2_sg, -(spaceDimZ_sg + 150));
	}
	LoadTextures();
	BuildRootSignature();
	BuildDescriptorHeaps();
	BuildShadersAndInputLayout();
	BuildShapeGeometry();
	BuildShapeGeometryDynamic();
	BuildMaterials();
	BuildRenderItems();
	BuildFrameResources(); // Lots of memory used
	BuildPSOs();
	//---------------------------
	BuildRootSignatureCS();
	BuildShadersAndInputLayoutCS();
	BuildPSO_CS();

	if (ComputeShaderActive) {
		UCP->BuildComputeCommandQLsim();
		UCP->BuildComputeBuffers();
		UCP->BuildComputeBuffersSpring();
		UCP->BuildComputeBuffersAnch2Seg();
		UCP->BuildComputeBuffersMove();
	}

	errno_t err = fopen_s(&clientPrint, "clientPrint.txt", "w");

	// Execute the initialization commands.
	ThrowIfFailed(mCommandList->Close());
	ID3D12CommandList* cmdsLists[] = { mCommandList.Get() };
	mCommandQueue->ExecuteCommandLists(_countof(cmdsLists), cmdsLists);

	// Wait until initialization is complete.
	FlushCommandQueue();

	return true;
}

bool RenderDX12::InitNetwork() {
	bool initWaitBool = true;
	varsPoint.paused = &PAUSED;
	varsPoint.mAppPaused = &mAppPaused;
	varsPoint.running = &end_CurrentTest;
	varsPoint.minutes = &time_simulatedMinutes;
	varsPoint.seconds = &time_simulatedSeconds;
	varsPoint.camXYZ = (XMFLOAT3*)&mCamera.mPosition;
	varsPoint.screenXZ[0] = &mClientWidth;
	varsPoint.screenXZ[1] = &mClientHeight;
	varsPoint.useRegion = &USE_REGION;
	varsPoint.dataMon[0] = &dataMontitor[0];
	varsPoint.dataMon[1] = &dataMontitor[1];
	varsPoint.inputLag[0] = &simInputLag.x;
	varsPoint.inputLag[1] = &simInputLag.y;
	varsPoint.controllerMats[0] = &vc.controllerMats16[0];
	varsPoint.controllerMats[1] = &vc.controllerMats16[1];
	varsPoint.controllerLoc[0] = &vc.lastControllerLoc[0]; // What if the DirectX side re fixes the particles?
	varsPoint.controllerLoc[1] = &vc.lastControllerLoc[1];

	void * inputs[18];
	inputs[0] = &varsPoint;
	inputs[1] = &host_segLocColor;
	inputs[2] = &MTCountsMain;
	inputs[3] = &MT_COUNT;
	inputs[4] = &initWaitBool;
	inputs[5] = &SEG_RenderCnt;///
	inputs[6] = &MOTOR_COUNT;
	inputs[7] = &host_motorLocations;
	inputs[8] = &host_motorQuat;
	inputs[9] = &host_segLoc;
	inputs[10] = &host_segColor;
	inputs[11] = &MOTOR_RenderCnt;///
	inputs[12] = &host_motorQuatLocXZ;
	inputs[13] = &SEGMENTCOUNT;
	inputs[14] = &NetworkLive;
	inputs[15] = &spaceDimX_sg;
	inputs[16] = &spaceDimY_sg;
	inputs[17] = &spaceDimZ_sg;
	PAUSED = true; // Can be set to false in later init phases

	if (USEstatData) {
		int err = pthread_create(&netStatusThread, NULL, networkStatusThread, inputs);
		if (err != 0) { wprintf(L"Can't Create Status Socket thread error code %d\n", err); }
		else { wprintf(L"Created Socket Compute Server thread\n"); }

		int err1 = pthread_create(&netDataThread, NULL, networkDataThread, inputs);
		if (err1 != 0) { wprintf(L"Can't Create Data Socket thread error code %d\n", err1); }
		else { wprintf(L"Created Socket Compute Server thread\n"); }
	}
	else {
		int err1 = pthread_create(&netDataThread, NULL, serverThread, inputs);
		if (err1 != 0) { wprintf(L"Can't Create Data Socket thread error code %d\n", err1); }
		else { wprintf(L"Created Socket Compute Server thread\n"); }
	}

	while (initWaitBool) { Sleep(2); }

	UINT wtf = sizeof(quatLocXZ);
	USE_REGION = *varsPoint.useRegion;
	host_segLocColor = new XMFLOAT4[SEGMENTCOUNT];
	host_segLoc = new XMFLOAT3[SEGMENTCOUNT];
	host_segColor = new float[SEGMENTCOUNT];
	host_motorLocations = new XMFLOAT3[MOTOR_COUNT];
	host_motorQuatLocXZ = new quatLocXZ[MOTOR_COUNT];
	host_motorQuat = new XMFLOAT4[MOTOR_COUNT];
	return 0;
}

void RenderDX12::LoadTextures()
{
	std::vector<std::string> textureNamelist;
	std::vector<std::wstring> texturePathlist;

	textureNamelist.push_back("bricksTex");
	texturePathlist.push_back(L"Textures/bricks.dds");

	textureNamelist.push_back("stoneTex");
	texturePathlist.push_back(L"Textures/stone.dds");

	textureNamelist.push_back("crystalTex");
	texturePathlist.push_back(L"Textures/eye.dds");

	textureNamelist.push_back("crateTex");
	texturePathlist.push_back(L"Textures/WoodCrate01.dds");

	textureNamelist.push_back("iceTex");
	texturePathlist.push_back(L"Textures/ice.dds");

	textureNamelist.push_back("matrixAquaTex");
	texturePathlist.push_back(L"Textures/matrixAqua.dds");

	textureNamelist.push_back("defaultTex");
	texturePathlist.push_back(L"Textures/white1x1.dds");

	textureNamelist.push_back("controllerTex");
	texturePathlist.push_back(L"Textures/controllerInfo.dds");

	textureNamelist.push_back("controllerMode0");
	texturePathlist.push_back(L"Textures/controllerMode0.dds");

	textureNamelist.push_back("controllerMode1");
	texturePathlist.push_back(L"Textures/controllerMode1.dds");

	textureNamelist.push_back("controllerMode2");
	texturePathlist.push_back(L"Textures/controllerMode2.dds");

	for (int i = 0; i < textureNamelist.size(); i++) {
		auto texDiscrip = std::make_unique<Texture>();
		texDiscrip->Name = textureNamelist[i];
		texDiscrip->Filename = texturePathlist[i];
		ThrowIfFailed(DirectX::CreateDDSTextureFromFile12(md3dDevice.Get(),
			mCommandList.Get(), texDiscrip->Filename.c_str(),
			texDiscrip->Resource, texDiscrip->UploadHeap));

		mTextures[texDiscrip->Name] = std::move(texDiscrip);
	}
	/*auto bricksTex = std::make_unique<Texture>();
	bricksTex->Name = "bricksTex";
	bricksTex->Filename = L"Textures/bricks.dds";
	ThrowIfFailed(DirectX::CreateDDSTextureFromFile12(md3dDevice.Get(),
		mCommandList.Get(), bricksTex->Filename.c_str(),
		bricksTex->Resource, bricksTex->UploadHeap));

	auto stoneTex = std::make_unique<Texture>();
	stoneTex->Name = "stoneTex";
	stoneTex->Filename = L"Textures/stone.dds";
	ThrowIfFailed(DirectX::CreateDDSTextureFromFile12(md3dDevice.Get(),
		mCommandList.Get(), stoneTex->Filename.c_str(),
		stoneTex->Resource, stoneTex->UploadHeap));

	auto crystalTex = std::make_unique<Texture>();
	crystalTex->Name = "crystalTex";
	crystalTex->Filename = L"Textures/eye.dds";
	ThrowIfFailed(DirectX::CreateDDSTextureFromFile12(md3dDevice.Get(),
		mCommandList.Get(), crystalTex->Filename.c_str(),
		crystalTex->Resource, crystalTex->UploadHeap));

	auto crateTex = std::make_unique<Texture>();
	crateTex->Name = "crateTex";
	crateTex->Filename = L"Textures/WoodCrate01.dds";
	ThrowIfFailed(DirectX::CreateDDSTextureFromFile12(md3dDevice.Get(),
		mCommandList.Get(), crateTex->Filename.c_str(),
		crateTex->Resource, crateTex->UploadHeap));

	auto iceTex = std::make_unique<Texture>();
	iceTex->Name = "iceTex";
	iceTex->Filename = L"Textures/ice.dds";
	ThrowIfFailed(DirectX::CreateDDSTextureFromFile12(md3dDevice.Get(),
		mCommandList.Get(), iceTex->Filename.c_str(),
		iceTex->Resource, iceTex->UploadHeap));

	auto matrixAquaTex = std::make_unique<Texture>();
	matrixAquaTex->Name = "matrixAquaTex";
	matrixAquaTex->Filename = L"Textures/matrixAqua.dds";
	ThrowIfFailed(DirectX::CreateDDSTextureFromFile12(md3dDevice.Get(),
		mCommandList.Get(), matrixAquaTex->Filename.c_str(),
		matrixAquaTex->Resource, matrixAquaTex->UploadHeap));

	auto defaultTex = std::make_unique<Texture>();
	defaultTex->Name = "defaultTex";
	defaultTex->Filename = L"Textures/white1x1.dds";
	ThrowIfFailed(DirectX::CreateDDSTextureFromFile12(md3dDevice.Get(),
		mCommandList.Get(), defaultTex->Filename.c_str(),
		defaultTex->Resource, defaultTex->UploadHeap));

	auto controllerTex = std::make_unique<Texture>();
	controllerTex->Name = "controllerTex";
	controllerTex->Filename = L"Textures/controllerInfo.dds";
	ThrowIfFailed(DirectX::CreateDDSTextureFromFile12(md3dDevice.Get(),
		mCommandList.Get(), controllerTex->Filename.c_str(),
		controllerTex->Resource, controllerTex->UploadHeap));

	auto controllerMode0 = std::make_unique<Texture>();
	controllerMode0->Name = "controllerMode0";
	controllerMode0->Filename = L"Textures/controllerMode0.dds";
	ThrowIfFailed(DirectX::CreateDDSTextureFromFile12(md3dDevice.Get(),
		mCommandList.Get(), controllerMode0->Filename.c_str(),
		controllerMode0->Resource, controllerMode0->UploadHeap));

	auto controllerMode1 = std::make_unique<Texture>();
	controllerMode1->Name = "controllerMode1";
	controllerMode1->Filename = L"Textures/controllerMode1.dds";
	ThrowIfFailed(DirectX::CreateDDSTextureFromFile12(md3dDevice.Get(),
		mCommandList.Get(), controllerMode1->Filename.c_str(),
		controllerMode1->Resource, controllerMode1->UploadHeap));

	auto controllerMode2 = std::make_unique<Texture>();
	controllerMode2->Name = "controllerMode2";
	controllerMode2->Filename = L"Textures/controllerMode2.dds";
	ThrowIfFailed(DirectX::CreateDDSTextureFromFile12(md3dDevice.Get(),
		mCommandList.Get(), controllerMode2->Filename.c_str(),
		controllerMode2->Resource, controllerMode2->UploadHeap));

	mTextures[bricksTex->Name] = std::move(bricksTex);
	mTextures[stoneTex->Name] = std::move(stoneTex);
	mTextures[crystalTex->Name] = std::move(crystalTex);
	mTextures[crateTex->Name] = std::move(crateTex);
	mTextures[iceTex->Name] = std::move(iceTex);
	mTextures[matrixAquaTex->Name] = std::move(matrixAquaTex);
	mTextures[defaultTex->Name] = std::move(defaultTex);
	mTextures[controllerTex->Name] = std::move(controllerTex);
	mTextures[controllerMode0->Name] = std::move(controllerMode0);
	mTextures[controllerMode1->Name] = std::move(controllerMode1);
	mTextures[controllerMode2->Name] = std::move(controllerMode2);*/
}

void RenderDX12::BuildRootSignature()
{
	CD3DX12_DESCRIPTOR_RANGE texTable;
	texTable.Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 7, 0, 0);

	// Root parameter can be a table, root descriptor or root constants.
	CD3DX12_ROOT_PARAMETER slotRootParameter[4];

	// Perfomance TIP: Order from most frequent to least frequent.
	slotRootParameter[0].InitAsShaderResourceView(0, 1);
	slotRootParameter[1].InitAsShaderResourceView(1, 1);
	slotRootParameter[2].InitAsConstantBufferView(0);
	slotRootParameter[3].InitAsDescriptorTable(1, &texTable, D3D12_SHADER_VISIBILITY_PIXEL);

	auto staticSamplers = GetStaticSamplers();

	// A root signature is an array of root parameters.
	CD3DX12_ROOT_SIGNATURE_DESC rootSigDesc(4, slotRootParameter,
		(UINT)staticSamplers.size(), staticSamplers.data(),
		D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

	// create a root signature with a single slot which points to a descriptor range consisting of a single constant buffer
	ComPtr<ID3DBlob> serializedRootSig = nullptr;
	ComPtr<ID3DBlob> errorBlob = nullptr;
	//This function has been superceded by D3D12SerializeVersionedRootSignature as of the Windows 10 Anniversary Update (14393).
	HRESULT hr = D3D12SerializeRootSignature(&rootSigDesc, D3D_ROOT_SIGNATURE_VERSION_1,
		serializedRootSig.GetAddressOf(), errorBlob.GetAddressOf());

	if (errorBlob != nullptr)
	{
		::OutputDebugStringA((char*)errorBlob->GetBufferPointer());
	}
	ThrowIfFailed(hr);

	ThrowIfFailed(md3dDevice->CreateRootSignature(
		0,
		serializedRootSig->GetBufferPointer(),
		serializedRootSig->GetBufferSize(),
		IID_PPV_ARGS(mRootSignature.GetAddressOf())));

}

void RenderDX12::BuildDescriptorHeaps()
{
	//
	// Create the SRV heap.
	//
	D3D12_DESCRIPTOR_HEAP_DESC srvHeapDesc = {};
	srvHeapDesc.NumDescriptors = 11; ///CAUTION <---X
	srvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
	srvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
	ThrowIfFailed(md3dDevice->CreateDescriptorHeap(&srvHeapDesc, IID_PPV_ARGS(&mSrvDescriptorHeap)));

	//
	// Fill out the heap with actual descriptors.
	//
	CD3DX12_CPU_DESCRIPTOR_HANDLE hDescriptor(mSrvDescriptorHeap->GetCPUDescriptorHandleForHeapStart());

	D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
	srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
	srvDesc.Texture2D.MostDetailedMip = 0;
	srvDesc.Texture2D.ResourceMinLODClamp = 0.0f;
	srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;

	std::vector<Microsoft::WRL::ComPtr<ID3D12Resource>> textureList;
	textureList.push_back(mTextures["bricksTex"]->Resource);
	textureList.push_back(mTextures["stoneTex"]->Resource);
	textureList.push_back(mTextures["crystalTex"]->Resource);
	textureList.push_back(mTextures["crateTex"]->Resource);
	textureList.push_back(mTextures["iceTex"]->Resource);
	textureList.push_back(mTextures["matrixAquaTex"]->Resource);
	textureList.push_back(mTextures["defaultTex"]->Resource);
	textureList.push_back(mTextures["controllerTex"]->Resource);
	textureList.push_back(mTextures["controllerMode0"]->Resource);
	textureList.push_back(mTextures["controllerMode1"]->Resource);
	textureList.push_back(mTextures["controllerMode2"]->Resource);

	for (int i = 0; i < textureList.size(); i++) {
		srvDesc.Format = textureList[i]->GetDesc().Format;
		srvDesc.Texture2D.MipLevels = textureList[i]->GetDesc().MipLevels;
		md3dDevice->CreateShaderResourceView(textureList[i].Get(), &srvDesc, hDescriptor);

		if(i < textureList.size()-1)
			hDescriptor.Offset(1, mCbvSrvDescriptorSize);
	}
}

void RenderDX12::BuildShadersAndInputLayout()
{
	const D3D_SHADER_MACRO alphaTestDefines[] =
	{
		"ALPHA_TEST", "1",
		NULL, NULL
	};

	mShaders["standardVS"] = d3dUtil::CompileShader(L"Shaders\\Default.hlsl", nullptr, "VS", "vs_5_1");
	mShaders["opaquePS"] = d3dUtil::CompileShader(L"Shaders\\Default.hlsl", nullptr, "PS", "ps_5_1");

	mShaders["texColorVS"] = d3dUtil::CompileShader(L"Shaders\\Default.hlsl", nullptr, "VS2", "vs_5_1");
	mShaders["texColorPS"] = d3dUtil::CompileShader(L"Shaders\\Default.hlsl", nullptr, "PS2", "ps_5_1");

	mInputLayout =
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
		{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
		{ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 24, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
		//	{ "COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 1, 32,  D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
	};
}

void RenderDX12::BuildSkullGeometry()
{
	std::ifstream fin("Models/skull.txt");

	if (!fin)
	{
		MessageBox(0, L"Models/skull.txt not found.", 0, 0);
		return;
	}

	UINT vcount = 0;
	UINT tcount = 0;
	std::string ignore;

	fin >> ignore >> vcount;
	fin >> ignore >> tcount;
	fin >> ignore >> ignore >> ignore >> ignore;

	XMFLOAT3 vMinf3(+MathHelper::Infinity, +MathHelper::Infinity, +MathHelper::Infinity);
	XMFLOAT3 vMaxf3(-MathHelper::Infinity, -MathHelper::Infinity, -MathHelper::Infinity);

	XMVECTOR vMin = XMLoadFloat3(&vMinf3);
	XMVECTOR vMax = XMLoadFloat3(&vMaxf3);

	std::vector<Vertex> vertices(vcount);
	for (UINT i = 0; i < vcount; ++i)
	{
		fin >> vertices[i].Pos.x >> vertices[i].Pos.y >> vertices[i].Pos.z;
		fin >> vertices[i].Normal.x >> vertices[i].Normal.y >> vertices[i].Normal.z;

		XMVECTOR P = XMLoadFloat3(&vertices[i].Pos);

		// Project point onto unit sphere and generate spherical texture coordinates.
		XMFLOAT3 spherePos;
		XMStoreFloat3(&spherePos, XMVector3Normalize(P));

		float theta = atan2f(spherePos.z, spherePos.x);

		// Put in [0, 2pi].
		if (theta < 0.0f)
			theta += XM_2PI;

		float phi = acosf(spherePos.y);

		float u = theta / (2.0f*XM_PI);
		float v = phi / XM_PI;

		vertices[i].TexC = { u, v };

		vMin = XMVectorMin(vMin, P);
		vMax = XMVectorMax(vMax, P);
	}

	BoundingBox bounds;
	XMStoreFloat3(&bounds.Center, 0.5f*(vMin + vMax));
	XMStoreFloat3(&bounds.Extents, 0.5f*(vMax - vMin));

	fin >> ignore;
	fin >> ignore;
	fin >> ignore;

	std::vector<std::int32_t> indices(3 * tcount);
	for (UINT i = 0; i < tcount; ++i)
	{
		fin >> indices[i * 3 + 0] >> indices[i * 3 + 1] >> indices[i * 3 + 2];
	}

	fin.close();

	//
	// Pack the indices of all the meshes into one index buffer.
	//

	const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);

	const UINT ibByteSize = (UINT)indices.size() * sizeof(std::int32_t);

	auto geo = std::make_unique<MeshGeometry>();
	geo->Name = "skullGeo";

	ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
	CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

	ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
	CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

	geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
		mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

	geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
		mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

	geo->VertexByteStride = sizeof(Vertex);
	geo->VertexBufferByteSize = vbByteSize;
	geo->IndexFormat = DXGI_FORMAT_R32_UINT;
	geo->IndexBufferByteSize = ibByteSize;

	SubmeshGeometry submesh;
	submesh.IndexCount = (UINT)indices.size();
	submesh.StartIndexLocation = 0;
	submesh.BaseVertexLocation = 0;
	submesh.Bounds = bounds;

	geo->DrawArgs["skull"] = submesh;

	mGeometries[geo->Name] = std::move(geo);
}

void RenderDX12::BuildShapeGeometry()
{
	GeometryGenerator geoGen;
	const UINT objCount = 7;

	GeometryGenerator::MeshData meshDataList[12];
	meshDataList[0] = geoGen.CreateBox(300, .1f, 100, 1);
	meshDataList[1] = geoGen.CreateGrid(geoGridx, geoGridz, (uint32_t)(geoGridx / gridDivisorx), (uint32_t)(geoGridz / gridDivisorz));
	//meshDataList[1] = geoGen.CreateGrid(2, 2, 2, 2);
	meshDataList[2] = geoGen.CreateSphere(sphereRadius, 12, 6); //geoGen.CreateSphere(sphereRadius, 16, 8);//
	meshDataList[3] = geoGen.CreateCylinder(0.3f, 0.6f, 4.0f, 20, 20);
	meshDataList[4] = geoGen.CreateLine(1.0f);

	GeometryGenerator::LoadData loadObjDataList[4];

	//Load in Controller, takes ages in debug mode so using a simpler model when debugging
#if defined(DEBUG) | defined(_DEBUG)
	geoGen.parseScene("Models\\controlerSmooth2.obj", &loadObjDataList[0]);
#else
	geoGen.parseScene("Models\\controlerDL.obj", &loadObjDataList[0]); //https://sketchfab.com/models/ab115087cc724ee193287cd8557c862a#
#endif
	meshDataList[5] = geoGen.CreateOBJ(loadObjDataList[0]);

	//Loading in motor model
	geoGen.parseScene("Models\\motor.obj", &loadObjDataList[1]);
	meshDataList[6] = geoGen.CreateOBJ(loadObjDataList[1]);

	//
	// We are concatenating all the geometry into one big vertex/index buffer.  So
	// define the regions in the buffer each submesh covers.
	//

	// Cache the vertex offsets to each object in the concatenated vertex buffer.
	UINT vertexOffsetList[objCount] = { 0 };
	for (int i = 1; i < objCount; i++) {
		vertexOffsetList[i] = meshDataList[i - 1].Vertices.size() + vertexOffsetList[i - 1];
	}

	// Cache the starting index for each object in the concatenated index buffer.
	UINT indexOffsetList[objCount] = { 0 };
	for (int i = 1; i < objCount; i++) {
		indexOffsetList[i] = meshDataList[i - 1].Indices32.size() + indexOffsetList[i - 1];
	}

	//Save indexing/offset information
	SubmeshGeometry subMeshGeoList[objCount]; // Random Length
	for (int i = 0; i < objCount; i++) {
		subMeshGeoList[i].IndexCount = (UINT)meshDataList[i].Indices32.size();
		subMeshGeoList[i].StartIndexLocation = indexOffsetList[i];
		subMeshGeoList[i].BaseVertexLocation = vertexOffsetList[i];
	}

	//
	// Extract the vertex elements we are interested in and pack the
	// vertices of all the meshes into one vertex buffer.
	//

	UINT totalVertexCount = 0;
	for (int i = 0; i < objCount; i++) {
		totalVertexCount += meshDataList[i].Vertices.size();
	}

	std::vector<Vertex> vertices(totalVertexCount);

	//Loop trhough tall the mesh data and load all vertex info into a single vertex list
	UINT k = 0;
	for (size_t obj = 0; obj < objCount; ++obj)
	{
		for (size_t i = 0; i < meshDataList[obj].Vertices.size(); ++i, ++k)
		{
			vertices[k].Pos = meshDataList[obj].Vertices[i].Position;
			vertices[k].Normal = meshDataList[obj].Vertices[i].Normal;
			vertices[k].TexC = meshDataList[obj].Vertices[i].TexC;
		}
	}

	//Loop through meshData to add all obj indices to one main indice list
	std::vector<std::uint16_t> indices;
	for (int i = 0; i < objCount; i++) {
		indices.insert(indices.end(), std::begin(meshDataList[i].GetIndices16()), std::end(meshDataList[i].GetIndices16()));
	}

	const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);
	const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

	auto geo = std::make_unique<MeshGeometry>();
	geo->Name = "shapeGeo";

	ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
	CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

	ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
	CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

	geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
		mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

	geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
		mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

	geo->VertexByteStride = sizeof(Vertex);
	geo->VertexBufferByteSize = vbByteSize;
	geo->IndexFormat = DXGI_FORMAT_R16_UINT;
	geo->IndexBufferByteSize = ibByteSize;

	geo->DrawArgs["box"] = subMeshGeoList[0];
	geo->DrawArgs["grid"] = subMeshGeoList[1];
	geo->DrawArgs["sphere"] = subMeshGeoList[2];
	geo->DrawArgs["cylinder"] = subMeshGeoList[3];
	geo->DrawArgs["line"] = subMeshGeoList[4];
	geo->DrawArgs["controller"] = subMeshGeoList[5];
	geo->DrawArgs["motor"] = subMeshGeoList[6];

	mGeometries[geo->Name] = std::move(geo);
}

void RenderDX12::BuildShapeGeometry2()
{
	GeometryGenerator geoGen;

	GeometryGenerator::MeshData box = geoGen.CreateBox(2.0f, 1.5f, 4.0f, 1);
	GeometryGenerator::MeshData grid = geoGen.CreateGrid(geoGridx, geoGridz, (uint32_t)(geoGridx / gridDivisorx), (uint32_t)(geoGridz / gridDivisorz));
	GeometryGenerator::MeshData sphere = geoGen.CreateSphere(sphereRadius, 12, 6); //geoGen.CreateSphere(sphereRadius, 16, 8);//
	GeometryGenerator::MeshData cylinder = geoGen.CreateCylinder(0.3f, 0.6f, 4.0f, 20, 20);
	GeometryGenerator::MeshData line = geoGen.CreateLine(1.0f);

	GeometryGenerator geoTest;
	GeometryGenerator::LoadData test;
	geoTest.parseScene("Models\\controlerSmooth2.obj", &test);
	GeometryGenerator::MeshData OBJ = geoTest.CreateOBJ(test);

	UINT objCount = 6;

	//
	// We are concatenating all the geometry into one big vertex/index buffer.  So
	// define the regions in the buffer each submesh covers.
	//

	// Cache the vertex offsets to each object in the concatenated vertex buffer.

	UINT boxVertexOffset = 0;
	UINT gridVertexOffset = (UINT)box.Vertices.size();
	UINT sphereVertexOffset = gridVertexOffset + (UINT)grid.Vertices.size();
	UINT cylinderVertexOffset = sphereVertexOffset + (UINT)sphere.Vertices.size();
	UINT lineVertexOffset = cylinderVertexOffset + (UINT)cylinder.Vertices.size();
	UINT OBJVertexOffset = lineVertexOffset + (UINT)line.Vertices.size();

	// Cache the starting index for each object in the concatenated index buffer.

	UINT boxIndexOffset = 0;
	UINT gridIndexOffset = (UINT)box.Indices32.size();
	UINT sphereIndexOffset = gridIndexOffset + (UINT)grid.Indices32.size();
	UINT cylinderIndexOffset = sphereIndexOffset + (UINT)sphere.Indices32.size();
	UINT lineIndexOffset = cylinderIndexOffset + (UINT)cylinder.Indices32.size();
	UINT OBJIndexOffset = lineIndexOffset + (UINT)line.Indices32.size();

	SubmeshGeometry boxSubmesh;
	boxSubmesh.IndexCount = (UINT)box.Indices32.size();
	boxSubmesh.StartIndexLocation = boxIndexOffset;
	boxSubmesh.BaseVertexLocation = boxVertexOffset;

	SubmeshGeometry gridSubmesh;
	gridSubmesh.IndexCount = (UINT)grid.Indices32.size();
	gridSubmesh.StartIndexLocation = gridIndexOffset;
	gridSubmesh.BaseVertexLocation = gridVertexOffset;

	SubmeshGeometry sphereSubmesh;
	sphereSubmesh.IndexCount = (UINT)sphere.Indices32.size();
	sphereSubmesh.StartIndexLocation = sphereIndexOffset;
	sphereSubmesh.BaseVertexLocation = sphereVertexOffset;

	SubmeshGeometry cylinderSubmesh;
	cylinderSubmesh.IndexCount = (UINT)cylinder.Indices32.size();
	cylinderSubmesh.StartIndexLocation = cylinderIndexOffset;
	cylinderSubmesh.BaseVertexLocation = cylinderVertexOffset;

	SubmeshGeometry lineSubmesh;
	lineSubmesh.IndexCount = (UINT)line.Indices32.size();
	lineSubmesh.StartIndexLocation = lineIndexOffset;
	lineSubmesh.BaseVertexLocation = lineVertexOffset;

	SubmeshGeometry ObjSubmesh;
	ObjSubmesh.IndexCount = (UINT)OBJ.Indices32.size();
	ObjSubmesh.StartIndexLocation = OBJIndexOffset;
	ObjSubmesh.BaseVertexLocation = OBJVertexOffset;


	//
	// Extract the vertex elements we are interested in and pack the
	// vertices of all the meshes into one vertex buffer.
	//

	UINT totalVertexCount =
		box.Vertices.size() +
		grid.Vertices.size() +
		sphere.Vertices.size() +
		cylinder.Vertices.size() +
		line.Vertices.size() +
		OBJ.Vertices.size();

	std::vector<Vertex> vertices(totalVertexCount);

	UINT k = 0;
	for (size_t i = 0; i < box.Vertices.size(); ++i, ++k)
	{
		vertices[k].Pos = box.Vertices[i].Position;
		vertices[k].Normal = box.Vertices[i].Normal;
		vertices[k].TexC = box.Vertices[i].TexC;
	}
	for (size_t i = 0; i < grid.Vertices.size(); ++i, ++k)
	{
		vertices[k].Pos = grid.Vertices[i].Position;
		vertices[k].Normal = grid.Vertices[i].Normal;
		vertices[k].TexC = grid.Vertices[i].TexC;
		//unsigned int r = 0;
		//unsigned int g = ((vertices[k].Pos.x + (gridx / 2.0f)) / gridx) * 255.0f;
		//unsigned int b = ((vertices[k].Pos.z + (gridz / 2.0f)) / gridz) * 255.0f;
		//unsigned int a = 255;
		//float rg = ((unsigned int)r << 8) | g;
		//float ba = ((unsigned int)b << 8) | a;
		//vertices[k].TexC = { rg, ba };
	}
	for (size_t i = 0; i < sphere.Vertices.size(); ++i, ++k)
	{
		vertices[k].Pos = sphere.Vertices[i].Position;
		vertices[k].Normal = sphere.Vertices[i].Normal;
		vertices[k].TexC = sphere.Vertices[i].TexC;
		//float theta = atan2f(vertices[k].Pos.z, vertices[k].Pos.x);
		//// Put in [0, 2pi].
		//if (theta < 0.0f)
		//	theta += XM_2PI;
		//float normPrePhi = (vertices[k].Pos.y - (-sphereRadius)) / (sphereRadius - (-sphereRadius));
		//float phi = acosf(normPrePhi);
		//float u = theta / (2.0f*XM_PI);
		//float v = phi / XM_PI;
		//vertices[k].TexC = { u, v };
	}
	for (size_t i = 0; i < cylinder.Vertices.size(); ++i, ++k)
	{
		vertices[k].Pos = cylinder.Vertices[i].Position;
		vertices[k].Normal = cylinder.Vertices[i].Normal;
	}
	for (size_t i = 0; i < line.Vertices.size(); ++i, ++k)
	{
		vertices[k].Pos = line.Vertices[i].Position;
		vertices[k].Normal = line.Vertices[i].Normal;
		vertices[k].TexC = line.Vertices[i].TexC;
	}
	for (size_t i = 0; i < OBJ.Vertices.size(); ++i, ++k)
	{
		vertices[k].Pos = OBJ.Vertices[i].Position;
		vertices[k].Normal = OBJ.Vertices[i].Normal;
		vertices[k].TexC = OBJ.Vertices[i].TexC;
	}

	std::vector<std::uint16_t> indices;
	indices.insert(indices.end(), std::begin(box.GetIndices16()), std::end(box.GetIndices16()));
	indices.insert(indices.end(), std::begin(grid.GetIndices16()), std::end(grid.GetIndices16()));
	indices.insert(indices.end(), std::begin(sphere.GetIndices16()), std::end(sphere.GetIndices16()));
	indices.insert(indices.end(), std::begin(cylinder.GetIndices16()), std::end(cylinder.GetIndices16()));
	indices.insert(indices.end(), std::begin(line.GetIndices16()), std::end(line.GetIndices16()));
	indices.insert(indices.end(), std::begin(OBJ.GetIndices16()), std::end(OBJ.GetIndices16()));

	const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);
	const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

	auto geo = std::make_unique<MeshGeometry>();
	geo->Name = "shapeGeo";

	ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
	CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

	ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
	CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

	geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
		mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

	geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
		mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

	geo->VertexByteStride = sizeof(Vertex);
	geo->VertexBufferByteSize = vbByteSize;
	geo->IndexFormat = DXGI_FORMAT_R16_UINT;
	geo->IndexBufferByteSize = ibByteSize;

	geo->DrawArgs["box"] = boxSubmesh;
	geo->DrawArgs["grid"] = gridSubmesh;
	geo->DrawArgs["sphere"] = sphereSubmesh;
	geo->DrawArgs["cylinder"] = cylinderSubmesh;
	geo->DrawArgs["line"] = lineSubmesh;
	geo->DrawArgs["controller"] = ObjSubmesh;

	mGeometries[geo->Name] = std::move(geo);
}

void RenderDX12::BuildShapeGeometryDynamic()
{
	GeometryGenerator geoGen;
	GeometryGenerator::MeshData box = geoGen.CreateBox(1.5f, 0.5f, 1.5f, 1);
	float gridx = size1D_fg + size1D_fg*0.1f; float gridz = size1D_fg + size1D_fg*0.1f;
	int vertCntx = (int)gridx / vertDivisor_fg; int vertCntz = (int)gridz / vertDivisor_fg;
	GeometryGenerator::MeshData grid = geoGen.CreateGrid(gridx, gridz, vertCntx, vertCntz);
	float sphereRadius = 2.f;
	GeometryGenerator::MeshData sphere = geoGen.CreateSphere(sphereRadius, 12, 6);
	GeometryGenerator::MeshData cylinder = geoGen.CreateCylinder(0.5f, 0.3f, 3.0f, 20, 20);

	// Cache the vertex offsets to each object in the concatenated vertex buffer.
	UINT boxVertexOffset = 0;
	UINT gridVertexOffset = (UINT)box.Vertices.size();
	UINT sphereVertexOffset = gridVertexOffset + (UINT)grid.Vertices.size();
	UINT cylinderVertexOffset = sphereVertexOffset + (UINT)sphere.Vertices.size();

	// Cache the starting index for each object in the concatenated index buffer.
	UINT boxIndexOffset = 0;
	UINT gridIndexOffset = (UINT)box.Indices32.size();
	UINT sphereIndexOffset = gridIndexOffset + (UINT)grid.Indices32.size();
	UINT cylinderIndexOffset = sphereIndexOffset + (UINT)sphere.Indices32.size();

	SubmeshGeometry boxSubmesh;
	boxSubmesh.IndexCount = (UINT)box.Indices32.size();
	boxSubmesh.StartIndexLocation = (INT)boxIndexOffset;
	boxSubmesh.BaseVertexLocation = (INT)boxVertexOffset;
	boxSubmesh.VertexCount = (INT)box.Vertices.size();

	SubmeshGeometry gridSubmesh;
	gridSubmesh.IndexCount = (UINT)grid.Indices32.size();
	gridSubmesh.StartIndexLocation = (INT)gridIndexOffset;
	gridSubmesh.BaseVertexLocation = (INT)gridVertexOffset;
	gridSubmesh.VertexCount = (INT)grid.Vertices.size();

	SubmeshGeometry sphereSubmesh;
	sphereSubmesh.IndexCount = (UINT)sphere.Indices32.size();
	sphereSubmesh.StartIndexLocation = (INT)sphereIndexOffset;
	sphereSubmesh.BaseVertexLocation = (INT)sphereVertexOffset;
	sphereSubmesh.VertexCount = (INT)sphere.Vertices.size();

	SubmeshGeometry cylinderSubmesh;
	cylinderSubmesh.IndexCount = (UINT)cylinder.Indices32.size();
	cylinderSubmesh.StartIndexLocation = (INT)cylinderIndexOffset;
	cylinderSubmesh.BaseVertexLocation = (INT)cylinderVertexOffset;
	cylinderSubmesh.VertexCount = (INT)cylinder.Vertices.size();

	//
	// Extract the vertex elements we are interested in and pack the
	// vertices of all the meshes into one vertex buffer.
	//

	UINT totalVertexCount =
		(UINT)box.Vertices.size() +
		(UINT)grid.Vertices.size() +
		(UINT)sphere.Vertices.size() +
		(UINT)cylinder.Vertices.size();

	mVertexCountDynamic = (UINT)totalVertexCount;
	mDynamicVBCPU.resize(mVertexCountDynamic);

	UINT k = 0;
	for (size_t i = 0; i < box.Vertices.size(); ++i, ++k)
	{
		mDynamicVBCPU[k].Pos = box.Vertices[i].Position;
		mDynamicVBCPU[k].Normal = box.Vertices[i].Normal;
	}

	for (size_t i = 0; i < grid.Vertices.size(); ++i, ++k)
	{
		mDynamicVBCPU[k].Pos = grid.Vertices[i].Position;
		mDynamicVBCPU[k].Normal = grid.Vertices[i].Normal;
		unsigned int r = (UINT)(((mDynamicVBCPU[k].Pos.x + (gridx / 2.0f)) / gridx) * 255.0f);
		unsigned int g = 0;
		unsigned int b = (UINT)(((mDynamicVBCPU[k].Pos.z + (gridz / 2.0f)) / gridz) * 255.0f);
		unsigned int a = 255;
		float rg = (float)((r << 8) | g);
		float ba = (float)((b << 8) | a);
		mDynamicVBCPU[k].TexC = { rg, ba };
	}

	for (size_t i = 0; i < sphere.Vertices.size(); ++i, ++k)
	{
		mDynamicVBCPU[k].Pos = sphere.Vertices[i].Position;
		mDynamicVBCPU[k].Normal = sphere.Vertices[i].Normal;

		float theta = atan2f(mDynamicVBCPU[k].Pos.z, mDynamicVBCPU[k].Pos.x);
		// Put in [0, 2pi].
		if (theta < 0.0f)
			theta += XM_2PI;
		float normPrePhi = (mDynamicVBCPU[k].Pos.y - (-sphereRadius)) / (sphereRadius - (-sphereRadius));
		float phi = acosf(normPrePhi);
		float u = theta / (2.0f*XM_PI);
		float v = phi / XM_PI;
		mDynamicVBCPU[k].TexC = { u, v };
	}

	for (size_t i = 0; i < cylinder.Vertices.size(); ++i, ++k)
	{
		mDynamicVBCPU[k].Pos = cylinder.Vertices[i].Position;
		mDynamicVBCPU[k].Normal = cylinder.Vertices[i].Normal;
	}

	std::vector<std::uint16_t> indices;
	indices.insert(indices.end(), std::begin(box.GetIndices16()), std::end(box.GetIndices16()));
	indices.insert(indices.end(), std::begin(grid.GetIndices16()), std::end(grid.GetIndices16()));
	indices.insert(indices.end(), std::begin(sphere.GetIndices16()), std::end(sphere.GetIndices16()));
	indices.insert(indices.end(), std::begin(cylinder.GetIndices16()), std::end(cylinder.GetIndices16()));

	const UINT vbByteSize = (UINT)mDynamicVBCPU.size() * sizeof(Vertex);
	const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

	auto geo = std::make_unique<MeshGeometry>();
	geo->Name = "shapeGeoDy";

	//ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
	//CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

	ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
	CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

	//geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
	//	mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

	geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
		mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

	geo->VertexByteStride = sizeof(Vertex);
	geo->VertexBufferByteSize = vbByteSize;
	geo->IndexFormat = DXGI_FORMAT_R16_UINT;
	geo->IndexBufferByteSize = ibByteSize;

	geo->DrawArgs["boxDy"] = boxSubmesh;
	geo->DrawArgs["gridDy"] = gridSubmesh;
	geo->DrawArgs["sphereDy"] = sphereSubmesh;
	geo->DrawArgs["cylinderDy"] = cylinderSubmesh;

	mGeometries[geo->Name] = std::move(geo);
}

void RenderDX12::BuildMaterials()
{
	float matAlpha = 0.5f;

	auto bricks0 = std::make_unique<Material>();
	bricks0->Name = "bricks0";
	bricks0->MatCBIndex = 0;
	bricks0->DiffuseSrvHeapIndex = 0; // Texture index 'CreateShaderResourceView'
	bricks0->DiffuseAlbedo = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
	bricks0->FresnelR0 = XMFLOAT3(0.02f, 0.02f, 0.02f);
	bricks0->Roughness = 0.1f;

	auto stone0 = std::make_unique<Material>();
	stone0->Name = "stone0";
	stone0->MatCBIndex = 1;
	stone0->DiffuseSrvHeapIndex = 1;
	stone0->DiffuseAlbedo = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
	stone0->FresnelR0 = XMFLOAT3(0.05f, 0.05f, 0.05f);
	stone0->Roughness = 0.3f;

	auto crystalMat = std::make_unique<Material>();
	crystalMat->Name = "crystalMat";
	crystalMat->MatCBIndex = 2;
	crystalMat->DiffuseSrvHeapIndex = 2;
	crystalMat->DiffuseAlbedo = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
	crystalMat->FresnelR0 = XMFLOAT3(0.02f, 0.02f, 0.02f);
	crystalMat->Roughness = 0.3f;

	auto crate0 = std::make_unique<Material>();
	crate0->Name = "checkboard0";
	crate0->MatCBIndex = 3;
	crate0->DiffuseSrvHeapIndex = 3;
	crate0->DiffuseAlbedo = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
	crate0->FresnelR0 = XMFLOAT3(0.05f, 0.05f, 0.05f);
	crate0->Roughness = 0.2f;

	auto ice0 = std::make_unique<Material>();
	ice0->Name = "ice0";
	ice0->MatCBIndex = 4;
	ice0->DiffuseSrvHeapIndex = 4;
	ice0->DiffuseAlbedo = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
	ice0->FresnelR0 = XMFLOAT3(0.1f, 0.1f, 0.1f);
	ice0->Roughness = 0.0f;

	auto matrixAquaMat = std::make_unique<Material>();
	matrixAquaMat->Name = "matrixAquaMat";
	matrixAquaMat->MatCBIndex = 5;
	matrixAquaMat->DiffuseSrvHeapIndex = 5; //matrixAquaTex
	matrixAquaMat->DiffuseAlbedo = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
	matrixAquaMat->FresnelR0 = XMFLOAT3(0.05f, 0.05f, 0.05f);
	matrixAquaMat->Roughness = 0.2f;

	auto greenMat = std::make_unique<Material>();
	greenMat->Name = "greenMat";
	greenMat->MatCBIndex = 6;
	greenMat->DiffuseSrvHeapIndex = 6; //default
	greenMat->DiffuseAlbedo = XMFLOAT4(0.1f, 1.0f, 0.3f, matAlpha);
	greenMat->FresnelR0 = XMFLOAT3(0.05f, 0.05f, 0.05f);
	greenMat->Roughness = 0.5f;

	auto blackMat = std::make_unique<Material>();
	blackMat->Name = "blackMat";
	blackMat->MatCBIndex = 7;
	blackMat->DiffuseSrvHeapIndex = 6; //default
	blackMat->DiffuseAlbedo = XMFLOAT4(0.0f, 0.0f, 0.05f, matAlpha);
	blackMat->FresnelR0 = XMFLOAT3(0.1f, 0.1f, 0.1f);
	blackMat->Roughness = 0.5f;

	auto purpleMat = std::make_unique<Material>();
	purpleMat->Name = "purpleMat";
	purpleMat->MatCBIndex = 8;
	purpleMat->DiffuseSrvHeapIndex = 6; //default
	purpleMat->DiffuseAlbedo = XMFLOAT4(1.0f, 1.0f, 1.0f, matAlpha);
	purpleMat->FresnelR0 = XMFLOAT3(0.05f, 0.05f, 0.05f);
	purpleMat->Roughness = 0.5f;

	auto controllerInfo = std::make_unique<Material>();
	controllerInfo->Name = "controllerInfo";
	controllerInfo->MatCBIndex = 9;
	controllerInfo->DiffuseSrvHeapIndex = 7; // Texture index 'CreateShaderResourceView'
	controllerInfo->DiffuseAlbedo = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
	controllerInfo->FresnelR0 = XMFLOAT3(0.02f, 0.02f, 0.02f);
	controllerInfo->Roughness = 0.1f;

	auto forceMat = std::make_unique<Material>();
	forceMat->Name = "forceMat";
	forceMat->MatCBIndex = 10;
	forceMat->DiffuseSrvHeapIndex = 6; //default
	forceMat->DiffuseAlbedo = XMFLOAT4(1.0f, 0.1f, 0.1f, 0.2f);
	forceMat->FresnelR0 = XMFLOAT3(0.05f, 0.05f, 0.05f);
	forceMat->Roughness = 0.5f;

	auto controllerMode0 = std::make_unique<Material>();
	controllerMode0->Name = "controllerMode0";
	controllerMode0->MatCBIndex = 11;
	controllerMode0->DiffuseSrvHeapIndex = 8;
	controllerMode0->DiffuseAlbedo = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
	controllerMode0->FresnelR0 = XMFLOAT3(0.02f, 0.02f, 0.02f);
	controllerMode0->Roughness = 0.1f;

	auto controllerMode1 = std::make_unique<Material>();
	controllerMode1->Name = "controllerMode1";
	controllerMode1->MatCBIndex = 12;
	controllerMode1->DiffuseSrvHeapIndex = 9;
	controllerMode1->DiffuseAlbedo = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
	controllerMode1->FresnelR0 = XMFLOAT3(0.02f, 0.02f, 0.02f);
	controllerMode1->Roughness = 0.1f;

	auto controllerMode2 = std::make_unique<Material>();
	controllerMode2->Name = "controllerMode2";
	controllerMode2->MatCBIndex = 13;
	controllerMode2->DiffuseSrvHeapIndex = 10; // Texture index 'CreateShaderResourceView'
	controllerMode2->DiffuseAlbedo = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
	controllerMode2->FresnelR0 = XMFLOAT3(0.02f, 0.02f, 0.02f);
	controllerMode2->Roughness = 0.1f;

	mMaterials["bricks0"] = std::move(bricks0);
	mMaterials["stone0"] = std::move(stone0);
	mMaterials["crystalMat"] = std::move(crystalMat);
	mMaterials["crate0"] = std::move(crate0);
	mMaterials["ice0"] = std::move(ice0);
	mMaterials["matrixAquaMat"] = std::move(matrixAquaMat);
	mMaterials["greenMat"] = std::move(greenMat);
	mMaterials["blackMat"] = std::move(blackMat);
	mMaterials["purpleMat"] = std::move(purpleMat);
	mMaterials["controllerInfo"] = std::move(controllerInfo);
	mMaterials["forceMat"] = std::move(forceMat);
	mMaterials["controllerMode0"] = std::move(controllerMode0);
	mMaterials["controllerMode1"] = std::move(controllerMode1);
	mMaterials["controllerMode2"] = std::move(controllerMode2);
}

void RenderDX12::BuildRenderItems()
{
	//sphereRenderNumber++;
	auto gridRirem = std::make_unique<RenderItem>();
	gridRirem->World = MathHelper::Identity4x4();
	gridRirem->TexTransform = MathHelper::Identity4x4();
	gridRirem->ObjCBIndex = 0;
	gridRirem->Mat = mMaterials["crystalMat"].get();
	gridRirem->Geo = mGeometries["shapeGeo"].get();
	gridRirem->PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
	gridRirem->InstanceCount = 1;
	gridRirem->IndexCount = gridRirem->Geo->DrawArgs["grid"].IndexCount;
	gridRirem->StartIndexLocation = gridRirem->Geo->DrawArgs["grid"].StartIndexLocation;
	gridRirem->BaseVertexLocation = gridRirem->Geo->DrawArgs["grid"].BaseVertexLocation;
	gridRirem->Bounds = gridRirem->Geo->DrawArgs["grid"].Bounds;

	// Generate instance data.
	const int gridCount = 1;
	instancingCounts.maxCounts[instancingCounts.instanceGroupCount] = gridCount;
	strncpy_s(instancingCounts.name[instancingCounts.instanceGroupCount], 64, "grid", std::string("grid").length());
	instancingCounts.instanceGroupCount++;
	//mInstanceGridCount = gridCount;
	gridRirem->Instances.resize(gridCount);

	for (int index = 0; index < gridCount; ++index)
	{
		// Position instanced along a 3D grid.

		gridRirem->Instances[index].World = XMFLOAT4X4(
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			(0.0f), (0.0f), (0.0f), 1.0f);

		//gridRirem->Instances[index].Color = XMFLOAT3(1.0f, 1.0f, 1.0f);
		gridRirem->Instances[index].Color = XMFLOAT3(0.0f, 0.0f, 0.3f);

		XMStoreFloat4x4(&gridRirem->Instances[index].TexTransform, XMMatrixScaling(1.0f, 1.0f, 1.0f));
		gridRirem->Instances[index].MaterialIndex = 4;
	}

	mAllRitems.push_back(std::move(gridRirem));


	//-------------------------------------------------------------------
	if (flexiGrid) {
		//sphereRenderNumber++;
		auto gridDynamicRirem = std::make_unique<RenderItem>();
		gridDynamicRirem->World = MathHelper::Identity4x4();
		gridDynamicRirem->TexTransform = MathHelper::Identity4x4();
		gridDynamicRirem->ObjCBIndex = 0;
		gridDynamicRirem->Mat = mMaterials["crystalMat"].get();
		gridDynamicRirem->Geo = mGeometries["shapeGeoDy"].get();
		gridDynamicRirem->PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
		gridDynamicRirem->InstanceCount = 0;
		gridDynamicRirem->IndexCount = gridDynamicRirem->Geo->DrawArgs["gridDy"].IndexCount;
		gridDynamicRirem->StartIndexLocation = gridDynamicRirem->Geo->DrawArgs["gridDy"].StartIndexLocation;
		gridDynamicRirem->BaseVertexLocation = gridDynamicRirem->Geo->DrawArgs["gridDy"].BaseVertexLocation;
		gridDynamicRirem->Bounds = gridDynamicRirem->Geo->DrawArgs["gridDy"].Bounds;

		// Generate instance data.
		const int gridDyCount = 1;
		instancingCounts.maxCounts[instancingCounts.instanceGroupCount] = gridDyCount;
		strncpy_s(instancingCounts.name[instancingCounts.instanceGroupCount], 64, "flexiGrid", std::string("flexiGrid").length());
		instancingCounts.instanceGroupCount++;
		//mInstanceGridDyCount = gridDyCount;
		gridDynamicRirem->Instances.resize(gridDyCount);
		gridDynamicRirem->InstanceCount = gridDyCount;


		for (int index = 0; index < gridDyCount; ++index)
		{
			// Position instanced along a 3D grid.

			gridDynamicRirem->Instances[index].World = XMFLOAT4X4(
				1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				(size1D_fg), (0.0f), (0.0f), 1.0f);

			gridDynamicRirem->Instances[index].Color = XMFLOAT3(0.0f, 0.0f, 0.0f);

			XMStoreFloat4x4(&gridDynamicRirem->Instances[index].TexTransform, XMMatrixScaling(1.f, 1.f, 1.f));
			gridDynamicRirem->Instances[index].MaterialIndex = index % mMaterials.size();
		}

		mDynamicRitem = gridDynamicRirem.get();


		mAllRitems.push_back(std::move(gridDynamicRirem));
	}

	//-------------------------------------------------------------------

	//sphereRenderNumber++;
	auto controllerRI = std::make_unique<RenderItem>();
	controllerRI->World = MathHelper::Identity4x4();
	controllerRI->TexTransform = MathHelper::Identity4x4();
	controllerRI->ObjCBIndex = 0;
	controllerRI->Mat = mMaterials["crystalMat"].get();
	controllerRI->Geo = mGeometries["shapeGeo"].get();
	controllerRI->PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
	controllerRI->InstanceCount = 0;
	controllerRI->IndexCount = controllerRI->Geo->DrawArgs["controller"].IndexCount;
	controllerRI->StartIndexLocation = controllerRI->Geo->DrawArgs["controller"].StartIndexLocation;
	controllerRI->BaseVertexLocation = controllerRI->Geo->DrawArgs["controller"].BaseVertexLocation;
	controllerRI->Bounds = controllerRI->Geo->DrawArgs["controller"].Bounds;

	// Generate instance data.
	UINT controllerCount = 2;
	instancingCounts.maxCounts[instancingCounts.instanceGroupCount] = controllerCount;
	strncpy_s(instancingCounts.name[instancingCounts.instanceGroupCount], 64, "controller", std::string("controller").length());
	instancingCounts.instanceGroupCount++;
	controllerRI->Instances.resize(controllerCount);
	controllerRI->InstanceCount = controllerCount;

	for (UINT k = 0; k < controllerCount; ++k)
	{
		controllerRI->Instances[k].World = XMFLOAT4X4(
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			-20.0f + (k*40.0f), CONTROLERy, -100.0f, 1.0f);

		controllerRI->Instances[k].Color = XMFLOAT3(1.0f, 1.0f, 1.0f);

		XMStoreFloat4x4(&controllerRI->Instances[k].TexTransform, XMMatrixScaling(1.f, 1.f, 1.f));
		controllerRI->Instances[k].MaterialIndex = k + 6;
	}

	mAllRitems.push_back(std::move(controllerRI));

	///---------------------------------------------------------------------

	UINT sphereCount1, sphereCount2;
	if (SEGMENTCOUNT > renderCntMax) {
		sphereCount1 = SEGMENTCOUNT / 2;
		sphereCount2 = SEGMENTCOUNT - sphereCount1;
		inst1Max = sphereCount1;
	}
	else {
		sphereCount1 = SEGMENTCOUNT;
		sphereCount2 = 0;
	}
	auto sphereRitem = std::make_unique<RenderItem>();
	sphereRitem->World = MathHelper::Identity4x4();
	sphereRitem->TexTransform = MathHelper::Identity4x4();
	sphereRitem->ObjCBIndex = 0;
	sphereRitem->Mat = mMaterials["crystalMat"].get();
	sphereRitem->Geo = mGeometries["shapeGeo"].get();
	sphereRitem->PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
	sphereRitem->InstanceCount = 0;
	sphereRitem->IndexCount = sphereRitem->Geo->DrawArgs["sphere"].IndexCount;
	sphereRitem->StartIndexLocation = sphereRitem->Geo->DrawArgs["sphere"].StartIndexLocation;
	sphereRitem->BaseVertexLocation = sphereRitem->Geo->DrawArgs["sphere"].BaseVertexLocation;
	sphereRitem->Bounds = sphereRitem->Geo->DrawArgs["sphere"].Bounds;
	/*sphereRitem->IndexCount = sphereRitem->Geo->DrawArgs["box"].IndexCount;
	sphereRitem->StartIndexLocation = sphereRitem->Geo->DrawArgs["box"].StartIndexLocation;
	sphereRitem->BaseVertexLocation = sphereRitem->Geo->DrawArgs["box"].BaseVertexLocation;
	sphereRitem->Bounds = sphereRitem->Geo->DrawArgs["box"].Bounds;*/

	// Generate instance data.
	//mInstanceSphereCount = SEGMENTCOUNT;
	instancingCounts.maxCounts[instancingCounts.instanceGroupCount] = sphereCount1;
	strncpy_s(instancingCounts.name[instancingCounts.instanceGroupCount], 64, "segments", std::string("segments").length());
	instancingCounts.instanceGroupCount++;
	sphereRitem->Instances.resize(sphereCount1);
	sphereRitem->InstanceCount = (useNetworkCompute) ? 1 : sphereCount1;

	for (UINT k = 0; k < (UINT)sphereCount1; ++k)
	{
		sphereRitem->Instances[k].World = XMFLOAT4X4(
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 5.0f, 0.0f, 1.0f);

		sphereRitem->Instances[k].Color = XMFLOAT3(0.0f, 0.0f, 0.0f);

		XMStoreFloat4x4(&sphereRitem->Instances[k].TexTransform, XMMatrixScaling(2.f, 2.f, 1.f));
		sphereRitem->Instances[k].MaterialIndex = k % mMaterials.size();
	}

	mAllRitems.push_back(std::move(sphereRitem));

	if (sphereCount2 != 0) {
		auto sphereRitem2 = std::make_unique<RenderItem>();
		sphereRitem2->World = MathHelper::Identity4x4();
		sphereRitem2->TexTransform = MathHelper::Identity4x4();
		sphereRitem2->ObjCBIndex = 0;
		sphereRitem2->Mat = mMaterials["crystalMat"].get();
		sphereRitem2->Geo = mGeometries["shapeGeo"].get();
		sphereRitem2->PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
		sphereRitem2->InstanceCount = 0;
		sphereRitem2->IndexCount = sphereRitem2->Geo->DrawArgs["sphere"].IndexCount;
		sphereRitem2->StartIndexLocation = sphereRitem2->Geo->DrawArgs["sphere"].StartIndexLocation;
		sphereRitem2->BaseVertexLocation = sphereRitem2->Geo->DrawArgs["sphere"].BaseVertexLocation;
		sphereRitem2->Bounds = sphereRitem2->Geo->DrawArgs["sphere"].Bounds;

		instancingCounts.maxCounts[instancingCounts.instanceGroupCount] = sphereCount2;
		strncpy_s(instancingCounts.name[instancingCounts.instanceGroupCount], 64, "segments", std::string("segments").length());
		instancingCounts.instanceGroupCount++;
		sphereRitem2->Instances.resize(sphereCount2);
		sphereRitem2->InstanceCount = (useNetworkCompute) ? 1 : sphereCount2;

		for (UINT k = 0; k < (UINT)sphereCount2; ++k)
		{
			sphereRitem2->Instances[k].World = XMFLOAT4X4(
				1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				5.0f, 5.0f, 0.0f, 1.0f);

			sphereRitem2->Instances[k].Color = XMFLOAT3(0.0f, 0.0f, 0.0f);

			XMStoreFloat4x4(&sphereRitem2->Instances[k].TexTransform, XMMatrixScaling(2.f, 2.f, 1.f));
			sphereRitem2->Instances[k].MaterialIndex = k % mMaterials.size();
		}

		mAllRitems.push_back(std::move(sphereRitem2));
	}

	///-----------------------------------------------------------------
	if (MOTOR_COUNT > 0) {
		auto motorRitem = std::make_unique<RenderItem>();
		motorRitem->World = MathHelper::Identity4x4();
		motorRitem->TexTransform = MathHelper::Identity4x4();
		motorRitem->ObjCBIndex = 0;
		motorRitem->Mat = mMaterials["crystalMat"].get();
		motorRitem->Geo = mGeometries["shapeGeo"].get();
		motorRitem->PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
		motorRitem->InstanceCount = 0;
		motorRitem->IndexCount = motorRitem->Geo->DrawArgs["motor"].IndexCount;
		motorRitem->StartIndexLocation = motorRitem->Geo->DrawArgs["motor"].StartIndexLocation;
		motorRitem->BaseVertexLocation = motorRitem->Geo->DrawArgs["motor"].BaseVertexLocation;
		motorRitem->Bounds = motorRitem->Geo->DrawArgs["motor"].Bounds;

		// Generate instance data.
		instancingCounts.maxCounts[instancingCounts.instanceGroupCount] = MOTOR_COUNT;
		strncpy_s(instancingCounts.name[instancingCounts.instanceGroupCount], 64, "motor", std::string("motor").length());
		instancingCounts.instanceGroupCount++;
		motorRitem->Instances.resize(MOTOR_COUNT);
		motorRitem->InstanceCount = MOTOR_COUNT;

		for (UINT k = 0; k < (UINT)MOTOR_COUNT; ++k)
		{
			motorRitem->Instances[k].World = XMFLOAT4X4(
				1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f);

			motorRitem->Instances[k].Color = XMFLOAT3(1.0f, 0.0f, 0.0f);

			XMStoreFloat4x4(&motorRitem->Instances[k].TexTransform, XMMatrixScaling(2.f, 2.f, 1.f));
			motorRitem->Instances[k].MaterialIndex = k % mMaterials.size();
		}

		mAllRitems.push_back(std::move(motorRitem));
	}
	//-----------------------------------------------------------------

	auto lineRitem = std::make_unique<RenderItem>();
	lineRitem->World = MathHelper::Identity4x4();
	lineRitem->TexTransform = MathHelper::Identity4x4();
	lineRitem->ObjCBIndex = 0;
	lineRitem->Mat = mMaterials["crystalMat"].get();
	lineRitem->Geo = mGeometries["shapeGeo"].get();
	lineRitem->PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_LINELIST;
	lineRitem->InstanceCount = 0;
	lineRitem->IndexCount = lineRitem->Geo->DrawArgs["line"].IndexCount;
	lineRitem->StartIndexLocation = lineRitem->Geo->DrawArgs["line"].StartIndexLocation;
	lineRitem->BaseVertexLocation = lineRitem->Geo->DrawArgs["line"].BaseVertexLocation;
	lineRitem->Bounds = lineRitem->Geo->DrawArgs["line"].Bounds;

	// Generate instance data.
	const UINT lineCount = 10;
	instancingCounts.maxCounts[instancingCounts.instanceGroupCount] = lineCount;
	strncpy_s(instancingCounts.name[instancingCounts.instanceGroupCount], 64, "line", std::string("line").length());
	instancingCounts.instanceGroupCount++;
	lineRitem->Instances.resize(lineCount);
	lineRitem->InstanceCount = lineCount;

	XMFLOAT3 locs[lineCount] = {
		XMFLOAT3(spaceDimXD2_sg, 0.0f, spaceDimZD2_sg),
		XMFLOAT3(-spaceDimXD2_sg, 0.0f, spaceDimZD2_sg),
		XMFLOAT3(spaceDimXD2_sg, 0.0f, -spaceDimZD2_sg),
		XMFLOAT3(-spaceDimXD2_sg, 0.0f, -spaceDimZD2_sg),
		XMFLOAT3(-spaceDimXD2_sg, spaceDimY_sg, spaceDimZD2_sg), //Top
		XMFLOAT3(-spaceDimXD2_sg, spaceDimY_sg, -spaceDimZD2_sg),
		XMFLOAT3(spaceDimXD2_sg, spaceDimY_sg, spaceDimZD2_sg),
		XMFLOAT3(-spaceDimXD2_sg, spaceDimY_sg, spaceDimZD2_sg),
		XMFLOAT3(-20.0f, CONTROLERy, -100.0f),
		XMFLOAT3(+20.0f, CONTROLERy, -100.0f)
	};
	float scaleY[lineCount] = {
		spaceDimY_sg,
		spaceDimY_sg,
		spaceDimY_sg,
		spaceDimY_sg,
		spaceDimX_sg,
		spaceDimX_sg,
		spaceDimZ_sg,
		spaceDimZ_sg,
		30.0f,
		30.0f
	};

	for (UINT k = 0; k < (UINT)lineCount; ++k)
	{
		XMMATRIX scale = XMMATRIX(
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, scaleY[k], 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f);

		XMMATRIX rotate = XMMATRIX(
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f);

		if (k == 6 || k == 7) {
			rotate = XMMATRIX(
				1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, cos(1.570796f), -sin(1.570796f), 0.0f,
				0.0f, sin(1.570796f), cos(1.570796f), 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f);
		}
		if (k == 4 || k == 5) {
			rotate = XMMATRIX(
				cos(1.570796f), -sin(1.570796f), 0.0f, 0.0f,
				sin(1.570796f), cos(1.570796f), 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f);
		}

		XMMATRIX transform = XMMATRIX(
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			locs[k].x, locs[k].y, locs[k].z, 1.0f);

		scale = XMMatrixMultiply(scale, rotate);
		scale = XMMatrixMultiply(scale, transform);
		XMStoreFloat4x4(&lineRitem->Instances[k].World, scale);

		lineRitem->Instances[k].Color = XMFLOAT3(0.2f, 0.2f, 0.2f);

		if (k >= 8) {
			lineRitem->Instances[k].Color = XMFLOAT3(1.0f, 0.2f, 1.0f);
		}
		XMStoreFloat4x4(&lineRitem->Instances[k].TexTransform, XMMatrixScaling(1.0f, 1.0f, 1.0f));
		lineRitem->Instances[k].MaterialIndex = 4;
	}

	mAllRitems.push_back(std::move(lineRitem));

	//-----------------------------------------------------------------------------------------------------------

	if (1) {
		/*auto menuRitem = std::make_unique<RenderItem>();
		menuRitem->World = MathHelper::Identity4x4();
		menuRitem->TexTransform = MathHelper::Identity4x4();
		menuRitem->ObjCBIndex = 0;
		menuRitem->Mat = mMaterials["controllerInfo"].get();
		menuRitem->Geo = mGeometries["shapeGeo"].get();
		menuRitem->PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
		menuRitem->InstanceCount = 0;
		menuRitem->IndexCount = menuRitem->Geo->DrawArgs["box"].IndexCount;
		menuRitem->StartIndexLocation = menuRitem->Geo->DrawArgs["box"].StartIndexLocation;
		menuRitem->BaseVertexLocation = menuRitem->Geo->DrawArgs["box"].BaseVertexLocation;
		menuRitem->Bounds = menuRitem->Geo->DrawArgs["box"].Bounds;

		// Generate instance data.
		instancingCounts.maxCounts[instancingCounts.instanceGroupCount] = 1;
		strncpy_s(instancingCounts.name[instancingCounts.instanceGroupCount], 64, "boxMenue", std::string("boxMenue").length());
		instancingCounts.instanceGroupCount++;
		menuRitem->Instances.resize(1);
		menuRitem->InstanceCount = 1;

		for (UINT k = 0; k < (UINT)1; ++k)
		{
			//XMStoreFloat4x4(&menuRitem->Instances[k].World, XMMatrixRotationX(-2.0f));/// MENU ITEM
			XMStoreFloat4x4(&menuRitem->Instances[k].World, XMMatrixRotationX(-1.5708f));/// MENU ITEM
			menuRitem->Instances[k].World._41 = 0.0f;
			//menuRitem->Instances[k].World._42 = spaceDimY_sg - 30;
			menuRitem->Instances[k].World._42 = spaceDimY_sg - 80;
			menuRitem->Instances[k].World._43 = spaceDimZD2_sg + 20;

			menuRitem->Instances[k].Color = XMFLOAT3(1.0f, 1.0f, 1.0f);

			XMStoreFloat4x4(&menuRitem->Instances[k].TexTransform, XMMatrixScaling(1.f, 1.f, 1.f));
			menuRitem->Instances[k].MaterialIndex = 9;
		}

		mAllRitems.push_back(std::move(menuRitem));*/

		/*auto controllModes = std::make_unique<RenderItem>();
		controllModes->World = MathHelper::Identity4x4();
		controllModes->TexTransform = MathHelper::Identity4x4();
		controllModes->ObjCBIndex = 0;
		controllModes->Mat = mMaterials["controllerMode0"].get();
		controllModes->Geo = mGeometries["shapeGeo"].get();
		controllModes->PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
		controllModes->InstanceCount = 0;
		controllModes->IndexCount = controllModes->Geo->DrawArgs["box"].IndexCount;
		controllModes->StartIndexLocation = controllModes->Geo->DrawArgs["box"].StartIndexLocation;
		controllModes->BaseVertexLocation = controllModes->Geo->DrawArgs["box"].BaseVertexLocation;
		controllModes->Bounds = controllModes->Geo->DrawArgs["box"].Bounds;

		instancingCounts.maxCounts[instancingCounts.instanceGroupCount] = 3;
		strncpy_s(instancingCounts.name[instancingCounts.instanceGroupCount], 64, "controllModes", std::string("controllModes").length());
		instancingCounts.instanceGroupCount++;
		controllModes->Instances.resize(3);
		controllModes->InstanceCount = 3;

		for (UINT k = 0; k < (UINT)3; ++k)
		{
			XMStoreFloat4x4(&controllModes->Instances[k].World, XMMatrixMultiply(XMMatrixRotationX(-1.5708f), XMMatrixScaling(0.2f, 0.2f, 1.0f)));

			controllModes->Instances[k].World._41 = 0.0f;
			controllModes->Instances[k].World._42 = (spaceDimY_sg - 80) - ((k * 23) + 70);
			controllModes->Instances[k].World._43 = spaceDimZD2_sg + 20;

			//controllModes->Instances[k].Color = XMFLOAT3(k == 0, k == 1, k == 2);
			controllModes->Instances[k].Color = XMFLOAT3(1, 1, 1);
			controllModes->Instances[k].MaterialIndex = 11 + k;
		}

		mAllRitems.push_back(std::move(controllModes));*/

		auto forceParticle = std::make_unique<RenderItem>();
		forceParticle->World = MathHelper::Identity4x4();
		forceParticle->TexTransform = MathHelper::Identity4x4();
		forceParticle->ObjCBIndex = 0;
		forceParticle->Mat = mMaterials["forceMat"].get();
		forceParticle->Geo = mGeometries["shapeGeo"].get();
		forceParticle->PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
		forceParticle->InstanceCount = 0;
		forceParticle->IndexCount = forceParticle->Geo->DrawArgs["sphere"].IndexCount;
		forceParticle->StartIndexLocation = forceParticle->Geo->DrawArgs["sphere"].StartIndexLocation;
		forceParticle->BaseVertexLocation = forceParticle->Geo->DrawArgs["sphere"].BaseVertexLocation;
		forceParticle->Bounds = forceParticle->Geo->DrawArgs["sphere"].Bounds;

		instancingCounts.maxCounts[instancingCounts.instanceGroupCount] = 2;
		strncpy_s(instancingCounts.name[instancingCounts.instanceGroupCount], 64, "forceParticle", std::string("forceParticle").length());
		instancingCounts.instanceGroupCount++;
		forceParticle->Instances.resize(2);
		forceParticle->InstanceCount = 2;

		for (UINT k = 0; k < (UINT)2; ++k)
		{
			forceParticle->Instances[k].World = XMFLOAT4X4(
				2.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 2.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 2.0f, 0.0f,
				10.0f + (-20.0f*k), CONTROLERy, 0.0f, 1.0f);

			forceParticle->Instances[k].Color = XMFLOAT3(1.0f, 1.0f, 1.0f);
			forceParticle->Instances[k].MaterialIndex = 10;
		}

		mAllRitems.push_back(std::move(forceParticle));
	}
	// All the render items are opaque.
	for (auto& e : mAllRitems)
		mOpaqueRitems.push_back(e.get());
}

void RenderDX12::BuildFrameResources()
{
	for (int i = 0; i < gNumFrameResources; ++i)
	{
		mFrameResources.push_back(std::make_unique<FrameResource>(md3dDevice.Get(),
			1, instancingCounts, flexiGrid, mVertexCountDynamic, (UINT)mMaterials.size()));
	}
	ThrowIfFailed(mCommandQueue->GetTimestampFrequency(&mDirectCommandQueueTimestampFrequencies));
}

void RenderDX12::BuildPSOs()
{
	D3D12_GRAPHICS_PIPELINE_STATE_DESC linePsoDesc;

	//
	// PSO for opaque objects.
	//
	ZeroMemory(&linePsoDesc, sizeof(D3D12_GRAPHICS_PIPELINE_STATE_DESC));
	linePsoDesc.InputLayout = { mInputLayout.data(), (UINT)mInputLayout.size() };
	linePsoDesc.pRootSignature = mRootSignature.Get();
	linePsoDesc.VS =
	{
		reinterpret_cast<BYTE*>(mShaders["standardVS"]->GetBufferPointer()),
		mShaders["standardVS"]->GetBufferSize()
	};
	linePsoDesc.PS =
	{
		reinterpret_cast<BYTE*>(mShaders["opaquePS"]->GetBufferPointer()),
		mShaders["opaquePS"]->GetBufferSize()
	};
	/*linePsoDesc.VS =
	{
	reinterpret_cast<BYTE*>(mShaders["texColorVS"]->GetBufferPointer()),
	mShaders["texColorVS"]->GetBufferSize()
	};
	linePsoDesc.PS =
	{
	reinterpret_cast<BYTE*>(mShaders["texColorPS"]->GetBufferPointer()),
	mShaders["texColorPS"]->GetBufferSize()
	};*/
	//StreamOutput.???? https://msdn.microsoft.com/en-us/library/windows/desktop/bb205122(v=vs.85).aspx
	linePsoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
	linePsoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
	linePsoDesc.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC(D3D12_DEFAULT);
	linePsoDesc.SampleMask = UINT_MAX;             // No samples are disabled with 0xffffffff
	linePsoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE;
	linePsoDesc.NumRenderTargets = 1;              // Render Targets 
	linePsoDesc.RTVFormats[0] = mBackBufferFormat; // With Render targets
	linePsoDesc.SampleDesc.Count = m4xMsaaState ? msaaFactor : 1;
	linePsoDesc.SampleDesc.Quality = m4xMsaaState ? (m4xMsaaQuality - 1) : 0;
	linePsoDesc.DSVFormat = mDepthStencilFormat;
	ThrowIfFailed(md3dDevice->CreateGraphicsPipelineState(&linePsoDesc, IID_PPV_ARGS(&mPSOs["line"])));

	D3D12_GRAPHICS_PIPELINE_STATE_DESC opaquePsoDesc;

	//
	// PSO for opaque objects.
	//
	ZeroMemory(&opaquePsoDesc, sizeof(D3D12_GRAPHICS_PIPELINE_STATE_DESC));
	opaquePsoDesc.InputLayout = { mInputLayout.data(), (UINT)mInputLayout.size() };
	opaquePsoDesc.pRootSignature = mRootSignature.Get();
	opaquePsoDesc.VS =
	{
		reinterpret_cast<BYTE*>(mShaders["standardVS"]->GetBufferPointer()),
		mShaders["standardVS"]->GetBufferSize()
	};
	opaquePsoDesc.PS =
	{
		reinterpret_cast<BYTE*>(mShaders["opaquePS"]->GetBufferPointer()),
		mShaders["opaquePS"]->GetBufferSize()
	};
	//StreamOutput.???? https://msdn.microsoft.com/en-us/library/windows/desktop/bb205122(v=vs.85).aspx
	opaquePsoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
	opaquePsoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
	opaquePsoDesc.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC(D3D12_DEFAULT);
	opaquePsoDesc.SampleMask = UINT_MAX;             // No samples are disabled with 0xffffffff
	opaquePsoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
	opaquePsoDesc.NumRenderTargets = 1;              // Render Targets 
	opaquePsoDesc.RTVFormats[0] = mBackBufferFormat; // With Render targets
	opaquePsoDesc.SampleDesc.Count = m4xMsaaState ? msaaFactor : 1;
	opaquePsoDesc.SampleDesc.Quality = m4xMsaaState ? (m4xMsaaQuality - 1) : 0;
	opaquePsoDesc.DSVFormat = mDepthStencilFormat;
	ThrowIfFailed(md3dDevice->CreateGraphicsPipelineState(&opaquePsoDesc, IID_PPV_ARGS(&mPSOs["opaque"])));

	D3D12_GRAPHICS_PIPELINE_STATE_DESC texColorPsoDesc;

	//
	// PSO for texColor objects.
	//
	ZeroMemory(&texColorPsoDesc, sizeof(D3D12_GRAPHICS_PIPELINE_STATE_DESC));
	texColorPsoDesc.InputLayout = { mInputLayout.data(), (UINT)mInputLayout.size() };
	texColorPsoDesc.pRootSignature = mRootSignature.Get();
	texColorPsoDesc.VS =
	{
		reinterpret_cast<BYTE*>(mShaders["texColorVS"]->GetBufferPointer()),
		mShaders["texColorVS"]->GetBufferSize()
	};
	texColorPsoDesc.PS =
	{
		reinterpret_cast<BYTE*>(mShaders["texColorPS"]->GetBufferPointer()),
		mShaders["texColorPS"]->GetBufferSize()
	};
	//StreamOutput.???? https://msdn.microsoft.com/en-us/library/windows/desktop/bb205122(v=vs.85).aspx
	texColorPsoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
	texColorPsoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
	texColorPsoDesc.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC(D3D12_DEFAULT);
	texColorPsoDesc.SampleMask = UINT_MAX;             // No samples are disabled with 0xffffffff
	texColorPsoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
	texColorPsoDesc.NumRenderTargets = 1;              // Render Targets 
	texColorPsoDesc.RTVFormats[0] = mBackBufferFormat; // With Render targets
	texColorPsoDesc.SampleDesc.Count = m4xMsaaState ? msaaFactor : 1;
	texColorPsoDesc.SampleDesc.Quality = m4xMsaaState ? (m4xMsaaQuality - 1) : 0;
	texColorPsoDesc.DSVFormat = mDepthStencilFormat;
	ThrowIfFailed(md3dDevice->CreateGraphicsPipelineState(&texColorPsoDesc, IID_PPV_ARGS(&mPSOs["texColor"])));

	//--------------------------------------------------------------------------------------------------------------

	//
	// PSO for transparent objects
	//
	D3D12_GRAPHICS_PIPELINE_STATE_DESC transparentPsoDesc = opaquePsoDesc;
	D3D12_RENDER_TARGET_BLEND_DESC transparencyBlendDesc;

	transparencyBlendDesc.BlendEnable = true;
	transparencyBlendDesc.LogicOpEnable = false;
	transparencyBlendDesc.SrcBlend = D3D12_BLEND_SRC_ALPHA;
	transparencyBlendDesc.DestBlend = D3D12_BLEND_INV_SRC_ALPHA;
	transparencyBlendDesc.BlendOp = D3D12_BLEND_OP_ADD;
	transparencyBlendDesc.SrcBlendAlpha = D3D12_BLEND_ONE;
	transparencyBlendDesc.DestBlendAlpha = D3D12_BLEND_ZERO;
	transparencyBlendDesc.BlendOpAlpha = D3D12_BLEND_OP_ADD;
	transparencyBlendDesc.LogicOp = D3D12_LOGIC_OP_NOOP;
	transparencyBlendDesc.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;

	transparentPsoDesc.BlendState.RenderTarget[0] = transparencyBlendDesc;
	ThrowIfFailed(md3dDevice->CreateGraphicsPipelineState(&transparentPsoDesc, IID_PPV_ARGS(&mPSOs["transparent"])));
}

XMMATRIX RenderDX12::GetHMDMatrixProjectionEye(vr::Hmd_Eye nEye)
{
	if (!m_pOpenVRSystem)
		return XMMATRIX();

	vr::HmdMatrix44_t mat;
	mat = m_pOpenVRSystem->GetProjectionMatrix(nEye, 3.0f, CAMERA_ZFAR); // near far no apparent impact on paralax 

	return XMMATRIX(mat.m[0][0], mat.m[1][0], mat.m[2][0], mat.m[3][0],
		mat.m[0][1], mat.m[1][1], mat.m[2][1], mat.m[3][1],
		-mat.m[0][2], -mat.m[1][2], -mat.m[2][2], -mat.m[3][2],
		mat.m[0][3], mat.m[1][3], mat.m[2][3], mat.m[3][3]);
}

void RenderDX12::SetupCameras()
{
	mProjectionLeft = GetHMDMatrixProjectionEye(vr::Eye_Left);
	mProjectionRight = GetHMDMatrixProjectionEye(vr::Eye_Right);
}