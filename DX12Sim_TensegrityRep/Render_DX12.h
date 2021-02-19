#pragma once

#include "Common/d3dApp.h"
#include "Common/MathHelper.h"
#include "Common/UploadBuffer.h"
#include "Common/GeometryGenerator.h"
#include "Common/Camera.h"
#include "FrameResource.h"
#include <omp.h>

#include <iostream>  
#include <random>
#include <chrono>

#include "inc/ScreenGrab.h"
#include "D2d1.h"

#include "D3d12.h"
#include "D3d11on12.h"

//////////////////////////////////////////LEAP MOTION//////////////////////////////////////////
#include "Leap.h"
#include "LeapMath.h"
#include "HandData2.h" ///////////////////////////////Hands segments position data////////////
//#include "FXHlib.h" /////////////////////////////Fuji-Xerox Haptic library//////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

#define HAVE_STRUCT_TIMESPEC
#include "Common/pthread.h"

using Microsoft::WRL::ComPtr;
using namespace DirectX;
using namespace DirectX::PackedVector;

#pragma comment(lib, "d3dcompiler.lib")
#pragma comment(lib, "D3D12.lib")
#pragma comment(lib, "openvr_api.lib")

#define USE_ControllerThread 0
#define CONTROLERy -20
#define USEstatData 0 ///
#define VR_EXPLICIT_TIMING 1

struct D3D12TextureData_t
{
	ID3D12Resource *m_pResource;
	ID3D12CommandQueue *m_pCommandQueue;
	uint32_t m_nNodeMask;
};

struct timer_ms {
	double PCFreq;
	__int64 CounterStart;
};

struct quatLocXZ {
	XMFLOAT4 q;
	XMFLOAT2 pos;
	//XMFLOAT2 pad;
};

struct float4x4 {
	float m00;
	float m01;
	float m02;
	float m03;
	float m10;
	float m11;
	float m12;
	float m13;
	float m20;
	float m21;
	float m22;
	float m23;
	float m30;
	float m31;
	float m32;
	float m33;
};

//ClientFileHeader.h Ln41
struct statusVariablesPoint {
	int *running;
	bool *paused;
	bool *mAppPaused;
	int *minutes;
	double *seconds;
	int *screenXZ[2];
	XMFLOAT3 *camXYZ;
	int *useRegion;
	float *dataMon[2];
	unsigned int *inputLag[2]; //x Send, y Recv
	float4x4 *controllerMats[2];
	XMFLOAT3 *controllerLoc[2];
};

struct VIVEController {
	//Controller - Tracking, Input, Render, Interaction
	bool VRTriggerPressed[2] = { 0, 0 };		/// True when pressed - <recorded in Render_DX or VIVEController - used in ObjectUpdate> DXSim
	bool VRTriggerSearching[2] = { 0, 0 };		/// Set to false after picking algorithm - <recorded in Render_DX or VIVEController - used in ObjectUpdate> DXSim
	XMMATRIX controllerMatrixSave[2];			/// Special Format for TubeWand - <recorded in Render_DX or VIVEController - used in ObjectUpdate> DXSim
	float4x4 controllerMats16[2];				/// Special Format for TubeWand - <recorded in Render_DX or VIVEController> CUDA
	XMFLOAT3 lastControllerLoc[2] = { XMFLOAT3(0.0f, 0.0f, 0.0f), XMFLOAT3(0.0f, 0.0f, 0.0f) }; /// Used to compute Delta - <recorded in Render_DX or VIVEController> DXSim/CUDA
	XMFLOAT3 controllerDeltaLoc[2] = { XMFLOAT3(0.0f, 0.0f, 0.0f), XMFLOAT3(0.0f, 0.0f, 0.0f) };/// Controller Delta Loc - <no longer used>
	XMFLOAT3 lastGrabLoc[2] = { XMFLOAT3(0.0f, 0.0f, 0.0f), XMFLOAT3(0.0f, 0.0f, 0.0f) };		/// Offset Controller loc for the grab point of the controller - <recorded in Render_DX or VIVEController - used in ObjectUpdate> DXSim
	XMFLOAT3 grabDeltaLoc[2] = { XMFLOAT3(0.0f, 0.0f, 0.0f), XMFLOAT3(0.0f, 0.0f, 0.0f) };		/// Controller Delta Grab Loc - <recorded in Render_DX or VIVEController - used in ObjectUpdate> DXSim
	XMFLOAT3 controllerLook[2] = { XMFLOAT3(0.0f, 0.0f, 0.0f), XMFLOAT3(0.0f, 0.0f, 0.0f) };	/// Vector where the controller is pointing - <recorded in Render_DX or VIVEController - used in ObjectUpdate> DXSim
	float grabForOffset = 30.0f;

	float VIVEtoWorldScaling = 60.0f;
	XMFLOAT3 VR_MoveOffset = XMFLOAT3(0.0f, 0.0f, 0.0f);
};

struct InteractionParameters {
	float INTER_E; //Increasing increases distance (equilibrium for LJP)
	float INTER_S;
	float FORCE_RANGE;
	unsigned int NEG_FORCES_ON;
	float NEG_F_MULTIPLY;
	unsigned int GRAVITY_ON = 0;
	unsigned int FORCECOLOR_ON = 0;
	unsigned int BOUNDED = 0;

	float FPS = 90.0f; //Assuming frame per second.

	//Drag / Dampening { x1 & x2 can be unstable if v magnitute > 1 }
	unsigned int dragType = 1;
	float x0Drag = 0.0001f;		//Costant   Drag: v = -x0Drag + v
	float x1Drag = 0.1f;		//Linear    Drag: v = x1Drag*v //xxxxxxxxxxxxxxxxxxxxx
	float x2Drag = 0.025f;		//Quadratic Drag: v = x2Drag*v*v
	float x3Drag = 0.100f;		//Cubic     Drag: v = x3Drag*v*v*v
	unsigned int dampenType = 0;
	float x0Dampen = 0.0001f;	//Costant   Drag: v = -x0Drag + v
	float x1Dampen = 0.005f;	//Linear    Dampening: v = x1Dampen*v
	float x2Dampen = 0.025f;	//Quadratic Dampening: v = x2Dampen*v*v
	float x3Dampen = 0.100f;	//Cubic     Dampening: v = x3Dampen*v*v*v
};

//-----------------------------------------------------------------------------------------------------------
///////////////////////////////////////////LEAP MOTION//////////////////////////////////////////
//-----------------------------------------------------------------------------------------------------------
XMMATRIX leapToDXTransform(Leap::Vector pos, Leap::Matrix basMatx, float posScale, XMMATRIX cam);
XMMATRIX leapToDX_HMD_Transform(Leap::Vector pos, Leap::Matrix basMatx, float posScale, XMMATRIX cam);

/*struct HapticPars {	//This HapticPard is for storing the haptic parameters.
	int freq[12];
	int amp[12];
	int press[12];
};*/

class LeapSegments {
private:
	float* x;
	float* y;
	float* z;
	float* radius;
	int numSeg;
	HandData handData; //////////////////////////////////////////HandData object/////////////////////
	int numHands = 0;

public:
	LeapSegments(int ns);
	~LeapSegments();

	void setX(int idx, float value);	//To set the x value.
	void setY(int idx, float value);	//To set the y value.
	void setZ(int idx, float value);	//To set the z value.
	float getX(int idx);				//To get the x value.
	float getY(int idx);				//To get the y value.
	float getZ(int idx);				//To get the z value.
	float getRad(int idx);				//To get the radius value.
	int getNumSeg();					//To get the the number of segments used for virtual hands.
	void setNumHands(int n);
	int getNumHands();
	void segmentsPlacing(bool isLeft, Leap::Finger::Type finger, Leap::Bone::Type bone, XMMATRIX pos);		//To arrange segments to form virtual hands.
	void segments_HMD_Placing(bool isLeft, Leap::Finger::Type finger, Leap::Bone::Type bone, XMMATRIX pos);	//To arrange segments to form virtual hands in VR mode.
};
//-----------------------------------------------------------------------------------------------------------

// Lightweight structure stores parameters to draw a shape.  This will
// vary from app-to-app.
struct RenderItem
{
	RenderItem() = default;
	RenderItem(const RenderItem& rhs) = delete;

	// World matrix of the shape that describes the object's local space
	// relative to the world space, which defines the position, orientation,
	// and scale of the object in the world.
	XMFLOAT4X4 World = MathHelper::Identity4x4();

	XMFLOAT4X4 TexTransform = MathHelper::Identity4x4();

	// Dirty flag indicating the object data has changed and we need to update the constant buffer.
	// Because we have an object cbuffer for each FrameResource, we have to apply the
	// update to each FrameResource.  Thus, when we modify obect data we should set 
	// NumFramesDirty = gNumFrameResources so that each frame resource gets the update.
	int NumFramesDirty = gNumFrameResources;

	// Index into GPU constant buffer corresponding to the ObjectCB for this render item.
	UINT ObjCBIndex = -1;

	Material* Mat = nullptr;
	MeshGeometry* Geo = nullptr;

	// Primitive topology.
	D3D12_PRIMITIVE_TOPOLOGY PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;

	BoundingBox Bounds;
	std::vector<InstanceData> Instances;

	// DrawIndexedInstanced parameters.
	UINT IndexCount = 0;
	UINT InstanceCount = 0; //Set in RenderDX12::UpdateInstanceData used to render a subset
	UINT StartIndexLocation = 0;
	int BaseVertexLocation = 0;
};

class RenderDX12 : public D3DApp
{
public:
	RenderDX12(HINSTANCE hInstance);
	RenderDX12(const RenderDX12& rhs) = delete;
	RenderDX12& operator=(const RenderDX12& rhs) = delete;
	~RenderDX12();
	void saferDelete(void * dataToDelete);
	void StartCounter_ms(timer_ms &t);
	double GetCounter_ms(timer_ms &t);

	virtual bool Initialize()override;

	// --------Compute Shaders-----------
	ComPtr<ID3D12RootSignature> mRootSignatureCS = nullptr;
	ComPtr<ID3D12RootSignature> mRootSignatureMoveCS = nullptr;
	ComPtr<ID3D12RootSignature> mRootSignatureSpringCS = nullptr;
	ComPtr<ID3D12RootSignature> mRootSignatureAnch2SegCS = nullptr;
	ComPtr<ID3D12RootSignature> mRootSignatureDataCS = nullptr;
	ComPtr<ID3D12RootSignature> mRootSignatureDataCSLocal = nullptr;
	void InitGPUMemOut(ComPtr<ID3D12Resource> * memory, UINT64 byteSize, std::wstring name);
	void InitGPUMemIn(ComPtr<ID3D12Resource> * memory, UINT64 byteSize, std::wstring name);
	void InitGPUMemInOut(ComPtr<ID3D12Resource> * memory, UINT64 byteSize, std::wstring name);
	void InitGPUMemBuffered(ComPtr<ID3D12Resource> * memory, UINT64 byteSize, std::wstring name);
	void BarriorCopyFromGPU(ComPtr<ID3D12GraphicsCommandList> &commandList, ComPtr<ID3D12Resource> * memory);
	void BarriorCopyToGPU(ComPtr<ID3D12GraphicsCommandList> &commandList, ComPtr<ID3D12Resource> * memory);
	void BuildRootSignatureCS();
	void BuildShadersAndInputLayoutCS();
	void BuildPSO_CS();
	//void RecordComputeWork();
	UINT getInstIdx(const char * name);
	void packi32b(unsigned char *buf, unsigned long int i);
	XMMATRIX quatToMatrix_r(XMFLOAT4 q);
	XMFLOAT4X4 quatToMatrix_r4(XMFLOAT4 q);
	void make16mat(float * out, XMMATRIX in);
	void make16mat(float4x4 *out, XMMATRIX in);
	void copy16mat(float4x4 *out, float4x4 in);

private:
	virtual void OnResize()override;
	virtual void Update(const GameTimer& gt)override;
	virtual void Draw(const GameTimer& gt)override;
	virtual void DrawStereo(const GameTimer& gt)override;
	void DrawUpscale(const GameTimer& gt);

	virtual void OnMouseDown(WPARAM btnState, int x, int y)override;
	virtual void OnMouseUp(WPARAM btnState, int x, int y)override;
	virtual void OnMouseMove(WPARAM btnState, int x, int y)override;

	void DirectionalMovement(const GameTimer& gt);
	void OnKeyboardInput(const GameTimer& gt);
	void AnimateMaterials(const GameTimer& gt);
	void LoadNetworkDataRegion();
	void LoadNetworkDataAll();
	UINT UpdateInstanceData(const GameTimer& gt);
	void UpdateMotorColors();
	void UpdateMaterialBuffer(const GameTimer& gt);
	void UpdateMainPassCB(const GameTimer& gt);
	void UpdateMainPassCBStereo(const GameTimer& gt);

	bool InitNetwork();
	bool InitControllerThread();
	bool CloseControllerThread();
	void LoadTextures();
	void BuildRootSignature();
	void BuildDescriptorHeaps();
	void BuildShadersAndInputLayout();
	void BuildSkullGeometry();
	void BuildShapeGeometry();			void BuildShapeGeometry2();
	void BuildShapeGeometryDynamic();
	void BuildPSOs();
	void BuildFrameResources();
	void BuildMaterials();
	void BuildRenderItems();
	void DrawRenderItems(ID3D12GraphicsCommandList* cmdList, const std::vector<RenderItem*>& ritems);

	std::array<const CD3DX12_STATIC_SAMPLER_DESC, 6> GetStaticSamplers();

	bool saveImage = 0; /////////////////////////////////////////////////////////////////////////////////////////////
	bool imageCounter = 0;
	int lastCapturedSec = 0;
	int CAPTURE_INTERVAL_SEC = 10;
	void PrintScreenImage(int offset);
	void RenderUI();

	// --------Open VR-------------------
	void SubmitToHMD();
	void UpdateEyeTwo(int eye);
	void UpdateController(const GameTimer& gt);
	void ControllerButtonPress(vr::VRControllerState_t *state, RenderItem * contRI, const GameTimer& gt, int controllerID);
	void TakeControllerInputTwo(const GameTimer& gt, int controllerID);
	bool TryActivateVR();
	bool TryActivateOpenVR();
	void DeactivateVR();
	bool IsVRActive() const { return m_pOpenVRSystem; }
	void TubeWand(int c);
	void TubeWandRobo(int c, const GameTimer& gt);
	void RenderDX12::TubeWandRoboLagTest(int c, const GameTimer& gt);
	void TubeWandOverride(XMMATRIX controllerMatrix, XMFLOAT3 locationAcc);
	void TubeWandOverrideDebug(XMMATRIX *controllerMatrix, XMFLOAT3 *locationAcc, int cID, int start);
	void TubeWandOverride(XMMATRIX *controllerMatrix, XMFLOAT3 *locationAcc, int cID, int start);

	XMMATRIX GetHMDMatrixProjectionEye(vr::Hmd_Eye nEye);
	void SetupCameras();

	//-----------------------------------------------------------------------------------------------------------
	//////////////////////////////////////////LEAP MOTION//////////////////////////////////////////
	//-----------------------------------------------------------------------------------------------------------
	Leap::Controller leapController;
	Leap::Frame leapFrame;
	Leap::HandList leapHandList;
	float leapVRscale = 5.0f;			//The scale of the virtual hands.
	void updateLeapMotionHands();		//For updating hand segments positions each frame.
	void updateLeapMotionHandsVR();		//For updating hand segments positions each frame in VR mode.
	//-----------------------------------------------------------------------------------------------------------

public:

	//-----------------------------------------------------------------------------------------------------------
	//////////////////////////////////////////LEAP MOTION//////////////////////////////////////////
	//-----------------------------------------------------------------------------------------------------------
	std::string testString = "---";	//For display text in the simulation panel.
	HandData hd1;					//Model for creating virtual hands.
	UINT LEAP_SEG_CNT;				//Tne number of segments used for creating virtual hands.
	LeapSegments* LS;				//Leap Motion segments manager.
	//-----------------------------------------------------------------------------------------------------------

	std::vector<std::unique_ptr<FrameResource>> mFrameResources;
	FrameResource* mCurrFrameResource = nullptr;
	int mCurrFrameResourceIndex = 0;

private:

	UINT mCbvSrvDescriptorSize = 0;

	ComPtr<ID3D12RootSignature> mRootSignature = nullptr;

	ComPtr<ID3D12DescriptorHeap> mSrvDescriptorHeap = nullptr;

	std::unordered_map<std::string, std::unique_ptr<MeshGeometry>> mGeometries;
	std::unordered_map<std::string, std::unique_ptr<Material>> mMaterials;
	std::unordered_map<std::string, std::unique_ptr<Texture>> mTextures;
	std::unordered_map<std::string, ComPtr<ID3DBlob>> mShaders;

	std::vector<D3D12_INPUT_ELEMENT_DESC> mInputLayout;

	// List of all the render items.
	std::vector<std::unique_ptr<RenderItem>> mAllRitems;

	UINT mVertexCountDynamic = 0;

	RenderItem* mDynamicRitem = nullptr;
	std::vector<Vertex> mDynamicVBCPU;

	bool mFrustumCullingEnabled = true;

	BoundingFrustum mCamFrustum;

	PassConstants mMainPassCBL;
	PassConstants mMainPassCBR;

	POINT mLastMousePos;

public:

	Camera mCamera;
	float cameraTeta = 0;
	std::unordered_map<std::string, ComPtr<ID3D12PipelineState>> mPSOs;
	InstanceItemCounts instancingCounts;

	// Render items divided by PSO.
	std::vector<RenderItem*> mOpaqueRitems;

	float increment;
	int frameCount;
	timer_ms timer_a;
	timer_ms timerAnimate;
	timer_ms timerSimLag;
	timer_ms timerDraw;
	timer_ms timerVRSync;
	
	double timeCumulativeInst = 0;
	double timeCumulativeAnima = 0;
	double timeCumulativeText = 0;
	double timeCumulativePick = 0;

	double timeCumulativeSort = 0;
	double timeCumulativeLJP = 0;
	double timeCumulativeMove = 0;
	double timeCumulativeDraw = 0;
	double timeCumulativeVRSync = 0;

	double timeModInst = 0;
	double timeModAnima = 0;
	double timeModText = 0;
	double timeModPick = 0;

	double timeModSort = 0;
	double timeModLJP = 0;
	double timeModMove = 0;
	double timeModDraw = 0;
	double timeModVRSync = 0;

	double timerDXRender = 0;
	UINT64 accumDXRender = 0;
	double timerDXCSsim = 0;
	UINT64 accumDXCSsim = 0;
	double timerDXCScpy = 0;
	UINT64 accumDXCScpy = 0;

	UINT inputLagCount = 0;
	double timeCumulativeIpLag = 0;
	double timeModIpLag = 0;

	const int gNumFrameResources = 3;
	bool transparentPS = 0;
	float CAMERA_ZFAR = 9000.0f; ///////////////////////////////////////////////////////////
	UINT sphereRenderNumber = -1; // Delete carefully
	std::string inputLines[32];
	float averageSegmentsFoundLJP = 0;
	float averageSegmentsComputedLJP = 0;
	float totalEnergy = 0.0f;

	InteractionParameters param;

	//Input file related values
	unsigned int SEGMENTCOUNT;
	unsigned int SEG_RenderCnt;
	UINT renderCntMax = 1000000 * 27; // 1mill * 20
	UINT inst1Max = 0xFFFFFFFF;
	UINT sphereRenderCnt1, sphereRenderCnt2;

	//Sim space
	float spaceDimX_sg;
	float spaceDimZ_sg;
	float spaceDimY_sg;
	float spaceDimXD2_sg;
	float spaceDimZD2_sg;
	float spaceDimYD2_sg;

	int coarseGraining = 10;
	float geoGridx = 10;	// Should never use defult values
	float geoGridz = 10;	//-------------------------------
	int gridDivisorx = 1;	//-------------------------------
	int gridDivisorz = 1;	//-------------------------------
	float sphereRadius = 1.0f;
	bool flexiGrid = 0;
	const float size1D_fg = 400.0f;
	const int vertDivisor_fg = 2;

	//Demo control variables
	bool PAUSED = true;
	bool NPRESS = false;
	int time_simulatedMinutes = 0;
	double time_simulatedSeconds = 0.0;
	float dataMontitor[2];
	bool simLagStart = true;
	XMUINT2 simInputLag = XMUINT2(0,0);
	bool holdWall = false;
	bool sendWall = false;
	bool wallLeft = true;
	float tubeRadius = 7;///--------WAND
	UINT tubeLength = 18;///--------WAND
	std::string buildString = "";

	bool isFolded = false;				///////////The folding of the anchored spring object.
	bool isFoldChanged = false;			///////////The folding status is changed.
	bool isForceApplied = false;		///////////The start status for applying force to the object.
	bool isRecording = false;			///////////The status for start recording the data to an output file.
	bool isForceChanged = false;		///////////The status which indicates whether the applying force is changed.
	
	XMFLOAT2 INC_TEMP = { 0, 1 };

	//-----------VR-------------------------------------------------------
	int holdingIdx[2] = { -1, -1 };
	
	float holdingStrength = 0.0f;// broken
	XMFLOAT3 pullForward = XMFLOAT3(0.0f, 0.0f, 0.0f);
	bool MouseRightPressed = false;		//True when pressed
	bool MouseRightSearching = false;	//Set to false after picking algorithm
	XMFLOAT3 lastMouseLoc = XMFLOAT3(0.0f, 0.0f, 0.0f);
	XMFLOAT3 mouseDeltaLoc = XMFLOAT3(0.0f, 0.0f, 0.0f);
	XMFLOAT2 lastMousePos = XMFLOAT2(0.0f, 0.0f);

	//Controller
	VIVEController vc;
	//See struct

	//HMD
	static const int hMemIterCnt = 5;
	XMFLOAT3 lastHeadLoc = XMFLOAT3(0.0f, 0.0f, 0.0f);
	XMFLOAT3 headDeltaLoc = XMFLOAT3(0.0f, 0.0f, 0.0f);
	XMFLOAT3 hLocSave[hMemIterCnt] = { XMFLOAT3(0.0f, 0.0f, 0.0f) };
	int hMemIter = 0; 

	pthread_t controllerThread;
	void * cInputs[9];
	void *returnValCT;
	
	XMMATRIX mProjectionLeft;
	XMMATRIX mProjectionRight;
	UINT64 mDirectCommandQueueTimestampFrequencies;
	FILE * clientPrint = NULL;

	//----------------COMPUTE RELATED------------------------------------
	bool useLists = 1; // Dont set to false hash is commented out
	bool ComputeShaderActive = 1;
	const static unsigned int heapCount = 3;
	ComPtr<ID3D12Resource> OR_originalIndex[heapCount] = { nullptr };		// Sort::RW - Compute::R
	ComPtr<ID3D12Resource> OR_segLocSorted_x[heapCount] = { nullptr };		// Sort::RW - Compute::R
	ComPtr<ID3D12Resource> OR_segLocSorted_y[heapCount] = { nullptr };		// ^^^^^^^^^
	ComPtr<ID3D12Resource> OR_segLocSorted_z[heapCount] = { nullptr };		// ^^^^
	ComPtr<ID3D12Resource> OR_cellStart[heapCount] = { nullptr };			// Sort::RW - Compute::R
	ComPtr<ID3D12Resource> OR_cellEnd[heapCount] = { nullptr };				// Sort::RW - Compute::R

	ComPtr<ID3D12Resource> OR_forwardVec_x[heapCount] = { nullptr };	// Compute::RW
	ComPtr<ID3D12Resource> OR_forwardVec_y[heapCount] = { nullptr };	// ^^^^^^^^^
	ComPtr<ID3D12Resource> OR_forwardVec_z[heapCount] = { nullptr };	// ^^^^
	ComPtr<ID3D12Resource> OR_colors_x[heapCount] = { nullptr };			// Compute::RW
	ComPtr<ID3D12Resource> OR_colors_y[heapCount] = { nullptr };			// ^^^^^^^^^
	ComPtr<ID3D12Resource> OR_colors_z[heapCount] = { nullptr };			// ^^^^

	ComPtr<ID3D12Resource> OR_segRadiSort[heapCount] = { nullptr };			// Addition
	
	ComPtr<ID3D12Resource> OR_segLocID_x[heapCount] = { nullptr };		// Sort::RW - Compute::R
	ComPtr<ID3D12Resource> OR_segLocID_y[heapCount] = { nullptr };		// ^^^^^^^^^ ID = Instance Data
	ComPtr<ID3D12Resource> OR_segLocID_z[heapCount] = { nullptr };		// ^^^^
	ComPtr<ID3D12Resource> OR_colorsID_x[heapCount] = { nullptr };			// Compute::RW
	ComPtr<ID3D12Resource> OR_colorsID_y[heapCount] = { nullptr };			// ^^^^^^^^^
	ComPtr<ID3D12Resource> OR_colorsID_z[heapCount] = { nullptr };			// ^^^^

	ComPtr<ID3D12Resource> OR_segLocM_x[heapCount] = { nullptr };		// Sort::RW - Compute::R
	ComPtr<ID3D12Resource> OR_segLocM_y[heapCount] = { nullptr };		// ^^^^^^^^^ delete and use ID?
	ComPtr<ID3D12Resource> OR_segLocM_z[heapCount] = { nullptr };		// ^^^^

	ComPtr<ID3D12Resource> OR_springDistance[heapCount] = { nullptr };	//
	ComPtr<ID3D12Resource> OR_massID[heapCount] = { nullptr };			//
	ComPtr<ID3D12Resource> OR_springConstant[heapCount] = { nullptr };	//

	ComPtr<ID3D12Resource> OR_anchor_x[heapCount] = { nullptr };	//
	ComPtr<ID3D12Resource> OR_anchor_y[heapCount] = { nullptr };	//
	ComPtr<ID3D12Resource> OR_anchor_z[heapCount] = { nullptr };	//
	ComPtr<ID3D12Resource> OR_numConSeg[heapCount] = { nullptr };	//
	ComPtr<ID3D12Resource> OR_anch2SegmentID[heapCount] = { nullptr };	//
	ComPtr<ID3D12Resource> OR_anch2SegDistance[heapCount] = { nullptr };	//
	ComPtr<ID3D12Resource> OR_anch2SegConst[heapCount] = { nullptr };	//
	ComPtr<ID3D12Resource> OR_anchor_2SegStart[heapCount] = { nullptr };	//
	ComPtr<ID3D12Resource> OR_anchor_veloX[heapCount] = { nullptr };	//
	ComPtr<ID3D12Resource> OR_anchor_veloY[heapCount] = { nullptr };	//
	ComPtr<ID3D12Resource> OR_anchor_veloZ[heapCount] = { nullptr };	//

	//-------------NETWORK RELATED---------------------------

	pthread_t netStatusThread, netDataThread;
	void *returnValST; void *returnValDT;

	bool useNetworkCompute = 0; // MAIN on off
	int NetworkLive = 1;
	unsigned int MT_COUNT = 0;
	//unsigned int MT_SEGCOUNT = 0;
	int USE_REGION = 0; // Sim Input File Controlls this
	int end_CurrentTest = 0;
	statusVariablesPoint varsPoint;
	unsigned int MOTOR_COUNT = 0;
	unsigned int MOTOR_RenderCnt = 0;
	int * MTCountsMain = NULL;

	XMFLOAT4 * host_segLocColor = NULL;		// Render Region
	XMFLOAT3 * host_segLoc = NULL;			// Render All
	float * host_segColor = NULL;			// Render All

	quatLocXZ* host_motorQuatLocXZ = NULL;	// Render Region
	XMFLOAT3 * host_motorLocations = NULL;	// Render All
	XMFLOAT4 * host_motorQuat = NULL;		// Render All

	bool useCSInstanceCopy = 0;
	UINT currentInstanceBuffer = 0; // (must be 2 or 3)
	ComPtr<ID3D12Resource> OR_segLocColor[heapCount] = { nullptr };
	ComPtr<ID3D12Resource> OR_segLoc[heapCount] = { nullptr };
	ComPtr<ID3D12Resource> OR_segColor[heapCount] = { nullptr };

	ComPtr<ID3D12Resource> OR_motorQuatLocXZ[heapCount] = { nullptr };
	ComPtr<ID3D12Resource> OR_motorLocations[heapCount] = { nullptr };
	ComPtr<ID3D12Resource> OR_motorQuat[heapCount] = { nullptr };

	ComPtr<ID3D12Resource> OR_InstancedData[2][heapCount] = { nullptr };
};

// 5.30.2017
//-----------
// At 90 FPS
//    VR 400k 
// nonVR 400K
//-----------
// At 30 FPS
// nonVR 1.1M
// blend 850k

// 40,000 seg 400 area 6-8 cube bounded