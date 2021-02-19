
#pragma once

#if defined(DEBUG) || defined(_DEBUG)
#include "D3d12sdklayers.h"
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#endif

#include "d3dUtil.h"
#include "GameTimer.h"
#include "D3d11on12.h"
#include "D2d1_1.h"
#include "D2d1_3.h"
#include "Dwrite.h"
#include "Objbase.h"

#include "pix3.h"
#include "openvr.h"

#define HAVE_STRUCT_TIMESPEC
#include "pthread.h"

////////////////////////////////////////////////////////////////////////////////////////////////
#include "FXHlib.h" /////////////////////////////Fuji-Xerox Haptic library//////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

// Link necessary d3d12 libraries.
#pragma comment(lib,"d3dcompiler.lib")
#pragma comment(lib, "D3D12.lib")
#pragma comment(lib, "dxgi.lib")

//////////////////////////////////FOR HAPTIC///////////////////////////////////////////////////
struct HapticPars {
	int freq[12];
	int amp[12];
	int press[12];
	FXH::FXHdevice fxHapDevice; /////////////////////Fuji-Xerox haptic device handler///////////
};

void* FXhapticThread(void* hp);
///////////////////////////////////////////////////////////////////////////////////////////////

class D3DApp
{
protected:

    D3DApp(HINSTANCE hInstance);
    D3DApp(const D3DApp& rhs) = delete;
    D3DApp& operator=(const D3DApp& rhs) = delete;
    virtual ~D3DApp();

public:

    static D3DApp* GetApp();
    
	HINSTANCE AppInst()const;
	HWND      MainWnd()const;
	float     AspectRatio()const;

    bool Get4xMsaaState()const;
    void Set4xMsaaState(bool value);

	//////////////////////////////////FOR HAPTIC///////////////////////////////////////////////////
	HapticPars hapticResult;
	///////////////////////////////////////////////////////////////////////////////////////////////
	
	int Run();
 
    virtual bool Initialize();
    virtual LRESULT MsgProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);

	// VR resources
	bool								vrActivate = false;
	vr::IVRSystem*						m_pOpenVRSystem = nullptr;
	vr::IVRCompositor*					m_pOpenVRCompositor;
	vr::TrackedDevicePose_t				m_poseOpenVR[vr::k_unMaxTrackedDeviceCount];
	bool								m_rbShowTrackedDevice[16];
	float								VRoffsetFlat = 0.033; /// was 0.026 --------------------------------------------------------XXXXXXXXXXXX
	float								VRoffsetStereo = 1.2; /// was 0.026 --------------------------------------------------------XXXXXXXXXXXX

protected:
    virtual void CreateRtvAndDsvDescriptorHeaps();
	virtual void OnResize(); 
	virtual void Update(const GameTimer& gt)=0;
    virtual void Draw(const GameTimer& gt)=0;
	virtual void DrawStereo(const GameTimer& gt)=0;

	// Convenience overrides for handling mouse input.
	virtual void OnMouseDown(WPARAM btnState, int x, int y){ }
	virtual void OnMouseUp(WPARAM btnState, int x, int y)  { }
	virtual void OnMouseMove(WPARAM btnState, int x, int y){ }

protected:

	bool InitMainWindow();
	void GetHardwareAdapter(IDXGIFactory4* pFactory, IDXGIAdapter1** ppAdapter);
	bool InitDirect3D();
	bool InitD3D11();
	void CreateCommandObjects();
    void CreateSwapChain();

	void FlushCommandQueue();
	void FlushTextData();

	ID3D12Resource* CurrentBackBuffer()const;
	D3D12_CPU_DESCRIPTOR_HANDLE CurrentBackBufferView()const;
	D3D12_CPU_DESCRIPTOR_HANDLE DepthStencilView()const;

	void CalculateFrameStats();

    void LogAdapters();
    void LogAdapterOutputs(IDXGIAdapter* adapter);
    void LogOutputDisplayModes(IDXGIOutput* output, DXGI_FORMAT format);

protected:

    static D3DApp* mApp;

    HINSTANCE mhAppInst = nullptr; // application instance handle
    HWND      mhMainWnd = nullptr; // main window handle
	bool      mAppPaused = false;  // is the application paused?
	bool      mMinimized = false;  // is the application minimized?
	bool      mMaximized = false;  // is the application maximized?
	bool      mResizing = false;   // are the resize bars being dragged?
    bool      mFullscreenState = false;// fullscreen enabled

	// Used to keep track of the “delta-time” and game time (§4.4).
	GameTimer mTimer;
	
    Microsoft::WRL::ComPtr<IDXGIFactory4> mdxgiFactory;
    Microsoft::WRL::ComPtr<IDXGISwapChain> mSwapChain;
    Microsoft::WRL::ComPtr<ID3D12Device> md3dDevice;

    Microsoft::WRL::ComPtr<ID3D12Fence> mFence;
    UINT64 mCurrentFence = 0;
	
    Microsoft::WRL::ComPtr<ID3D12CommandQueue> mCommandQueue;
    Microsoft::WRL::ComPtr<ID3D12CommandAllocator> mDirectCmdListAlloc;
    Microsoft::WRL::ComPtr<ID3D12GraphicsCommandList> mCommandList;

	static const int SwapChainBufferCount = 2;
	int mCurrBackBuffer = 0;
    Microsoft::WRL::ComPtr<ID3D12Resource> mSwapChainBuffer[SwapChainBufferCount];
    Microsoft::WRL::ComPtr<ID3D12Resource> mDepthStencilBuffer;

    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> mRtvHeap;
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> mDsvHeap;

    D3D12_VIEWPORT mScreenViewport; 
    D3D12_RECT mScissorRect;
	D3D12_VIEWPORT m_viewportA[2];
	D3D12_RECT m_scissorRectA[2];

	UINT mRtvDescriptorSize = 0;
	UINT mDsvDescriptorSize = 0;
	UINT mCbvSrvUavDescriptorSize = 0;

	// Derived class should set these in derived constructor to customize starting values.
	std::wstring mMainWndCaption = L"Gutmann";
	D3D_DRIVER_TYPE md3dDriverType = D3D_DRIVER_TYPE_HARDWARE;
    DXGI_FORMAT mBackBufferFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
	DXGI_FORMAT mDepthStencilFormat = DXGI_FORMAT_D24_UNORM_S8_UINT;
	D3D_FEATURE_LEVEL mFeatureLevel = D3D_FEATURE_LEVEL_12_0;
	int		mClientWidth = 1920;
	int		mClientHeight = 1080;
	//int		mClientWidth = 1890;
	//int		mClientHeight = 1680;
	bool	mTextToScreen = 0; //////////////////////////////////////////////////////////////////////////////////////////////////
	float screenFontSize = 20;
	int STEREO_RENDER = 0;

	Microsoft::WRL::ComPtr<ID3D11DeviceContext> m_d3d11DeviceContext;
	Microsoft::WRL::ComPtr<ID3D11On12Device> m_d3d11On12Device;
	Microsoft::WRL::ComPtr<IDWriteFactory> m_dWriteFactory;
	Microsoft::WRL::ComPtr<ID2D1Factory3> m_d2dFactory;
	Microsoft::WRL::ComPtr<ID2D1Device2> m_d2dDevice;
	Microsoft::WRL::ComPtr<ID2D1DeviceContext2> m_d2dDeviceContext;
	Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> m_rtvHeap;
	Microsoft::WRL::ComPtr<ID3D11Resource> m_wrappedBackBuffers[SwapChainBufferCount];
	Microsoft::WRL::ComPtr<ID2D1Bitmap1> m_d2dRenderTargets[SwapChainBufferCount];
	Microsoft::WRL::ComPtr<ID3D12CommandAllocator> m_commandAllocators[SwapChainBufferCount];
	Microsoft::WRL::ComPtr<ID2D1SolidColorBrush> m_textBrush;
	Microsoft::WRL::ComPtr<IDWriteTextFormat> m_textFormat;
	UINT m_rtvDescriptorSize;

	// Set true to use #X MSAA (§4.1.8).  The default is false.
	bool		m4xMsaaState = 1;		// #X MSAA enabled
	UINT		m4xMsaaQuality = 0;		// quality level of #X MSAA
	UINT		msaaFactor = 4;			// MSAA sample count
	Microsoft::WRL::ComPtr<ID3D12Resource> msaaResource = NULL;			// MSAA
	Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> mRtvMSAAHeap = NULL;	// MSAA
	
};

// Assign a name to the object to aid with debugging.
#if defined(_DEBUG)
inline void SetName(ID3D12Object* pObject, LPCWSTR name)
{
	pObject->SetName(name);
}
inline void SetNameIndexed(ID3D12Object* pObject, LPCWSTR name, UINT index)
{
	WCHAR fullName[50];
	if (swprintf_s(fullName, L"%s[%u]", name, index) > 0)
	{
		pObject->SetName(fullName);
	}
}
#else
inline void SetName(ID3D12Object*, LPCWSTR)
{
}
inline void SetNameIndexed(ID3D12Object*, LPCWSTR, UINT)
{
}
#endif

// Naming helper for ComPtr<T>.
// Assigns the name of the variable as the name of the object.
// The indexed variant will include the index in the name of the object.
#define NAME_D3D12_OBJECT(x) SetName(x.Get(), L#x)
#define NAME_D3D12_OBJECT_INDEXED(x, n) SetNameIndexed(x[n].Get(), L#x, n)
