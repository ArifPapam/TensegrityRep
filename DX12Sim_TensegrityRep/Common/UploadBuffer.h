#pragma once

#include "d3dUtil.h"

template<typename T>
class UploadBuffer
{
public:
    UploadBuffer(ID3D12Device* device, UINT elementCount, bool isConstantBuffer) : 
        mIsConstantBuffer(isConstantBuffer)
    {
        mElementByteSize = sizeof(T);

        // Constant buffer elements need to be multiples of 256 bytes.
        // This is because the hardware can only view constant data 
        // at m*256 byte offsets and of n*256 byte lengths. 
        // typedef struct D3D12_CONSTANT_BUFFER_VIEW_DESC {
        // UINT64 OffsetInBytes; // multiple of 256
        // UINT   SizeInBytes;   // multiple of 256
        // } D3D12_CONSTANT_BUFFER_VIEW_DESC;
        if(isConstantBuffer)
            mElementByteSize = d3dUtil::CalcConstantBufferByteSize(sizeof(T));

		ThrowIfFailed(device->CreateCommittedResource(
				&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
				D3D12_HEAP_FLAG_NONE,
				&CD3DX12_RESOURCE_DESC::Buffer((UINT64)mElementByteSize*elementCount),
				D3D12_RESOURCE_STATE_GENERIC_READ,
				nullptr,
				IID_PPV_ARGS(&mUploadBuffer)));

        ThrowIfFailed(mUploadBuffer->Map(0, nullptr, reinterpret_cast<void**>(&mMappedData)));

        // We do not need to unmap until we are done with the resource.  However, we must not write to
        // the resource while it is in use by the GPU (so we must use synchronization techniques).
    }

    UploadBuffer(const UploadBuffer& rhs) = delete;
    UploadBuffer& operator=(const UploadBuffer& rhs) = delete;
    ~UploadBuffer()
    {
        if(mUploadBuffer != nullptr)
            mUploadBuffer->Unmap(0, nullptr);

        mMappedData = nullptr;
    }

    ID3D12Resource* Resource()const
    {
        return mUploadBuffer.Get();
    }

    inline void CopyData(int elementIndex, const T& data)
    {
        memcpy(&mMappedData[(UINT64)elementIndex*mElementByteSize], &data, sizeof(T));
    }

	void CopyAllData(int size, const T * data)
	{
		memcpy(mMappedData, data, sizeof(T)*size);
	}

	inline void streamToBuffer(int elementIndex, void* src)
	{
		for (int k = 0; k < 9; k++) {
			const __m128i srcPtr = _mm_load_si128((__m128i*)((char*)src + (k * 16)));
			_mm_stream_si128((__m128i*)((char*)&mMappedData[elementIndex*mElementByteSize] + (k * 16)), srcPtr);
		}
	}

	inline void streamBuffer(int elementIndex, void* src)
	{
		//Unroll Manually
		{
			char *line = (char*)src;
			char *dstline = (char*)&mMappedData[elementIndex*mElementByteSize];
			// prefetch next line
			_mm_prefetch(line + 16, _MM_HINT_NTA);
			__m128i srcPtr = _mm_load_si128((__m128i *)line);
			_mm_stream_si128((__m128i*)dstline, srcPtr);
			srcPtr =  _mm_load_si128((__m128i *)(line + 16));
			_mm_stream_si128((__m128i*)(dstline + 16), srcPtr);
			srcPtr = _mm_load_si128((__m128i *)(line + 32));
			_mm_stream_si128((__m128i*)(dstline + 32), srcPtr);
			srcPtr = _mm_load_si128((__m128i *)(line + 48));
			_mm_stream_si128((__m128i*)(dstline + 48), srcPtr);
			srcPtr = _mm_load_si128((__m128i *)(line + 64));
			_mm_stream_si128((__m128i*)(dstline + 64), srcPtr);
			srcPtr = _mm_load_si128((__m128i *)(line + 80));
			_mm_stream_si128((__m128i*)(dstline + 80), srcPtr);
			srcPtr = _mm_load_si128((__m128i *)(line + 96));
			_mm_stream_si128((__m128i*)(dstline + 96), srcPtr);
			srcPtr = _mm_load_si128((__m128i *)(line + 112));
			_mm_stream_si128((__m128i*)(dstline + 112), srcPtr);
			srcPtr = _mm_load_si128((__m128i *)(line + 128));
			_mm_stream_si128((__m128i*)(dstline + 128), srcPtr);
		}
		//memcpy(&mMappedData[elementIndex*mElementByteSize], (char*)src, mElementByteSize);
	}

	BYTE* mMappedData = nullptr;

private:
    Microsoft::WRL::ComPtr<ID3D12Resource> mUploadBuffer;
    

    UINT64 mElementByteSize = 0;
    bool mIsConstantBuffer = false;
};