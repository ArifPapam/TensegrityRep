#include "FrameResource.h"

FrameResource::FrameResource(ID3D12Device* device, UINT passCount, InstanceItemCounts items, bool flexiGrid, UINT totalDyVertexCount, UINT materialCount)
{
	ThrowIfFailed(device->CreateCommandAllocator(
		D3D12_COMMAND_LIST_TYPE_DIRECT,
		IID_PPV_ARGS(CmdListAlloc.GetAddressOf())));

	PassCBL = std::make_unique<UploadBuffer<PassConstants>>(device, passCount, true);
	PassCBR = std::make_unique<UploadBuffer<PassConstants>>(device, passCount, true);
	MaterialBuffer = std::make_unique<UploadBuffer<MaterialData>>(device, materialCount, false);

	for (int i = 0; i < (int)items.instanceGroupCount; i++) {
		InstanceBuffer[i] = std::make_unique<UploadBuffer<InstanceData>>(device, items.maxCounts[i], false);
	}
	if (flexiGrid) {
		DynamicVB = std::make_unique<UploadBuffer<Vertex>>(device, totalDyVertexCount, false);
	}
	
	// Create query heaps and result buffers.
	{
		// Two timestamps for each frame and timer.
		const UINT resultCount = 2;
		const UINT resultBufferSize = resultCount * sizeof(UINT64);

		D3D12_QUERY_HEAP_DESC timestampHeapDesc = {};
		timestampHeapDesc.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP;
		timestampHeapDesc.Count = resultCount;


		ThrowIfFailed(device->CreateCommittedResource(
			&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_READBACK),
			D3D12_HEAP_FLAG_NONE,
			&CD3DX12_RESOURCE_DESC::Buffer(resultBufferSize),
			D3D12_RESOURCE_STATE_COPY_DEST,
			nullptr,
			IID_PPV_ARGS(&mTimestampResultBuffers)));

		ThrowIfFailed(device->CreateQueryHeap(&timestampHeapDesc, IID_PPV_ARGS(&mTimestampQueryHeaps)));

	}
	
}

FrameResource::~FrameResource()
{
	//Needed?
	mTimestampResultBuffers.Reset();
	mTimestampQueryHeaps.Reset();
}