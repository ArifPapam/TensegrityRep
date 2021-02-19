cbuffer cbSettings : register(b0)
{
	unsigned int numAnchors;    //0
	unsigned int fixAnchors;      //1
};

RWStructuredBuffer<float> anchor_x : register(u0);			//Input
RWStructuredBuffer<float> anchor_y : register(u1);			//Input
RWStructuredBuffer<float> anchor_z : register(u2);			//Input
RWStructuredBuffer<unsigned int> numConSeg : register(u3);			//Input
RWStructuredBuffer<unsigned int> anch2SegmentID : register(u4);	//Input
RWStructuredBuffer<float> anch2SegDistance : register(u5);	//Input
RWStructuredBuffer<float> anch2SegConst : register(u6);		//Input
RWStructuredBuffer<unsigned int> anchor_2SegStart : register(u7);	//Input
RWStructuredBuffer<float> segLoc_x : register(u8);			//Input
RWStructuredBuffer<float> segLoc_y : register(u9);			//Input
RWStructuredBuffer<float> segLoc_z : register(u10);			//Input
RWStructuredBuffer<float> anchor_veloX : register(u11);		//Input-Output
RWStructuredBuffer<float> anchor_veloY : register(u12);		//Input-Output
RWStructuredBuffer<float> anchor_veloZ : register(u13);		//Input-Output
RWStructuredBuffer<float> forward_x : register(u14);		//Input-Output
RWStructuredBuffer<float> forward_y : register(u15);		//Input-Output
RWStructuredBuffer<float> forward_z : register(u16);		//Input-Output

float dot(float3 a) {
	return ((a.x * a.x) + (a.y * a.y)) + (a.z * a.z);
}

float3 multiply(float f, float3 a) {
	float3 ret;
	ret.x = a.x * f;
	ret.y = a.y * f;
	ret.z = a.z * f;
	return ret;
}

float3 add(float3 a, float3 b) {
	float3 ret;
	ret.x = a.x + b.x;
	ret.y = a.y + b.y;
	ret.z = a.z + b.z;
	return ret;
}

[numthreads(256, 1, 1)]
void computeAnchorSegment(int3 dtid : SV_DispatchThreadID) {
	int anchID = dtid.x;
	if (anchID < numAnchors) {
		float3 anch = { anchor_x[anchID], anchor_y[anchID], anchor_z[anchID] };
		float3 forceAcc = { 0.0f, 0.0f, 0.0f };
		float3 anchForce = { 0.0f, 0.0f, 0.0f };
		
		float anchRatio = (1.0f / numConSeg[anchID]);

		for (unsigned int conSeg = 0; conSeg < numConSeg[anchID]; conSeg++) {
			unsigned int seg = anch2SegmentID[anchor_2SegStart[anchID] + conSeg];
			float3 vec = { segLoc_x[seg] - anch.x, segLoc_y[seg] - anch.y, segLoc_z[seg] - anch.z };
			float dist = sqrt(dot(vec));
			float force = (dist - anch2SegDistance[anchor_2SegStart[anchID] + conSeg]);
			float3 forceVec = multiply(force * anch2SegConst[anchor_2SegStart[anchID] + conSeg], normalize(vec));
			forceAcc = add(forceAcc, forceVec);

			//For devided spring constant.
			float3 anchVec = multiply(force * anchRatio, normalize(vec));
			anchForce = add(anchForce, anchVec);

			forward_x[seg] -= forceAcc.x;
			forward_y[seg] -= forceAcc.y;
			forward_z[seg] -= forceAcc.z;
		}

		if ((anchID + 1) > fixAnchors) { //To bind one of the MT's end
			anchor_veloX[anchID] += anchForce.x;
			anchor_veloY[anchID] += anchForce.y;
			anchor_veloZ[anchID] += anchForce.z;
		}
	}
}