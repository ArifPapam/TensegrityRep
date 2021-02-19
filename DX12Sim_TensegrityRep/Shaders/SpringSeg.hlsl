cbuffer cbSettings : register(b0)
{
	unsigned int startIdx;    //0
	unsigned int endIdx;      //1
	unsigned int MAXSPRING;   //2
	unsigned int SPRINGCOLOR; //3
	float color_range;        //4
};

RWStructuredBuffer<float> segLoc_x : register(u0);			//Input
RWStructuredBuffer<float> segLoc_y : register(u1);			//Input
RWStructuredBuffer<float> segLoc_z : register(u2);			//Input
RWStructuredBuffer<float> forward_x : register(u3);			//Input-Output
RWStructuredBuffer<float> forward_y : register(u4);			//Input-Output
RWStructuredBuffer<float> forward_z : register(u5);			//Input-Output
RWStructuredBuffer<float> springDistance : register(u6);	//Input
RWStructuredBuffer<unsigned int> massID : register(u7);		//Input
RWStructuredBuffer<float> springConstant : register(u8);	//Input
RWStructuredBuffer<float> colors_x : register(u9);			//Output
RWStructuredBuffer<float> colors_y : register(u10);			//Output
RWStructuredBuffer<float> colors_z : register(u11);			//Output

float3 getColourR2G(float v, float max) {
	float3 c = { 1.0f,1.0f,1.0f }; // white
	float dv;

	if (v < -max)
		v = -max;
	if (v > max)
		v = (max);

	if (v < (-max / 2.0f)) {
		c.y = (2.0f * v / max) + 2.0f;
		c.z = 0.0f;
	}
	else if (v < 0.0f) {
		c.x = -2.0f * v / max;
		c.z = 0.0f;
	}
	else if (v < (max / 2.0f)) {
		c.x = 0.0f;
		c.z = 2.0f * v / max;
	}
	else {
		c.x = 0.0f;
		c.y = 2.0f - (2.0f * v / max);
	}

	return(c);
}

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
void computeSpringSeg(int3 dtid : SV_DispatchThreadID) {
	unsigned int seg = dtid.x + startIdx;
	if (seg < endIdx) {
		float3 forceAcc = { 0.0f, 0.0f, 0.0f };
		float forceColor = 0.0f;
		int foundCnt = 0;
		while (massID[seg * MAXSPRING + foundCnt] != 0xFFFFFFFF) {
			unsigned int target = massID[seg * MAXSPRING + foundCnt];
			float3 vec = { segLoc_x[target] - segLoc_x[seg], segLoc_y[target] - segLoc_y[seg], segLoc_z[target] - segLoc_z[seg] };
			float dist = sqrt(dot(vec));
			float force = (dist - springDistance[seg * MAXSPRING + foundCnt]) * springConstant[seg * MAXSPRING + foundCnt];
			forceColor += force;
			forceAcc = add(forceAcc, multiply(force, normalize(vec)));
			foundCnt++;
		}
		forward_x[seg] += forceAcc.x;
		forward_y[seg] += forceAcc.y;
		forward_z[seg] += forceAcc.z;

		if (SPRINGCOLOR) {
			//float3 colorSpr = getColourR2G(sqrt(dot(forceAcc)),color_range);
			float3 colorSpr = getColourR2G(forceColor, color_range);
			colors_x[seg] = colorSpr.x;
			colors_y[seg] = colorSpr.y;
			colors_z[seg] = colorSpr.z;
		}
	}
}