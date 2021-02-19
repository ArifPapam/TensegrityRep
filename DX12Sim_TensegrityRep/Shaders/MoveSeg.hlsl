
cbuffer cbSettings : register(b0) // 11 constants
{
	float WALLx_D2;				//0
	float WALLz_D2;				//1
	float WALLy;				//2
	float radius;				//3
	float DRAG_VAL;				//4
	float DAMPEN_VAL;			//5
	unsigned int DRAG_TYPE;		//6
	unsigned int DAMPEN_TYPE;	//7	
	unsigned int BOUNDED;		//8
	unsigned int GRAVITY_ON;	//9
	unsigned int count;			//10
};

RWStructuredBuffer<float> segLoc_x : register(u0);
RWStructuredBuffer<float> segLoc_y : register(u1);
RWStructuredBuffer<float> segLoc_z : register(u2);
RWStructuredBuffer<float> forward_x : register(u3);
RWStructuredBuffer<float> forward_y : register(u4);
RWStructuredBuffer<float> forward_z : register(u5);

float3 GetColour(float v, float vmin, float vmax)
{
	float3 c = { 1.0f,1.0f,1.0f }; // white
	float dv;

	if (v < vmin)
		v = vmin;
	if (v > vmax)
		v = vmax;
	dv = vmax - vmin;

	if (v < (vmin + 0.25f * dv)) {
		c.x = 0.0f;
		c.y = 4.0f * (v - vmin) / dv;
	}
	else if (v < (vmin + 0.5f * dv)) {
		c.x = 0.0f;
		c.z = 1.0f + 4.0f * (vmin + 0.25f * dv - v) / dv;
	}
	else if (v < (vmin + 0.75f * dv)) {
		c.x = 4.0f * (v - vmin - 0.5f * dv) / dv;
		c.z = 0.0f;
	}
	else {
		c.y = 1.0f + 4.0f * (vmin + 0.75f * dv - v) / dv;
		c.z = 0.0f;
	}

	return(c);
}
float3 m_f3(float x, float y, float z)
{
	float3 f3;
	f3.x = x;
	f3.y = y;
	f3.z = z;
	return f3;
}

float min0(float a) {
	return (float)(a > 0.0f)*a;
}
float sgn(float val) {
	return (0.0f < val) - (val < 0.0f);
}

[numthreads(256, 1, 1)]
void CS(int3 dtid : SV_DispatchThreadID)
{
	unsigned int segNode = dtid.x;
	if (segNode < count) {

		float3 locVec;
		float3 forVec;
		locVec.x = segLoc_x[segNode];
		locVec.y = segLoc_y[segNode];
		locVec.z = segLoc_z[segNode];
		forVec.x = forward_x[segNode];
		forVec.y = forward_y[segNode];
		forVec.z = forward_z[segNode];

		//Checks boundary and alters location and forward vector if needed
		//Boundaries - adjusted for radius of object
		float adj_X = WALLx_D2 - radius;
		float adj_Z = WALLz_D2 - radius;
		float adj_Ytop = WALLy - radius;
		float adj_Ybase = 0.0f + radius;

		if (BOUNDED) {
			if (GRAVITY_ON)
				forVec.y += -0.02f;
			//Boundary conditions
			float DragVal = (1.0f - DAMPEN_VAL);
			if (locVec.x < -adj_X) {
				forVec.x *= -DragVal;
				locVec.x = -adj_X;
			}
			if (locVec.x > adj_X) {
				forVec.x *= -DragVal;
				locVec.x = adj_X;
			}
			if (locVec.y < adj_Ybase) {
				forVec.y *= -DragVal;
				locVec.y = adj_Ybase;
			}
			if (locVec.y > adj_Ytop) {
				forVec.y *= -DragVal;
				locVec.y = adj_Ytop;
			}
			if (locVec.z < -adj_Z) {
				forVec.z *= -DragVal;
				locVec.z = -adj_Z;
			}
			if (locVec.z > adj_Z) {
				forVec.z *= -DragVal;
				locVec.z = adj_Z;
			}
		}
		else {
			if (GRAVITY_ON)
				forVec.y += -0.00008f;
			//Boundary conditions
			if (locVec.x < -adj_X) {
				locVec.x = adj_X;
			}
			if (locVec.x > adj_X) {
				locVec.x = -adj_X;
			}
			if (locVec.y < adj_Ybase) {
				locVec.y = adj_Ytop;
			}
			if (locVec.y > adj_Ytop) {
				locVec.y = adj_Ybase;
			}
			if (locVec.z < -adj_Z) {
				locVec.z = adj_Z;
			}
			if (locVec.z > adj_Z) {
				locVec.z = -adj_Z;
			}
		}

		//Movement below

		//Drag
		if (DRAG_TYPE == 4) {
			//No Drag
		}
		else if (DRAG_TYPE == 0) {
			forVec.x = min0(abs(forVec.x) - DRAG_VAL)*sgn(forVec.x);
			forVec.y = min0(abs(forVec.y) - DRAG_VAL)*sgn(forVec.y);
			forVec.z = min0(abs(forVec.z) - DRAG_VAL)*sgn(forVec.z);
		}
		else if (DRAG_TYPE == 1) {
			forVec.x *= (1.0f - DRAG_VAL);
			forVec.y *= (1.0f - DRAG_VAL);
			forVec.z *= (1.0f - DRAG_VAL);
		}
		else if (DRAG_TYPE == 2) {
			forVec.x *= (1.0f - (DRAG_VAL*abs(forVec.x)));
			forVec.y *= (1.0f - (DRAG_VAL*abs(forVec.y)));
			forVec.z *= (1.0f - (DRAG_VAL*abs(forVec.z)));
		}
		else if (DRAG_TYPE == 3) {
			forVec.x *= (1.0f - (DRAG_VAL*forVec.x*forVec.x));
			forVec.y *= (1.0f - (DRAG_VAL*forVec.y*forVec.y));
			forVec.z *= (1.0f - (DRAG_VAL*forVec.z*forVec.z));
		}

		//Move Forward
		locVec.x += forVec.x;
		locVec.y += forVec.y;
		locVec.z += forVec.z;

		//Update Locations
		segLoc_x[segNode] = locVec.x;
		segLoc_y[segNode] = locVec.y;
		segLoc_z[segNode] = locVec.z;
		forward_x[segNode] = forVec.x;
		forward_y[segNode] = forVec.y;
		forward_z[segNode] = forVec.z;
	}
}