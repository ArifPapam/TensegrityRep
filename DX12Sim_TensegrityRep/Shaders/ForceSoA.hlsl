
cbuffer cbSettings : register(b0) // 14 constants
{
	unsigned int startOBJ;		//0
	unsigned int endOBJ;		//1
	float forceRange;			//2
	float e;					//3
	float s;					//4
	unsigned int negForces;		//5
	float NEG_F_MULTIPLY;		//6
	unsigned int COLOR_FORCE;	//7	
	unsigned int CUBE;			//8
	float GRIDxHALF;			//9
	float GRIDzHALF;			//10
	float GRIDyDEPTH_BUFFER;	//11
	unsigned int CUBE_COUNTx_1D;//12
	unsigned int CUBE_COUNTz_1D;//13
	unsigned int HASH_ENTRIE;	//14
	unsigned int LEAP_SEG_CNT;	//15
};

RWStructuredBuffer<unsigned int> originalIndex : register(u0); 
RWStructuredBuffer<float> segLocSorted_x : register(u1);
RWStructuredBuffer<float> segLocSorted_y : register(u2);
RWStructuredBuffer<float> segLocSorted_z : register(u3);
RWStructuredBuffer<unsigned int> cellStart : register(u4);
RWStructuredBuffer<unsigned int> cellEnd : register(u5);

RWStructuredBuffer<float> objDataToModifiy_x : register(u6);
RWStructuredBuffer<float> objDataToModifiy_y : register(u7);
RWStructuredBuffer<float> objDataToModifiy_z : register(u8);
RWStructuredBuffer<float> colors_x : register(u9);
RWStructuredBuffer<float> colors_y : register(u10);
RWStructuredBuffer<float> colors_z : register(u11);

RWStructuredBuffer<float> segRadiSort : register(u12);		//Addition

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
unsigned int hashGPU(float x, float y, float z) {
	unsigned int xCube, yCube, zCube;
	xCube = (x + GRIDxHALF) / CUBE;
	zCube = (z + GRIDzHALF) / CUBE;
	yCube = (y + GRIDyDEPTH_BUFFER) / CUBE;
	return (yCube*CUBE_COUNTz_1D*CUBE_COUNTx_1D) + (zCube + xCube * CUBE_COUNTz_1D);
}

float3 multiply(float f, float3 a) {
	float3 ret;
	ret.x = a.x * f;
	ret.y = a.y * f;
	ret.z = a.z * f;
	return ret;
}
float3 multiply(float3 a, float3 b) {
	return m_f3(a.x * b.x, a.y * b.y, a.z * b.z);
}
float3 add(float3 a, float3 b) {
	float3 ret;
	ret.x = a.x + b.x;
	ret.y = a.y + b.y;
	ret.z = a.z + b.z;
	return ret;
}
float3 subtract(float3 a, float3 b) {
	return m_f3(a.x - b.x, a.y - b.y, a.z - b.z);
}
float3 divide(float3 a, float f) {
	return m_f3(a.x / f, a.y / f, a.z / f);
}
float dot(float3 a) {
	return ((a.x * a.x) + (a.y * a.y)) + (a.z * a.z);
}
float return_inverse(float k) {
	if (k == 0.0f)
		return 0.0f;
	else
		return (1.0f / k);
}
float3 normalize(float3 q)
{
	float num2 = dot(q);
	float num = return_inverse(sqrt(num2));
	float3 norm;
	norm.x = q.x * num;
	norm.y = q.y * num;
	norm.z = q.z * num;
	return norm;
}

[numthreads(256, 1, 1)]
void CS2(int3 dtid : SV_DispatchThreadID)
{
	unsigned int segNode = dtid.x;
	if (segNode < endOBJ) {
		//Used for coloring based on forces acting on a segment
		float forceSave = 0.0f;

		//Local accumulator
		float3 pull_Singletotal = { 0.0f, 0.0f, 0.0f };

		//Saving location of segment of focus for performance reasons (more critical on GPU)
		unsigned int segNodeIndex = originalIndex[segNode];
		float xLoc = segLocSorted_x[segNode];
		float yLoc = segLocSorted_y[segNode];
		float zLoc = segLocSorted_z[segNode];
		float segR = segRadiSort[segNode];		//Addition
		unsigned int tubeGrid = hashGPU(xLoc, yLoc, zLoc);

		//Loop over the 27 locations in a 3x3x3 box
		for (unsigned int t = 0; t < 27; t++) {
			//The below line computes 3D grid locations using a basic incrementing iterator and the hash dimentions 
			unsigned int targetGrid = (CUBE_COUNTx_1D*CUBE_COUNTz_1D*((t / 9) - 1)) + ((tubeGrid - CUBE_COUNTz_1D + (((t % 9) / 3)*CUBE_COUNTz_1D)) - 1 + ((t) % 3));
			if (targetGrid < HASH_ENTRIE && targetGrid >= 0) {
				unsigned int startIndex = cellStart[targetGrid];
				unsigned int endIndex = cellEnd[targetGrid];
				if (startIndex != 0xffffffff)          // cell is not empty
				{
					for (unsigned int cNode = startIndex; cNode < endIndex; cNode++) {
						//Make sure it doesnt compute against itself
						if (cNode != segNode && cNode < endOBJ) {
							float3 vectorA = { xLoc - segLocSorted_x[cNode], yLoc - segLocSorted_y[cNode], zLoc - segLocSorted_z[cNode] };
							float dist = sqrt(vectorA.x*vectorA.x + vectorA.y*vectorA.y + vectorA.z*vectorA.z);
							//Check if the objects are in the interaction range
							if (dist < forceRange) {
								float maxForce = s*3.0f; //Could be anything
								//Lennard-Jones potential with lower exponents and avoiding the pow() function for performance
								if (dist == 0.0f) { dist = 0.00001f; }
								float radiusCNode = segRadiSort[cNode];	//Addition
								float equ = radiusCNode + segR;			//Addition
								float innerNum = (equ / dist);			//Addition
								float x2 = innerNum*innerNum;
								float x4 = x2*x2;
								float x8 = x4*x4;
								float force = s*(x8 - x4);

								if (force > maxForce) { force = maxForce; }

								if (force < 0.0f) {
									force *= negForces;
									force *= NEG_F_MULTIPLY;
								}
								unsigned int segOriginalIndex = originalIndex[segNode];
								unsigned int targetOriginalIndex = originalIndex[cNode];
								if (segOriginalIndex > LEAP_SEG_CNT || targetOriginalIndex > LEAP_SEG_CNT) {
									forceSave += force;
								}

								pull_Singletotal = add(pull_Singletotal, multiply(force, normalize(vectorA)));
							}
						}
					}
				}
			}
		}
		objDataToModifiy_x[segNodeIndex] += pull_Singletotal.x;
		objDataToModifiy_y[segNodeIndex] += pull_Singletotal.y;
		objDataToModifiy_z[segNodeIndex] += pull_Singletotal.z;

		//Applying color to the segment
		if (COLOR_FORCE) {
			float3 color;
			//Color based on the accumulation of positive and negative forces acting on the segment
			color = GetColour(forceSave, -0.3f, 0.4f);
			//Setting the color to the global structure
			colors_x[segNodeIndex] = color.x;
			colors_y[segNodeIndex] = color.y;
			colors_z[segNodeIndex] = color.z;
		}
	}
}