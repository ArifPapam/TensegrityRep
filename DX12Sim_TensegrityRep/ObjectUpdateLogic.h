#pragma once

#include "Render_DX12.h"
#include "managedMem.h"
#include "Readers.h"
#include "atomRadii.h"
#include <random>
#include <vector>
#include <queue>
#include <string>
//#include "FXHlib.h" /////////////////////////////Fuji-Xerox Haptic library//////////////////////

class UpdateClass : public RenderDX12
{
public:
	UpdateClass(HINSTANCE hInstance);

	//Hash table entry
	struct Entry {
		unsigned int    key;
		Entry           *next;
	};

	//Layout for better vectorization - however many loops are very complex and may not vectorize for other reasons
	struct floatArray3 {
		float *x;
		float *y;
		float *z;
	};

	struct uint4Array6 { // Made to support up to 6 connected segments
		XMFLOAT4 * a;
		XMFLOAT4 * b;
		XMFLOAT4 * c;
		XMFLOAT4 * d;
		XMFLOAT4 * e;
		XMFLOAT4 * f;
	};

	struct mapH {
		UINT hashVal;
		UINT orig;
	};
	struct KeyValue {
		UINT kHash;
		UINT vIdx;
	};
	struct StartEnd {
		UINT s;
		UINT e;
	};

	managedMemClass	mem;
	floatArray3		segLocation;					//**Location (x,y,z) 
	floatArray3		segLocationSorted;				//**Sorted Location for better memory patterns
	floatArray3		transVelocity;					//**Translational Velocity (movement in 3D space)
	floatArray3		segColor;						//  RGB segment color
	float *			forceSave;						//	Used for coloring based on forces acting on a segment
	//floatArray3		directionalForceAccumulator;	//	Used to mirror GPU thread parallelism on the CPU across 2 loops
	float *			segRadius;						//**Segmant Radius
	float *			segRadiusSorted;				//**Segmant Radius Sorted
	float *			segTempFactor;					//  Segment temperature factor

	//For Random generator
	unsigned int seed1;
	std::default_random_engine generator;

	//-----------------------------------------------------------------------------------------------------------
	//HAPTIC
	//-----------------------------------------------------------------------------------------------------------
	//FXH::FXHdevice fxHapDevice;				//Fuji-Xerox haptic device handdler///////////
	void setHapticFeedback(float limitSense);	//For generating haptic feedback.
	float hapticForceThreshold = 0.00f;			//For a threshold value of interacted segment force to trigger a haptic feedback.
	//-----------------------------------------------------------------------------------------------------------

	ofstream resultFile;			//Output file stream object for writing results to a .csv file.
	UINT iCycle = 0;				//For writing iteration/computing cycle number.
	double upForce, segDiameter;
	float MTlength;
	float initY;
	UINT fixAnchors, objType, lastSegmentPlus1;
	float colorRange = 0.0025f;

	//-----------------------------------------------------------------------------------------------------------
	//ObjectParameters is a structure for storing object parameter values.
	//-----------------------------------------------------------------------------------------------------------
	struct ObjectParameters {
		float segToSegConstant;		//Spring constant value between a segment and other segment.
		float anchToAnchConstant;	//Spring constant value between an anchor and other anchor.
		float anchToSegConstant;	//Spring constant value between an anchor and segments.
		float anchorDistance;		//Distance between each anchor and center of the object.
		float jointAnchDistance;	//Distance between each joint anchor and their joint center.
		
		UINT numSection;			//Number of section in the object.
		UINT sectionLength;			//Length of the object. (MT => length in the number of dimer)
		float segWidth;				//Segment's width. (MT => dimer's width)

		UINT numSegMove;			//Number of moving segment in the end part of the object.
		float upMove;				//Distance to move in 1 compute cycle (moving part).

		ObjectParameters() {}
		ObjectParameters(float s2sConst, float a2aConst, float a2sConst, float anchDist, float jointDist, UINT nSection, UINT sectionL, float segW, UINT nSegMov, float up) {
			segToSegConstant = s2sConst;
			anchToAnchConstant = a2aConst;
			anchToSegConstant = a2sConst;
			anchorDistance = anchDist;
			jointAnchDistance = jointDist;
			numSection = nSection;
			sectionLength = sectionL;
			segWidth = segW;
			numSegMove = nSegMov;
			upMove = up;
		}
	} objPar1, objPar2;
	//-----------------------------------------------------------------------------------------------------------

	struct Anchor2SegmentLinks {
		std::vector<UINT> anch2SegLink;
	};

	//-----------------------------------------------------------------------------------------------------------
	//Anchor structure represents an anchor.
	//-----------------------------------------------------------------------------------------------------------
	struct Anchor {
		std::vector<float> x;
		std::vector<float> y;
		std::vector<float> z;

		std::vector<float> veloX;
		std::vector<float> veloY;
		std::vector<float> veloZ;

		std::vector<Anchor2SegmentLinks> connectedSegment;
		std::vector<UINT> numConSeg;
	};
	//-----------------------------------------------------------------------------------------------------------

	//-----------------------------------------------------------------------------------------------------------
	//ObjectManager is a class for managing Tensegrty representation objects is the simulation.
	//-----------------------------------------------------------------------------------------------------------
	class ObjectManager {
	private:
		UINT numObj = 0;                        //Number of object built using segments.
		UINT numAnch = 0;                       //Total number of anchors.
		std::vector<UINT> objSegStart;          //Array of object's start segments.
		std::vector<UINT> objSegCount;          //Array of object's segments counts.
		std::vector<UINT> objAnchorStart;       //Array of object's start anchors.
		std::vector<UINT> objAnchorCount;       //Array of object's anchors counts.
		
		Anchor anchors;                         //All anchors in the world.
		
		std::vector<UINT> anch2AnchorID;                //List per 10 elements.
		std::vector<float> anch2AnchorDistance;
		std::vector<float> anch2AnchorSpringConst;
		UINT anch2AnchMax = 15;

		std::vector<UINT> anchor2SegConnect;
		std::vector<UINT> anch2SegmentConnect;
		std::vector<float> anchorToSegmentDistance;
		std::vector<float> anchorToSegmentSpringConst;

		std::vector<UINT> anch2SegmentStart;	//This data was created to make GPU compute shader easier by allowing it to be copied to GPU memory.
		
	public:
		//To get address of the begining of the anchors.numConSeg
		UINT *getAddressNumConSeg() { return &anchors.numConSeg[0]; }

		//To get address of the begining of the anchors.x.
		float *getAddressAnchorX() { return &anchors.x[0]; }

		//To get address of the begining of the anchors.y.
		float *getAddressAnchorY() { return &anchors.y[0]; }

		//To get address of the begining of the anchors.z.
		float *getAddressAnchorZ() { return &anchors.z[0]; }

		//To get address of the begining of the anchors.veloX.
		float *getAddressAnchVeloX() { return &anchors.veloX[0]; }

		//To get address of the begining of the anchors.veloY.
		float *getAddressAnchVeloY() { return &anchors.veloY[0]; }

		//To get address of the begining of the anchors.veloZ.
		float *getAddressAnchVeloZ() { return &anchors.veloZ[0]; }

		//To get address of the begining of the Anch2SegmentConnect.
		UINT *getAddressAnch2SegmentID() { return &anch2SegmentConnect[0]; }

		//To get address of the begining of the AnchorToSegmentDistanc.
		float *getAddressAnch2SegDistance() { return &anchorToSegmentDistance[0]; }

		//To get address of the begining of the AnchorToSegmentSpringConst.
		float *getAddressAnch2SegConst() { return &anchorToSegmentSpringConst[0]; }

		//To get address of the begining of the anch2SegmentStart.
		UINT *getAddressAnch2SegmentStart() { return &anch2SegmentStart[0]; }

		//To get the number of objects.
		UINT getNumAnch2Seg() { return anch2SegmentConnect.size(); }

		//To initiate object manager.
		//It is mandatory to execute this after creating ObjectManager object, before creating any 3D object.
		//handSegCount = number of segments needed for virtual hands.
		void init(UINT handSegCount);

		//To set anch2SegmentStart that will be used for compute shader data.
		//It is mandatory to execute this after creating all 3D objects.
		//Execute this after all objects has been created.
		void initAnch2Seg();

		//To add a new object.
		//segCount = number of segments needed.
		//anchCount = number of anchors needed.
		UINT addObject(UINT segCount, UINT anchCount);

		//To get the number of objects.
		UINT getNumObj() { return numObj; }

		//To get the number of anchors.
		UINT getNumAnchor() { return numAnch; }

		//To get the maximum number of anchors connected to an anchor.
		UINT getMaxAnchor2Anchor() { return anch2AnchMax; }

		//To get the first segment ID of this object.
		UINT getObjSegStart(UINT objectID) { return objSegStart[objectID]; }

		//To get the number of segment in this object.
		UINT getObjSegCount(UINT objectID) { return objSegCount[objectID]; }

		//To get the first anchor ID of this object.
		UINT getObjAnchorStart(UINT objectID) { return objAnchorStart[objectID]; }

		//To get the number of anchors in this object.
		UINT getObjAnchorCount(UINT objectID) { return objAnchorCount[objectID]; }

		//To get anchor location data.
		XMFLOAT3 getAnchorPos(UINT anchorID) { return XMFLOAT3(anchors.x[anchorID], anchors.y[anchorID], anchors.z[anchorID]); }
		XMFLOAT3 getAnchorPos(UINT objectID, UINT objAnchorN) { return XMFLOAT3(anchors.x[objAnchorStart[objectID] + objAnchorN], anchors.y[objAnchorStart[objectID] + objAnchorN], anchors.z[objAnchorStart[objectID] + objAnchorN]); }

		//To get the number of segments connected to this anchor (ID=anchorID).
		UINT getNumConSeg(UINT anchorID) { return anchors.connectedSegment[anchorID].anch2SegLink.size(); }
		UINT getNumConSeg(UINT objectID, UINT objAnchorN) { return anchors.connectedSegment[objAnchorStart[objectID] + objAnchorN].anch2SegLink.size(); }

		//To get a connected segment to this anchor (ID=anchorID);
		UINT getConSegID(UINT anchorID, UINT conSegN) { return anch2SegmentConnect[anchors.connectedSegment[anchorID].anch2SegLink[conSegN]]; }

		//To get the original distance of a connected segment to this anchor (ID=anchorID);
		float getConSegDistance(UINT anchorID, UINT conSegN) { return anchorToSegmentDistance[anchors.connectedSegment[anchorID].anch2SegLink[conSegN]]; }

		//To get the spring constant of a connected segment to this anchor (ID=anchorID);
		float getConSegConst(UINT anchorID, UINT conSegN) { return anchorToSegmentSpringConst[anchors.connectedSegment[anchorID].anch2SegLink[conSegN]]; }

		//To set the original distance of a connected segment to this anchor (ID=anchorID);
		void setConSegDistance(UINT anchorID, UINT conSegN, float value) { anchorToSegmentDistance[anchors.connectedSegment[anchorID].anch2SegLink[conSegN]] = value; }

		//To get connected anchor with this anchor.
		UINT getAnchor2AnchorID(UINT anchorID, UINT conAnchorN) { return anch2AnchorID[anchorID * anch2AnchMax + conAnchorN]; }
		UINT getAnchor2AnchorID(UINT objectID, UINT objAnchorN, UINT conAnchorN) { return anch2AnchorID[(objAnchorStart[objectID] + objAnchorN) * anch2AnchMax + conAnchorN]; }

		//To get connected anchor's distance with this anchor.
		float getAnchor2AnchorDist(UINT anchorID, UINT conAnchorN) { return anch2AnchorDistance[anchorID * anch2AnchMax + conAnchorN]; }
		float getAnchor2AnchorDist(UINT objectID, UINT objAnchorN, UINT conAnchorN) { return anch2AnchorDistance[(objAnchorStart[objectID] + objAnchorN) * anch2AnchMax + conAnchorN]; }

		//To get connected anchor's spring constant with this anchor.
		float getAnchor2AnchorConst(UINT anchorID, UINT conAnchorN) { return anch2AnchorSpringConst[anchorID * anch2AnchMax + conAnchorN]; }
		float getAnchor2AnchorConst(UINT objectID, UINT objAnchorN, UINT conAnchorN) { return anch2AnchorSpringConst[(objAnchorStart[objectID] + objAnchorN) * anch2AnchMax + conAnchorN]; }

		//To set an anchor's position.
		void setAnchorPos(UINT anchorID, XMFLOAT3 value);
		void setAnchorPos(UINT objectID, UINT objAnchorN, XMFLOAT3 value);

		//To add value to anchor's velocity.
		void addAnchorVelo(UINT anchorID, XMFLOAT3 value);
		void addAnchorVelo(UINT objectID, UINT objAnchorN, XMFLOAT3 value);

		//To add anchor's velocity to anchor's position.
		void addAnchorVelo2Pos(UINT anchorID);

		//To connect two anchors.
		bool setAnchor2Anchor(UINT anchor1, UINT anchor2, float springConstant);

		//To set all 4 anchors in a tetrahedronal positions.
		void set4Anchors(UINT objectID, UINT objSubID, XMFLOAT3 objCenter, float distanceToCenter, bool isReverse, float springConstant);

		//To set all 3 anchors in a triangle positions.
		void set3Anchors(UINT objectID, UINT anchorN, XMFLOAT3 objCenter, float distanceToCenter, bool isReverse, bool connectedTriangle, float springConstant);

		//To set all 4 anchors in a triangle positions.
		void set4FlatAnchors(UINT objectID, UINT anchorN, XMFLOAT3 objCenter, float distanceToCenter, bool isDiagonal, float springConstant);

		//To set all 6 anchors in 3 axis directions.
		void set6Anchors(UINT objectID, UINT objSubID, XMFLOAT3 objCenter, float distanceToCenter, float springConstant);

		//To add an anchor-to-segment connection between anchorID and segmentID.
		void addAnchor2Segment(UINT anchorID, UINT segmentID, float distance, float springConstant);
	};
	ObjectManager* objMan;    //This is the object manager for this simulation.
	//-----------------------------------------------------------------------------------------------------------

	AtmRad::AtomRadii atmRad; //This is for providing atomic radii data.
	
	PDBreader::FileReader* pdbFile = new PDBreader::FileReader("1jff.pdb");  //PDB file reader
	
	float hapticLevelValue[7][2] = {
		{ 0.22772705696203f, 20.0f },
		{ 0.24928516624041f, 30.0f },
		{ 0.27955384615385f, 40.0f },
		{ 0.50848322851153f, 50.0f },
		{ 0.72653609341826f, 60.0f },
		{ 0.8239395280236f,  70.0f },
		{ 1.0f, 80.0f} };

	//This is for creating an object from .pdb file.
	//isAnchored==true to use Tensegrity representation method.
	XMFLOAT3* createPdbAnchObj(PDBreader::FileReader* pdb, float scale, bool isAnchored, XMMATRIX transform);
	
	//This is to make spring compound to segments (spring-to-spring connection based on distance).
	void initSpringCompound(UINT startSeg, UINT endSeg, float distance, float springC);

	//This is just for testing the idea of anchored object model.
	UINT testObj(UINT length, UINT width, XMFLOAT3 origin);

	//This is for folding an anchored object model.
	UINT testObjFolded(UINT length, UINT width, XMFLOAT3 origin);

	//This is for making a long anchored object model.
	UINT testObjLong(UINT length, UINT width, UINT section, XMFLOAT3 origin);

	//This is for making anchored object with joint.
	//
	UINT testJointObj(UINT length1, UINT length2, UINT width, UINT jointType, XMFLOAT3 origin);

	//This is for making a microtubule anchored object model.
	UINT createMT1(XMFLOAT3 origin, ObjectParameters objParam); //1 anchor per section.
	UINT createMT2(XMFLOAT3 origin, ObjectParameters objParam); //4 anchors per section without section connection.
	UINT createMT3(XMFLOAT3 origin, ObjectParameters objParam); //4 anchors per section, connect all adjacent section anchors.
	UINT createMT4(XMFLOAT3 origin, ObjectParameters objParam); //4 anchors per section, 3 joint anchors.
	UINT createMT5(XMFLOAT3 origin, ObjectParameters objParam); //2x3 anchors per section, overlapping anchors.
	UINT createMT6(XMFLOAT3 origin, ObjectParameters objParam); //4 anchors per section, 3 joint anchors, connect adjacent joint anchors.
	UINT createMT7(XMFLOAT3 origin, ObjectParameters objParam, UINT bridge); //2x4 flat anchors per section, overlapping anchors, connect with bridges.

	//This is for making a filamen anchored object model (one array of segments)
	UINT createFilamen1(XMFLOAT3 origin, ObjectParameters objParam, UINT bridge);	//2x4 flat anchors per section, overlapping anchors, connect with bridges.
	UINT createFilamen2(XMFLOAT3 origin, ObjectParameters objParam);				//4 anchors per section, 3 joint anchors.
	UINT createFilamen3(XMMATRIX oriMx, ObjectParameters objParam, UINT bridge);	//2x4 flat anchors per section, overlapping anchors, connect with bridges, position by oriMx.

	//This is for making microtubule from some data.
	//mtPos.x axis is from left (0.0f) to right; mtPos.y axis is from forward to backward; mtPos.z axis is from bottom to top.
	//Rot is rotation with the axis the same as mtPos.
	//adjuster is another position adjuster. See the function definition for more clear.
	UINT createSmoothMuscleMT(XMFLOAT3 mtPos, XMFLOAT3 axis, float rot, ObjectParameters objParam, UINT bridge, float magnifier, XMFLOAT3 adjuster);	//Create filament object with rotates around an arbitrary axis.
	UINT createSmoothMuscleMT(XMFLOAT3 mtPos, XMFLOAT3 rot, ObjectParameters objParam, UINT bridge, float magnifier, XMFLOAT3 adjuster);				//Create filament object with XYZ rotations.
																																						
	//This is for arranging segments to form a microtubule.
	void arrangeMT(UINT startSegment, UINT numSegment, UINT length, UINT dimerLength, UINT dimerWidth, float springConst, XMFLOAT3 origin);

private:
	int springConnect(UINT segment1ID, UINT segment2ID, float stringC);
	bool springHalfConnect(UINT segment1ID, UINT segment2ID, float stringC);
	
	UINT foldingID;		//ID for folding setting.
	UINT unfoldingID;	//ID for unfolding setting.
	UINT foldedObjID;	//ID for folded/unfolded object.
	UINT longObjID;		//ID for long object.

	/*To get the center of spring mass of a spring object.
	startSeg is the start segment index of the object.
	endSeg is the end segment index of the object (excluding anchors).*/
	XMFLOAT3 centerSpringMass(UINT startSeg, UINT endSeg);

	/*To get the center of spring mass of a spring object.
	segmentArray is the array of the object's segments.
	startSeg is the start segment index in the array.
	endSeg is the end segment index in the array (excluding anchors).*/
	XMFLOAT3 centerSpringMass(XMFLOAT3* segmentArray, UINT startSeg, UINT endSeg);

public:
	UINT *flag;
	KeyValue *KVList;
	KeyValue *KVTemp;
	static const UINT sizeDa = 32768;
	UINT * countBit;
	UINT * countBitCounts;
	UINT * countBitStarts;
	UINT bitCountNeeded = 1;

	UINT * segOriginalIndex;
	UINT * segCellStart;
	UINT * segCellEnd;
	UINT * pickingVisited;

	float *avgSegmentsFoundLJP;
	float *avgSegmentsComputedLJP;

	///Sim space and grid
	unsigned int CUBE; //Cube makes big changes
	const static int GRID_BUFFER = 50;
	int WALLx_D2;
	int WALLz_D2;
	int WALLy_D2;
	int WALLy;
	UINT CUBE_COUNTx_1D;
	UINT CUBE_COUNTx_1D_HALF;
	float GRIDxHALF;
	UINT CUBE_COUNTz_1D;
	UINT CUBE_COUNTz_1D_HALF;
	float GRIDzHALF;
	UINT CUBE_COUNTy_1D;
	UINT CUBE_COUNTy_1D_HALF;
	float GRIDyDEPTH_BUFFER; //Used as a shift so it is positive, value is the units below 0 the y cube grid extends
	float GRIDyELEVATION_BUFFER;
	UINT HASH_ENTRIE;

	UINT SEGMENT_COUNT;
	UINT OBJECT_COUNT;

	Microsoft::WRL::ComPtr<ID3D12CommandQueue> mCommandQueue_Sim;
	Microsoft::WRL::ComPtr<ID3D12CommandAllocator> CmdListAlloc_Sim;
	Microsoft::WRL::ComPtr<ID3D12GraphicsCommandList> mCommandList_Sim;
	Microsoft::WRL::ComPtr<ID3D12Fence> mFence_Sim;
	UINT64 mCurrentFence_Sim = 0;
	Microsoft::WRL::ComPtr<ID3D12QueryHeap> mTimestampQueryHeaps_Sim;
	Microsoft::WRL::ComPtr<ID3D12Resource> mTimestampResultBuffers_Sim;

	Microsoft::WRL::ComPtr<ID3D12CommandQueue> mCommandQueue_Cpy;
	Microsoft::WRL::ComPtr<ID3D12CommandAllocator> CmdListAlloc_Cpy;
	Microsoft::WRL::ComPtr<ID3D12GraphicsCommandList> mCommandList_Cpy;
	Microsoft::WRL::ComPtr<ID3D12Fence> mFence_Cpy;
	UINT64 mCurrentFence_Cpy = 0;
	Microsoft::WRL::ComPtr<ID3D12QueryHeap> mTimestampQueryHeaps_Cpy;
	Microsoft::WRL::ComPtr<ID3D12Resource> mTimestampResultBuffers_Cpy;

	void initUpdateClass(float spaceDimXD2_sg, float spaceDimZD2_sg, unsigned int * segCount, int cubeLength, float * averageSegmentsFoundLJP, float * averageSegmentsComputedLJP);
	void closeUpdateLogic();

	//Compute Operations
	void MainUpdateMethod(float dt, LeapSegments* ls);
	void propagateHandForceAcrossSprings();
	void SimDataToInstData(int renderNumber);
	void CPUReSortObjects(int hash_entries, int elements, floatArray3 &locations);
	void force_searchList_CPU(const UINT * originalIndex, const floatArray3 segLocSorted, floatArray3 objDataToModifiy, floatArray3 colors, const UINT *cellStart, const UINT *cellEnd, int startOBJ, int endOBJ,
		float forceRange, float e, float s, UINT negForces, UINT COLOR_FORCE, float *avgSegmentsFound, float *avgSegmentsComputed);

	float LJPController(float xStatic, float yStatic, float zStatic, float xDy, float yDy, float zDy, float forceRange, float e, float s, bool negForces, XMFLOAT3 &pull_Singletotal);
	int LJP3(int segNode, int currentNode, float xLoc, float yLoc, float zLoc, floatArray3 segLoc, float forceRange, float e, float s, bool negForces, XMFLOAT3 &pull_Singletotal, float * forceSave, const UINT * originalIndex);
	int LJP3_Hand(int segNode, int cNode, UINT segNodeOrig, float xLoc, float yLoc, float zLoc, float segR, floatArray3 segLoc, float radiusCNode, float forceRange, float s, UINT negForces, XMFLOAT3 &pull_Singletotal, XMFLOAT3 &pull_Handtotal, float * forceSave, const UINT * originalIndex);
	void accumulateHapticSegment(float F, UINT segment, UINT target);

	void Tube();
	void TubeWand(XMMATRIX * controllerMatrix, XMFLOAT3 * locationAcc, int s, int e);
	void TubeWand16(XMMATRIX * controllerMatrix, XMFLOAT3 * locationAcc, int s, int e);

	void VRControllerInteractions();
	void VRCollisionInteractions(int s, int e);
	void MouseInteractions();
	void Pick(int sx, int sy);
	void PickOpti(int sx, int sy);
	void controller_pickingL(floatArray3 segLocSorted, const UINT * originalIndex, const UINT *cellStart, const UINT *cellEnd, int controller, XMFLOAT3 grabLoc, float forceRange, int * selectedSeg);
	int Controller(int segNode, int currentNode, float xLoc, float yLoc, float zLoc, floatArray3 segLoc, floatArray3 segFor, float forceRange, XMFLOAT3 controllerDelta, floatArray3 colors, bool COLOR_FORCE);
	
	void applyDrag(XMFLOAT3 &forVec);
	void applyDragIF(XMFLOAT3 &forVec);
	void CheckBoundary(bool bounded, float radius, XMFLOAT3 &locVec, XMFLOAT3 &forVec);

	//-----------------------------------------------------------------------------------------------------------
	//Spring force////////////////////////////////////////////////////////////////////////
	//-----------------------------------------------------------------------------------------------------------
	float *springDistance;
	float *springConstant;
	UINT *massID;
	UINT MAXSPRING = 100;			//This is maximum number of segments to connect by springs per segment.
	void computeSpringObject();		//This is for computing segment-to-segment connection (CPU).
	void computeAnchorSegment();	//This is for computing anchor-to-segment connection (CPU).
	void computeAnchors();			//This is for computing anchor-to-anchor connection (CPU).
	void moveAnchors();				//This is for integrating the movement of anchors.
	//-----------------------------------------------------------------------------------------------------------
	
	//Brownian Motion
	UINT brownian1perN; //Randomize 1 segment per this value to apply Brownian motion.
	UINT numBrownianSeg;
	UINT *brownianSegmentID;
	UINT brownianIteration = 0;
	float *brownianSegmentX;
	float *brownianSegmentY;
	float *brownianSegmentZ;
	void randomizeBrownian(float magnitude);
	void useBrownianMotion(UINT maxBrownianRadius);

	//-----------------------------------------------------------------------------------------------------------
	//Compute Things
	//-----------------------------------------------------------------------------------------------------------
	void BuildComputeCommandQLsim();
	void BuildComputeCommandQLcpy();
	void FlushCommandQueueCSsim();
	void FlushCommandQueueCScpy();

	//Lennard-Jones potential (collision calculation) compute shader.
	void LJP_CS();
	void BuildComputeBuffers();		//Build
	void RecordComputeWork();		//Record
	void CShaderDataInput();		//In
	void CShaderDataOutput();		//Out
	void CloseComputeBuffers();		//Close

	//Segment-to-segment compute shader.
	void Spring_CS();
	void BuildComputeBuffersSpring();	//Build
	void RecordComputeWorkSpring();		//Record
	void CShaderDataInputSpring();		//In
	void CShaderDataOutputSpring();		//Out

	//Anchor-to-segment compute shader. (Under construction)
	void Anch2Seg_CS();
	void BuildComputeBuffersAnch2Seg();	//Build
	void RecordComputeWorkAnch2Seg();	//Record
	void CShaderDataInputAnch2Seg();	//In
	void CShaderDataOutputAnch2Seg();	//Out
	
	//Segment movement integration compute shader.
	void Move_CS();
	void BuildComputeBuffersMove();	//Build
	void RecordComputeWorkMove();	//Record
	void CShaderDataInputMove();	//In
	void CShaderDataOutputMove();	//Out
	//-----------------------------------------------------------------------------------------------------------

	//Bonus Operations
	void holdWall();
	void sendWall(bool left);
	void CubeSetFunc();
	void updateLocation(UINT i, float x, float y, float z);
	void shiftLocation(UINT i, float x, float y, float z);
	void updateForward(UINT i, float x, float y, float z);
	void shiftForward(UINT i, float f);
	void shiftColor(UINT i, float f);

	// Sorting Operations
	UINT hashCPU(float x, float y, float z);
	void merge(struct mapH *mapP, int n, int m);
	void merge_sort(struct mapH *mapP, int n);
	void sortListCPU(floatArray3 locations, floatArray3 locationsSorted, UINT * originalIndex, UINT * cellStart, UINT * cellEnd);

	void InitRadix(UINT length);
	void CloseRadix();
	void checkCount2(const KeyValue * __restrict list, KeyValue * __restrict temp, UINT * __restrict countBit, UINT bitLoc, const int blockId, const int blockCnt, const int max);
	void pack2(KeyValue * __restrict list, const KeyValue * __restrict temp, const UINT * __restrict countBit, const UINT * __restrict countBitStart, const int blockId, const int blockCount, const int max);
	void checkKVList(KeyValue * KVli, UINT flag);
	int Radix1DSeg2Kv(floatArray3 locations, floatArray3 locationsSorted, UINT * originalIndex, UINT * cellStart, UINT * cellEnd, UINT length, UINT bitCount);

	//Math and Helpers
	float dot(XMFLOAT3 a);
	float dot(XMFLOAT4 a);
	float dot(float x, float y, float z);
	float dot(XMFLOAT3 a, XMFLOAT3 b);
	float return_inverse(float k);
	XMFLOAT3 normalize(XMFLOAT3 q);
	XMFLOAT3 normalize(XMFLOAT3 q, float m);
	XMFLOAT3 normalize(float x, float y, float z);
	XMFLOAT4 normalize(XMFLOAT4 q);
	XMFLOAT4 normalize(XMFLOAT4 q, float m);
	XMFLOAT3 multiplyF3(XMFLOAT3 a, float b);
	XMFLOAT4 multiplyF4(XMFLOAT4 a, float b);
	void swap(float *a, float *b);
	bool solveQuadratic(const float &a, const float &b, const float &c, float &x0, float &x1);
	XMFLOAT3 multiply(float f, XMFLOAT3 a);
	XMFLOAT3 divide(XMFLOAT3 a, float f);
	XMFLOAT3 add(XMFLOAT3 a, XMFLOAT3 b);
	XMFLOAT3 subtract(XMFLOAT3 a, XMFLOAT3 b);
	int IntersectRaySphere(XMFLOAT3 origin, XMFLOAT3 ray, XMFLOAT3 s, float radius, float &t, XMFLOAT3 &q);
	bool intersectGeo(const XMFLOAT3 &rayOrig, const XMFLOAT3 &rayDir, const XMFLOAT3 &sphereCentLoc, const float &sphereRadius2, float &t);
	bool intersectAny(const XMFLOAT3 &rayOrig, const XMFLOAT3 &rayDir, const XMFLOAT3 &sphereCentLoc, const float &sphereRadius2, float &t);
	bool IntersectRayAABB(XMFLOAT3 origin, XMFLOAT3 ray, XMFLOAT3 boxMin, XMFLOAT3 boxMax, float &t);
	void Matrix4x4Multiply(float src1[4][4], float src2[4][4], float dest[4][4]);
	void Matrix4x4Multiply(float src1[16], float src2[16], float dest[16]);
	void Matrix4x4Multiply(float4x4 src1, float4x4 src2, float4x4 dest);
	void make16mat(float * out, XMMATRIX in);

	XMFLOAT3 GetColour(float v, float vmin, float vmax);
	void UpdateClass::timeCumulative(double * cumulative, double * mod);
	XMFLOAT3 moveFVect(float distance, XMFLOAT3 location, XMFLOAT3 forw);
	XMFLOAT4 Multiply(XMFLOAT4 quaternion1, XMFLOAT4 quaternion2);
	XMFLOAT4 Multiply(XMFLOAT4 quaternion1, float scaleFactor);
	XMFLOAT4 CreateFromYawPitchRoll(float yaw, float pitch, float roll);

	XMFLOAT3 getColourR2G(float v, float max);
};
