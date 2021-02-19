#pragma once

#include "ObjectUpdateLogic.h"

#include <iostream>
#include <fstream>
#include <ctime>

#define HAND_SPRING_PROPAGATE 0

inline float frand()
{
	return rand() / (float)RAND_MAX;
}

UpdateClass::UpdateClass(HINSTANCE hInstance)
	: RenderDX12(hInstance)
{
	//fxHapDevice.connect(L"COM3"); /////////////To connect Fuji-Xerox haptic device to port COM3///////////
}

//Class Init and Close///////////////////////////////////////////
void UpdateClass::initUpdateClass(float spaceDimXD2_sg, float spaceDimZD2_sg, unsigned int * segCount, int cubeLength, float * averageSegmentsFoundLJP, float * averageSegmentsComputedLJP) {

	SEGMENT_COUNT = *segCount;

	//*segCount += 2; //Increment CPU side too

	OBJECT_COUNT = *segCount;

	CUBE = cubeLength;
	param.FORCE_RANGE = CUBE*1.5f;

	//Bounding area - can be switched to non square objects
	WALLx_D2 = (int)spaceDimXD2_sg;
	WALLz_D2 = (int)spaceDimZD2_sg;
	WALLy_D2 = (int)spaceDimYD2_sg;
	WALLy = (int)spaceDimYD2_sg * 2;

	//Used in the hash operations - TODO change Gridbuffer based on cube size
	CUBE_COUNTx_1D = (int)(((WALLx_D2 * 2) / CUBE) + GRID_BUFFER);
	CUBE_COUNTz_1D = (int)(((WALLz_D2 * 2) / CUBE) + GRID_BUFFER);
	CUBE_COUNTy_1D = (int)(((WALLy_D2 * 2) / CUBE) + GRID_BUFFER);
	CUBE_COUNTx_1D_HALF = CUBE_COUNTx_1D / 2;
	CUBE_COUNTz_1D_HALF = CUBE_COUNTz_1D / 2;
	CUBE_COUNTy_1D_HALF = CUBE_COUNTy_1D / 2;
	GRIDxHALF = CUBE_COUNTx_1D_HALF*CUBE;
	GRIDzHALF = CUBE_COUNTz_1D_HALF*CUBE;
	GRIDyDEPTH_BUFFER = CUBE_COUNTy_1D_HALF*CUBE - WALLy_D2;
	GRIDyELEVATION_BUFFER = CUBE_COUNTy_1D_HALF*CUBE*2.0f - GRIDyDEPTH_BUFFER;
	HASH_ENTRIE = CUBE_COUNTx_1D*CUBE_COUNTz_1D*CUBE_COUNTy_1D;

	avgSegmentsFoundLJP = averageSegmentsFoundLJP;
	avgSegmentsComputedLJP = averageSegmentsComputedLJP;

	//Creates the structure for the hash table used to sort objects by location
	//create_table_CPU(host_segTable_hash, HASH_ENTRIE, OBJECT_COUNT);
	InitRadix(OBJECT_COUNT);
	while (pow(2, bitCountNeeded) < HASH_ENTRIE) {
		bitCountNeeded++;
	}
	if (bitCountNeeded % 2 == 1) {
		bitCountNeeded++;
	}
	mem.allocateMem(&countBit, sizeDa, hostMem, "countBit");
	mem.allocateMem(&countBitCounts, sizeDa, hostMem, "countBitCounts");
	mem.allocateMem(&countBitStarts, sizeDa, hostMem, "countBitStarts");

	//Allocations needed for LJP using my memory allocating code <may differ for other simulations>
	mem.allocateMem(&segLocation.x, OBJECT_COUNT, hostMem, "segLocation.x");
	mem.allocateMem(&segLocation.y, OBJECT_COUNT, hostMem, "segLocation.y");
	mem.allocateMem(&segLocation.z, OBJECT_COUNT, hostMem, "segLocation.z");

	mem.allocateMem(&segLocationSorted.x, OBJECT_COUNT, hostMem, "segLocationSorted.x");
	mem.allocateMem(&segLocationSorted.y, OBJECT_COUNT, hostMem, "segLocationSorted.y");
	mem.allocateMem(&segLocationSorted.z, OBJECT_COUNT, hostMem, "segLocationSorted.z");

	mem.allocateMem(&transVelocity.x, OBJECT_COUNT * 2, hostMem, "transVelocity.x"); //Translational Velocity
	mem.allocateMem(&transVelocity.y, OBJECT_COUNT * 2, hostMem, "transVelocity.y");
	mem.allocateMem(&transVelocity.z, OBJECT_COUNT * 2, hostMem, "transVelocity.z");

	mem.allocateMem(&segColor.x, OBJECT_COUNT, hostMem, "segColor.x");
	mem.allocateMem(&segColor.y, OBJECT_COUNT, hostMem, "segColor.y");
	mem.allocateMem(&segColor.z, OBJECT_COUNT, hostMem, "segColor.z");

	//mem.allocateMem(&directionalForceAccumulator.x, OBJECT_COUNT, hostMem, "directionalForceAccumulator.x"); // Used to mirror GPU thread parallelism on the CPU across 2 loops
	//mem.allocateMem(&directionalForceAccumulator.y, OBJECT_COUNT, hostMem, "directionalForceAccumulator.y");
	//mem.allocateMem(&directionalForceAccumulator.z, OBJECT_COUNT, hostMem, "directionalForceAccumulator.z");

	mem.allocateMem(&segRadius, OBJECT_COUNT, hostMem, "segRadius");
	mem.allocateMem(&segRadiusSorted, OBJECT_COUNT, hostMem, "segRadiusSorted");

	mem.allocateMem(&segTempFactor, OBJECT_COUNT, hostMem, "segTempFactor");

	mem.allocateMem(&forceSave, OBJECT_COUNT, hostMem, "forceSave");	// Used for coloring based on forces acting on a segment
	//mem.allocateMem(&frictionAccumulate, OBJECT_COUNT, hostMem, "frictionAccumulate"); //////////Friction thing////////////////////

	mem.allocateMem(&segOriginalIndex, OBJECT_COUNT, hostMem, "segOriginalIndex"); // Sort Stuff
	mem.allocateMem(&segCellStart, HASH_ENTRIE, hostMem, "segCellStart");
	mem.allocateMem(&segCellEnd, HASH_ENTRIE, hostMem, "segCellEnd");
	mem.allocateMem(&pickingVisited, HASH_ENTRIE, hostMem, "pickingVisited");

	//Spring force////////////////////////////////////////////////////////////////////////
	mem.allocateMem(&springDistance, SEGMENTCOUNT * 100, hostMem, "springDistance");
	mem.allocateMem(&springConstant, SEGMENTCOUNT * 100, hostMem, "springConstant");
	mem.allocateMem(&massID, SEGMENTCOUNT * 100, hostMem, "massID");
	//Spring force////////////////////////////////////////////////////////////////////////

	//Much stronger random than rand()
	seed1 = (unsigned int)std::chrono::system_clock::now().time_since_epoch().count();
	generator.seed(seed1);

	//Initiallizing data for LJP interactions
	float buffer = 10.0f;
	float spreadX = spaceDimXD2_sg - buffer;
	float spreadZ = spaceDimZD2_sg - buffer;

	/*std::uniform_real_distribution<float> uni_placementX(-10, 10);
	std::uniform_real_distribution<float> uni_placementY(2, 22);
	std::uniform_real_distribution<float> uni_placementZ(-10, 10);*/
	std::uniform_real_distribution<float> uni_placementX(-spreadX, spreadX);
	std::uniform_real_distribution<float> uni_placementY(buffer, 30.0f);//spaceDimY_sg - buffer
	std::uniform_real_distribution<float> uni_placementZ(-spreadZ, spreadZ);
	for (UINT i = 0; i < OBJECT_COUNT; i++) {
		if (i < SEGMENT_COUNT) {
			segLocation.x[i] = uni_placementX(generator);
			segLocation.y[i] = uni_placementY(generator);
			segLocation.z[i] = uni_placementZ(generator);
		}
		else {
			segLocation.x[i] = 0;
			segLocation.y[i] = 0;
			segLocation.z[i] = 0;
		}
		segRadius[i] = 0.5f;
		segRadiusSorted[i] = 0.5f;
	}

	float initMotion = 0.05f;
	std::uniform_real_distribution<float> uni_forwardX(-initMotion, initMotion);
	std::uniform_real_distribution<float> uni_forwardY(-initMotion, initMotion);
	std::uniform_real_distribution<float> uni_forwardZ(-initMotion, initMotion);
	for (UINT i = 0; i < OBJECT_COUNT; i++) {
		if (i < SEGMENT_COUNT) {
			/*transVelocity.x[i] = uni_forwardX(generator);
			transVelocity.y[i] = uni_forwardY(generator);
			transVelocity.z[i] = uni_forwardZ(generator);*/
			transVelocity.x[i] = 0.0f;
			transVelocity.y[i] = 0.0f;
			transVelocity.z[i] = 0.0f;
		}
		else {
			transVelocity.x[i] = 0.0f;
			transVelocity.y[i] = 0.0f;
			transVelocity.z[i] = 0.0f;
		}
	}

	std::uniform_real_distribution<float> uni_R(0, 1);
	std::uniform_real_distribution<float> uni_G(0, 1);
	std::uniform_real_distribution<float> uni_B(0, 1);
	for (UINT i = 0; i < OBJECT_COUNT; i++) {
		if (i < SEGMENT_COUNT) {
			/*segColor.x[i] = uni_R(generator);
			segColor.y[i] = uni_G(generator);
			segColor.z[i] = uni_B(generator);*/
			segColor.x[i] = 0.2;
			segColor.y[i] = 0.8;
			segColor.z[i] = 0.2;
		}
		else {
			segColor.x[i] = 0.2;
			segColor.y[i] = 0.8;
			segColor.z[i] = 0.2;
		}
	}

	objMan = new ObjectManager();	//Creating ObjectManajer.
	objMan->init(LEAP_SEG_CNT);		//This is mandatory. See the declaration of the function.

	for (UINT i = 0; i < LEAP_SEG_CNT; i++) {
		segRadius[i] = LS->getRad(i);
	}

	//-----------------------------------------------------------------------------------------------------------
	//OBJECT CREATIONS///////////////////////////////////////////////////////////////////////////////////////////
	//-----------------------------------------------------------------------------------------------------------
/*	createPdbAnchObj(pdbFile, 1.0f, true, XMMATRIX(
				1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				-100.0f, 120.0f, -120.0f, 1.0f));							//This is for creating an object from a .pdb file.
*/

/*	UINT a = 10;
	UINT b = 5;
	unfoldingID = testObj(a, b, XMFLOAT3{ 0.0f, 50.0f, -70.0f });			//This object is the reference for "foldedObjID" object in unfolding state. It should be hidden from the scene.
	foldingID = testObjFolded(a, b, XMFLOAT3{ 0.0f, 50.0f, -50.0f });		//This object is the reference for "foldedObjID" object in folding state. It should be hidden from the scene.
																			//Hide "unfoldingID" and "foldingID" objects away in the scene.
	foldedObjID = testObj(a, b, XMFLOAT3{ -160.0f, 120.0f, -70.0f });		//This is the object that can folded and unfolded.
*/	
//	testJointObj(10, 7, 7, 0, XMFLOAT3{ -60.0f, 120.0f, -70.0f });			//This object has hinge joint.
//	testJointObj(10, 5, 7, 1, XMFLOAT3{ 60.0f, 120.0f, -70.0f });			//This object has rotational joint.
//	testJointObj(10, 7, 3, 2, XMFLOAT3{ 160.0f, 120.0f, -70.0f });			//This object has ball & socket joint.

	objPar2 = ObjectParameters(0.1f, 0.15f, 0.1f, 8.0f, 8.0f, 10, 20, 0.5f, 1, 0.1f);	//This is ObjectManager for filament objects.
	UINT bridge = 0;		//Bridge connection parameter.
	float scale = 20.0f;	//Make MT objects 10 times larger.
	XMFLOAT3 adjustPos = XMFLOAT3(-400.0f, 100.0f, -400.0f);	//Place MT objects 150 units length to the left, 100 units lenght higher, and 200 units lenght closer.
	UINT L2R = 4;
	UINT F2B = 3;
	for (int i = 0; i < L2R; i++) {
		for (int j = 0; j < F2B; j++) {
			createSmoothMuscleMT(XMFLOAT3(10.0f + 7.0f * i, 15.0f + 7.0f * j, 7.0f), XMFLOAT3(0.0f, 0.0f, 0.0f), objPar2, bridge, scale, adjustPos);
		}
	}
	//-----------------------------------------------------------------------------------------------------------

	time_t now = time(0);
	tm *ltime = localtime(&now);

	//-----------------------------------------------------------------------------------------------------------
	//Prepare file name and path for the flexural rigidity measurement (microtubule bending)./////////////////////
	//-----------------------------------------------------------------------------------------------------------
	std::string writeFile = "Results\\";
	writeFile += to_string(1900 + ltime->tm_year) + "-" +
		to_string(1 + ltime->tm_mon) + "-" +
		to_string(ltime->tm_mday) + " " +
		to_string(ltime->tm_hour) + ";" +
		to_string(ltime->tm_min) + ";" +
		to_string(ltime->tm_sec) + ".csv";
	//-----------------------------------------------------------------------------------------------------------

	//-----------------------------------------------------------------------------------------------------------
	//This line of code below is for activating the recording of microtubule bending flexural rigidity measurement.
	//The result is a .csv file stored in ../Result subfolder.
	//-----------------------------------------------------------------------------------------------------------
	//resultFile = ofstream(writeFile);
	/*if (resultFile.is_open())
	testString = "Write OK.";//////test
	else
	testString = "It doesn't works.";//////test*/
	//-----------------------------------------------------------------------------------------------------------


	//-----------------------------------------------------------------------------------------------------------
	//FLEXURAL RIGIDITY MEASUREMENT OF MICROTUBULE/////////////////////////////////////////////////////////////////
	//-----------------------------------------------------------------------------------------------------------
/*	std::string drgType;
	float drgVal;
	if (param.dragType == 0) {
		drgType = "Constant Drag";
		drgVal = param.x0Drag;
	}
	else if (param.dragType == 1) {
		drgType = "Linear Drag";
		drgVal = param.x1Drag;
	}
	else if (param.dragType == 2) {
		drgType = "Quadratic Drag";
		drgVal = param.x2Drag;
	}
	else if (param.dragType == 3) {
		drgType = "Cubic Drag";
		drgVal = param.x3Drag;
	}

	objPar1 = ObjectParameters(0.1f, 0.2f, 0.1f, 8.0f, 10.0f, 10, 5, 1.0f, 52, 0.01f);	//This is ObjectManager for MTmodel 1 to 7.
	objPar2 = ObjectParameters(0.1f, 0.15f, 0.1f, 20.0f, 8.0f, 10, 20, 1.0f, 1, 0.1f);	//This is ObjectManager for MTmodel 11 and 12.
	UINT bridge = 0;

	XMFLOAT3 startMT;
	int MTmodel = 11;
	if (MTmodel < 11) {
		objType = 0;
		startMT = XMFLOAT3{ -5.884615f, 100.0f, 0.0f };
	}
	else {
		objType = 1;
		startMT = XMFLOAT3{ -2.5f, 100.0f, 0.0f };
	}

	switch (MTmodel) {
	case 1:
		longObjID = createMT1(startMT, objPar1);
		fixAnchors = 2;
		resultFile << "MICROTUBULE MODEL=,1 anchor per section""\n\n";
		break;
	case 2:
		longObjID = createMT2(startMT, objPar1);
		fixAnchors = 4;
		resultFile << "MICROTUBULE MODEL=,4 anchors per section without section connection\n\n";
		break;
	case 3:
		longObjID = createMT3(startMT, objPar1);
		fixAnchors = 4;
		resultFile << "MICROTUBULE MODEL=,4 anchors per section_ connect all adjacent section anchors\n\n";
		break;
	case 4:
		longObjID = createMT4(startMT, objPar1);
		fixAnchors = 4;
		resultFile << "MICROTUBULE MODEL=,4 anchors per section_ 3 joint anchors\n\n";
		break;
	case 5:
		longObjID = createMT5(startMT, objPar1);
		fixAnchors = 3;
		resultFile << "MICROTUBULE MODEL=,2x3 anchors per section_ overlapping anchors\n\n";
		break;
	case 6:
		longObjID = createMT6(startMT, objPar1);
		fixAnchors = 4;
		resultFile << "MICROTUBULE MODEL=,4 anchors per section_ 3 joint anchors_ connect adjacent joint anchors\n\n";
		break;
	case 7:
		longObjID = createMT7(startMT, objPar1, 0);
		fixAnchors = 4;
		resultFile << "MICROTUBULE MODEL=,2x4 flat anchors per section_ overlapping anchors_ connect with bridges\n\n";
		break;
	case 11:
		longObjID = createFilamen1(startMT, objPar2, bridge);
		fixAnchors = 4;
		resultFile << "FILAMEN MODEL=,Filament 2x4 flat anchors per section_ overlapping anchors_ connect with bridges\n\n";
		break;
	case 12:
		longObjID = createFilamen2(startMT, objPar2);
		fixAnchors = 4;
		resultFile << "FILAMEN MODEL=,Filament 4 anchors per section_ 3 joint anchors\n\n";
		break;
	default:
		resultFile << "No MT selected\n\n";
		testString = "No MT selected";
	}

	//F = m * a = m * d / t^2 = (upForce)
	//m = mass of the dimers being moved upward
	//d = distance to move in 1 compute cycle
	//t = time per compute cycle = 1 / frame per second
	double segMass = 0.0;
	float xShift = 0.0f;
	if (objType == 0) {
		segMass = 8.302695333e-23;	//This is a segment (tubulin) mass in kilogram (kg).
		segDiameter = 4.0e-9;		//This is a segment/tubulin diameter in meter (m).
		xShift = 0.884615f;			//This is a shift in x-axis for the beginning of the object.

		double m = objPar1.numSegMove * segMass;
		upForce = m * (objPar1.upMove * segDiameter / objPar1.segWidth) * param.FPS * param.FPS;
		
		float MTL = objPar1.numSection * objPar1.sectionLength * objPar1.segWidth * 2 - 1 - (objPar1.sectionLength + (objPar1.numSegMove / 26) - 2) * objPar1.segWidth; //Note: Length of N segments is (N-1)*segmentWidth.
		MTlength = MTL * segDiameter;
		initY = startMT.y;

		resultFile << "Length (L)=," + to_string(MTL) + "," + to_string(MTlength * 1.0e9) + ",nm\n" +
			"Number of section=," + to_string(objPar1.numSection) + "\n" +
			"Section length=," + to_string(objPar1.sectionLength) + ",particles\n" +
			"Particle diameter=," + to_string(objPar1.segWidth) + "," + to_string(segDiameter * 1.0e9) + ",nm\n" +
			"Particle to particle constant=," + to_string(objPar1.segToSegConstant) + "\n" +
			"Anchor to anchor constant=," + to_string(objPar1.anchToAnchConstant) + "\n" +
			"Anchor to particle constant=," + to_string(objPar1.anchToSegConstant) + "\n" +
			"Anchor distance=," + to_string(objPar1.anchorDistance) + ",dist unit\n" +
			"Joint anchor distance=," + to_string(objPar1.jointAnchDistance) + ",dist unit\n" +
			"Drag type=," + drgType + "\n" +
			"Drag value=," + to_string(drgVal) + "\n" +
			"Upward force=," + to_string(upForce * 1.0e26) + ",E-26,N\n" +
			"Raw object parameter=," + to_string(objPar1.segToSegConstant) + "," + to_string(objPar1.anchToAnchConstant) + "," + to_string(objPar1.anchToSegConstant) + "," + to_string(objPar1.anchorDistance) + "," + to_string(objPar1.jointAnchDistance) + "," + to_string(objPar1.numSection) + "," + to_string(objPar1.sectionLength) + "," + to_string(objPar1.segWidth) + "," + to_string(objPar1.numSegMove) + "," + to_string(objPar1.upMove) + "\n" +
			"Bridge=," + to_string(bridge) + "\n\n" +
			"x(s=0)=," + to_string(startMT.x + xShift + objPar1.sectionLength * objPar1.segWidth) +
			",dist unit,,x(s=L)=," + to_string(startMT.x + xShift + objPar1.sectionLength * objPar1.segWidth + MTL) +
			",dist unit\ny(s=0)=," + to_string(startMT.y) +
			",dist unit,,y(s=L)=," + to_string(startMT.y) +
			",dist unit\nz(s=0)=," + to_string(startMT.z) +
			",dist unit,,z(s=L)=," + to_string(startMT.z) +
			",dist unit\n\ni compute cycle,x(s=L),y(s=L),z(s=L),EI(xE-40),x(s=0),y(s=0),z(s=0)\n";
	} else if (objType == 1) {
		segMass = 6.47610235974e-21;	//This is a segment (group of tubulin) mass in kilogram (kg).
		segDiameter = 24.0e-9;			//This is a segment diameter in meter (m).
		xShift = 0.0f;					//This is a shift in x-axis for the beginning of the object.
		UINT numSeg = objPar2.numSection * objPar2.sectionLength;

		double m = objPar2.numSegMove * segMass;
		upForce = m * (objPar2.upMove * segDiameter / objPar2.segWidth) * param.FPS * param.FPS;

		float MTL = (numSeg - 1) * objPar2.segWidth - (objPar2.numSegMove - 1) * objPar2.segWidth / 2.0f; //Note: Length of N segments is (N-1)*segmentWidth.
		MTlength = MTL * segDiameter;
		initY = startMT.y;

		resultFile << "Length (L)=," + to_string(MTL) + "," + to_string(MTlength * 1.0e9) + ",nm\n" +
			"Number of section=," + to_string(objPar2.numSection) + "\n" +
			"Section length=," + to_string(objPar2.sectionLength) + ",particles\n" +
			"Particle diameter=," + to_string(objPar2.segWidth) + "," + to_string(segDiameter * 1.0e9) + ",nm\n" +
			"Particle to particle constant=," + to_string(objPar2.segToSegConstant) + "\n" +
			"Anchor to anchor constant=," + to_string(objPar2.anchToAnchConstant) + "\n" +
			"Anchor to particle constant=," + to_string(objPar2.anchToSegConstant) + "\n" +
			"Anchor distance=," + to_string(objPar2.anchorDistance) + ",dist unit\n" +
			"Joint anchor distance=," + to_string(objPar2.jointAnchDistance) + ",dist unit\n" +
			"Drag type=," + drgType + "\n" +
			"Drag value=," + to_string(drgVal) + "\n" +
			"Upward force=," + to_string(upForce * 1.0e26) + ",E-26,N\n" +
			"Raw object parameter=," + to_string(objPar2.segToSegConstant) + "," + to_string(objPar2.anchToAnchConstant) + "," + to_string(objPar2.anchToSegConstant) + "," + to_string(objPar2.anchorDistance) + "," + to_string(objPar2.jointAnchDistance) + "," + to_string(objPar2.numSection) + "," + to_string(objPar2.sectionLength) + "," + to_string(objPar2.segWidth) + "," + to_string(objPar2.numSegMove) + "," + to_string(objPar2.upMove) + "\n" +
			"Bridge=," + to_string(bridge) + "\n\n" +
			"x(s=0)=," + to_string(startMT.x + xShift + objPar2.sectionLength * objPar2.segWidth / 2.0f) +
			",dist unit,,x(s=L)=," + to_string(startMT.x + xShift + objPar2.sectionLength * objPar2.segWidth / 2.0f + MTL) +
			",dist unit\ny(s=0)=," + to_string(startMT.y) +
			",dist unit,,y(s=L)=," + to_string(startMT.y) +
			",dist unit\nz(s=0)=," + to_string(startMT.z) +
			",dist unit,,z(s=L)=," + to_string(startMT.z) +
			",dist unit\n\ndist unit\n\ni compute cycle";

		for (UINT i = 0; i < numSeg; i++) {
			resultFile << ",y(" + to_string(i) + ")";
		}
		resultFile << "\n";
	}
		
	testString = "upForce=" + to_string(upForce * 1.0e26) + "x1.0E-26 N";//////test
*/	//-----------------------------------------------------------------------------------------------------------


	//-----------------------------------------------------------------------------------------------------------
	//Allocate memory for Brownian motion random buffer.
	//-----------------------------------------------------------------------------------------------------------
	brownian1perN = 50; //Randomize 1 segment per this value to apply Brownian motion.
	numBrownianSeg = (objMan->getObjSegStart(objMan->getNumObj() - 1) + objMan->getObjSegCount(objMan->getNumObj() - 1)) / brownian1perN;
	brownianSegmentID = new UINT[numBrownianSeg];
	brownianSegmentX = new float[numBrownianSeg];
	brownianSegmentY = new float[numBrownianSeg];
	brownianSegmentZ = new float[numBrownianSeg];
	//-----------------------------------------------------------------------------------------------------------

	objMan->initAnch2Seg();	//This is mandatory. See the declaration of the function.
}

void UpdateClass::closeUpdateLogic() {

	//Clears table
	//free_table_CPU(host_segTable_hash);
	CloseRadix();
	//Deletes all memory allocated made using my memory allocation function
	mem.deleteAllAllocations();
}

void UpdateClass::computeSpringObject() {
	int seg;
	int end = objMan->getObjSegCount(objMan->getNumObj() - 1) + objMan->getObjSegStart(objMan->getNumObj() - 1);
#pragma omp parallel for
	for (seg = objMan->getObjSegStart(1); seg < end; ++seg)
	{
		XMFLOAT3 forceAcc = XMFLOAT3(0.0f, 0.0f, 0.0f);
		float forceColor = 0.0f;
		int foundCnt = 0;
		while (massID[seg * MAXSPRING + foundCnt] != UINT_MAX)
		{
			UINT target = massID[seg * MAXSPRING + foundCnt];
			XMFLOAT3 vec = XMFLOAT3(segLocation.x[target] - segLocation.x[seg], segLocation.y[target] - segLocation.y[seg], segLocation.z[target] - segLocation.z[seg]);
			float dist = sqrtf(dot(vec));
			float force = (dist - springDistance[seg * MAXSPRING + foundCnt]) * springConstant[seg * MAXSPRING + foundCnt];
			forceColor += force;
			forceAcc = add(forceAcc, multiply(force, normalize(vec)));
			foundCnt++;
		}
		transVelocity.x[seg] += forceAcc.x;
		transVelocity.y[seg] += forceAcc.y;
		transVelocity.z[seg] += forceAcc.z;
		/*XMFLOAT3 colorSpr = GetColour(sqrtf(dot(forceAcc)), 0.0f, 0.005f);
		segColor.x[seg] = colorSpr.x;
		segColor.y[seg] = colorSpr.y;
		segColor.z[seg] = colorSpr.z;*/
		XMFLOAT3 colorSpr = getColourR2G(forceColor, colorRange);
		segColor.x[seg] = colorSpr.x;
		segColor.y[seg] = colorSpr.y;
		segColor.z[seg] = colorSpr.z;
	}
}

void UpdateClass::computeAnchorSegment() {
#pragma omp parallel for
	for (int i = 0; i < objMan->getNumAnchor(); i++) {
		XMFLOAT3 anch = objMan->getAnchorPos(i);
		XMFLOAT3 forceAcc = XMFLOAT3(0.0f, 0.0f, 0.0f);
		XMFLOAT3 anchForce = XMFLOAT3(0.0f, 0.0f, 0.0f); //For devided spring constant.
		UINT numConSeg = objMan->getNumConSeg(i);

		float anchRatio = (1.0f / numConSeg);

		for (size_t j = 0; j < numConSeg; j++) {
			UINT seg = objMan->getConSegID(i, j);
			XMFLOAT3 vec = XMFLOAT3(segLocation.x[seg] - anch.x, segLocation.y[seg] - anch.y, segLocation.z[seg] - anch.z);
			float dist = sqrtf(dot(vec));
			float force = (dist - objMan->getConSegDistance(i, j));
			XMFLOAT3 forceVec = multiply(force * objMan->getConSegConst(i, j), normalize(vec));
			forceAcc = add(forceAcc, forceVec);

			//For devided spring constant.
			XMFLOAT3 anchVec = multiply(force * anchRatio, normalize(vec));
			anchForce = add(anchForce, anchVec);

			transVelocity.x[seg] -= forceVec.x;
			transVelocity.y[seg] -= forceVec.y;
			transVelocity.z[seg] -= forceVec.z;
		}

		if ((i + 1) > fixAnchors) //To bind one of the MT's end
		objMan->addAnchorVelo(i, anchForce);
	}
}

void UpdateClass::computeAnchors() {
#pragma omp parallel for
	for (int i = 0; i < objMan->getNumAnchor(); i++) {
		XMFLOAT3 anch = objMan->getAnchorPos(i);
		UINT maxAnchSet = objMan->getMaxAnchor2Anchor();
		XMFLOAT3 forceAcc = XMFLOAT3(0.0f, 0.0f, 0.0f);
		
		for (int j = 0; j < maxAnchSet; j++) {
			if (objMan->getAnchor2AnchorID(i, j) == UINT_MAX) break;
			else {
				XMFLOAT3 targetAnch = objMan->getAnchorPos(objMan->getAnchor2AnchorID(i, j));
				XMFLOAT3 vec = XMFLOAT3(targetAnch.x - anch.x, targetAnch.y - anch.y, targetAnch.z - anch.z);
				float dist = sqrtf(dot(vec));
				float force = (dist - objMan->getAnchor2AnchorDist(i, j)) * objMan->getAnchor2AnchorConst(i, j);
				forceAcc = add(forceAcc, multiply(force, normalize(vec)));
			}
		}
		
		if ((i + 1) > fixAnchors) //To bind one of the MT's end.
		objMan->addAnchorVelo(i, forceAcc);
	}
}

void UpdateClass::moveAnchors() {
#pragma omp parallel for
	for (int i = 0; i < objMan->getNumAnchor(); i++) {
		objMan->addAnchorVelo2Pos(i);
	}
}

void UpdateClass::randomizeBrownian(float magnitude) {
	std::uniform_int_distribution<int> randID(0, brownian1perN - 1);
	std::uniform_real_distribution<float> brownMotX(-magnitude, magnitude);
	std::uniform_real_distribution<float> brownMotY(-magnitude, magnitude);
	std::uniform_real_distribution<float> brownMotZ(-magnitude, magnitude);

	//testString = "objMan->brownianSegmentID=";//////test
	for (int randSeg = 0; randSeg < numBrownianSeg; randSeg++) {
		brownianSegmentID[randSeg] = randID(generator) + (randSeg * brownian1perN) + objMan->getObjSegStart(1);
		brownianSegmentX[randSeg] = brownMotX(generator);
		brownianSegmentY[randSeg] = brownMotY(generator);
		brownianSegmentZ[randSeg] = brownMotZ(generator);

		//testString += to_string(brownianSegmentID[randSeg]) + ", ";//////test
	}	
}

void UpdateClass::useBrownianMotion(UINT maxBrownianRadius) {
#pragma omp parallel for
	for (int brownIdx = 0; brownIdx < numBrownianSeg; ++brownIdx) {
		transVelocity.x[brownianSegmentID[brownIdx]] += brownianSegmentX[brownIdx];
		transVelocity.y[brownianSegmentID[brownIdx]] += brownianSegmentY[brownIdx];
		transVelocity.z[brownianSegmentID[brownIdx]] += brownianSegmentZ[brownIdx];

		std::uniform_int_distribution<int> randBrownRad(1, maxBrownianRadius);
		UINT neighbour = randBrownRad(generator);
		for (UINT i = 1; i < neighbour; i++) {
			//float forX2 = (i / (neighbour + 1));
			float multiplier = 1.0f;//(1 - (forX2 * forX2));
			transVelocity.x[brownianSegmentID[brownIdx] + i] += brownianSegmentX[brownIdx] * multiplier;
			transVelocity.y[brownianSegmentID[brownIdx] + i] += brownianSegmentY[brownIdx] * multiplier;
			transVelocity.z[brownianSegmentID[brownIdx] + i] += brownianSegmentZ[brownIdx] * multiplier;
			transVelocity.x[brownianSegmentID[brownIdx] - i] += brownianSegmentX[brownIdx] * multiplier;
			transVelocity.y[brownianSegmentID[brownIdx] - i] += brownianSegmentY[brownIdx] * multiplier;
			transVelocity.z[brownianSegmentID[brownIdx] - i] += brownianSegmentZ[brownIdx] * multiplier;
		}
	}
}

XMFLOAT3* UpdateClass::createPdbAnchObj(PDBreader::FileReader* pdb, float scale, bool isAnchored, XMMATRIX transform) {
	UINT objID = objMan->getNumObj();
	UINT numAtoms = pdb->atoms.getNumAtoms();
	objMan->addObject(numAtoms, 6);
	XMFLOAT3* segArray = new XMFLOAT3[numAtoms];
	XMMATRIX segArr = XMMatrixIdentity();
	UINT startSegment = objMan->getObjSegStart(objID);
	UINT startAnchor = objMan->getObjAnchorStart(objID);
	float s2sConst = 0.09f;
	float a2aConst = 0.1f;
	float a2sConst = 0.25f;

	int i, j;
	for (i = 0; i < numAtoms; i++) {
		segRadius[i + startSegment] = atmRad.getRadius(pdb->atoms.atom[i].element) * 7 * scale;
		segArr = XMMATRIX(
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			pdb->atoms.atom[i].x * scale, pdb->atoms.atom[i].y * scale, -pdb->atoms.atom[i].z * scale, 1.0f);

		segArr *= transform;

		segArray[i].x = XMVectorGetX(segArr.r[3]);
		segArray[i].y = XMVectorGetY(segArr.r[3]);
		segArray[i].z = XMVectorGetZ(segArr.r[3]);
		segTempFactor[i + startSegment] = pdb->atoms.atom[i].tempFactor;

		segLocation.x[i + startSegment] = segArray[i].x;
		segLocation.y[i + startSegment] = segArray[i].y;
		segLocation.z[i + startSegment] = segArray[i].z;
	}

	initSpringCompound(startSegment, startSegment + numAtoms, 4, s2sConst);

	XMFLOAT3 center = centerSpringMass(startSegment, startSegment + numAtoms - 1);
	//objMan->set4Anchors(objID, 0, center, 50.0f, false, a2aConst);
	objMan->set6Anchors(objID, 0, center, 50.0f, a2aConst);
	//Make springs between segments and anchors
	if (isAnchored)
	for (UINT i = startAnchor; i < startAnchor + 6; i++) {
		XMFLOAT3 anch = objMan->getAnchorPos(i);
		for (UINT j = startSegment; j < startSegment + numAtoms; j++) {
			XMFLOAT3 vec = XMFLOAT3(anch.x - segLocation.x[j], anch.y - segLocation.y[j], anch.z - segLocation.z[j]);
			float dist = sqrtf(dot(vec));
			objMan->addAnchor2Segment(i, j, dist, a2sConst);
		}
	}

	return segArray;
}

void UpdateClass::initSpringCompound(UINT startSeg, UINT endSeg, float distance, float springC) {
	float avgFound = 0;
	for (int seg = startSeg; seg < endSeg; ++seg)
	{
		int foundCnt = 0;
		for (int search = startSeg; search < endSeg; ++search)
		{
			if (seg != search) {
				XMFLOAT3 vec = XMFLOAT3(segLocation.x[search] - segLocation.x[seg], segLocation.y[search] - segLocation.y[seg], segLocation.z[search] - segLocation.z[seg]);
				float dist = sqrtf(dot(vec));
				if (dist < distance) {
					springDistance[seg * MAXSPRING + foundCnt] = dist;
					springConstant[seg * MAXSPRING + foundCnt] = springC;
					massID[seg * MAXSPRING + foundCnt] = search;
					foundCnt++;
				}
			}
			if (foundCnt >= MAXSPRING - 1)
				break;
		}
		for (int extra = foundCnt; extra < MAXSPRING; ++extra)
		{
			springDistance[seg * MAXSPRING + extra] = FLT_MAX;
			springConstant[seg * MAXSPRING + extra] = FLT_MAX;
			massID[seg * MAXSPRING + extra] = UINT_MAX;
		}
		avgFound += foundCnt;
	}
}

int UpdateClass::springConnect(UINT segment1ID, UINT segment2ID, float stringC) {
	XMFLOAT3 vec = XMFLOAT3(segLocation.x[segment1ID] - segLocation.x[segment2ID], segLocation.y[segment1ID] - segLocation.y[segment2ID], segLocation.z[segment1ID] - segLocation.z[segment2ID]);
	float dist = sqrtf(dot(vec));

	bool isConnected = false;
	for (int i = 0; i < MAXSPRING; i++) {
		if (massID[segment1ID * MAXSPRING + i] == UINT_MAX) {
			springDistance[segment1ID * MAXSPRING + i] = dist;
			springConstant[segment1ID * MAXSPRING + i] = stringC;
			massID[segment1ID * MAXSPRING + i] = segment2ID;
			isConnected = true;
			break;
		}
	}
	if (!isConnected) return -1;

	isConnected = false;
	for (int i = 0; i < MAXSPRING; i++) {
		if (massID[segment2ID * MAXSPRING + i] == UINT_MAX) {
			springDistance[segment2ID * MAXSPRING + i] = dist;
			springConstant[segment2ID * MAXSPRING + i] = stringC;
			massID[segment2ID * MAXSPRING + i] = segment1ID;
			isConnected = true;
			break;
		}
	}
	if (!isConnected) return -2;
	return 0;
}

bool UpdateClass::springHalfConnect(UINT segment1ID, UINT segment2ID, float stringC) {
	XMFLOAT3 vec = XMFLOAT3(segLocation.x[segment1ID] - segLocation.x[segment2ID], segLocation.y[segment1ID] - segLocation.y[segment2ID], segLocation.z[segment1ID] - segLocation.z[segment2ID]);
	float dist = sqrtf(dot(vec));

	for (int i = 0; i < MAXSPRING; i++) {
		if (massID[segment1ID * MAXSPRING + i] == UINT_MAX) {
			springDistance[segment1ID * MAXSPRING + i] = dist;
			springConstant[segment1ID * MAXSPRING + i] = stringC;
			massID[segment1ID * MAXSPRING + i] = segment2ID;
			return true;
		}
	}
	return false;
}

XMFLOAT3 UpdateClass::centerSpringMass(UINT startSeg, UINT endSeg) {
	XMFLOAT3 center;
	XMFLOAT3 temp;
	int numSeg = (endSeg - startSeg) + 1;
	
	temp.x = 0.0f;
	temp.y = 0.0f;
	temp.z = 0.0f;

	for (int i = startSeg; i <= endSeg; i++) {
		temp.x = temp.x + segLocation.x[i];
		temp.y = temp.y + segLocation.y[i];
		temp.z = temp.z + segLocation.z[i];
	}

	center.x = temp.x / numSeg;
	center.y = temp.y / numSeg;
	center.z = temp.z / numSeg;

	return center;
}

XMFLOAT3 UpdateClass::centerSpringMass(XMFLOAT3* segmentArray, UINT startSeg, UINT endSeg) {
	XMFLOAT3 center;
	XMFLOAT3 temp;
	int numSeg = (endSeg - startSeg) + 1;

	temp.x = 0.0f;
	temp.y = 0.0f;
	temp.z = 0.0f;

	for (int i = startSeg; i <= endSeg; i++) {
		temp.x = temp.x + segmentArray[i].x;
		temp.y = temp.y + segmentArray[i].y;
		temp.z = temp.z + segmentArray[i].z;
	}

	center.x = temp.x / numSeg;
	center.y = temp.y / numSeg;
	center.z = temp.z / numSeg;

	return center;
}

UINT UpdateClass::testObj(UINT length, UINT width, XMFLOAT3 origin) {
	UINT objID = objMan->addObject(length * width, 4);
	UINT startSegment = objMan->getObjSegStart(objID);
	UINT startAnchor = objMan->getObjAnchorStart(objID);
	UINT numSeg = objMan->getObjSegCount(objID);
	
	for (UINT i = 0; i < width; i++) {
		for (UINT j = 0; j < length; j++) {
			segRadius[i * length + j + startSegment] = 1.0f;
			segLocation.x[i * length + j + startSegment] = 2.0f * segRadius[i * length + j + startSegment] * j + origin.x;
			segLocation.y[i * length + j + startSegment] = 0.0f + origin.y;
			segLocation.z[i * length + j + startSegment] = 2.0f * segRadius[i * length + j + startSegment] * i + origin.z;
		}
	}

	XMFLOAT3 center = centerSpringMass(startSegment, startSegment + numSeg - 1);

	//Prepare the springs
	for (UINT i = startSegment * MAXSPRING; i < (startSegment + numSeg) * MAXSPRING; i++) {
		springDistance[i] = FLT_MAX;
		springConstant[i] = FLT_MAX;
		massID[i] = UINT_MAX;
	}

	objMan->set4Anchors(objID, 0, center, 10.0f, false, 0.1f);

	float isc = 0.3f; //This is spring constant value for the inner springs (segment to segment).
	float a2sConstant = 0.1f; //This is spring constant value between an anchor and a segment.

	//Make springs for the square
	for (UINT i = 0; i < width - 1; i++) {
		for (UINT j = 0; j < length - 1; j++) {
			springConnect(i * length + j + startSegment, i * length + j + startSegment + 1, isc);
			springConnect(i * length + j + startSegment, (i + 1) * length + j + startSegment, isc);
		}
		springConnect((i + 1) * length - 1 + startSegment, (i + 2) * length - 1 + startSegment, isc);
	}
	for (UINT j = 0; j < length - 1; j++) {
		springConnect((width - 1) * length + j + startSegment, (width - 1) * length + j + startSegment + 1, isc);
	}

	//Make springs between segments and anchors
	for (UINT i = startAnchor; i < startAnchor + 4; i++) {
		XMFLOAT3 anch = objMan->getAnchorPos(i);
		for (UINT j = startSegment; j < startSegment + numSeg; j++) {
			XMFLOAT3 vec = XMFLOAT3(anch.x - segLocation.x[j], anch.y - segLocation.y[j], anch.z - segLocation.z[j]);
			float dist = sqrtf(dot(vec));
			objMan->addAnchor2Segment(i, j, dist, a2sConstant);
		}
	}

	return objID;
}

UINT UpdateClass::testObjFolded(UINT length, UINT width, XMFLOAT3 origin) {
	UINT objID = objMan->addObject(length * width, 4);
	UINT startSegment = objMan->getObjSegStart(objID);
	UINT startAnchor = objMan->getObjAnchorStart(objID);
	UINT numSeg = objMan->getObjSegCount(objID);

	for (UINT i = 0; i < width; i++) {
		for (UINT j = 0; j < length / 2; j++) {
			segRadius[i * length + j + startSegment] = 1.0f;
			segLocation.x[i * length + j + startSegment] = 2.0f * segRadius[i * length + j + startSegment] * j + origin.x;
			segLocation.y[i * length + j + startSegment] = 0.0f + origin.y;
			segLocation.z[i * length + j + startSegment] = 2.0f * segRadius[i * length + j + startSegment] * i + origin.z;
		}
		for (UINT j = length / 2; j < length; j++) {
			segRadius[i * length + j + startSegment] = 1.0f;
			segLocation.x[i * length + j + startSegment] = 2.0f * segRadius[i * length + j + startSegment] * length / 2 + origin.x;
			segLocation.y[i * length + j + startSegment] = 2.0f * segRadius[i * length + j + startSegment] * (j - length / 2) + origin.y;
			segLocation.z[i * length + j + startSegment] = 2.0f * segRadius[i * length + j + startSegment] * i + origin.z;
		}
	}

	XMFLOAT3 center = centerSpringMass(startSegment, startSegment + numSeg - 1);

	//Prepare the springs
	for (UINT i = startSegment * MAXSPRING; i < (startSegment + numSeg) * MAXSPRING; i++) {
		springDistance[i] = FLT_MAX;
		springConstant[i] = FLT_MAX;
		massID[i] = UINT_MAX;
	}

	objMan->set4Anchors(objID, 0, center, 10.0f, false, 0.1f);

	float isc = 0.3f; //This is spring constant value for the inner springs (segment to segment).
	float a2sConstant = 0.1f; //This is spring constant value between an anchor and a segment.

							  //Make springs for the square
	for (UINT i = 0; i < width - 1; i++) {
		for (UINT j = 0; j < length - 1; j++) {
			springConnect(i * length + j + startSegment, i * length + j + startSegment + 1, isc);
			springConnect(i * length + j + startSegment, (i + 1) * length + j + startSegment, isc);
		}
		springConnect((i + 1) * length - 1 + startSegment, (i + 2) * length - 1 + startSegment, isc);
	}
	for (UINT j = 0; j < length - 1; j++) {
		springConnect((width - 1) * length + j + startSegment, (width - 1) * length + j + startSegment + 1, isc);
	}

	//Make springs between segments and anchors
	for (UINT i = startAnchor; i < startAnchor + 4; i++) {
		XMFLOAT3 anch = objMan->getAnchorPos(i);
		for (UINT j = startSegment; j < startSegment + numSeg; j++) {
			XMFLOAT3 vec = XMFLOAT3(anch.x - segLocation.x[j], anch.y - segLocation.y[j], anch.z - segLocation.z[j]);
			float dist = sqrtf(dot(vec));
			objMan->addAnchor2Segment(i, j, dist, a2sConstant);
		}
	}

	return objID;
}

UINT UpdateClass::testObjLong(UINT length, UINT width, UINT section, XMFLOAT3 origin) {
	UINT numSeg = length * width * section;
	UINT numSegPerSec = length * width;
	UINT objID = objMan->addObject(numSeg, section * 4);
	UINT startSegment = objMan->getObjSegStart(objID);
	UINT startAnchor = objMan->getObjAnchorStart(objID);
	XMFLOAT3 *center = new XMFLOAT3[section];

	for (UINT s = 0; s < section; s++) {
		for (UINT i = 0; i < width; i++) {
			for (UINT j = 0; j < length; j++) {
				segLocation.x[i * length + j + startSegment + s * numSegPerSec] = 2.0f * j + origin.x + s * 2.f * length;
				segLocation.y[i * length + j + startSegment + s * numSegPerSec] = 0.0f + origin.y;
				segLocation.z[i * length + j + startSegment + s * numSegPerSec] = 2.0f * i + origin.z;
			}
		}
		center[s] = centerSpringMass(startSegment + s * numSegPerSec, startSegment + (s + 1) * numSegPerSec - 1);
	}

	//Prepare the springs
	for (UINT i = startSegment * MAXSPRING; i < (startSegment + numSeg) * MAXSPRING; i++) {
		springDistance[i] = FLT_MAX;
		springConstant[i] = FLT_MAX;
		massID[i] = UINT_MAX;
	}

	//Set anchors
	/*for (int i = 0; i < section; i++) {
		objMan->set4Anchors(objID, i, center[i], 7.5f, 0.2f);
	}*/
	objMan->set4Anchors(objID, 0, center[0], 15.0f, false, 0.2f);
	objMan->set4Anchors(objID, 1, center[1], 7.5f, false, 0.2f);
	objMan->set4Anchors(objID, 2, center[2], 7.5f, false, 0.2f);
	objMan->set4Anchors(objID, 3, center[3], 7.5f, false, 0.2f);
	objMan->set4Anchors(objID, 4, center[4], 7.5f, false, 0.2f);
	
	float isc = 0.5f; //This is spring constant value for the inner springs (segment to segment).
	float a2sConstant = 0.2f; //This is spring constant value between an anchor and a segment.

	//Make springs for the square
	for (UINT s = 0; s < section; s++) {
		for (UINT i = 0; i < width - 1; i++) {
			for (UINT j = 0; j < length - 1; j++) {
				springConnect(i * length + j + s * numSegPerSec + startSegment, i * length + j + s * numSegPerSec + startSegment + 1, isc);
				springConnect(i * length + j + s * numSegPerSec + startSegment, (i + 1) * length + j + s * numSegPerSec + startSegment, isc);
			}
			springConnect((i + 1) * length - 1 + s * numSegPerSec + startSegment, (i + 2) * length - 1 + s * numSegPerSec + startSegment, isc);
		}
		for (UINT j = 0; j < length - 1; j++) {
			springConnect((width - 1) * length + j + s * numSegPerSec + startSegment, (width - 1) * length + j + s * numSegPerSec + startSegment + 1, isc);
		}
	}
	for (UINT s = 0; s < section - 1; s++) {
		for (UINT i = 0; i < width; i++) {
			springConnect((length - 1) + i * length + s * numSegPerSec + startSegment, i * length + (s + 1) * numSegPerSec + startSegment, isc);
		}
	}

	//Make springs between segments and anchors
	for (UINT s = 0; s < section; s++) {
		for (UINT i = startAnchor + s * 4; i < startAnchor + s * 4 + 4; i++) {
			XMFLOAT3 anch = objMan->getAnchorPos(i);
			for (UINT j = startSegment + s * numSegPerSec; j < startSegment + (s + 1) * numSegPerSec; j++) {
				XMFLOAT3 vec = XMFLOAT3(anch.x - segLocation.x[j], anch.y - segLocation.y[j], anch.z - segLocation.z[j]);
				float dist = sqrtf(dot(vec));
				objMan->addAnchor2Segment(i, j, dist, a2sConstant);
			}
		}
	}

	return objID;
}

UINT UpdateClass::testJointObj(UINT length1, UINT length2, UINT width, UINT jointType, XMFLOAT3 origin) {
	UINT objID;
	if (jointType == 0) {	//Object with hinge joint.
		objID = objMan->addObject((length1 + length2) * width, 10);
	} if (jointType == 1) {	//Object with rotational joint.
		objID = objMan->addObject((length1 + length2) * width, 10);
	} if (jointType == 2) {	//Object with ball & socket joint.
		objID = objMan->addObject((length1 + length2) * width, 9);
	}

	UINT startSegment = objMan->getObjSegStart(objID);
	UINT startAnchor = objMan->getObjAnchorStart(objID);
	UINT numSeg = objMan->getObjSegCount(objID);
	UINT numSeg1 = length1 * width;
	float s2sconst = 0.3f;	//This is segment-to-segment spring constant value.
	float a2aconst = 0.1f;	//This is anchor-to-anchor spring constant value.
	float a2sconst = 0.1f;	//This is anchor-to-segment spring constant value.

	//Arrange segments.
	for (UINT i = 0; i < length1; i++) {
		for (UINT j = 0; j < width; j++) {
			segRadius[i * width + j + startSegment] = 1.0f;
			segLocation.x[i * width + j + startSegment] = 2.0f * segRadius[i * width + j + startSegment] * i + origin.x;
			segLocation.y[i * width + j + startSegment] = 0.0f + origin.y;
			segLocation.z[i * width + j + startSegment] = 2.0f * segRadius[i * width + j + startSegment] * j + origin.z;
		}
	}
	for (UINT i = 0; i < length2; i++) {
		for (UINT j = 0; j < width; j++) {
			segRadius[i * width + j + startSegment + numSeg1] = 1.0f;
			segLocation.x[i * width + j + startSegment + numSeg1] = 2.0f * segRadius[i * width + j + startSegment + numSeg1] * (i + length1) + origin.x;
			segLocation.y[i * width + j + startSegment + numSeg1] = 0.0f + origin.y;
			segLocation.z[i * width + j + startSegment + numSeg1] = 2.0f * segRadius[i * width + j + startSegment + numSeg1] * j + origin.z;
		}
	}

	//Define center of the section.
	XMFLOAT3 center1 = centerSpringMass(startSegment, startSegment + numSeg1 - 1);
	XMFLOAT3 center2 = centerSpringMass(startSegment + numSeg1, startSegment + numSeg - 1);

	//Prepare the springs
	for (UINT i = startSegment * MAXSPRING; i < (startSegment + numSeg) * MAXSPRING; i++) {
		springDistance[i] = FLT_MAX;
		springConstant[i] = FLT_MAX;
		massID[i] = UINT_MAX;
	}

	//Arrange anchors.
	objMan->set4Anchors(objID, 0, center1, 5.0f, false, a2aconst);
	objMan->set4Anchors(objID, 1, center2, 5.0f, true, a2aconst);

	//Create segment-to-segment springs.
	for (UINT i = 0; i < length1 - 1; i++) {	//Segment1
		for (UINT j = 0; j < width - 1; j++) {
			springConnect(i * width + j + startSegment, i * width + j + startSegment + 1, s2sconst);
			springConnect(i * width + j + startSegment, (i + 1) * width + j + startSegment, s2sconst);
		}
		springConnect((i + 1) * width - 1 + startSegment, (i + 2) * width - 1 + startSegment, s2sconst);
	}
	for (UINT j = 0; j < width - 1; j++) {
		springConnect((length1 - 1) * width + j + startSegment, (length1 - 1) * width + j + startSegment + 1, s2sconst);
	}
	for (UINT i = 0; i < length2 - 1; i++) {	//Segment2
		for (UINT j = 0; j < width - 1; j++) {
			springConnect(i * width + j + startSegment + numSeg1, i * width + j + startSegment + numSeg1 + 1, s2sconst);
			springConnect(i * width + j + startSegment + numSeg1, (i + 1) * width + j + startSegment + numSeg1, s2sconst);
		}
		springConnect((i + 1) * width - 1 + startSegment + numSeg1, (i + 2) * width - 1 + startSegment + numSeg1, s2sconst);
	}
	for (UINT j = 0; j < width - 1; j++) {
		springConnect((length2 - 1) * width + j + startSegment + numSeg1, (length2 - 1) * width + j + startSegment + numSeg1 + 1, s2sconst);
	}

	//Create anchor-to-segment springs.
	for (UINT i = startAnchor; i < startAnchor + 4; i++) {
		XMFLOAT3 anch = objMan->getAnchorPos(i);	//Segment1
		for (UINT j = startSegment; j < startSegment + numSeg1; j++) {
			XMFLOAT3 vec = XMFLOAT3(anch.x - segLocation.x[j], anch.y - segLocation.y[j], anch.z - segLocation.z[j]);
			float dist = sqrtf(dot(vec));
			objMan->addAnchor2Segment(i, j, dist, a2sconst);
		}
		anch = objMan->getAnchorPos(i + 4);	//Segment2
		for (UINT j = startSegment + numSeg1; j < startSegment + numSeg; j++) {
			XMFLOAT3 vec = XMFLOAT3(anch.x - segLocation.x[j], anch.y - segLocation.y[j], anch.z - segLocation.z[j]);
			float dist = sqrtf(dot(vec));
			objMan->addAnchor2Segment(i + 4, j, dist, a2sconst);
		}
	}

	//Add joint anchors.
	XMFLOAT3 centerJoint = XMFLOAT3(center1.x + (float)length1 * segRadius[startSegment], center1.y, center1.z);	//segRadius[startSegment] = 2.0f * segRadius[startSegment] / 2.0f
	if (jointType == 0) {
		XMFLOAT3 HingeAnchor1 = XMFLOAT3(centerJoint.x, centerJoint.y, centerJoint.z + (float)width * segRadius[startSegment]);	//segRadius[startSegment] = 2.0f * segRadius[startSegment] / 2.0f
		XMFLOAT3 HingeAnchor2 = XMFLOAT3(centerJoint.x, centerJoint.y, centerJoint.z - (float)width * segRadius[startSegment]);	//segRadius[startSegment] = 2.0f * segRadius[startSegment] / 2.0f
		objMan->setAnchorPos(objID, 8, HingeAnchor1);
		objMan->setAnchorPos(objID, 9, HingeAnchor2);

		objMan->setAnchor2Anchor(startAnchor + 8, startAnchor + 9, a2aconst);
		for (int i = startAnchor; i < startAnchor + 8; i++) {
			objMan->setAnchor2Anchor(i, startAnchor + 8, a2aconst);
			objMan->setAnchor2Anchor(i, startAnchor + 9, a2aconst);
		}
	} if (jointType == 1) {
		objMan->setAnchorPos(objID, 8, center1);
		objMan->setAnchorPos(objID, 9, center2);

		objMan->setAnchor2Anchor(startAnchor + 8, startAnchor + 9, a2aconst);
		for (int i = startAnchor; i < startAnchor + 8; i++) {
			objMan->setAnchor2Anchor(i, startAnchor + 8, a2aconst);
			objMan->setAnchor2Anchor(i, startAnchor + 9, a2aconst);
		}
	} if (jointType == 2) {
		objMan->setAnchorPos(objID, 8, centerJoint);

		for (int i = startAnchor; i < startAnchor + 8; i++) {
			objMan->setAnchor2Anchor(i, startAnchor + 8, a2aconst);
		}
	}

	return objID;
}

UINT UpdateClass::createMT1(XMFLOAT3 origin, ObjectParameters objParam) {
	UINT numSection = objParam.numSection;
	UINT length = objParam.sectionLength * numSection;
	UINT objID = objMan->addObject(length * 26, numSection);
	UINT startSeg = objMan->getObjSegStart(objID);
	UINT startAnchor = objMan->getObjAnchorStart(objID);
	UINT numSeg = objMan->getObjSegCount(objID); 
	XMFLOAT3 *center = new XMFLOAT3[numSection];

	float s2sConst = objParam.segToSegConstant;
	float a2aConst = objParam.anchToAnchConstant; 
	float a2sConst = objParam.anchToSegConstant;

	//Prepare the springs
	for (UINT i = startSeg * MAXSPRING; i < (startSeg + numSeg) * MAXSPRING; i++) {
		springDistance[i] = FLT_MAX;
		springConstant[i] = FLT_MAX;
		massID[i] = UINT_MAX;
	}

	arrangeMT(startSeg, numSeg, length, 2 * objParam.segWidth, objParam.segWidth, s2sConst, origin);

	//Set center of each section.
	UINT numSegPerSec = numSeg / numSection;
	for (UINT i = 0; i < numSection; i++) {
		center[i] = centerSpringMass(startSeg + i * numSegPerSec, startSeg + ((i + 1) * numSegPerSec) - 1);
	}

	//Set anchors.
	for (UINT i = 0; i < numSection; i++) {
		objMan->setAnchorPos(startAnchor + i, center[i]);
	}
	for (UINT i = 1; i < numSection; i++) {
		objMan->setAnchor2Anchor(startAnchor + i - 1, startAnchor + i, a2aConst);
	}

	//Make springs between segments and anchors.
	for (UINT i = startAnchor; i < startAnchor + numSection; i++) {
		XMFLOAT3 anch = objMan->getAnchorPos(i);
		UINT numSec = i - startAnchor;
		for (UINT j = startSeg + (numSec * numSegPerSec); j < startSeg + ((numSec + 1) * numSegPerSec); j++) {
			XMFLOAT3 vec = XMFLOAT3(anch.x - segLocation.x[j], anch.y - segLocation.y[j], anch.z - segLocation.z[j]);
			float dist = sqrtf(dot(vec));
			objMan->addAnchor2Segment(i, j, dist, a2sConst);
		}
	}

	return objID;
}

UINT UpdateClass::createMT2(XMFLOAT3 origin, ObjectParameters objParam) {
	UINT numSection = objParam.numSection;
	UINT length = objParam.sectionLength * numSection;
	UINT objID = objMan->addObject(length * 26, numSection * 4);
	UINT startSeg = objMan->getObjSegStart(objID);
	UINT startAnchor = objMan->getObjAnchorStart(objID);
	UINT numSeg = objMan->getObjSegCount(objID);
	XMFLOAT3 *center = new XMFLOAT3[numSection];

	float s2sConst = objParam.segToSegConstant;
	float a2aConst = objParam.anchToAnchConstant;
	float a2sConst = objParam.anchToSegConstant;

	//Prepare the springs
	for (UINT i = startSeg * MAXSPRING; i < (startSeg + numSeg) * MAXSPRING; i++) {
		springDistance[i] = FLT_MAX;
		springConstant[i] = FLT_MAX;
		massID[i] = UINT_MAX;
	}

	//Arrange all segments.
	arrangeMT(startSeg, numSeg, length, 2 * objParam.segWidth, objParam.segWidth, s2sConst, origin);

	//Set center of each section.
	UINT numSegPerSec = numSeg / numSection;
	for (UINT i = 0; i < numSection; i++) {
		center[i] = centerSpringMass(startSeg + i * numSegPerSec, startSeg + ((i + 1) * numSegPerSec) - 1);
	}

	//Set anchors.
	for (UINT i = 0; i < numSection; i++) {
		objMan->set4Anchors(objID, i, center[i], objParam.anchorDistance, i % 2, a2aConst);
	}

	//Make springs between segments and anchors.
	for (UINT i = startAnchor; i < startAnchor + objMan->getObjAnchorCount(objID); i++) {
		XMFLOAT3 anch = objMan->getAnchorPos(i);
		UINT numSec = (i - startAnchor) / 4;
		for (UINT j = startSeg + (numSec * numSegPerSec); j < startSeg + ((numSec + 1) * numSegPerSec); j++) {
			XMFLOAT3 vec = XMFLOAT3(anch.x - segLocation.x[j], anch.y - segLocation.y[j], anch.z - segLocation.z[j]);
			float dist = sqrtf(dot(vec));
			objMan->addAnchor2Segment(i, j, dist, a2sConst);
		}
	}

	return objID;
}

UINT UpdateClass::createMT3(XMFLOAT3 origin, ObjectParameters objParam) {
	UINT numSection = objParam.numSection;
	UINT length = objParam.sectionLength * numSection;
	UINT objID = objMan->addObject(length * 26, numSection * 4);
	UINT startSeg = objMan->getObjSegStart(objID);
	UINT startAnchor = objMan->getObjAnchorStart(objID);
	UINT numSeg = objMan->getObjSegCount(objID);
	XMFLOAT3 *center = new XMFLOAT3[numSection];

	float s2sConst = objParam.segToSegConstant;
	float a2aConst = objParam.anchToAnchConstant;
	float a2sConst = objParam.anchToSegConstant;

	//Prepare the springs
	for (UINT i = startSeg * MAXSPRING; i < (startSeg + numSeg) * MAXSPRING; i++) {
		springDistance[i] = FLT_MAX;
		springConstant[i] = FLT_MAX;
		massID[i] = UINT_MAX;
	}

	//Arrange all segments.
	arrangeMT(startSeg, numSeg, length, 2 * objParam.segWidth, objParam.segWidth, s2sConst, origin);

	//Set center of each section.
	UINT numSegPerSec = numSeg / numSection;
	for (UINT i = 0; i < numSection; i++) {
		center[i] = centerSpringMass(startSeg + i * numSegPerSec, startSeg + ((i + 1) * numSegPerSec) - 1);
	}

	//Set anchors.
	for (UINT i = 0; i < numSection; i++) {
		objMan->set4Anchors(objID, i, center[i], objParam.anchorDistance, i % 2, a2aConst);
	}
	for (UINT s = 1; s < numSection; s++) {
		for (UINT i = 0; i < 4; i++) {
			for (UINT j = 0; j < 4; j++) {
				objMan->setAnchor2Anchor(startAnchor + (s - 1) * 4 + i, startAnchor + s * 4 + j, a2aConst);
			}
		}
	}

	//Make springs between segments and anchors.
	for (UINT i = startAnchor; i < startAnchor + objMan->getObjAnchorCount(objID); i++) {
		XMFLOAT3 anch = objMan->getAnchorPos(i);
		UINT numSec = (i - startAnchor) / 4;
		for (UINT j = startSeg + (numSec * numSegPerSec); j < startSeg + ((numSec + 1) * numSegPerSec); j++) {
			XMFLOAT3 vec = XMFLOAT3(anch.x - segLocation.x[j], anch.y - segLocation.y[j], anch.z - segLocation.z[j]);
			float dist = sqrtf(dot(vec));
			objMan->addAnchor2Segment(i, j, dist, a2sConst);
		}
	}

	return objID;
}

UINT UpdateClass::createMT4(XMFLOAT3 origin, ObjectParameters objParam) {
	UINT numSection = objParam.numSection;
	UINT length = objParam.sectionLength * numSection;
	UINT objID = objMan->addObject(length * 26, numSection * 7 - 3);
	UINT startSeg = objMan->getObjSegStart(objID);
	UINT startAnchor = objMan->getObjAnchorStart(objID);
	UINT numSeg = objMan->getObjSegCount(objID);
	XMFLOAT3 *center = new XMFLOAT3[numSection];

	float s2sConst = objParam.segToSegConstant;
	float a2aConst = objParam.anchToAnchConstant;
	float a2sConst = objParam.anchToSegConstant;

	//Prepare the springs
	for (UINT i = startSeg * MAXSPRING; i < (startSeg + numSeg) * MAXSPRING; i++) {
		springDistance[i] = FLT_MAX;
		springConstant[i] = FLT_MAX;
		massID[i] = UINT_MAX;
	}

	//Arrange all segments.
	arrangeMT(startSeg, numSeg, length, 2 * objParam.segWidth, objParam.segWidth, s2sConst, origin);

	//Set center of each section.
	UINT numSegPerSec = numSeg / numSection;
	for (UINT i = 0; i < numSection; i++) {
		center[i] = centerSpringMass(startSeg + i * numSegPerSec, startSeg + ((i + 1) * numSegPerSec) - 1);
	}

	//Set anchors.
	for (UINT i = 0; i < numSection; i++) {
		objMan->set4Anchors(objID, i, center[i], objParam.anchorDistance, i % 2, a2aConst);
	}
	for (UINT s = 0; s < numSection - 1; s++) {
		XMFLOAT3 cntr = XMFLOAT3((center[s].x + center[s + 1].x) / 2, (center[s].y + center[s + 1].y) / 2, (center[s].z + center[s + 1].z) / 2);
		objMan->set3Anchors(objID, 4 * numSection + s * 3, cntr, objParam.jointAnchDistance, false, false, a2aConst);

		for (UINT i = 0; i < 3; i++) {
			for (UINT j = 0; j < 4; j++) {
				objMan->setAnchor2Anchor(startAnchor + 4 * s + j, startAnchor + 4 * numSection + 3 * s + i, a2aConst);
				objMan->setAnchor2Anchor(startAnchor + 4 * (s + 1) + j, startAnchor + 4 * numSection + 3 * s + i, a2aConst);
			}
		}
	}
	
	//Make springs between segments and anchors.
	for (UINT i = startAnchor; i < startAnchor + 4 * numSection; i++) {
		XMFLOAT3 anch = objMan->getAnchorPos(i);
		UINT numSec = (i - startAnchor) / 4;
		for (UINT j = startSeg + (numSec * numSegPerSec); j < startSeg + ((numSec + 1) * numSegPerSec); j++) {
			XMFLOAT3 vec = XMFLOAT3(anch.x - segLocation.x[j], anch.y - segLocation.y[j], anch.z - segLocation.z[j]);
			float dist = sqrtf(dot(vec));
			objMan->addAnchor2Segment(i, j, dist, a2sConst);
		}
	}

	return objID;
}

UINT UpdateClass::createMT5(XMFLOAT3 origin, ObjectParameters objParam) {
	UINT numSection = objParam.numSection;
	UINT length = objParam.sectionLength * numSection;
	UINT objID = objMan->addObject(length * 26, numSection * 3 + 3);
	UINT startSeg = objMan->getObjSegStart(objID);
	UINT startAnchor = objMan->getObjAnchorStart(objID);
	UINT numSeg = objMan->getObjSegCount(objID);
	XMFLOAT3 *center = new XMFLOAT3[numSection];

	float s2sConst = objParam.segToSegConstant;
	float a2aConst = objParam.anchToAnchConstant;
	float a2sConst = objParam.anchToSegConstant;

	//Prepare the springs
	for (UINT i = startSeg * MAXSPRING; i < (startSeg + numSeg) * MAXSPRING; i++) {
		springDistance[i] = FLT_MAX;
		springConstant[i] = FLT_MAX;
		massID[i] = UINT_MAX;
	}

	//Arrange all segments.
	arrangeMT(startSeg, numSeg, length, 2 * objParam.segWidth, objParam.segWidth, s2sConst, origin);

	//Set center of each section.
	UINT numSegPerSec = numSeg / numSection;
	for (UINT i = 0; i < numSection; i++) {
		center[i] = centerSpringMass(startSeg + i * numSegPerSec, startSeg + ((i + 1) * numSegPerSec) - 1);
	}
	
	//Set anchors.
	float ctrDist = center[1].x - center[0].x;
	float startX = center[0].x - (ctrDist / 2);
	for (UINT s = 0; s <= numSection; s++) {
		XMFLOAT3 cntr = XMFLOAT3(startX + s * ctrDist, center[0].y, center[0].z);
		objMan->set3Anchors(objID, s * 3, cntr, objParam.anchorDistance, /*s % 2*/false, true, a2aConst);
	}
	for (UINT s = 1; s <= numSection; s++) {
		for (UINT i = 0; i < 3; i++) {
			for (UINT j = 0; j < 3; j++) {
				objMan->setAnchor2Anchor(startAnchor + (s - 1) * 3 + i, startAnchor + s * 3 + j, a2aConst);
			}
		}
	}

	//Make springs between segments and anchors.
	for (UINT s = 0; s < numSection; s++) {
		for (UINT i = startAnchor + s * 3; i < startAnchor + 6 + s * 3; i++) {
			XMFLOAT3 anch = objMan->getAnchorPos(i);
			for (UINT j = startSeg + (s * numSegPerSec); j < startSeg + ((s + 1) * numSegPerSec); j++) {
				XMFLOAT3 vec = XMFLOAT3(anch.x - segLocation.x[j], anch.y - segLocation.y[j], anch.z - segLocation.z[j]);
				float dist = sqrtf(dot(vec));
				objMan->addAnchor2Segment(i, j, dist, a2sConst);
			}
		}
	}

	return objID;
}

UINT UpdateClass::createMT6(XMFLOAT3 origin, ObjectParameters objParam) {
	UINT numSection = objParam.numSection;
	UINT length = objParam.sectionLength * numSection;
	UINT objID = objMan->addObject(length * 26, numSection * 7 - 3);
	UINT startSeg = objMan->getObjSegStart(objID);
	UINT startAnchor = objMan->getObjAnchorStart(objID);
	UINT numSeg = objMan->getObjSegCount(objID);
	XMFLOAT3 *center = new XMFLOAT3[numSection];

	float s2sConst = objParam.segToSegConstant;
	float a2aConst = objParam.anchToAnchConstant;
	float a2sConst = objParam.anchToSegConstant;

	//Prepare the springs
	for (UINT i = startSeg * MAXSPRING; i < (startSeg + numSeg) * MAXSPRING; i++) {
		springDistance[i] = FLT_MAX;
		springConstant[i] = FLT_MAX;
		massID[i] = UINT_MAX;
	}

	//Arrange all segments.
	arrangeMT(startSeg, numSeg, length, 2 * objParam.segWidth, objParam.segWidth, s2sConst, origin);

	//Set center of each section.
	UINT numSegPerSec = numSeg / numSection;
	for (UINT i = 0; i < numSection; i++) {
		center[i] = centerSpringMass(startSeg + i * numSegPerSec, startSeg + ((i + 1) * numSegPerSec) - 1);
	}

	//Set anchors.
	for (UINT i = 0; i < numSection; i++) {
		objMan->set4Anchors(objID, i, center[i], objParam.anchorDistance, i % 2, a2aConst);
	}
	for (UINT s = 0; s < numSection - 1; s++) {
		XMFLOAT3 cntr = XMFLOAT3((center[s].x + center[s + 1].x) / 2, (center[s].y + center[s + 1].y) / 2, (center[s].z + center[s + 1].z) / 2);
		objMan->set3Anchors(objID, 4 * numSection + s * 3, cntr, objParam.jointAnchDistance, false, false, a2aConst);

		for (UINT i = 0; i < 3; i++) {
			for (UINT j = 0; j < 4; j++) {
				objMan->setAnchor2Anchor(startAnchor + 4 * s + j, startAnchor + 4 * numSection + 3 * s + i, a2aConst);
				objMan->setAnchor2Anchor(startAnchor + 4 * (s + 1) + j, startAnchor + 4 * numSection + 3 * s + i, a2aConst);
			}
		}
	}
	for (UINT s = 0; s < numSection - 2; s++) {
		for (UINT i = 0; i < 3; i++) {
			for (UINT j = 0; j < 3; j++) {
				objMan->setAnchor2Anchor(startAnchor + 4 * numSection + 3 * s + i, startAnchor + 4 * numSection + 3 * (s + 1) + j, a2aConst);
			}
		}
	}

	//Make springs between segments and anchors.
	for (UINT i = startAnchor; i < startAnchor + 4 * numSection; i++) {
		XMFLOAT3 anch = objMan->getAnchorPos(i);
		UINT numSec = (i - startAnchor) / 4;
		for (UINT j = startSeg + (numSec * numSegPerSec); j < startSeg + ((numSec + 1) * numSegPerSec); j++) {
			XMFLOAT3 vec = XMFLOAT3(anch.x - segLocation.x[j], anch.y - segLocation.y[j], anch.z - segLocation.z[j]);
			float dist = sqrtf(dot(vec));
			objMan->addAnchor2Segment(i, j, dist, a2sConst);
		}
	}

	return objID;
}

UINT UpdateClass::createMT7(XMFLOAT3 origin, ObjectParameters objParam, UINT bridge) {
	UINT numSection = objParam.numSection;
	UINT length = objParam.sectionLength * numSection;
	UINT objID = objMan->addObject(length * 26, numSection * 4 + 4);
	UINT startSeg = objMan->getObjSegStart(objID);
	UINT startAnchor = objMan->getObjAnchorStart(objID);
	UINT numSeg = objMan->getObjSegCount(objID);
	XMFLOAT3 *center = new XMFLOAT3[numSection];

	float s2sConst = objParam.segToSegConstant;
	float a2aConst = objParam.anchToAnchConstant;
	float a2sConst = objParam.anchToSegConstant;

	//Prepare the springs
	for (UINT i = startSeg * MAXSPRING; i < (startSeg + numSeg) * MAXSPRING; i++) {
		springDistance[i] = FLT_MAX;
		springConstant[i] = FLT_MAX;
		massID[i] = UINT_MAX;
	}

	//Arrange all segments.
	arrangeMT(startSeg, numSeg, length, 2 * objParam.segWidth, objParam.segWidth, s2sConst, origin);

	//Set center of each section.
	UINT numSegPerSec = numSeg / numSection;
	for (UINT i = 0; i < numSection; i++) {
		center[i] = centerSpringMass(startSeg + i * numSegPerSec, startSeg + ((i + 1) * numSegPerSec) - 1);
	}

	//Set anchors.
	float ctrDist = center[1].x - center[0].x;
	float startX = center[0].x - (ctrDist / 2);
	for (UINT s = 0; s <= numSection; s++) {
		XMFLOAT3 cntr = XMFLOAT3(startX + s * ctrDist, center[0].y, center[0].z);
		objMan->set4FlatAnchors(objID, s * 4, cntr, objParam.anchorDistance, true, a2aConst);
	}
	for (UINT s = 1; s <= numSection; s++) {
		for (UINT i = 0; i < 4; i++) {
			for (UINT j = 0; j < 4; j++) {
				objMan->setAnchor2Anchor(startAnchor + (s - 1) * 4 + i, startAnchor + s * 4 + j, a2aConst);
			}
		}
	}
	if (bridge > 0) {
		for (UINT s = 0; s < numSection - bridge; s++) {
			for (UINT i = 0; i < 4; i++) {
				objMan->setAnchor2Anchor(startAnchor + s * 4 + i, startAnchor + (s + bridge) * 4 + i, a2aConst);
			}
		}
	}

	//Make springs between segments and anchors.
	for (UINT s = 0; s < numSection; s++) {
		for (UINT i = startAnchor + s * 4; i < startAnchor + 8 + s * 4; i++) {
			XMFLOAT3 anch = objMan->getAnchorPos(i);
			for (UINT j = startSeg + (s * numSegPerSec); j < startSeg + ((s + 1) * numSegPerSec); j++) {
				XMFLOAT3 vec = XMFLOAT3(anch.x - segLocation.x[j], anch.y - segLocation.y[j], anch.z - segLocation.z[j]);
				float dist = sqrtf(dot(vec));
				objMan->addAnchor2Segment(i, j, dist, a2sConst);
			}
		}
	}

	return objID;
}

UINT UpdateClass::createFilamen1(XMFLOAT3 origin, ObjectParameters objParam, UINT bridge) {
	UINT numSection = objParam.numSection;
	UINT secLenght = objParam.sectionLength;
	UINT length = secLenght * numSection;
	UINT objID = objMan->addObject(length, numSection * 4 + 4);
	UINT startSeg = objMan->getObjSegStart(objID);
	UINT startAnchor = objMan->getObjAnchorStart(objID);
	UINT numSeg = objMan->getObjSegCount(objID);
	XMFLOAT3 *center = new XMFLOAT3[numSection];

	float s2sConst = objParam.segToSegConstant;
	float a2aConst = objParam.anchToAnchConstant;
	float a2sConst = objParam.anchToSegConstant;

	//Prepare the springs
	for (UINT i = startSeg * MAXSPRING; i < (startSeg + numSeg) * MAXSPRING; i++) {
		springDistance[i] = FLT_MAX;
		springConstant[i] = FLT_MAX;
		massID[i] = UINT_MAX;
	}

	//Arrange all segments.
	for (UINT i = 0; i < length; i++) {
		segLocation.x[startSeg + i] = i * objParam.segWidth + origin.x;
		segLocation.y[startSeg + i] = origin.y;
		segLocation.z[startSeg + i] = origin.z;
	}

	//Make all segment-to-segment springs.
	for (UINT i = 0; i < length - 1; i++) {
		segRadius[startSeg + i] = objParam.segWidth / 2;
		springConnect(startSeg + i, startSeg + i + 1, s2sConst);
	}
	segRadius[startSeg + length - 1] = objParam.segWidth / 2;

	//Set center of each section.
	float ctrX1 = origin.x + objParam.segWidth * (secLenght - 1) / 2;
	for (UINT s = 0; s < numSection; s++) {
		center[s].x = ctrX1 + s * secLenght;
		center[s].y = origin.y;
		center[s].z = origin.z;
	}
	
	//Set anchors.
	float ctrDist = center[1].x - center[0].x;
	float startX = center[0].x - (ctrDist / 2);
	for (UINT s = 0; s <= numSection; s++) {
		XMFLOAT3 cntr = XMFLOAT3(startX + s * ctrDist, center[0].y, center[0].z);
		objMan->set4FlatAnchors(objID, s * 4, cntr, objParam.anchorDistance, false, a2aConst);
	}
	for (UINT s = 1; s <= numSection; s++) {
		for (UINT i = 0; i < 4; i++) {
			for (UINT j = 0; j < 4; j++) {
				objMan->setAnchor2Anchor(startAnchor + (s - 1) * 4 + i, startAnchor + s * 4 + j, a2aConst);
			}
		}
	}
	if (bridge > 0) {
		for (UINT s = 0; s < numSection - bridge; s++) {
			for (UINT i = 0; i < 4; i++) {
				objMan->setAnchor2Anchor(startAnchor + s * 4 + i, startAnchor + (s + bridge) * 4 + i, a2aConst);
			}
		}
	}
	
	//Make springs between segments and anchors.
	for (UINT s = 0; s < numSection; s++) {
		for (UINT i = startAnchor + s * 4; i < startAnchor + 8 + s * 4; i++) {
			XMFLOAT3 anch = objMan->getAnchorPos(i);
			for (UINT j = startSeg + (s * secLenght); j < startSeg + ((s + 1) * secLenght); j++) {
				XMFLOAT3 vec = XMFLOAT3(anch.x - segLocation.x[j], anch.y - segLocation.y[j], anch.z - segLocation.z[j]);
				float dist = sqrtf(dot(vec));
				objMan->addAnchor2Segment(i, j, dist, a2sConst);
			}
		}
	}

	return objID;
}

UINT UpdateClass::createFilamen2(XMFLOAT3 origin, ObjectParameters objParam) {
	UINT numSection = objParam.numSection;
	UINT secLenght = objParam.sectionLength;
	UINT length = secLenght * numSection;
	UINT objID = objMan->addObject(length, numSection * 7 - 3);
	UINT startSeg = objMan->getObjSegStart(objID);
	UINT startAnchor = objMan->getObjAnchorStart(objID);
	UINT numSeg = objMan->getObjSegCount(objID);
	XMFLOAT3 *center = new XMFLOAT3[numSection];

	float s2sConst = objParam.segToSegConstant;
	float a2aConst = objParam.anchToAnchConstant;
	float a2sConst = objParam.anchToSegConstant;

	//Prepare the springs
	for (UINT i = startSeg * MAXSPRING; i < (startSeg + numSeg) * MAXSPRING; i++) {
		springDistance[i] = FLT_MAX;
		springConstant[i] = FLT_MAX;
		massID[i] = UINT_MAX;
	}

	//Arrange all segments.
	for (UINT i = 0; i < length; i++) {
		segLocation.x[startSeg + i] = i * objParam.segWidth + origin.x;
		segLocation.y[startSeg + i] = origin.y;
		segLocation.z[startSeg + i] = origin.z;
	}

	//Make all segment-to-segment springs.
	for (UINT i = 0; i < length - 1; i++) {
		segRadius[startSeg + i] = objParam.segWidth / 2;
		springConnect(startSeg + i, startSeg + i + 1, s2sConst);
	}
	segRadius[startSeg + length - 1] = objParam.segWidth / 2;

	//Set center of each section.
	float ctrX1 = origin.x + objParam.segWidth * (secLenght - 1) / 2;
	for (UINT s = 0; s < numSection; s++) {
		center[s].x = ctrX1 + s * secLenght;
		center[s].y = origin.y;
		center[s].z = origin.z;
	}

	//Set anchors.
	for (UINT i = 0; i < numSection; i++) {
		objMan->set4Anchors(objID, i, center[i], objParam.anchorDistance, i % 2, a2aConst);
	}
	for (UINT s = 0; s < numSection - 1; s++) {
		XMFLOAT3 cntr = XMFLOAT3((center[s].x + center[s + 1].x) / 2, (center[s].y + center[s + 1].y) / 2, (center[s].z + center[s + 1].z) / 2);
		objMan->set3Anchors(objID, 4 * numSection + s * 3, cntr, objParam.jointAnchDistance, s % 2, false, a2aConst);

		for (UINT i = 0; i < 3; i++) {
			for (UINT j = 0; j < 4; j++) {
				objMan->setAnchor2Anchor(startAnchor + 4 * s + j, startAnchor + 4 * numSection + 3 * s + i, a2aConst);
				objMan->setAnchor2Anchor(startAnchor + 4 * (s + 1) + j, startAnchor + 4 * numSection + 3 * s + i, a2aConst);
			}
		}
	}

	//Make springs between segments and anchors.
	for (UINT i = startAnchor; i < startAnchor + 4 * numSection; i++) {
		XMFLOAT3 anch = objMan->getAnchorPos(i);
		UINT numSec = (i - startAnchor) / 4;
		for (UINT j = startSeg + (numSec * secLenght); j < startSeg + ((numSec + 1) * secLenght); j++) {
			XMFLOAT3 vec = XMFLOAT3(anch.x - segLocation.x[j], anch.y - segLocation.y[j], anch.z - segLocation.z[j]);
			float dist = sqrtf(dot(vec));
			objMan->addAnchor2Segment(i, j, dist, a2sConst);
		}
	}

	return objID;
}

UINT UpdateClass::createFilamen3(XMMATRIX oriMx, ObjectParameters objParam, UINT bridge) {
	UINT numSection = objParam.numSection;
	UINT secLenght = objParam.sectionLength;
	UINT length = secLenght * numSection;
	UINT objID = objMan->addObject(length, numSection * 4 + 4);
	UINT startSeg = objMan->getObjSegStart(objID);
	UINT startAnchor = objMan->getObjAnchorStart(objID);
	UINT numSeg = objMan->getObjSegCount(objID);
	XMFLOAT3 *center = new XMFLOAT3[numSection];

	float s2sConst = objParam.segToSegConstant;
	float a2aConst = objParam.anchToAnchConstant;
	float a2sConst = objParam.anchToSegConstant;

	//Prepare the springs
	for (UINT i = startSeg * MAXSPRING; i < (startSeg + numSeg) * MAXSPRING; i++) {
		springDistance[i] = FLT_MAX;
		springConstant[i] = FLT_MAX;
		massID[i] = UINT_MAX;
	}

	//Arrange all segments.
	for (UINT i = 0; i < length; i++) {
		segLocation.x[startSeg + i] = i * objParam.segWidth;
		segLocation.y[startSeg + i] = 0;
		segLocation.z[startSeg + i] = 0;
	}

	//Make all segment-to-segment springs.
	for (UINT i = 0; i < length - 1; i++) {
		segRadius[startSeg + i] = objParam.segWidth / 2;
		springConnect(startSeg + i, startSeg + i + 1, s2sConst);
	}
	segRadius[startSeg + length - 1] = objParam.segWidth / 2;

	//Set center of each section.
	float ctrX1 = objParam.segWidth * (secLenght - 1) / 2;
	for (UINT s = 0; s < numSection; s++) {
		center[s].x = ctrX1 + s * secLenght;
		center[s].y = 0;
		center[s].z = 0;
	}

	//Set anchors.
	float ctrDist = center[1].x - center[0].x;
	float startX = center[0].x - (ctrDist / 2);
	for (UINT s = 0; s <= numSection; s++) {
		XMFLOAT3 cntr = XMFLOAT3(startX + s * ctrDist, center[0].y, center[0].z);
		objMan->set4FlatAnchors(objID, s * 4, cntr, objParam.anchorDistance, false, a2aConst);
	}
	for (UINT s = 1; s <= numSection; s++) {
		for (UINT i = 0; i < 4; i++) {
			for (UINT j = 0; j < 4; j++) {
				objMan->setAnchor2Anchor(startAnchor + (s - 1) * 4 + i, startAnchor + s * 4 + j, a2aConst);
			}
		}
	}
	if (bridge > 0) {
		for (UINT s = 0; s < numSection - bridge; s++) {
			for (UINT i = 0; i < 4; i++) {
				objMan->setAnchor2Anchor(startAnchor + s * 4 + i, startAnchor + (s + bridge) * 4 + i, a2aConst);
			}
		}
	}

	//Make springs between segments and anchors.
	for (UINT s = 0; s < numSection; s++) {
		for (UINT i = startAnchor + s * 4; i < startAnchor + 8 + s * 4; i++) {
			XMFLOAT3 anch = objMan->getAnchorPos(i);
			for (UINT j = startSeg + (s * secLenght); j < startSeg + ((s + 1) * secLenght); j++) {
				XMFLOAT3 vec = XMFLOAT3(anch.x - segLocation.x[j], anch.y - segLocation.y[j], anch.z - segLocation.z[j]);
				float dist = sqrtf(dot(vec));
				objMan->addAnchor2Segment(i, j, dist, a2sConst);
			}
		}
	}

	//Transform segments to the desired position.
	for (UINT i = 0; i < length; i++) {
		XMMATRIX segPos = XMMatrixTranslation(segLocation.x[startSeg + i], segLocation.y[startSeg + i], segLocation.z[startSeg + i]);
		segPos = segPos * oriMx;
		segLocation.x[startSeg + i] = XMVectorGetX(segPos.r[3]);
		segLocation.y[startSeg + i] = XMVectorGetY(segPos.r[3]);
		segLocation.z[startSeg + i] = XMVectorGetZ(segPos.r[3]);
	}

	//Transform anchors to the desired position.
	for (UINT i = 0; i < (numSection * 4 + 4); i++) {
		XMFLOAT3 anch = objMan->getAnchorPos(startAnchor + i);
		XMMATRIX anchPos = XMMatrixTranslation(anch.x, anch.y, anch.z);
		anchPos = anchPos * oriMx;
		anch.x = XMVectorGetX(anchPos.r[3]);
		anch.y = XMVectorGetY(anchPos.r[3]);
		anch.z = XMVectorGetZ(anchPos.r[3]);

		objMan->setAnchorPos(startAnchor + i, anch);
	}

	return objID;
}

UINT UpdateClass::createSmoothMuscleMT(XMFLOAT3 mtPos, XMFLOAT3 axis, float rot, ObjectParameters objParam, UINT bridge, float magnifier, XMFLOAT3 adjuster) {
	XMFLOAT3 matPos, rotAx;
	matPos.x = magnifier * mtPos.x + adjuster.x;
	matPos.y = magnifier * mtPos.y + adjuster.y;
	matPos.z = magnifier * mtPos.z + adjuster.z;
	rotAx.x = axis.x;
	rotAx.y = axis.y;
	rotAx.z = axis.z;

	XMMATRIX matRotAx = XMMatrixRotationAxis(XMLoadFloat3(&rotAx), rot);
	float xCenterShift = (objParam.sectionLength * objParam.numSection - 1) * objParam.segWidth / 2.0f;
	XMMATRIX zMinDir = XMMATRIX(
		0.0f, 1.0f, 0.0f, 0.0f,
		-1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, -xCenterShift, 0.0f, 1.0f);
	XMMATRIX mResult = zMinDir * matRotAx;
	mResult.r[3] += XMLoadFloat3(&matPos);

	return createFilamen3(mResult, objParam, bridge);
}

UINT UpdateClass::createSmoothMuscleMT(XMFLOAT3 mtPos, XMFLOAT3 rot, ObjectParameters objParam, UINT bridge, float magnifier, XMFLOAT3 adjuster) {
	XMFLOAT3 matPos;
	float rotX, rotY, rotZ;
	matPos.x = magnifier * mtPos.x + adjuster.x;
	matPos.y = magnifier * mtPos.z + adjuster.y;
	matPos.z = magnifier * mtPos.y + adjuster.z;
	rotX = XMConvertToRadians(rot.x);
	rotY = XMConvertToRadians(rot.z);
	rotZ = XMConvertToRadians(rot.y);

	float xCenterShift = (objParam.sectionLength * objParam.numSection - 1) * objParam.segWidth / 2.0f;
	XMMATRIX zMinDir = XMMATRIX(
		0.0f, 1.0f, 0.0f, 0.0f,
		-1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, -xCenterShift, 0.0f, 1.0f);
	XMMATRIX mResult = zMinDir * XMMatrixRotationX(-rotX) * XMMatrixRotationZ(-rotZ) * XMMatrixRotationY(-rotY);
	mResult.r[3] += XMLoadFloat3(&matPos);

	return createFilamen3(mResult, objParam, bridge);
}

void UpdateClass::arrangeMT(UINT startSegment, UINT numSegment, UINT length, UINT dimerLength, UINT dimerWidth, float springConst, XMFLOAT3 origin) {
	for (int i = 0; i < length; i++) {
		for (int j = 0; j < 26; j += 2) {
			segLocation.x[i * 26 + j + startSegment] = (i + (j * 0.057692307f)) * dimerLength + origin.x;
			segLocation.y[i * 26 + j + startSegment] = (2.1f * cos(j * 0.24166097f)) * dimerWidth + origin.y;
			segLocation.z[i * 26 + j + startSegment] = (2.1f * sin(j * 0.24166097f)) * dimerWidth + origin.z;

			segLocation.x[i * 26 + j + 1 + startSegment] = (i + ((j * 0.057692307f)) + 0.5f) * dimerLength + origin.x;
			segLocation.y[i * 26 + j + 1 + startSegment] = (2.1f * cos(j * 0.24166097f)) * dimerWidth + origin.y;
			segLocation.z[i * 26 + j + 1 + startSegment] = (2.1f * sin(j * 0.24166097f)) * dimerWidth + origin.z;
		}
	}

	//Make all segment-to-segment springs.
	for (UINT i = 0; i < length - 1; i++) {
		for (UINT j = 0; j < 24; j += 2) {
			springConnect(startSegment + j + (i * 26), startSegment + j + 1 + (i * 26), springConst);
			springConnect(startSegment + j + (i * 26), startSegment + j + 2 + (i * 26), springConst);
			springConnect(startSegment + j + 1 + (i * 26), startSegment + j + ((i + 1) * 26), springConst);
			springConnect(startSegment + j + 1 + (i * 26), startSegment + j + 2 + (i * 26), springConst);
		}
	}
	for (UINT i = 0; i < length - 2; i++) {
		springConnect(startSegment + 24 + (i * 26), startSegment + 25 + (i * 26), springConst);
		springConnect(startSegment + 24 + (i * 26), startSegment + 27 + (i * 26), springConst);
		springConnect(startSegment + 25 + (i * 26), startSegment + 50 + (i * 26), springConst);
		springConnect(startSegment + 25 + (i * 26), startSegment + 52 + (i * 26), springConst);
	}
	springConnect(startSegment - 28 + (length * 26), startSegment - 27 + (length * 26), springConst);
	springConnect(startSegment - 28 + (length * 26), startSegment - 25 + (length * 26), springConst);
	springConnect(startSegment - 27 + (length * 26), startSegment - 2 + (length * 26), springConst);
	for (UINT j = startSegment + numSegment - 26; j < startSegment + numSegment - 2; j += 2) {
		springConnect(j, j + 1, springConst);
		springConnect(j, j + 2, springConst);
		springConnect(j + 1, j + 3, springConst);
	}
	springConnect(startSegment + numSegment - 2, startSegment + numSegment - 1, springConst);
}

void UpdateClass::setHapticFeedback(float limitSense) {
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 6; j++) {
			hd1.accAvTemp[i][j] /= hd1.accNumSeg[i][j];
		}
	}

	float totalCol = 0;
	float avgSense = 0;
	
	//max = ((max < hd1.fingerVelocity[1][1]) && (hd1.fingerVelocity[1][1] < 200)) ? hd1.fingerVelocity[1][1] : max;
	//testString = std::to_string(max);//////test//////////////////////////////////////////////

	/////////////For right thumb finger///////////////////////////////////
	for (int i = hd1.getStartSegBone(0, 3) + hd1.getStartSegRight(); i <= hd1.getEndSegBone(0, 3) + hd1.getStartSegRight(); i++) {
		totalCol += segColor.x[i] - segColor.z[i];
		totalCol = std::abs(totalCol);
	}
	
	avgSense = totalCol / (hd1.getEndSegBone(0, 3) + 1 - hd1.getStartSegBone(0, 3));
	if (avgSense > limitSense) {
		hapticResult.freq[0] = 50 * hd1.fingerVelocity[0][0];
		for (int i = 0; i < 7; i++) {
			if (hd1.accAvTemp[0][0] < hapticLevelValue[i][0]) {
				hapticResult.amp[0] = (int) hapticLevelValue[i][1];
			}
		}
		int x = (int)(200 * avgSense);
		hapticResult.press[0] = (x < 54) ? (95 - x) : 41;
	}
	else {
		hapticResult.freq[0] = 0;
		hapticResult.amp[0] = 1;
		hapticResult.press[0] = 112;
	}
	totalCol = 0;
	avgSense = 0;
	//////////////////////////////////////////////////////////////////////
	/////////////For right index finger///////////////////////////////////
	for (int i = hd1.getStartSegBone(1, 3) + hd1.getStartSegRight(); i <= hd1.getEndSegBone(1, 3) + hd1.getStartSegRight(); i++) {
		totalCol += segColor.x[i] - segColor.z[i];
		totalCol = std::abs(totalCol);
	}

	avgSense = totalCol / (hd1.getEndSegBone(1, 3) + 1 - hd1.getStartSegBone(1, 3));
	if (avgSense > limitSense) {
		hapticResult.freq[1] = 50 * hd1.fingerVelocity[0][1];
		for (int i = 0; i < 7; i++) {
			if (hd1.accAvTemp[0][1] < hapticLevelValue[i][0]) {
				hapticResult.amp[1] = (int)hapticLevelValue[i][1];
			}
		}
		int x = (int)(200 * avgSense);
		hapticResult.press[1] = (x < 54) ? (95 - x) : 41;
	}
	else {
		hapticResult.freq[1] = 0;
		hapticResult.amp[1] = 1;
		hapticResult.press[1] = 112;
	}
	totalCol = 0;
	avgSense = 0;
	//////////////////////////////////////////////////////////////////////
	/////////////For right middle finger//////////////////////////////////
	for (int i = hd1.getStartSegBone(2, 3) + hd1.getStartSegRight(); i <= hd1.getEndSegBone(2, 3) + hd1.getStartSegRight(); i++) {
		totalCol += segColor.x[i] - segColor.z[i];
		totalCol = std::abs(totalCol);
	}

	avgSense = totalCol / (hd1.getEndSegBone(2, 3) + 1 - hd1.getStartSegBone(2, 3));
	if (avgSense > limitSense) {
		hapticResult.freq[2] = 50 * hd1.fingerVelocity[0][2];
		for (int i = 0; i < 7; i++) {
			if (hd1.accAvTemp[0][2] < hapticLevelValue[i][0]) {
				hapticResult.amp[2] = (int)hapticLevelValue[i][1];
			}
		}
		int x = (int)(200 * avgSense);
		hapticResult.press[2] = (x < 54) ? (95 - x) : 41;
	}
	else {
		hapticResult.freq[2] = 0;
		hapticResult.amp[2] = 1;
		hapticResult.press[2] = 112;
	}
	totalCol = 0;
	avgSense = 0;
	//////////////////////////////////////////////////////////////////////
	/////////////For right ring finger////////////////////////////////////
	for (int i = hd1.getStartSegBone(3, 3) + hd1.getStartSegRight(); i <= hd1.getEndSegBone(3, 3) + hd1.getStartSegRight(); i++) {
		totalCol += segColor.x[i] - segColor.z[i];
		totalCol = std::abs(totalCol);
	}

	avgSense = totalCol / (hd1.getEndSegBone(3, 3) + 1 - hd1.getStartSegBone(3, 3));
	if (avgSense > limitSense) {
		hapticResult.freq[3] = 50 * hd1.fingerVelocity[0][3];
		for (int i = 0; i < 7; i++) {
			if (hd1.accAvTemp[0][3] < hapticLevelValue[i][0]) {
				hapticResult.amp[3] = (int)hapticLevelValue[i][1];
			}
		}
		int x = (int)(200 * avgSense);
		hapticResult.press[3] = (x < 54) ? (95 - x) : 41;
	}
	else {
		hapticResult.freq[3] = 0;
		hapticResult.amp[3] = 1;
		hapticResult.press[3] = 112;
	}
	totalCol = 0;
	avgSense = 0;
	//////////////////////////////////////////////////////////////////////
	/////////////For right pinky finger///////////////////////////////////
	for (int i = hd1.getStartSegBone(4, 3) + hd1.getStartSegRight(); i <= hd1.getEndSegBone(4, 3) + hd1.getStartSegRight(); i++) {
		totalCol += segColor.x[i] - segColor.z[i];
		totalCol = std::abs(totalCol);
	}

	avgSense = totalCol / (hd1.getEndSegBone(4, 3) + 1 - hd1.getStartSegBone(4, 3));
	if (avgSense > limitSense) {
		hapticResult.freq[4] = 50 * hd1.fingerVelocity[0][4];
		for (int i = 0; i < 7; i++) {
			if (hd1.accAvTemp[0][4] < hapticLevelValue[i][0]) {
				hapticResult.amp[4] = (int)hapticLevelValue[i][1];
			}
		}
		int x = (int)(200 * avgSense);
		hapticResult.press[4] = (x < 54) ? (95 - x) : 41;
	}
	else {
		hapticResult.freq[4] = 0;
		hapticResult.amp[4] = 1;
		hapticResult.press[4] = 112;
	}
	totalCol = 0;
	avgSense = 0;
	//////////////////////////////////////////////////////////////////////
	/////////////For left thumb finger////////////////////////////////////
	for (int i = hd1.getStartSegBone(0, 3); i <= hd1.getEndSegBone(0, 3); i++) {
		totalCol += segColor.x[i] - segColor.z[i];
		totalCol = std::abs(totalCol);
	}

	avgSense = totalCol / (hd1.getEndSegBone(0, 3) + 1 - hd1.getStartSegBone(0, 3));
	if (avgSense > limitSense) {
		hapticResult.freq[6] = 50 * hd1.fingerVelocity[1][0];
		for (int i = 0; i < 7; i++) {
			if (hd1.accAvTemp[1][0] < hapticLevelValue[i][0]) {
				hapticResult.amp[6] = (int)hapticLevelValue[i][1];
			}
		}
		int x = (int)(200 * avgSense);
		hapticResult.press[6] = (x < 54) ? (95 - x) : 41;
	}
	else {
		hapticResult.freq[6] = 0;
		hapticResult.amp[6] = 1;
		hapticResult.press[6] = 112;
	}
	totalCol = 0;
	avgSense = 0;
	//////////////////////////////////////////////////////////////////////
	/////////////For left index finger////////////////////////////////////
	for (int i = hd1.getStartSegBone(1, 3); i <= hd1.getEndSegBone(1, 3); i++) {
		totalCol += segColor.x[i] - segColor.z[i];
		totalCol = std::abs(totalCol);
	}

	avgSense = totalCol / (hd1.getEndSegBone(1, 3) + 1 - hd1.getStartSegBone(1, 3));
	if (avgSense > limitSense) {
		hapticResult.freq[7] = 50 * hd1.fingerVelocity[1][1];
		for (int i = 0; i < 7; i++) {
			if (hd1.accAvTemp[1][1] < hapticLevelValue[i][0]) {
				hapticResult.amp[7] = (int)hapticLevelValue[i][1];
			}
		}
		int x = (int)(200 * avgSense);
		hapticResult.press[7] = (x < 54) ? (95 - x) : 41;
	}
	else {
		hapticResult.freq[7] = 0;
		hapticResult.amp[7] = 1;
		hapticResult.press[7] = 112;
	}
	totalCol = 0;
	avgSense = 0;
	//////////////////////////////////////////////////////////////////////
	/////////////For left middle finger////////////////////////////////////
	for (int i = hd1.getStartSegBone(2, 3); i <= hd1.getEndSegBone(2, 3); i++) {
		totalCol += segColor.x[i] - segColor.z[i];
		totalCol = std::abs(totalCol);
	}

	avgSense = totalCol / (hd1.getEndSegBone(2, 3) + 1 - hd1.getStartSegBone(2, 3));
	if (avgSense > limitSense) {
		hapticResult.freq[8] = 50 * hd1.fingerVelocity[1][2];
		for (int i = 0; i < 7; i++) {
			if (hd1.accAvTemp[1][2] < hapticLevelValue[i][0]) {
				hapticResult.amp[8] = (int)hapticLevelValue[i][1];
			}
		}
		int x = (int)(200 * avgSense);
		hapticResult.press[8] = (x < 54) ? (95 - x) : 41;
	}
	else {
		hapticResult.freq[8] = 0;
		hapticResult.amp[8] = 1;
		hapticResult.press[8] = 112;
	}
	totalCol = 0;
	avgSense = 0;
	//////////////////////////////////////////////////////////////////////
	/////////////For left ring finger////////////////////////////////////
	for (int i = hd1.getStartSegBone(3, 3); i <= hd1.getEndSegBone(3, 3); i++) {
		totalCol += segColor.x[i] - segColor.z[i];
		totalCol = std::abs(totalCol);
	}

	avgSense = totalCol / (hd1.getEndSegBone(3, 3) + 1 - hd1.getStartSegBone(3, 3));
	if (avgSense > limitSense) {
		hapticResult.freq[9] = 50 * hd1.fingerVelocity[1][3];
		for (int i = 0; i < 7; i++) {
			if (hd1.accAvTemp[1][3] < hapticLevelValue[i][0]) {
				hapticResult.amp[9] = (int)hapticLevelValue[i][1];
			}
		}
		int x = (int)(200 * avgSense);
		hapticResult.press[9] = (x < 54) ? (95 - x) : 41;
	}
	else {
		hapticResult.freq[9] = 0;
		hapticResult.amp[9] = 1;
		hapticResult.press[9] = 112;
	}
	totalCol = 0;
	avgSense = 0;
	//////////////////////////////////////////////////////////////////////
	/////////////For left pinky finger////////////////////////////////////
	for (int i = hd1.getStartSegBone(4, 3); i <= hd1.getEndSegBone(4, 3); i++) {
		totalCol += segColor.x[i] - segColor.z[i];
		totalCol = std::abs(totalCol);
	}

	avgSense = totalCol / (hd1.getEndSegBone(4, 3) + 1 - hd1.getStartSegBone(4, 3));
	if (avgSense > limitSense) {
		//hapticResult.freq[10] = 20 * hd1.accNumSeg[1][4];
		hapticResult.freq[10] = 50 * hd1.fingerVelocity[1][4];
		for (int i = 0; i < 7; i++) {
			if (hd1.accAvTemp[1][4] < hapticLevelValue[i][0]) {
				hapticResult.amp[10] = (int)hapticLevelValue[i][1];
			}
		}
		int x = (int)(200 * avgSense);
		hapticResult.press[10] = (x < 54) ? (95 - x) : 41;
	}
	else {
		hapticResult.freq[10] = 0;
		hapticResult.amp[10] = 1;
		hapticResult.press[10] = 112;
	}
	totalCol = 0;
	avgSense = 0;
	//////////////////////////////////////////////////////////////////////
}

void UpdateClass::BuildComputeCommandQLsim() {
	D3D12_COMMAND_QUEUE_DESC queueDesc = {};
	queueDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
	queueDesc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
	ThrowIfFailed(md3dDevice->CreateCommandQueue(&queueDesc, IID_PPV_ARGS(&mCommandQueue_Sim)));

	ThrowIfFailed(md3dDevice->CreateCommandAllocator(
		D3D12_COMMAND_LIST_TYPE_DIRECT,
		IID_PPV_ARGS(&CmdListAlloc_Sim)));

	ThrowIfFailed(md3dDevice->CreateCommandList(
		0,
		D3D12_COMMAND_LIST_TYPE_DIRECT,
		CmdListAlloc_Sim.Get(), // Associated command allocator
		nullptr,                   // Initial PipelineStateObject
		IID_PPV_ARGS(&mCommandList_Sim)));

	// Start off in a closed state.  This is because the first time we refer 
	// to the command list we will Reset it, and it needs to be closed before
	// calling Reset.
	mCommandList_Sim->Close();

	// Create query heaps and result buffers.
	{
		// Two timestamps for each frame and timer.
		const UINT resultCount = 2;
		const UINT resultBufferSize = resultCount * sizeof(UINT64);

		D3D12_QUERY_HEAP_DESC timestampHeapDesc = {};
		timestampHeapDesc.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP;
		timestampHeapDesc.Count = resultCount;


		ThrowIfFailed(md3dDevice->CreateCommittedResource(
			&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_READBACK),
			D3D12_HEAP_FLAG_NONE,
			&CD3DX12_RESOURCE_DESC::Buffer(resultBufferSize),
			D3D12_RESOURCE_STATE_COPY_DEST,
			nullptr,
			IID_PPV_ARGS(&mTimestampResultBuffers_Sim)));

		ThrowIfFailed(md3dDevice->CreateQueryHeap(&timestampHeapDesc, IID_PPV_ARGS(&mTimestampQueryHeaps_Sim)));

	}

	ThrowIfFailed(md3dDevice->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&mFence_Sim)));
}
void UpdateClass::BuildComputeCommandQLcpy() {
	D3D12_COMMAND_QUEUE_DESC queueDesc = {};
	queueDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
	queueDesc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
	ThrowIfFailed(md3dDevice->CreateCommandQueue(&queueDesc, IID_PPV_ARGS(&mCommandQueue_Cpy)));

	ThrowIfFailed(md3dDevice->CreateCommandAllocator(
		D3D12_COMMAND_LIST_TYPE_DIRECT,
		IID_PPV_ARGS(&CmdListAlloc_Cpy)));

	ThrowIfFailed(md3dDevice->CreateCommandList(
		0,
		D3D12_COMMAND_LIST_TYPE_DIRECT,
		CmdListAlloc_Cpy.Get(), // Associated command allocator
		nullptr,                   // Initial PipelineStateObject
		IID_PPV_ARGS(&mCommandList_Cpy)));

	// Start off in a closed state.  This is because the first time we refer 
	// to the command list we will Reset it, and it needs to be closed before
	// calling Reset.
	mCommandList_Cpy->Close();

	// Create query heaps and result buffers.
	{
		// Two timestamps for each frame and timer.
		const UINT resultCount = 2;
		const UINT resultBufferSize = resultCount * sizeof(UINT64);

		D3D12_QUERY_HEAP_DESC timestampHeapDesc = {};
		timestampHeapDesc.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP;
		timestampHeapDesc.Count = resultCount;


		ThrowIfFailed(md3dDevice->CreateCommittedResource(
			&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_READBACK),
			D3D12_HEAP_FLAG_NONE,
			&CD3DX12_RESOURCE_DESC::Buffer(resultBufferSize),
			D3D12_RESOURCE_STATE_COPY_DEST,
			nullptr,
			IID_PPV_ARGS(&mTimestampResultBuffers_Cpy)));

		ThrowIfFailed(md3dDevice->CreateQueryHeap(&timestampHeapDesc, IID_PPV_ARGS(&mTimestampQueryHeaps_Cpy)));

	}

	ThrowIfFailed(md3dDevice->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&mFence_Cpy)));
}
void UpdateClass::FlushCommandQueueCSsim()
{
	// Advance the fence value to mark commands up to this fence point.
	mCurrentFence_Sim++;

	// Add an instruction to the command queue to set a new fence point.  Because we 
	// are on the GPU timeline, the new fence point won't be set until the GPU finishes
	// processing all the commands prior to this Signal().
	ThrowIfFailed(mCommandQueue_Sim->Signal(mFence_Sim.Get(), mCurrentFence_Sim));

	// Wait until the GPU has completed commands up to this fence point.
	if (mFence_Sim->GetCompletedValue() < mCurrentFence_Sim)
	{
		HANDLE eventHandle = CreateEventEx(nullptr, false, false, EVENT_ALL_ACCESS);

		// Fire event when GPU hits current fence.  
		ThrowIfFailed(mFence_Sim->SetEventOnCompletion(mCurrentFence_Sim, eventHandle));

		// Wait until the GPU hits current fence event is fired.
		WaitForSingleObject(eventHandle, INFINITE);
		CloseHandle(eventHandle);
	}
}
void UpdateClass::FlushCommandQueueCScpy()
{
	// Advance the fence value to mark commands up to this fence point.
	mCurrentFence_Cpy++;

	// Add an instruction to the command queue to set a new fence point.  Because we 
	// are on the GPU timeline, the new fence point won't be set until the GPU finishes
	// processing all the commands prior to this Signal().
	ThrowIfFailed(mCommandQueue_Cpy->Signal(mFence_Cpy.Get(), mCurrentFence_Cpy));

	// Wait until the GPU has completed commands up to this fence point.
	if (mFence_Cpy->GetCompletedValue() < mCurrentFence_Cpy)
	{
		HANDLE eventHandle = CreateEventEx(nullptr, false, false, EVENT_ALL_ACCESS);

		// Fire event when GPU hits current fence.  
		ThrowIfFailed(mFence_Cpy->SetEventOnCompletion(mCurrentFence_Cpy, eventHandle));

		// Wait until the GPU hits current fence event is fired.
		WaitForSingleObject(eventHandle, INFINITE);
		CloseHandle(eventHandle);
	}
}
void UpdateClass::BuildComputeBuffers()
{
	UINT64 byteSize = SEGMENTCOUNT * sizeof(float);

	//Sort & find index
	sortListCPU(segLocation, segLocationSorted, segOriginalIndex, segCellStart, segCellEnd);

	InitGPUMemInOut(OR_originalIndex, byteSize, L"OR_originalIndex");
	InitGPUMemInOut(OR_segLocSorted_x, byteSize, L"OR_segLocSorted_x");
	InitGPUMemInOut(OR_segLocSorted_y, byteSize, L"OR_segLocSorted_y");
	InitGPUMemInOut(OR_segLocSorted_z, byteSize, L"OR_segLocSorted_z");
	InitGPUMemInOut(OR_cellStart, HASH_ENTRIE * sizeof(unsigned int), L"OR_cellStart");
	InitGPUMemInOut(OR_cellEnd, HASH_ENTRIE * sizeof(unsigned int), L"OR_cellEnd");
	InitGPUMemInOut(OR_forwardVec_x, byteSize * 2, L"OR_forwardVec_x");
	InitGPUMemInOut(OR_forwardVec_y, byteSize * 2, L"OR_forwardVec_y");
	InitGPUMemInOut(OR_forwardVec_z, byteSize * 2, L"OR_forwardVec_z");
	InitGPUMemInOut(OR_colors_x, byteSize, L"OR_colors_x");
	InitGPUMemInOut(OR_colors_y, byteSize, L"OR_colors_y");
	InitGPUMemInOut(OR_colors_z, byteSize, L"OR_colors_z");
	InitGPUMemInOut(OR_segRadiSort, byteSize, L"OR_segRadiSort");		//Addition

	// Map the data so we can read it on CPU.

	unsigned int* mapOriginalIndex = nullptr;
	float* mapSegSorted_x = nullptr;
	float* mapSegSorted_y = nullptr;
	float* mapSegSorted_z = nullptr;
	float* mapObjModify_x = nullptr;
	float* mapObjModify_y = nullptr;
	float* mapObjModify_z = nullptr;
	float* mapColors_x = nullptr;
	float* mapColors_y = nullptr;
	float* mapColors_z = nullptr;
	unsigned int* mapCellStart = nullptr;
	unsigned int* mapCellEnd = nullptr;
	float* mapSegRadiSort = nullptr;		//Addition
	CD3DX12_RANGE readRange(0, 0);
	ThrowIfFailed(OR_originalIndex[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapOriginalIndex)));
	ThrowIfFailed(OR_segLocSorted_x[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegSorted_x)));
	ThrowIfFailed(OR_segLocSorted_y[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegSorted_y)));
	ThrowIfFailed(OR_segLocSorted_z[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegSorted_z)));
	ThrowIfFailed(OR_forwardVec_x[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapObjModify_x)));
	ThrowIfFailed(OR_forwardVec_y[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapObjModify_y)));
	ThrowIfFailed(OR_forwardVec_z[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapObjModify_z)));
	ThrowIfFailed(OR_colors_x[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapColors_x)));
	ThrowIfFailed(OR_colors_y[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapColors_y)));
	ThrowIfFailed(OR_colors_z[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapColors_z)));
	ThrowIfFailed(OR_cellStart[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapCellStart)));
	ThrowIfFailed(OR_cellEnd[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapCellEnd)));
	ThrowIfFailed(OR_segRadiSort[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegRadiSort)));		//Addition

	/*ThrowIfFailed(OR_originalIndex[1]->Map(0, nullptr, reinterpret_cast<void**>(&mapOriginalIndex)));
	ThrowIfFailed(OR_segLocSorted_x[1]->Map(0, nullptr, reinterpret_cast<void**>(&mapSegSorted_x)));
	ThrowIfFailed(OR_segLocSorted_y[1]->Map(0, nullptr, reinterpret_cast<void**>(&mapSegSorted_y)));
	ThrowIfFailed(OR_segLocSorted_z[1]->Map(0, nullptr, reinterpret_cast<void**>(&mapSegSorted_z)));
	ThrowIfFailed(OR_forwardVec_x[1]->Map(0, nullptr, reinterpret_cast<void**>(&mapObjModify_x)));
	ThrowIfFailed(OR_forwardVec_y[1]->Map(0, nullptr, reinterpret_cast<void**>(&mapObjModify_y)));
	ThrowIfFailed(OR_forwardVec_z[1]->Map(0, nullptr, reinterpret_cast<void**>(&mapObjModify_z)));
	ThrowIfFailed(OR_colors_x[1]->Map(0, nullptr, reinterpret_cast<void**>(&mapColors_x)));
	ThrowIfFailed(OR_colors_y[1]->Map(0, nullptr, reinterpret_cast<void**>(&mapColors_y)));
	ThrowIfFailed(OR_colors_z[1]->Map(0, nullptr, reinterpret_cast<void**>(&mapColors_z)));
	ThrowIfFailed(OR_cellStart[1]->Map(0, nullptr, reinterpret_cast<void**>(&mapCellStart)));
	ThrowIfFailed(OR_cellEnd[1]->Map(0, nullptr, reinterpret_cast<void**>(&mapCellEnd)));*/

	//std::ofstream fout("results.txt");
	//TODO: test if memCopy or OpenMP is faster
	for (unsigned int i = 0; i < SEGMENTCOUNT; ++i)
	{
		mapOriginalIndex[i] = segOriginalIndex[i];
		mapSegSorted_x[i] = segLocationSorted.x[i];
		mapSegSorted_y[i] = segLocationSorted.y[i];
		mapSegSorted_z[i] = segLocationSorted.z[i];
		mapObjModify_x[i] = transVelocity.x[i];
		mapObjModify_y[i] = transVelocity.y[i];
		mapObjModify_z[i] = transVelocity.z[i];
		mapColors_x[i] = segColor.x[i];
		mapColors_y[i] = segColor.y[i];
		mapColors_z[i] = segColor.z[i];
		mapSegRadiSort[i] = segRadiusSorted[i];		//Addition
	}
	for (unsigned int i = 0; i < HASH_ENTRIE; ++i)
	{
		mapCellStart[i] = segCellStart[i];
		mapCellEnd[i] = segCellEnd[i];
		//fout << i << "\t(" << segCellStart[i] << ", " << segCellEnd[i] << ")" << std::endl;
	}

	OR_originalIndex[1]->Unmap(0, nullptr);
	OR_segLocSorted_x[1]->Unmap(0, nullptr);
	OR_segLocSorted_y[1]->Unmap(0, nullptr);
	OR_segLocSorted_z[1]->Unmap(0, nullptr);
	OR_forwardVec_x[1]->Unmap(0, nullptr);
	OR_forwardVec_y[1]->Unmap(0, nullptr);
	OR_forwardVec_z[1]->Unmap(0, nullptr);
	OR_colors_x[1]->Unmap(0, nullptr);
	OR_colors_y[1]->Unmap(0, nullptr);
	OR_colors_z[1]->Unmap(0, nullptr);
	OR_cellStart[1]->Unmap(0, nullptr);
	OR_cellEnd[1]->Unmap(0, nullptr);
	OR_segRadiSort[1]->Unmap(0, nullptr);		//Addition

	///////////////////////////////////////////////////////////////////////////////////////////////////////

	BarriorCopyToGPU(mCommandList, OR_originalIndex); //mCommandList_CS not used here; initialized using main command queue
	BarriorCopyToGPU(mCommandList, OR_segLocSorted_x);
	BarriorCopyToGPU(mCommandList, OR_segLocSorted_y);
	BarriorCopyToGPU(mCommandList, OR_segLocSorted_z);
	BarriorCopyToGPU(mCommandList, OR_cellStart);
	BarriorCopyToGPU(mCommandList, OR_cellEnd);
	BarriorCopyToGPU(mCommandList, OR_forwardVec_x);
	BarriorCopyToGPU(mCommandList, OR_forwardVec_y);
	BarriorCopyToGPU(mCommandList, OR_forwardVec_z);
	BarriorCopyToGPU(mCommandList, OR_colors_x);
	BarriorCopyToGPU(mCommandList, OR_colors_y);
	BarriorCopyToGPU(mCommandList, OR_colors_z);
	BarriorCopyToGPU(mCommandList, OR_segRadiSort);		//Addition
}
void UpdateClass::BuildComputeBuffersMove()
{
	UINT64 byteSize = SEGMENTCOUNT * sizeof(float);

	InitGPUMemInOut(OR_segLocM_x, byteSize, L"OR_segLocM_x");
	InitGPUMemInOut(OR_segLocM_y, byteSize, L"OR_segLocM_y");
	InitGPUMemInOut(OR_segLocM_z, byteSize, L"OR_segLocM_z");

	// Map the data so we can read it on CPU.
	float* mapSegLoc_x = nullptr;
	float* mapSegLoc_y = nullptr;
	float* mapSegLoc_z = nullptr;
	CD3DX12_RANGE readRange(0, 0);
	ThrowIfFailed(OR_segLocM_x[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegLoc_x)));
	ThrowIfFailed(OR_segLocM_y[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegLoc_y)));
	ThrowIfFailed(OR_segLocM_z[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegLoc_z)));

#if 0
	//TODO: test if memCopy or OpenMP is faster *need timers*
	//#pragma omp parallel for num_threads(10)
	for (int i = 0; i < SEGMENTCOUNT; ++i)
	{
		mapSegLoc_x[i] = segLocationSorted.x[i];
		mapSegLoc_y[i] = segLocationSorted.y[i];
		mapSegLoc_z[i] = segLocationSorted.z[i];
	}
#else
	memcpy(mapSegLoc_x, segLocation.x, SEGMENTCOUNT * sizeof(float));
	memcpy(mapSegLoc_y, segLocation.y, SEGMENTCOUNT * sizeof(float));
	memcpy(mapSegLoc_z, segLocation.z, SEGMENTCOUNT * sizeof(float));
#endif

	OR_segLocM_x[1]->Unmap(0, nullptr);
	OR_segLocM_y[1]->Unmap(0, nullptr);
	OR_segLocM_z[1]->Unmap(0, nullptr);

	///////////////////////////////////////////////////////////////////////////////////////////////////////

	BarriorCopyToGPU(mCommandList, OR_segLocM_x);
	BarriorCopyToGPU(mCommandList, OR_segLocM_y);
	BarriorCopyToGPU(mCommandList, OR_segLocM_z);
}
void UpdateClass::BuildComputeBuffersSpring() {
	UINT64 byteSize = SEGMENTCOUNT * 100 * sizeof(float);

	InitGPUMemInOut(OR_springDistance, byteSize, L"OR_springDistance");
	InitGPUMemInOut(OR_massID, byteSize, L"OR_massID");
	InitGPUMemInOut(OR_springConstant, byteSize, L"OR_springConstant");

	// Map the data so we can read it on CPU.
	float* mapSpringDistance = nullptr;
	float* mapMassID = nullptr;
	float* mapSpringConstant = nullptr;
	CD3DX12_RANGE readRange(0, 0);
	ThrowIfFailed(OR_springDistance[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSpringDistance)));
	ThrowIfFailed(OR_massID[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapMassID)));
	ThrowIfFailed(OR_springConstant[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSpringConstant)));

	memcpy(mapSpringDistance, springDistance, byteSize);
	memcpy(mapMassID, massID, byteSize);
	memcpy(mapSpringConstant, springConstant, byteSize);

	OR_springDistance[1]->Unmap(0, nullptr);
	OR_massID[1]->Unmap(0, nullptr);
	OR_springConstant[1]->Unmap(0, nullptr);

	///////////////////////////////////////////////////////////////////////////////////////////////////////

	BarriorCopyToGPU(mCommandList, OR_springDistance);
	BarriorCopyToGPU(mCommandList, OR_massID);
	BarriorCopyToGPU(mCommandList, OR_springConstant);
}
void UpdateClass::BuildComputeBuffersAnch2Seg() {
	UINT nAnchors = objMan->getNumAnchor();
	UINT nAnch2Seg = objMan->getNumAnch2Seg();

	InitGPUMemInOut(OR_anchor_x, nAnchors * sizeof(float), L"OR_anchor_x");
	InitGPUMemInOut(OR_anchor_y, nAnchors * sizeof(float), L"OR_anchor_y");
	InitGPUMemInOut(OR_anchor_z, nAnchors * sizeof(float), L"OR_anchor_z");
	InitGPUMemInOut(OR_numConSeg, nAnchors * sizeof(unsigned int), L"OR_numConSeg");
	InitGPUMemInOut(OR_anch2SegmentID, nAnch2Seg * sizeof(unsigned int), L"OR_anch2SegmentID");
	InitGPUMemInOut(OR_anch2SegDistance, nAnch2Seg * sizeof(float), L"OR_anch2SegDistance");
	InitGPUMemInOut(OR_anch2SegConst, nAnch2Seg * sizeof(float), L"OR_anch2SegConst");
	InitGPUMemInOut(OR_anchor_2SegStart, nAnchors * sizeof(unsigned int), L"OR_anchor_2SegStart");
	InitGPUMemInOut(OR_anchor_veloX, nAnchors * sizeof(float), L"OR_anchor_veloX");
	InitGPUMemInOut(OR_anchor_veloY, nAnchors * sizeof(float), L"OR_anchor_veloY");
	InitGPUMemInOut(OR_anchor_veloZ, nAnchors * sizeof(float), L"OR_anchor_veloZ");

	// Map the data so we can read it on CPU.
	float* mapAnchor_x = nullptr;
	float* mapAnchor_y = nullptr;
	float* mapAnchor_z = nullptr;
	unsigned int* mapNumConSeg = nullptr;
	unsigned int* mapAnch2SegmentID = nullptr;
	float* mapAnch2SegDistance = nullptr;
	float* mapAnch2SegConst = nullptr;
	unsigned int* mapAnchor_2SegStart = nullptr;
	float* mapAnchor_veloX = nullptr;
	float* mapAnchor_veloY = nullptr;
	float* mapAnchor_veloZ = nullptr;
	CD3DX12_RANGE readRange(0, 0);
	ThrowIfFailed(OR_anchor_x[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnchor_x)));
	ThrowIfFailed(OR_anchor_y[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnchor_y)));
	ThrowIfFailed(OR_anchor_z[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnchor_z)));
	ThrowIfFailed(OR_numConSeg[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapNumConSeg)));
	ThrowIfFailed(OR_anch2SegmentID[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnch2SegmentID)));
	ThrowIfFailed(OR_anch2SegDistance[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnch2SegDistance)));
	ThrowIfFailed(OR_anch2SegConst[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnch2SegConst)));
	ThrowIfFailed(OR_anchor_2SegStart[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnchor_2SegStart)));
	ThrowIfFailed(OR_anchor_veloX[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnchor_veloX)));
	ThrowIfFailed(OR_anchor_veloY[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnchor_veloY)));
	ThrowIfFailed(OR_anchor_veloZ[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnchor_veloZ)));

	memcpy(mapAnchor_x, objMan->getAddressAnchorX(), nAnchors * sizeof(float));
	memcpy(mapAnchor_y, objMan->getAddressAnchorY(), nAnchors * sizeof(float));
	memcpy(mapAnchor_z, objMan->getAddressAnchorZ(), nAnchors * sizeof(float));
	memcpy(mapNumConSeg, objMan->getAddressNumConSeg(), nAnchors * sizeof(unsigned int));
	memcpy(mapAnch2SegmentID, objMan->getAddressAnch2SegmentID(), nAnch2Seg * sizeof(unsigned int));
	memcpy(mapAnch2SegDistance, objMan->getAddressAnch2SegDistance(), nAnch2Seg * sizeof(float));
	memcpy(mapAnch2SegConst, objMan->getAddressAnch2SegConst(), nAnch2Seg * sizeof(float));
	memcpy(mapAnchor_2SegStart, objMan->getAddressAnch2SegmentStart(), nAnchors * sizeof(unsigned int));
	memcpy(mapAnchor_veloX, objMan->getAddressAnchVeloX(), nAnchors * sizeof(float));
	memcpy(mapAnchor_veloY, objMan->getAddressAnchVeloY(), nAnchors * sizeof(float));
	memcpy(mapAnchor_veloZ, objMan->getAddressAnchVeloZ(), nAnchors * sizeof(float));

	OR_anchor_x[1]->Unmap(0, nullptr);
	OR_anchor_y[1]->Unmap(0, nullptr);
	OR_anchor_z[1]->Unmap(0, nullptr);
	OR_numConSeg[1]->Unmap(0, nullptr);
	OR_anch2SegmentID[1]->Unmap(0, nullptr);
	OR_anch2SegDistance[1]->Unmap(0, nullptr);
	OR_anch2SegConst[1]->Unmap(0, nullptr);
	OR_anchor_2SegStart[1]->Unmap(0, nullptr);
	OR_anchor_veloX[1]->Unmap(0, nullptr);
	OR_anchor_veloY[1]->Unmap(0, nullptr);
	OR_anchor_veloZ[1]->Unmap(0, nullptr);

	///////////////////////////////////////////////////////////////////////////////////////////////////////

	BarriorCopyToGPU(mCommandList_Sim, OR_anchor_x);
	BarriorCopyToGPU(mCommandList_Sim, OR_anchor_y);
	BarriorCopyToGPU(mCommandList_Sim, OR_anchor_z);
	BarriorCopyToGPU(mCommandList_Sim, OR_numConSeg);
	BarriorCopyToGPU(mCommandList_Sim, OR_anch2SegmentID);
	BarriorCopyToGPU(mCommandList_Sim, OR_anch2SegDistance);
	BarriorCopyToGPU(mCommandList_Sim, OR_anch2SegConst);
	BarriorCopyToGPU(mCommandList_Sim, OR_anchor_2SegStart);
	BarriorCopyToGPU(mCommandList_Sim, OR_anchor_veloX);
	BarriorCopyToGPU(mCommandList_Sim, OR_anchor_veloY);
	BarriorCopyToGPU(mCommandList_Sim, OR_anchor_veloZ);
}

void UpdateClass::CloseComputeBuffers() {

	/*float* mapObjModify_x = nullptr;
	float* mapObjModify_y = nullptr;
	float* mapObjModify_z = nullptr;
	unsigned int* mapOrigninalIndex = nullptr;
	CD3DX12_RANGE readRange(0, SEGMENTCOUNT * sizeof(float));
	CD3DX12_RANGE readRange0(0, 0);
	OR_forwardVec_x[2]->Map(0, nullptr, reinterpret_cast<void**>(&mapObjModify_x));
	OR_forwardVec_y[2]->Map(0, &readRange, reinterpret_cast<void**>(&mapObjModify_y));
	OR_forwardVec_z[2]->Map(0, &readRange0, reinterpret_cast<void**>(&mapObjModify_z));
	OR_originalIndex[2]->Map(0, nullptr, reinterpret_cast<void**>(&mapOrigninalIndex));

	std::ofstream fout("results.txt");

	fout << "<Printing Out Final Results>" << std::endl;

	for (unsigned int i = 0; i < SEGMENTCOUNT; ++i)
	{
	fout << mapOrigninalIndex[i] << "(" << mapObjModify_x[i] << ", " << mapObjModify_y[i] << ", " << mapObjModify_z[i] << ")" << std::endl;
	}

	fout << "<End>" << std::endl;

	OR_forwardVec_x[2]->Unmap(0, nullptr);
	OR_forwardVec_y[2]->Unmap(0, nullptr);
	OR_forwardVec_z[2]->Unmap(0, nullptr);
	OR_originalIndex[2]->Unmap(0, nullptr);*/

	//Likely Not needed
	for (int heap = 0; heap < heapCount; heap++) {
		OR_originalIndex[heap].Reset();		// Sort::RW - Compute::R
		OR_segLocSorted_x[heap].Reset();		// Sort::RW - Compute::R
		OR_segLocSorted_y[heap].Reset();		// ^^^^^^^^^
		OR_segLocSorted_z[heap].Reset();		// ^^^^
		OR_cellStart[heap].Reset();			// Sort::RW - Compute::R
		OR_cellEnd[heap].Reset();				// Sort::RW - Compute::R
		OR_segRadiSort[heap].Reset();		//Addition

		OR_forwardVec_x[heap].Reset();	// Compute::RW
		OR_forwardVec_y[heap].Reset();	// ^^^^^^^^^
		OR_forwardVec_z[heap].Reset();	// ^^^^
		OR_colors_x[heap].Reset();			// Compute::RW
		OR_colors_y[heap].Reset();			// ^^^^^^^^^
		OR_colors_z[heap].Reset();			// ^^^^

		OR_segLocM_x[heap].Reset();			// Compute::RW
		OR_segLocM_y[heap].Reset();			// ^^^^^^^^^
		OR_segLocM_z[heap].Reset();			// ^^^^

		OR_springDistance[heap].Reset();	//
		OR_massID[heap].Reset();			//
		OR_springConstant[heap].Reset();	//

		OR_anchor_x[heap].Reset();			//
		OR_anchor_y[heap].Reset();			//
		OR_anchor_z[heap].Reset();			//
		OR_numConSeg[heap].Reset();			//
		OR_anch2SegmentID[heap].Reset();	//
		OR_anch2SegDistance[heap].Reset();	//
		OR_anch2SegConst[heap].Reset();		//
		OR_anchor_2SegStart[heap].Reset();	//
		OR_anchor_veloX[heap].Reset();		//
		OR_anchor_veloY[heap].Reset();		//
		OR_anchor_veloZ[heap].Reset();		//
	}
}


///Compute Functions//////////////////////////////////////////////
void UpdateClass::MainUpdateMethod(float dt, LeapSegments* ls)
{

	/// Sort Optimization ///
	StartCounter_ms(timer_a);
	CPUReSortObjects(HASH_ENTRIE, OBJECT_COUNT, segLocation);
	timeCumulativeSort += GetCounter_ms(timer_a);
	timeCumulative(&timeCumulativeSort, &timeModSort);
	
	/// NNS Interactions ///
	StartCounter_ms(timer_a);
	//The LJP - a quick implementation
	if ((!ComputeShaderActive) || 0) {
		force_searchList_CPU(segOriginalIndex, segLocationSorted, transVelocity, segColor, segCellStart, segCellEnd, 0, SEGMENT_COUNT, 
								param.FORCE_RANGE, param.INTER_E, param.INTER_S, param.NEG_FORCES_ON, param.FORCECOLOR_ON, avgSegmentsFoundLJP, avgSegmentsComputedLJP);
#if HAND_SPRING_PROPAGATE
		propagateHandForceAcrossSprings();
#endif
	}
	else {
		LJP_CS(); //Compute Shader Operation
	}
	timeCumulativeLJP += GetCounter_ms(timer_a);
	timeCumulative(&timeCumulativeLJP, &timeModLJP);
	
	//VRCollisionInteractions(0, 1); // Additional forward Vector modification so before the loc Update
	
	totalEnergy = 0.0f;
	
	//-----------------------------------------------------------------------------------------------------------
	//Script for shape transformation. 
	//-----------------------------------------------------------------------------------------------------------
	if (isFoldChanged) {
		UINT sfd;
		if (isFolded) {
			sfd = foldingID;
		}
		else {
			sfd = unfoldingID;
		}
		isFoldChanged = false;

		int i = objMan->getObjSegStart(foldedObjID) * MAXSPRING;
		int j = objMan->getObjSegStart(sfd) * MAXSPRING;
		for (; i < (objMan->getObjSegStart(foldedObjID) + objMan->getObjSegCount(foldedObjID)) * MAXSPRING; i++) {
			springDistance[i] = springDistance[j];
			j++;
		}

		i = objMan->getObjAnchorStart(foldedObjID);
		j = objMan->getObjAnchorStart(sfd);
		for (int k = 0; k < 4; k++) {
			for (int l = 0; l < objMan->getNumConSeg(i); l++) {
				objMan->setConSegDistance(i + k, l, objMan->getConSegDistance(j + k, l));
			}
		}
	}
	//-----------------------------------------------------------------------------------------------------------

	UINT lastObjIdx = objMan->getNumObj() - 1;
	if (lastObjIdx > 0) {
		if ((!ComputeShaderActive) || 1) {
			computeSpringObject();
		} else {
			Spring_CS();
		}

		if ((!ComputeShaderActive) || 1) {
			computeAnchorSegment();
		}
		else {
			Anch2Seg_CS();
		}

		computeAnchors();
		moveAnchors();
		
		//-----------------------------------------------------------------------------------------------------------
		//To show anchors
		//-----------------------------------------------------------------------------------------------------------
		/*UINT numAnch = objMan->getNumAnchor();
		for (int i = 0; i < numAnch; i++) {
			segLocation.x[SEGMENT_COUNT - numAnch + i] = objMan->getAnchorPos(i).x;
			segLocation.y[SEGMENT_COUNT - numAnch + i] = objMan->getAnchorPos(i).y;
			segLocation.z[SEGMENT_COUNT - numAnch + i] = objMan->getAnchorPos(i).z;
		}*/
		//-----------------------------------------------------------------------------------------------------------

		//-----------------------------------------------------------------------------------------------------------
		//Brownian Motion
		//-----------------------------------------------------------------------------------------------------------
		/*if (!(brownianIteration % 25))
			randomizeBrownian(0.03f);
		useBrownianMotion(20);
		brownianIteration++;*/
		//-----------------------------------------------------------------------------------------------------------
	}

	StartCounter_ms(timer_a);
	//Loop bellow handles boundary conditions, movement and updating the data that will be used to render
	int up;

	UINT numMovSeg;
	if (objType == 0) {
		numMovSeg = objPar1.numSegMove;
	}
	else if (objType == 1) {
		numMovSeg = objPar2.numSegMove;
	}
	lastSegmentPlus1 = objMan->getObjSegStart(lastObjIdx) + objMan->getObjSegCount(lastObjIdx);

	//-----------------------------------------------------------------------------------------------------------
	//To move some microtubule's end upward (to bend the microtubule).
	//-----------------------------------------------------------------------------------------------------------
	if (isForceApplied) {
		if (isForceChanged) {
			resultFile << "Force applied\n";
			isForceChanged = !isForceChanged;
		}
		for (UINT i = lastSegmentPlus1 - numMovSeg; i < lastSegmentPlus1; i++) {
			float up = (objType == 0) ? objPar1.upMove : objPar2.upMove;
			transVelocity.y[i] += up;
			segColor.x[i] = 1.0f;
			segColor.y[i] = 1.0f;
			segColor.z[i] = 0.0f;
		}
				
	} else if (isForceChanged) {
		resultFile << "Force removed\n";
		isForceChanged = !isForceChanged;

		testString = "Removed";//////test
	}
	//-----------------------------------------------------------------------------------------------------------

	/*if (resultFile.is_open())
		testString = "Write on the run OK.";//////test
	else
		testString = "It doesn't works.";//////test*/

	if ((!ComputeShaderActive)||1) { // CPU version of move
#pragma omp parallel for private (up)
		for (up = 0; up < lastSegmentPlus1; ++up)
		{

			XMFLOAT3 locVec, forVec;
			locVec.x = segLocation.x[up];
			locVec.y = segLocation.y[up];
			locVec.z = segLocation.z[up];
			forVec.x = transVelocity.x[up];
			forVec.y = transVelocity.y[up];
			forVec.z = transVelocity.z[up];

			//totalEnergy += sqrt(forVec.x*forVec.x + forVec.y*forVec.y + forVec.z*forVec.z);

			//Checks boundary and alters location and forward vector if needed
			CheckBoundary(param.BOUNDED, sphereRadius, locVec, forVec);

			//Movement below

			//Drag
			applyDragIF(forVec);

			//Move Forward
			locVec.x += forVec.x;
			locVec.y += forVec.y;
			locVec.z += forVec.z;

			//Update Locations
			segLocation.x[up] = locVec.x;
			segLocation.y[up] = locVec.y;
			segLocation.z[up] = locVec.z;
			transVelocity.x[up] = forVec.x;
			transVelocity.y[up] = forVec.y;
			transVelocity.z[up] = forVec.z;
		}
	}
	else {
		Move_CS(); //Compute Shader Operation
	}

	//If more than one of the 3 things below are enabled they will interfere with eachother
	int choice = 3;
	if (choice == 1) { //Stationary Tubes
		Tube();
	}
	if (choice == 2) {
		if (m_pOpenVRSystem) {
			TubeWand(vc.controllerMatrixSave, vc.lastControllerLoc, 1, 2); // Forcing a location so happens last
		}
	}
	if (choice == 3) {

		if (ls->getNumHands() > 0) {
			for (up = 0; up < ls->getNumSeg(); ++up) {
				segLocation.x[up] = ls->getX(up);
				segLocation.y[up] = ls->getY(up);
				segLocation.z[up] = ls->getZ(up);
			}
		}
		else {
			for (up = 0; up < ls->getNumSeg(); ++up) {
				segLocation.x[up] = 0.0f;
				segLocation.y[up] = -15.0f;
				segLocation.z[up] = 0.0f;
			}
		}
	}

	//-----------------------------------------------------------------------------------------------------------
	//For recording.
	//-----------------------------------------------------------------------------------------------------------
	XMFLOAT3 centerMovSeg = centerSpringMass(lastSegmentPlus1 - numMovSeg, lastSegmentPlus1 - 1);
	XMFLOAT3 centerStartMT;

	UINT start0, num0;
	UINT halfL = objPar1.sectionLength / 2;
	if (objPar1.sectionLength % 2) {
		start0 = halfL * 26 + objMan->getObjSegStart(lastObjIdx);
		num0 = 26;
	} else {
		start0 = (halfL - 1) * 26 + objMan->getObjSegStart(lastObjIdx);
		num0 = 52;
	}
	centerStartMT = centerSpringMass(start0, start0 + num0);

	//To show Flexural Rigidity at glance.
	//EI = F * L^3 / (3  * y)
	double EI = upForce * MTlength * MTlength * MTlength / (3 * (centerMovSeg.y - initY) * segDiameter);

	//testString = "EI=" + to_string(EI * 1.0E40) + "E40";//////test

	if (isRecording) {
		if (objType == 0) {
			resultFile << to_string(iCycle) + "," +
				to_string(centerMovSeg.x) + "," +
				to_string(centerMovSeg.y) + "," +
				to_string(centerMovSeg.z) + "," +
				to_string(EI * 1.0E40) + "," +
				to_string(centerStartMT.x) + "," +
				to_string(centerStartMT.y) + "," +
				to_string(centerStartMT.z) + "\n";
		} else if (objType == 1) {
			resultFile << to_string(iCycle);
			for (UINT i = objMan->getObjSegStart(lastObjIdx); i < objMan->getObjSegStart(lastObjIdx) + objMan->getObjSegCount(lastObjIdx); i++) {
				resultFile << "," + to_string(segLocation.y[i]);
			}
			resultFile << "\n";
		}
			iCycle++;
	}
	//-----------------------------------------------------------------------------------------------------------

	/*setHapticFeedback(hapticForceThreshold);
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 6; j++) {
			hd1.accNumSeg[i][j] = 0;
			hd1.accAvTemp[i][j] = 0;
		}*/

	timeCumulativeMove += GetCounter_ms(timer_a);
	timeCumulative(&timeCumulativeMove, &timeModMove);
}

void UpdateClass::propagateHandForceAcrossSprings() {
	int seg;
#pragma omp parallel for
	for (seg = LEAP_SEG_CNT; seg < OBJECT_COUNT; ++seg)
	{
		XMFLOAT3 forceAcc = XMFLOAT3(transVelocity.x[seg + OBJECT_COUNT], transVelocity.y[seg + OBJECT_COUNT], transVelocity.z[seg + OBJECT_COUNT]);
		int foundCnt = 0;
		while (massID[seg * MAXSPRING + foundCnt] != UINT_MAX)
		{
			UINT target = massID[seg * MAXSPRING + foundCnt];

			XMFLOAT3 neighboorVelocity = XMFLOAT3(transVelocity.x[target + OBJECT_COUNT], transVelocity.y[target + OBJECT_COUNT], transVelocity.z[target + OBJECT_COUNT]);
			forceAcc = add(forceAcc, neighboorVelocity);
			foundCnt++;
		}

		/*float magnitude = sqrtf(dot(forceAcc));
		XMFLOAT3 color = GetColour(magnitude, -0.2f, 1.0f);
		segColor.x[seg] = color.x;
		segColor.y[seg] = color.y;
		segColor.z[seg] = color.z;*/

		float scale = 0.1f;
		transVelocity.x[seg] += forceAcc.x * scale;
		transVelocity.y[seg] += forceAcc.y * scale;
		transVelocity.z[seg] += forceAcc.z * scale;
	}
}

void UpdateClass::SimDataToInstData(int renderNumber) {
	int up = 0;
	auto ri = mOpaqueRitems[renderNumber];
	int cnt = ri->InstanceCount;
#pragma omp parallel for private (up)
	for (up = 0; up < SEGMENT_COUNT; ++up)
	{
		float scale = segRadius[up] * 1.2f; //a bit bigger to look more full
		ri->Instances[up].World = XMFLOAT4X4(
			scale, 0.0f, 0.0f, 0.0f,
			0.0f, scale, 0.0f, 0.0f,
			0.0f, 0.0f, scale, 0.0f,
		segLocation.x[up], segLocation.y[up], segLocation.z[up], 1.0f);
		ri->Instances[up].Color.x = segColor.x[up];
		ri->Instances[up].Color.y = segColor.y[up];
		ri->Instances[up].Color.z = segColor.z[up];
	}
}

//Clears and resorts a CPU hash table
void UpdateClass::CPUReSortObjects(int hash_entries, int elements, floatArray3 &locations) {
		//sortListCPU(segLocation, segLocationSorted, segOriginalIndex, segCellStart, segCellEnd);
		Radix1DSeg2Kv(segLocation, segLocationSorted, segOriginalIndex, segCellStart, segCellEnd, OBJECT_COUNT, bitCountNeeded);
}

//Direction and velocity have been condenced into one vector - could also use a normalized direction vector with a velocity value - particles have no mass here
void UpdateClass::force_searchList_CPU(const UINT * originalIndex, const floatArray3 segLocSorted, floatArray3 objDataToModifiy, floatArray3 colors, const UINT *cellStart, const UINT *cellEnd, int startOBJ, int endOBJ,
	float forceRange, float e, float s, UINT negForces, UINT COLOR_FORCE, float *avgSegmentsFound, float *avgSegmentsComputed) {

	//For performance information
	int segNodeAvg = 0; int segNodeAvgCalc = 0;

	//Loop over all segments - make sure to keep thread safe with the OpenMP loop
	int segNode;
#pragma omp parallel for private (segNode)
	for (segNode = startOBJ; segNode < endOBJ; segNode++) {

		//For performance information
		int countOfSeg = 0; int countOfSeg2 = 0;

		//Used for coloring based on forces acting on a segment
		forceSave[segNode] = 0.0f;

		//Local accumulator
		XMFLOAT3 pull_Singletotal = { 0.0f, 0.0f, 0.0f };
		XMFLOAT3 pull_Handtotal = { 0.0f, 0.0f, 0.0f };

		//Saving location of segment of focus for performance reasons (more critical on GPU)
		UINT segNodeIndex = originalIndex[segNode];
		float xLoc = segLocSorted.x[segNode];
		float yLoc = segLocSorted.y[segNode];
		float zLoc = segLocSorted.z[segNode];
		float segR = segRadius[segNodeIndex];
		unsigned int tubeGrid = hashCPU(xLoc, yLoc, zLoc);

		//Loop over the 27 locations in a 3x3x3 box
		for (int t = 0; t < 27; t++) {
			//The below line computes 3D grid locations using a basic incrementing iterator and the hash dimentions 
			unsigned int targetGrid = (CUBE_COUNTx_1D*CUBE_COUNTz_1D*((t / 9) - 1)) +
				((tubeGrid - CUBE_COUNTz_1D + (((t % 9) / 3)*CUBE_COUNTz_1D)) - 1 + ((t) % 3));
			if (targetGrid < HASH_ENTRIE && targetGrid >= 0) {
				int startIndex = cellStart[targetGrid];
				int endIndex = cellEnd[targetGrid];
				if (startIndex != 0xffffffff)          // cell is not empty
				{
					for (int cNode = startIndex; cNode < endIndex; cNode++) {
						countOfSeg2++;
						//Make sure it doesnt compute against itself
						if (cNode != segNode && cNode < endOBJ) {
							//Call the interaction algorithm
#if 0
							countOfSeg += LJP3(segNode, cNode, xLoc, yLoc, zLoc, segLocSorted, forceRange, e, s, negForces, pull_Singletotal, forceSave, originalIndex);
#else
							float radiusCNode = segRadiusSorted[cNode]; // TODO
																		//countOfSeg += LJP3_DR(segNode, cNode, xLoc, yLoc, zLoc, segR, segLocSorted, radiusCNode, forceRange, s, negForces, pull_Singletotal, forceSave);
							countOfSeg += LJP3_Hand(segNode, cNode, segNodeIndex, xLoc, yLoc, zLoc, segR, segLocSorted, radiusCNode, forceRange, s, negForces, pull_Singletotal, pull_Handtotal, forceSave, originalIndex);
#endif
						}
					}
				}
			}
		}
		objDataToModifiy.x[segNodeIndex] += pull_Singletotal.x;
		objDataToModifiy.y[segNodeIndex] += pull_Singletotal.y;
		objDataToModifiy.z[segNodeIndex] += pull_Singletotal.z;

#if HAND_SPRING_PROPAGATE
		objDataToModifiy.x[segNodeIndex + OBJECT_COUNT] = pull_Handtotal.x;
		objDataToModifiy.y[segNodeIndex + OBJECT_COUNT] = pull_Handtotal.y;
		objDataToModifiy.z[segNodeIndex + OBJECT_COUNT] = pull_Handtotal.z;
#endif
		//Applying color to the segment
		if (COLOR_FORCE) {
			XMFLOAT3 color;
			//Color based on the accumulation of positive and negative forces acting on the segment
			color = GetColour(forceSave[segNode], -0.1f, 0.6f);
			//Setting the color to the global structure
			if (segNodeIndex == holdingIdx[0] || segNodeIndex == holdingIdx[1]) {
				colors.x[segNodeIndex] = color.x*2.0f;
				colors.y[segNodeIndex] = color.y*2.0f;
				colors.z[segNodeIndex] = color.z*2.0f;
			}
			else {
				colors.x[segNodeIndex] = color.x;
				colors.y[segNodeIndex] = color.y;
				colors.z[segNodeIndex] = color.z;
			}
		}

		//Not Parallel Safe... But seems to work
		segNodeAvg += countOfSeg2;
		segNodeAvgCalc += countOfSeg;

	}

	//For understanding performance hits - computing this does add some overhead above
	*avgSegmentsFound = segNodeAvg / (float)endOBJ;
	*avgSegmentsComputed = segNodeAvgCalc / (float)endOBJ;
}

void UpdateClass::LJP_CS() {
	if (ComputeShaderActive) {
		RecordComputeWork();

		/*If I want to make a full GPU process, then this is how*/
		//RecordComputeWorkSpring();
		//All other RecordComputeWork...()
		//This is called queuing work.

		//Code below is executed after finishing queuing.
		ThrowIfFailed(mCommandList_Sim->Close());
		ID3D12CommandList* cmdsListsCS[] = { mCommandList_Sim.Get() };
		mCommandQueue_Sim->ExecuteCommandLists(_countof(cmdsListsCS), cmdsListsCS);
		FlushCommandQueueCSsim();

		CShaderDataOutput(); 
	}
}
void UpdateClass::RecordComputeWork() {

	ThrowIfFailed(CmdListAlloc_Sim->Reset());
	ThrowIfFailed(mCommandList_Sim->Reset(CmdListAlloc_Sim.Get(), mPSOs["forceSoA"].Get()));

	CShaderDataInput();

	unsigned int startOBJ = 0;
	mCommandList_Sim->SetComputeRootSignature(mRootSignatureCS.Get());
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &startOBJ, 0);
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &SEGMENT_COUNT, 1);
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &param.FORCE_RANGE, 2);
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &param.INTER_E, 3);
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &param.INTER_S, 4);
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &param.NEG_FORCES_ON, 5);
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &param.NEG_F_MULTIPLY, 6);
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &param.FORCECOLOR_ON, 7);
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &CUBE, 8);
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &GRIDxHALF, 9);
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &GRIDzHALF, 10);
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &GRIDyDEPTH_BUFFER, 11);
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &CUBE_COUNTx_1D, 12);
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &CUBE_COUNTz_1D, 13);
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &HASH_ENTRIE, 14);
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &LEAP_SEG_CNT, 15);

	mCommandList_Sim->SetComputeRootUnorderedAccessView(1, OR_originalIndex[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(2, OR_segLocSorted_x[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(3, OR_segLocSorted_y[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(4, OR_segLocSorted_z[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(5, OR_cellStart[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(6, OR_cellEnd[0]->GetGPUVirtualAddress());
	
	mCommandList_Sim->SetComputeRootUnorderedAccessView(7, OR_forwardVec_x[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(8, OR_forwardVec_y[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(9, OR_forwardVec_z[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(10, OR_colors_x[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(11, OR_colors_y[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(12, OR_colors_z[0]->GetGPUVirtualAddress());

	mCommandList_Sim->SetComputeRootUnorderedAccessView(13, OR_segRadiSort[0]->GetGPUVirtualAddress());		//Addition

	// Get a timestamp at the start of the command list.
	//const UINT timestampHeapIndex = 2 * mCurrBackBuffer;
	//mCommandList_Sim->EndQuery(m_timestampQueryHeaps.Get(), D3D12_QUERY_TYPE_TIMESTAMP, timestampHeapIndex);
	// Get a timestamp at the start of the command list.
	const UINT timestampHeapIndex = 0;
	mCommandList_Sim->EndQuery(mTimestampQueryHeaps_Sim.Get(), D3D12_QUERY_TYPE_TIMESTAMP, timestampHeapIndex);

	mCommandList_Sim->Dispatch(static_cast<int>(ceil(SEGMENTCOUNT / 256.0f)), 1, 1);
	mCommandList_Sim->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::UAV(nullptr)); ///?

	if (1) {
		// Schedule to copy the data to the default buffer to the readback buffer.
		BarriorCopyFromGPU(mCommandList_Sim, OR_forwardVec_x);
		BarriorCopyFromGPU(mCommandList_Sim, OR_forwardVec_y);
		BarriorCopyFromGPU(mCommandList_Sim, OR_forwardVec_z);
		BarriorCopyFromGPU(mCommandList_Sim, OR_colors_x);
		BarriorCopyFromGPU(mCommandList_Sim, OR_colors_y);
		BarriorCopyFromGPU(mCommandList_Sim, OR_colors_z);
	}

	// Get a timestamp at the end of the command list and resolve the query data.
	mCommandList_Sim->EndQuery(mTimestampQueryHeaps_Sim.Get(), D3D12_QUERY_TYPE_TIMESTAMP, timestampHeapIndex + 1);
	mCommandList_Sim->ResolveQueryData(mTimestampQueryHeaps_Sim.Get(), D3D12_QUERY_TYPE_TIMESTAMP, timestampHeapIndex, 2, mTimestampResultBuffers_Sim.Get(), timestampHeapIndex * sizeof(UINT64));
}
void UpdateClass::CShaderDataInput() {
	// Map the data so we can read it on CPU.

	unsigned int* mapOriginalIndex = nullptr;
	float* mapSegSorted_x = nullptr;
	float* mapSegSorted_y = nullptr;
	float* mapSegSorted_z = nullptr;
	float* mapForVec_x = nullptr;
	float* mapForVec_y = nullptr;
	float* mapForVec_z = nullptr;
	unsigned int* mapCellStart = nullptr;
	unsigned int* mapCellEnd = nullptr;
	float* mapSegRadiSort = nullptr;		//Addition
	CD3DX12_RANGE readRange(0, 0);
	ThrowIfFailed(OR_originalIndex[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapOriginalIndex)));
	ThrowIfFailed(OR_segLocSorted_x[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegSorted_x)));
	ThrowIfFailed(OR_segLocSorted_y[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegSorted_y)));
	ThrowIfFailed(OR_segLocSorted_z[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegSorted_z)));
	ThrowIfFailed(OR_forwardVec_x[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapForVec_x)));
	ThrowIfFailed(OR_forwardVec_y[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapForVec_y)));
	ThrowIfFailed(OR_forwardVec_z[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapForVec_z)));
	ThrowIfFailed(OR_cellStart[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapCellStart)));
	ThrowIfFailed(OR_cellEnd[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapCellEnd)));
	ThrowIfFailed(OR_segRadiSort[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegRadiSort)));		//Addition

#if 0
	//TODO: test if memCopy or OpenMP is faster *need timers*
//#pragma omp parallel for num_threads(10)
	for (int i = 0; i < SEGMENTCOUNT; ++i)
	{
		mapOriginalIndex[i] = segOriginalIndex[i];
		mapSegSorted_x[i] = segLocationSorted.x[i];
		mapSegSorted_y[i] = segLocationSorted.y[i];
		mapSegSorted_z[i] = segLocationSorted.z[i];
		mapForVec_x[i] = transVelocity.x[i];
		mapForVec_y[i] = transVelocity.y[i];
		mapForVec_z[i] = transVelocity.z[i];
	}
//#pragma omp parallel for num_threads(10)
	for (int i = 0; i < HASH_ENTRIE; ++i)
	{
		mapCellStart[i] = segCellStart[i];
		mapCellEnd[i] = segCellEnd[i];
	}

#else

	memcpy(mapOriginalIndex, segOriginalIndex, SEGMENTCOUNT * sizeof(UINT));
	memcpy(mapSegSorted_x, segLocationSorted.x, SEGMENTCOUNT * sizeof(float));
	memcpy(mapSegSorted_y, segLocationSorted.y, SEGMENTCOUNT * sizeof(float));
	memcpy(mapSegSorted_z, segLocationSorted.z, SEGMENTCOUNT * sizeof(float));
	memcpy(mapForVec_x, transVelocity.x, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(mapForVec_y, transVelocity.y, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(mapForVec_z, transVelocity.z, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(mapSegRadiSort, segRadiusSorted, SEGMENTCOUNT * sizeof(float));		//Addition

	memcpy(mapCellStart, segCellStart, HASH_ENTRIE * sizeof(UINT));
	memcpy(mapCellEnd, segCellEnd, HASH_ENTRIE * sizeof(UINT));

#endif

	OR_originalIndex[1]->Unmap(0, nullptr);
	OR_segLocSorted_x[1]->Unmap(0, nullptr);
	OR_segLocSorted_y[1]->Unmap(0, nullptr);
	OR_segLocSorted_z[1]->Unmap(0, nullptr);
	OR_forwardVec_x[1]->Unmap(0, nullptr);
	OR_forwardVec_y[1]->Unmap(0, nullptr);
	OR_forwardVec_z[1]->Unmap(0, nullptr);
	OR_cellStart[1]->Unmap(0, nullptr);
	OR_cellEnd[1]->Unmap(0, nullptr);
	OR_segRadiSort[1]->Unmap(0, nullptr);		//Addition

	///////////////////////////////////////////////////////////////////////////////////////////////////////

	BarriorCopyToGPU(mCommandList_Sim, OR_originalIndex);
	BarriorCopyToGPU(mCommandList_Sim, OR_segLocSorted_x);
	BarriorCopyToGPU(mCommandList_Sim, OR_segLocSorted_y);
	BarriorCopyToGPU(mCommandList_Sim, OR_segLocSorted_z);
	BarriorCopyToGPU(mCommandList_Sim, OR_cellStart);
	BarriorCopyToGPU(mCommandList_Sim, OR_cellEnd);
	BarriorCopyToGPU(mCommandList_Sim, OR_forwardVec_x);
	BarriorCopyToGPU(mCommandList_Sim, OR_forwardVec_y);
	BarriorCopyToGPU(mCommandList_Sim, OR_forwardVec_z);
	BarriorCopyToGPU(mCommandList_Sim, OR_segRadiSort);		//Addition
}
void UpdateClass::CShaderDataOutput() {

	float* mapObjModify_x = nullptr;
	float* mapObjModify_y = nullptr;
	float* mapObjModify_z = nullptr;
	float* mapColor_x = nullptr;
	float* mapColor_y = nullptr;
	float* mapColor_z = nullptr;
	ThrowIfFailed(OR_forwardVec_x[2]->Map(0, nullptr, reinterpret_cast<void**>(&mapObjModify_x)));
	ThrowIfFailed(OR_forwardVec_y[2]->Map(0, nullptr, reinterpret_cast<void**>(&mapObjModify_y)));
	ThrowIfFailed(OR_forwardVec_z[2]->Map(0, nullptr, reinterpret_cast<void**>(&mapObjModify_z)));
	ThrowIfFailed(OR_colors_x[2]->Map(0, nullptr, reinterpret_cast<void**>(&mapColor_x)));
	ThrowIfFailed(OR_colors_y[2]->Map(0, nullptr, reinterpret_cast<void**>(&mapColor_y)));
	ThrowIfFailed(OR_colors_z[2]->Map(0, nullptr, reinterpret_cast<void**>(&mapColor_z)));

//Debugging
#if 0 
	force_searchList_CPU(segOriginalIndex, segLocationSorted, transVelocity, segColor, segCellStart, segCellEnd, 0, SEGMENT_COUNT, FORCE_RANGE, INTER_E, INTER_S, NEG_FORCES_ON, FORCECOLOR_ON, avgSegmentsFoundLJP, avgSegmentsComputedLJP);
	XMFLOAT3 Diff = { 0.0f, 0.0f, 0.0f};
	float xCDiff = 0.0f;
	float yCDiff = 0.0f;
	float zCDiff = 0.0f;
	for (unsigned int i = 0; i < SEGMENTCOUNT; ++i)
	{
		Diff.x += abs(transVelocity.x[i] - mapObjModify_x[i]);
		Diff.y += abs(transVelocity.y[i] - mapObjModify_y[i]);
		Diff.z += abs(transVelocity.z[i] - mapObjModify_z[i]);
		xCDiff += abs(segColor.x[i] - mapColor_x[i]);
		yCDiff += abs(segColor.y[i] - mapColor_y[i]);
		zCDiff += abs(segColor.z[i] - mapColor_z[i]);
	}
	Diff.x += 0.0f;
#endif

#if 0
//#pragma omp parallel for num_threads(10)
	for (int i = 0; i < SEGMENTCOUNT; ++i)
	{
		transVelocity.x[i] = mapObjModify_x[i];
		transVelocity.y[i] = mapObjModify_y[i];
		transVelocity.z[i] = mapObjModify_z[i];
		segColor.x[i] = mapColor_x[i];
		segColor.y[i] = mapColor_y[i];
		segColor.z[i] = mapColor_z[i];
	}
#else
	memcpy(transVelocity.x, mapObjModify_x, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(transVelocity.y, mapObjModify_y, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(transVelocity.z, mapObjModify_z, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(segColor.x, mapColor_x, SEGMENTCOUNT * sizeof(float));
	memcpy(segColor.y, mapColor_y, SEGMENTCOUNT * sizeof(float));
	memcpy(segColor.z, mapColor_z, SEGMENTCOUNT * sizeof(float));

#endif
	
	/*OR_forwardVec_x[2]->Unmap(0, nullptr);
	OR_forwardVec_y[2]->Unmap(0, nullptr);
	OR_forwardVec_z[2]->Unmap(0, nullptr);*/
	OR_colors_x[2]->Unmap(0, nullptr);
	OR_colors_y[2]->Unmap(0, nullptr);
	OR_colors_z[2]->Unmap(0, nullptr);

}

void UpdateClass::Spring_CS() {
	if (ComputeShaderActive) {
		RecordComputeWorkSpring();
		ThrowIfFailed(mCommandList_Sim->Close());
		ID3D12CommandList* cmdsListsCS[] = { mCommandList_Sim.Get() };
		mCommandQueue_Sim->ExecuteCommandLists(_countof(cmdsListsCS), cmdsListsCS);
		FlushCommandQueueCSsim();

		CShaderDataOutputSpring();
	}
}
void UpdateClass::RecordComputeWorkSpring() {
	ThrowIfFailed(CmdListAlloc_Sim->Reset());
	ThrowIfFailed(mCommandList_Sim->Reset(CmdListAlloc_Sim.Get(), mPSOs["springSeg"].Get()));

	CShaderDataInputSpring();

	UINT startIdx = objMan->getObjSegStart(1);
	UINT endIdx = objMan->getObjSegCount(objMan->getNumObj() - 1) + objMan->getObjSegStart(objMan->getNumObj() - 1);
	UINT SPRINGCOLOR = 1;

	mCommandList_Sim->SetComputeRootSignature(mRootSignatureSpringCS.Get());

	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &startIdx, 0);
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &endIdx, 1);
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &MAXSPRING, 2);
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &SPRINGCOLOR, 3);
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &colorRange, 4);

	mCommandList_Sim->SetComputeRootUnorderedAccessView(1, OR_segLocM_x[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(2, OR_segLocM_y[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(3, OR_segLocM_z[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(4, OR_forwardVec_x[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(5, OR_forwardVec_y[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(6, OR_forwardVec_z[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(7, OR_springDistance[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(8, OR_massID[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(9, OR_springConstant[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(10, OR_colors_x[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(11, OR_colors_y[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(12, OR_colors_z[0]->GetGPUVirtualAddress());

	mCommandList_Sim->Dispatch(static_cast<int>(ceil(SEGMENTCOUNT / 256.0f)), 1, 1);
	mCommandList_Sim->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::UAV(nullptr)); ///?

																				   // Schedule to copy the data to the default buffer to the readback buffer.
	BarriorCopyFromGPU(mCommandList_Sim, OR_forwardVec_x);
	BarriorCopyFromGPU(mCommandList_Sim, OR_forwardVec_y);
	BarriorCopyFromGPU(mCommandList_Sim, OR_forwardVec_z);
	BarriorCopyFromGPU(mCommandList_Sim, OR_colors_x);
	BarriorCopyFromGPU(mCommandList_Sim, OR_colors_y);
	BarriorCopyFromGPU(mCommandList_Sim, OR_colors_z);
}
void UpdateClass::CShaderDataInputSpring() {
	// Map the data so we can read it on CPU.

	float* mapSegLoc_x = nullptr;
	float* mapSegLoc_y = nullptr;
	float* mapSegLoc_z = nullptr;
	float* mapForVec_x = nullptr;
	float* mapForVec_y = nullptr;
	float* mapForVec_z = nullptr;
	float* mapSpringDistance = nullptr;
	unsigned int* mapMassID = nullptr;
	float* mapSpringConstant = nullptr;

	CD3DX12_RANGE readRange(0, 0);
	ThrowIfFailed(OR_segLocM_x[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegLoc_x)));
	ThrowIfFailed(OR_segLocM_y[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegLoc_y)));
	ThrowIfFailed(OR_segLocM_z[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegLoc_z)));
	ThrowIfFailed(OR_forwardVec_x[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapForVec_x)));
	ThrowIfFailed(OR_forwardVec_y[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapForVec_y)));
	ThrowIfFailed(OR_forwardVec_z[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapForVec_z)));
	ThrowIfFailed(OR_springDistance[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSpringDistance)));
	ThrowIfFailed(OR_massID[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapMassID)));
	ThrowIfFailed(OR_springConstant[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSpringConstant)));

	memcpy(mapSegLoc_x, segLocation.x, SEGMENTCOUNT * sizeof(float));
	memcpy(mapSegLoc_y, segLocation.y, SEGMENTCOUNT * sizeof(float));
	memcpy(mapSegLoc_z, segLocation.z, SEGMENTCOUNT * sizeof(float));
	memcpy(mapForVec_x, transVelocity.x, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(mapForVec_y, transVelocity.y, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(mapForVec_z, transVelocity.z, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(mapSpringDistance, springDistance, SEGMENTCOUNT * 100 * sizeof(float));
	memcpy(mapMassID, massID, SEGMENTCOUNT * 100 * sizeof(UINT));
	memcpy(mapSpringConstant, springConstant, SEGMENTCOUNT * 100 * sizeof(float));

	OR_segLocM_x[1]->Unmap(0, nullptr);
	OR_segLocM_y[1]->Unmap(0, nullptr);
	OR_segLocM_z[1]->Unmap(0, nullptr);
	OR_forwardVec_x[1]->Unmap(0, nullptr);
	OR_forwardVec_y[1]->Unmap(0, nullptr);
	OR_forwardVec_z[1]->Unmap(0, nullptr);
	OR_springDistance[1]->Unmap(0, nullptr);
	OR_massID[1]->Unmap(0, nullptr);
	OR_springConstant[1]->Unmap(0, nullptr);

	BarriorCopyToGPU(mCommandList_Sim, OR_segLocM_x);
	BarriorCopyToGPU(mCommandList_Sim, OR_segLocM_y);
	BarriorCopyToGPU(mCommandList_Sim, OR_segLocM_z);
	BarriorCopyToGPU(mCommandList_Sim, OR_forwardVec_x);
	BarriorCopyToGPU(mCommandList_Sim, OR_forwardVec_y);
	BarriorCopyToGPU(mCommandList_Sim, OR_forwardVec_z);
	BarriorCopyToGPU(mCommandList_Sim, OR_springDistance);
	BarriorCopyToGPU(mCommandList_Sim, OR_massID);
	BarriorCopyToGPU(mCommandList_Sim, OR_springConstant);
}
void UpdateClass::CShaderDataOutputSpring() {
	float* mapObjModify_x = nullptr;
	float* mapObjModify_y = nullptr;
	float* mapObjModify_z = nullptr;
	float* mapColor_x = nullptr;
	float* mapColor_y = nullptr;
	float* mapColor_z = nullptr;
	CD3DX12_RANGE readRange(0, 0);
	ThrowIfFailed(OR_forwardVec_x[2]->Map(0, &readRange, reinterpret_cast<void**>(&mapObjModify_x)));
	ThrowIfFailed(OR_forwardVec_y[2]->Map(0, &readRange, reinterpret_cast<void**>(&mapObjModify_y)));
	ThrowIfFailed(OR_forwardVec_z[2]->Map(0, &readRange, reinterpret_cast<void**>(&mapObjModify_z)));
	ThrowIfFailed(OR_colors_x[2]->Map(0, &readRange, reinterpret_cast<void**>(&mapColor_x)));
	ThrowIfFailed(OR_colors_y[2]->Map(0, &readRange, reinterpret_cast<void**>(&mapColor_y)));
	ThrowIfFailed(OR_colors_z[2]->Map(0, &readRange, reinterpret_cast<void**>(&mapColor_z)));

	memcpy(transVelocity.x, mapObjModify_x, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(transVelocity.y, mapObjModify_y, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(transVelocity.z, mapObjModify_z, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(segColor.x, mapColor_x, SEGMENTCOUNT * sizeof(float));
	memcpy(segColor.y, mapColor_y, SEGMENTCOUNT * sizeof(float));
	memcpy(segColor.z, mapColor_z, SEGMENTCOUNT * sizeof(float));

	OR_forwardVec_x[2]->Unmap(0, nullptr);
	OR_forwardVec_y[2]->Unmap(0, nullptr);
	OR_forwardVec_z[2]->Unmap(0, nullptr);
	OR_colors_x[2]->Unmap(0, nullptr);
	OR_colors_y[2]->Unmap(0, nullptr);
	OR_colors_z[2]->Unmap(0, nullptr);
}

void UpdateClass::Anch2Seg_CS() {
	if (ComputeShaderActive) {
		RecordComputeWorkAnch2Seg();
		ThrowIfFailed(mCommandList_Sim->Close());
		ID3D12CommandList* cmdsListsCS[] = { mCommandList_Sim.Get() };
		mCommandQueue_Sim->ExecuteCommandLists(_countof(cmdsListsCS), cmdsListsCS);
		FlushCommandQueueCSsim();

		CShaderDataOutputAnch2Seg();
	}
}
void UpdateClass::RecordComputeWorkAnch2Seg() {
	ThrowIfFailed(CmdListAlloc_Sim->Reset());
	ThrowIfFailed(mCommandList_Sim->Reset(CmdListAlloc_Sim.Get(), mPSOs["anch2Seg"].Get()));

	CShaderDataInputAnch2Seg();

	UINT numAnchors = objMan->getNumAnchor();

	mCommandList_Sim->SetComputeRootSignature(mRootSignatureAnch2SegCS.Get());

	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &numAnchors, 0);
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &fixAnchors, 1);
	
	mCommandList_Sim->SetComputeRootUnorderedAccessView(1, OR_anchor_x[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(2, OR_anchor_y[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(3, OR_anchor_z[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(4, OR_numConSeg[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(5, OR_anch2SegmentID[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(6, OR_anch2SegDistance[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(7, OR_anch2SegConst[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(8, OR_anchor_2SegStart[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(9, OR_segLocM_x[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(10, OR_segLocM_y[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(11, OR_segLocM_z[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(12, OR_anchor_veloX[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(13, OR_anchor_veloY[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(14, OR_anchor_veloZ[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(15, OR_forwardVec_x[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(16, OR_forwardVec_y[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(17, OR_forwardVec_z[0]->GetGPUVirtualAddress());

	mCommandList_Sim->Dispatch(static_cast<int>(ceil(SEGMENTCOUNT / 256.0f)), 1, 1);
	mCommandList_Sim->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::UAV(nullptr)); ///?

	// Schedule to copy the data to the default buffer to the readback buffer.
	BarriorCopyFromGPU(mCommandList_Sim, OR_anchor_veloX);
	BarriorCopyFromGPU(mCommandList_Sim, OR_anchor_veloY);
	BarriorCopyFromGPU(mCommandList_Sim, OR_anchor_veloZ);
	BarriorCopyFromGPU(mCommandList_Sim, OR_forwardVec_x);
	BarriorCopyFromGPU(mCommandList_Sim, OR_forwardVec_y);
	BarriorCopyFromGPU(mCommandList_Sim, OR_forwardVec_z);
}
void UpdateClass::CShaderDataInputAnch2Seg() {
	// Map the data so we can read it on CPU.

	float* mapAnchor_x = nullptr;
	float* mapAnchor_y = nullptr;
	float* mapAnchor_z = nullptr;
	unsigned int* mapNumConSeg = nullptr;
	unsigned int* mapAnch2SegmentID = nullptr;
	float* mapAnch2SegDistance = nullptr;
	float* mapAnch2SegConst = nullptr;
	unsigned int* mapAnchor_2SegStart = nullptr;
	float* mapSegLoc_x = nullptr;
	float* mapSegLoc_y = nullptr;
	float* mapSegLoc_z = nullptr;
	float* mapAnchor_veloX = nullptr;
	float* mapAnchor_veloY = nullptr;
	float* mapAnchor_veloZ = nullptr;
	float* mapForward_x = nullptr;
	float* mapForward_y = nullptr;
	float* mapForward_z = nullptr;

	UINT nAnchors = objMan->getNumAnchor();
	UINT nAnch2Seg = objMan->getNumAnch2Seg();

	CD3DX12_RANGE readRange(0, 0);
	ThrowIfFailed(OR_anchor_x[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnchor_x)));
	ThrowIfFailed(OR_anchor_y[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnchor_y)));
	ThrowIfFailed(OR_anchor_z[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnchor_z)));
	ThrowIfFailed(OR_numConSeg[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapNumConSeg)));
	ThrowIfFailed(OR_anch2SegmentID[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnch2SegmentID)));
	ThrowIfFailed(OR_anch2SegDistance[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnch2SegDistance)));
	ThrowIfFailed(OR_anch2SegConst[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnch2SegConst)));
	ThrowIfFailed(OR_anchor_2SegStart[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnchor_2SegStart)));
	ThrowIfFailed(OR_segLocM_x[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegLoc_x)));
	ThrowIfFailed(OR_segLocM_y[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegLoc_y)));
	ThrowIfFailed(OR_segLocM_z[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegLoc_z)));
	ThrowIfFailed(OR_anchor_veloX[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnchor_veloX)));
	ThrowIfFailed(OR_anchor_veloY[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnchor_veloY)));
	ThrowIfFailed(OR_anchor_veloZ[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnchor_veloZ)));
	ThrowIfFailed(OR_forwardVec_x[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapForward_x)));
	ThrowIfFailed(OR_forwardVec_y[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapForward_y)));
	ThrowIfFailed(OR_forwardVec_z[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapForward_z)));

	memcpy(mapAnchor_x, objMan->getAddressAnchorX(), nAnchors * sizeof(float));
	memcpy(mapAnchor_y, objMan->getAddressAnchorY() , nAnchors * sizeof(float));
	memcpy(mapAnchor_z, objMan->getAddressAnchorZ(), nAnchors * sizeof(float));
	memcpy(mapNumConSeg, objMan->getAddressNumConSeg(), nAnchors * sizeof(unsigned int));
	memcpy(mapAnch2SegmentID, objMan->getAddressAnch2SegmentID(), nAnch2Seg * sizeof(unsigned int));
	memcpy(mapAnch2SegDistance, objMan->getAddressAnch2SegDistance(), nAnch2Seg * sizeof(float));
	memcpy(mapAnch2SegConst, objMan->getAddressAnch2SegConst(), nAnch2Seg * sizeof(float));
	memcpy(mapAnchor_2SegStart, objMan->getAddressAnch2SegmentStart(), nAnchors * sizeof(unsigned int));
	memcpy(mapSegLoc_x, segLocation.x, SEGMENTCOUNT * sizeof(float));
	memcpy(mapSegLoc_y, segLocation.y, SEGMENTCOUNT * sizeof(float));
	memcpy(mapSegLoc_z, segLocation.z, SEGMENTCOUNT * sizeof(float));
	memcpy(mapAnchor_veloX, objMan->getAddressAnchVeloX(), nAnchors * sizeof(float));
	memcpy(mapAnchor_veloY, objMan->getAddressAnchVeloY(), nAnchors * sizeof(float));
	memcpy(mapAnchor_veloZ, objMan->getAddressAnchVeloZ(), nAnchors * sizeof(float));
	memcpy(mapForward_x, transVelocity.x, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(mapForward_y, transVelocity.y, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(mapForward_z, transVelocity.z, SEGMENTCOUNT * sizeof(float) * 2);

	OR_anchor_x[1]->Unmap(0, nullptr);
	OR_anchor_y[1]->Unmap(0, nullptr);
	OR_anchor_z[1]->Unmap(0, nullptr);
	OR_numConSeg[1]->Unmap(0, nullptr);
	OR_anch2SegmentID[1]->Unmap(0, nullptr);
	OR_anch2SegDistance[1]->Unmap(0, nullptr);
	OR_anch2SegConst[1]->Unmap(0, nullptr);
	OR_anchor_2SegStart[1]->Unmap(0, nullptr);
	OR_segLocM_x[1]->Unmap(0, nullptr);
	OR_segLocM_y[1]->Unmap(0, nullptr);
	OR_segLocM_z[1]->Unmap(0, nullptr);
	OR_anchor_veloX[1]->Unmap(0, nullptr);
	OR_anchor_veloY[1]->Unmap(0, nullptr);
	OR_anchor_veloZ[1]->Unmap(0, nullptr);
	OR_forwardVec_x[1]->Unmap(0, nullptr);
	OR_forwardVec_y[1]->Unmap(0, nullptr);
	OR_forwardVec_z[1]->Unmap(0, nullptr);
	
	BarriorCopyToGPU(mCommandList_Sim, OR_anchor_x);
	BarriorCopyToGPU(mCommandList_Sim, OR_anchor_y);
	BarriorCopyToGPU(mCommandList_Sim, OR_anchor_z);
	BarriorCopyToGPU(mCommandList_Sim, OR_numConSeg);
	BarriorCopyToGPU(mCommandList_Sim, OR_anch2SegmentID);
	BarriorCopyToGPU(mCommandList_Sim, OR_anch2SegDistance);
	BarriorCopyToGPU(mCommandList_Sim, OR_anch2SegConst);
	BarriorCopyToGPU(mCommandList_Sim, OR_anchor_2SegStart);
	BarriorCopyToGPU(mCommandList_Sim, OR_segLocM_x);
	BarriorCopyToGPU(mCommandList_Sim, OR_segLocM_y);
	BarriorCopyToGPU(mCommandList_Sim, OR_segLocM_z);
	BarriorCopyToGPU(mCommandList_Sim, OR_anchor_veloX);
	BarriorCopyToGPU(mCommandList_Sim, OR_anchor_veloY);
	BarriorCopyToGPU(mCommandList_Sim, OR_anchor_veloZ);
	BarriorCopyToGPU(mCommandList_Sim, OR_forwardVec_x);
	BarriorCopyToGPU(mCommandList_Sim, OR_forwardVec_y);
	BarriorCopyToGPU(mCommandList_Sim, OR_forwardVec_z);
}
void UpdateClass::CShaderDataOutputAnch2Seg() {
	float* mapAnchor_veloX = nullptr;
	float* mapAnchor_veloY = nullptr;
	float* mapAnchor_veloZ = nullptr;
	float* mapObjModify_x = nullptr;
	float* mapObjModify_y = nullptr;
	float* mapObjModify_z = nullptr;

	UINT nAnchors = objMan->getNumAnchor();

	CD3DX12_RANGE readRange(0, 0);
	ThrowIfFailed(OR_anchor_veloX[2]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnchor_veloX)));
	ThrowIfFailed(OR_anchor_veloY[2]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnchor_veloY)));
	ThrowIfFailed(OR_anchor_veloZ[2]->Map(0, &readRange, reinterpret_cast<void**>(&mapAnchor_veloZ)));
	ThrowIfFailed(OR_forwardVec_x[2]->Map(0, &readRange, reinterpret_cast<void**>(&mapObjModify_x)));
	ThrowIfFailed(OR_forwardVec_y[2]->Map(0, &readRange, reinterpret_cast<void**>(&mapObjModify_y)));
	ThrowIfFailed(OR_forwardVec_z[2]->Map(0, &readRange, reinterpret_cast<void**>(&mapObjModify_z)));

	memcpy(objMan->getAddressAnchVeloX(), mapAnchor_veloX, nAnchors * sizeof(float));
	memcpy(objMan->getAddressAnchVeloY(), mapAnchor_veloY, nAnchors * sizeof(float));
	memcpy(objMan->getAddressAnchVeloZ(), mapAnchor_veloZ, nAnchors * sizeof(float));
	memcpy(transVelocity.x, mapObjModify_x, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(transVelocity.y, mapObjModify_y, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(transVelocity.z, mapObjModify_z, SEGMENTCOUNT * sizeof(float) * 2);

	OR_colors_x[2]->Unmap(0, nullptr);
	OR_colors_y[2]->Unmap(0, nullptr);
	OR_colors_z[2]->Unmap(0, nullptr);
	OR_forwardVec_x[2]->Unmap(0, nullptr);
	OR_forwardVec_y[2]->Unmap(0, nullptr);
	OR_forwardVec_z[2]->Unmap(0, nullptr);
}

void UpdateClass::Move_CS() {
	if (ComputeShaderActive) {
		RecordComputeWorkMove();
		ThrowIfFailed(mCommandList_Sim->Close());
		ID3D12CommandList* cmdsListsCS[] = { mCommandList_Sim.Get() };
		mCommandQueue_Sim->ExecuteCommandLists(_countof(cmdsListsCS), cmdsListsCS);
		FlushCommandQueueCSsim();

		CShaderDataOutputMove(); 
	}
}
void UpdateClass::RecordComputeWorkMove() {

	ThrowIfFailed(CmdListAlloc_Sim->Reset());
	ThrowIfFailed(mCommandList_Sim->Reset(CmdListAlloc_Sim.Get(), mPSOs["moveSeg"].Get()));
	//mCommandList_Sim->SetPipelineState(mPSOs["moveSeg"].Get());

	CShaderDataInputMove();

	float DRAG_VAL = 0;
	if (param.dragType == -1) {/*none*/ }
	else if (param.dragType == 0) { DRAG_VAL = param.x0Drag; }
	else if (param.dragType == 1) { DRAG_VAL = param.x1Drag; }
	else if (param.dragType == 2) { DRAG_VAL = param.x2Drag; }
	else if (param.dragType == 3) { DRAG_VAL = param.x3Drag; }
	float wx = WALLx_D2, wz = WALLz_D2, wy = WALLy;
	
	mCommandList_Sim->SetComputeRootSignature(mRootSignatureMoveCS.Get());

	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &wx, 0);				//float x
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &wz, 1);				//float x
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &wy, 2);				//float x
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &sphereRadius, 3);		//float
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &DRAG_VAL, 4);			//float
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &param.x1Dampen, 5);	//float
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &param.dragType, 6);			//unsigned int x
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &param.dampenType, 7);			//unsigned int x
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &param.BOUNDED, 8);	//unsigned int
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &param.GRAVITY_ON, 9);	//unsigned int
	mCommandList_Sim->SetComputeRoot32BitConstants(0, 1, &lastSegmentPlus1, 10);	//unsigned int

	mCommandList_Sim->SetComputeRootUnorderedAccessView(1, OR_segLocM_x[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(2, OR_segLocM_y[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(3, OR_segLocM_z[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(4, OR_forwardVec_x[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(5, OR_forwardVec_y[0]->GetGPUVirtualAddress());
	mCommandList_Sim->SetComputeRootUnorderedAccessView(6, OR_forwardVec_z[0]->GetGPUVirtualAddress());

	/*// Get a timestamp at the start of the command list.
	//const UINT timestampHeapIndex = 2 * mCurrBackBuffer;
	//mCommandList_Sim->EndQuery(m_timestampQueryHeaps.Get(), D3D12_QUERY_TYPE_TIMESTAMP, timestampHeapIndex);
	// Get a timestamp at the start of the command list.
	const UINT timestampHeapIndex = 0;
	mCommandList_Sim->EndQuery(mTimestampQueryHeaps_Sim.Get(), D3D12_QUERY_TYPE_TIMESTAMP, timestampHeapIndex);*/

	mCommandList_Sim->Dispatch(static_cast<int>(ceil(SEGMENTCOUNT / 256.0f)), 1, 1);
	mCommandList_Sim->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::UAV(nullptr)); ///?

	if (1) {
		// Schedule to copy the data to the default buffer to the readback buffer.
		BarriorCopyFromGPU(mCommandList_Sim, OR_forwardVec_x);
		BarriorCopyFromGPU(mCommandList_Sim, OR_forwardVec_y);
		BarriorCopyFromGPU(mCommandList_Sim, OR_forwardVec_z);
		BarriorCopyFromGPU(mCommandList_Sim, OR_segLocM_x);
		BarriorCopyFromGPU(mCommandList_Sim, OR_segLocM_y);
		BarriorCopyFromGPU(mCommandList_Sim, OR_segLocM_z);
	}

	/*// Get a timestamp at the end of the command list and resolve the query data.
	mCommandList_Sim->EndQuery(mTimestampQueryHeaps_Sim.Get(), D3D12_QUERY_TYPE_TIMESTAMP, timestampHeapIndex + 1);
	mCommandList_Sim->ResolveQueryData(mTimestampQueryHeaps_Sim.Get(), D3D12_QUERY_TYPE_TIMESTAMP, timestampHeapIndex, 2, mTimestampResultBuffers_Sim.Get(), timestampHeapIndex * sizeof(UINT64));*/
}
void UpdateClass::CShaderDataInputMove() {
	// Map the data so we can read it on CPU.
	float* mapSegLoc_x = nullptr;
	float* mapSegLoc_y = nullptr;
	float* mapSegLoc_z = nullptr;
	float* mapForVec_x = nullptr;//TODO DELETE
	float* mapForVec_y = nullptr;
	float* mapForVec_z = nullptr;
	CD3DX12_RANGE readRange(0, 0);
	ThrowIfFailed(OR_segLocM_x[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegLoc_x)));
	ThrowIfFailed(OR_segLocM_y[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegLoc_y)));
	ThrowIfFailed(OR_segLocM_z[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegLoc_z)));
	ThrowIfFailed(OR_forwardVec_x[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapForVec_x)));
	ThrowIfFailed(OR_forwardVec_y[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapForVec_y)));
	ThrowIfFailed(OR_forwardVec_z[1]->Map(0, &readRange, reinterpret_cast<void**>(&mapForVec_z)));

#if 0
	//TODO: test if memCopy or OpenMP is faster *need timers*
	//#pragma omp parallel for num_threads(10)
	for (int i = 0; i < SEGMENTCOUNT; ++i)
	{
		mapSegLoc_x[i] = segLocationSorted.x[i];
		mapSegLoc_y[i] = segLocationSorted.y[i];
		mapSegLoc_z[i] = segLocationSorted.z[i];
		mapForVec_x[i] = transVelocity.x[i];
		mapForVec_y[i] = transVelocity.y[i];
		mapForVec_z[i] = transVelocity.z[i];
	}
#else
	memcpy(mapSegLoc_x, segLocation.x, SEGMENTCOUNT * sizeof(float));
	memcpy(mapSegLoc_y, segLocation.y, SEGMENTCOUNT * sizeof(float));
	memcpy(mapSegLoc_z, segLocation.z, SEGMENTCOUNT * sizeof(float));
	memcpy(mapForVec_x, transVelocity.x, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(mapForVec_y, transVelocity.y, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(mapForVec_z, transVelocity.z, SEGMENTCOUNT * sizeof(float) * 2);
#endif

	OR_segLocM_x[1]->Unmap(0, nullptr);
	OR_segLocM_y[1]->Unmap(0, nullptr);
	OR_segLocM_z[1]->Unmap(0, nullptr); 
	OR_forwardVec_x[1]->Unmap(0, nullptr);
	OR_forwardVec_y[1]->Unmap(0, nullptr);
	OR_forwardVec_z[1]->Unmap(0, nullptr);

	///////////////////////////////////////////////////////////////////////////////////////////////////////

	BarriorCopyToGPU(mCommandList_Sim, OR_segLocM_x);
	BarriorCopyToGPU(mCommandList_Sim, OR_segLocM_y);
	BarriorCopyToGPU(mCommandList_Sim, OR_segLocM_z);
	BarriorCopyToGPU(mCommandList_Sim, OR_forwardVec_x);
	BarriorCopyToGPU(mCommandList_Sim, OR_forwardVec_y);
	BarriorCopyToGPU(mCommandList_Sim, OR_forwardVec_z);
}
void UpdateClass::CShaderDataOutputMove() {

	float* mapSegLoc_x = nullptr;
	float* mapSegLoc_y = nullptr;
	float* mapSegLoc_z = nullptr;
	float* mapForVec_x = nullptr;
	float* mapForVec_y = nullptr;
	float* mapForVec_z = nullptr;
	CD3DX12_RANGE readRange(0, 0);
	ThrowIfFailed(OR_segLocM_x[2]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegLoc_x)));
	ThrowIfFailed(OR_segLocM_y[2]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegLoc_y)));
	ThrowIfFailed(OR_segLocM_z[2]->Map(0, &readRange, reinterpret_cast<void**>(&mapSegLoc_z)));
	ThrowIfFailed(OR_forwardVec_x[2]->Map(0, &readRange, reinterpret_cast<void**>(&mapForVec_x)));
	ThrowIfFailed(OR_forwardVec_y[2]->Map(0, &readRange, reinterpret_cast<void**>(&mapForVec_y)));
	ThrowIfFailed(OR_forwardVec_z[2]->Map(0, &readRange, reinterpret_cast<void**>(&mapForVec_z)));

#if 0
	//#pragma omp parallel for num_threads(10)
	for (int i = 0; i < SEGMENTCOUNT; ++i)
	{
		transVelocity.x[i] = mapForVec_x[i];
		transVelocity.y[i] = mapForVec_y[i];
		transVelocity.z[i] = mapForVec_z[i];
		segLocation.x[i] = mapColor_x[i];
		segLocation.y[i] = mapColor_y[i];
		segLocation.z[i] = mapColor_z[i];
	}
#else

	memcpy(transVelocity.x, mapForVec_x, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(transVelocity.y, mapForVec_y, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(transVelocity.z, mapForVec_z, SEGMENTCOUNT * sizeof(float) * 2);
	memcpy(segLocation.x, mapSegLoc_x, SEGMENTCOUNT * sizeof(float));
	memcpy(segLocation.y, mapSegLoc_y, SEGMENTCOUNT * sizeof(float));
	memcpy(segLocation.z, mapSegLoc_z, SEGMENTCOUNT * sizeof(float));

#endif

	OR_forwardVec_x[2]->Unmap(0, nullptr);
	OR_forwardVec_y[2]->Unmap(0, nullptr);
	OR_forwardVec_z[2]->Unmap(0, nullptr);
	OR_segLocM_x[2]->Unmap(0, nullptr);
	OR_segLocM_y[2]->Unmap(0, nullptr);
	OR_segLocM_z[2]->Unmap(0, nullptr);

}


void UpdateClass::Tube() {
	int up;
	float spee = 0.7f;
	float separation = 1.0f;
	int fAcross = ((WALLz_D2 + 10) * 2) / spee;
	if (frameCount > fAcross * 1) { //Stationary Tubes
		float pi = 3.14159265f;
		float tau = 6.2831853f;
		float ttau = 12.56637f;
		XMFLOAT3 origin = { -(tubeLength*separation) / 2.0f, 10, -((float)WALLz_D2 + 20) };

		//tubeRadius -= ((frameCount / 25) % 60);
		origin.z += ((frameCount) % fAcross)*spee; //-80 z origin
		up = 0;
		for (int tube = 0; tube < 1; ++tube)
		{
			for (int itr = 0; itr < tubeRadius*tubeLength; ++itr, ++up)
			{
				XMFLOAT3 tmpLoc = { 0, separation, 0 };
				XMFLOAT4 tmpQuat = { 0, 1, 0, 0 };
				XMFLOAT4 rotQuat = CreateFromYawPitchRoll(0, (ttau / tubeRadius)*(float)(itr % (int)tubeRadius), 0);
				XMFLOAT4 fQuat = Multiply(tmpQuat, rotQuat);
				float ci = sqrt(separation*separation * 2) / 2.0f;
				float ra = (separation*tubeRadius) / tau;
				XMFLOAT4 f2Quat = Multiply(fQuat, ra);
				segLocation.x[up] = origin.x + f2Quat.x + ((itr / tubeRadius)*separation);
				segLocation.y[up] = origin.y + f2Quat.y + (tube * 15);
				segLocation.z[up] = origin.z + f2Quat.z + (tube * 60);
			}
		}
	}
}
void UpdateClass::TubeWand(XMMATRIX * controllerMatrix, XMFLOAT3 * locationAcc, int s, int e) {
	int up;
	float spee = 0.7f;
	float separation = 2.2f;
	if (m_pOpenVRSystem) { //Stationary Tubes
		float tau = 6.2831853f;
		float ttau = 12.56637f;
		up = 0;
		for (int tube = s; tube < e; ++tube)
		{
			unsigned short hapticRed = 0;
			for (int itr = 0; itr < tubeRadius*tubeLength; ++itr, ++up)
			{
				XMFLOAT4 tmpQuat = { 1, 0, 0, 0 };
				XMFLOAT4 rotQuat = CreateFromYawPitchRoll((ttau / tubeRadius)*(float)(itr % (int)tubeRadius), 0, 0);
				XMFLOAT4 fQuat = Multiply(tmpQuat, rotQuat);
				float ci = sqrt(separation*separation * 2) / 2.0f;
				float ra = (separation*tubeRadius) / tau;
				XMFLOAT4 f2Quat = Multiply(fQuat, ra);

				XMFLOAT3 particleLoc;
				particleLoc.x = f2Quat.x;
				particleLoc.y = f2Quat.y + ((itr / tubeRadius)*separation);
				particleLoc.z = f2Quat.z;

				XMMATRIX scaleLine = XMMATRIX(
					1.0f, 0.0f, 0.0f, 0.0f,
					0.0f, 1.0f, 0.0f, 0.0f,
					0.0f, 0.0f, 1.0f, 0.0f,
					particleLoc.x, particleLoc.y, particleLoc.z, 1.0f);
				XMMATRIX XfixRotation = XMMATRIX(
					1.0f, 0.0f, 0.0f, 0.0f,
					0.0f, cos(1.570796f), sin(1.570796f), 0.0f,
					0.0f, -sin(1.570796f), cos(1.570796f), 0.0f,
					0.0f, 0.0f, 0.0f, 1.0f);

				XMMATRIX lineMatrix = XMMatrixMultiply(scaleLine, XMMatrixMultiply(XfixRotation, controllerMatrix[tube]));

				segLocation.x[up] = lineMatrix.r[3].m128_f32[0] + locationAcc[tube].x;
				segLocation.y[up] = lineMatrix.r[3].m128_f32[1] + locationAcc[tube].y;
				segLocation.z[up] = lineMatrix.r[3].m128_f32[2] + locationAcc[tube].z;

				float red = (segColor.x[up] < 0.6f) ? 0 : segColor.x[up];
				hapticRed += (unsigned short)(red*11.0);
			}
			if(hapticRed > 10)
				m_pOpenVRSystem->TriggerHapticPulse(tube+1, 0, hapticRed);  // made for GetColour(forceSave, -0.04f, 0.4f);
		}
	}
}
void UpdateClass::TubeWand16(XMMATRIX * controllerMatrix, XMFLOAT3 * locationAcc, int s, int e) {
	int up;
	float spee = 0.7f;
	float separation = 1.6f;
	int fAcross = ((WALLz_D2 + 10) * 2) / spee;
	if (m_pOpenVRSystem) { //Stationary Tubes
		float tau = 6.2831853f;
		float ttau = 12.56637f;
		up = 0;
		for (int tube = s; tube < e; ++tube)
		{
			float controller16[16];
			make16mat(controller16, controllerMatrix[tube]);

			unsigned short hapticRed = 0;
			for (int itr = 0; itr < tubeRadius*tubeLength; ++itr, ++up)
			{
				XMFLOAT4 tmpQuat = { 1, 0, 0, 0 };
				XMFLOAT4 rotQuat = CreateFromYawPitchRoll((ttau / tubeRadius)*(float)(itr % (int)tubeRadius), 0, 0);
				XMFLOAT4 fQuat = Multiply(tmpQuat, rotQuat);
				float ci = sqrt(separation*separation * 2) / 2.0f;
				float ra = (separation*tubeRadius) / tau;
				XMFLOAT4 f2Quat = Multiply(fQuat, ra);

				XMFLOAT3 particleLoc;
				particleLoc.x = f2Quat.x;
				particleLoc.y = f2Quat.y + ((itr / tubeRadius)*separation);
				particleLoc.z = f2Quat.z;

				float scaleLine16[16] = {
					1.0f, 0.0f, 0.0f, 0.0f,
					0.0f, 1.0f, 0.0f, 0.0f,
					0.0f, 0.0f, 1.0f, 0.0f,
					particleLoc.x, particleLoc.y, particleLoc.z, 1.0f };
				float XfixRotation16[16] = {
					1.0f, 0.0f, 0.0f, 0.0f,
					0.0f, cos(1.570796f), sin(1.570796f), 0.0f,
					0.0f, -sin(1.570796f), cos(1.570796f), 0.0f,
					0.0f, 0.0f, 0.0f, 1.0f };

				float res1[16]; float res2[16];
				Matrix4x4Multiply(XfixRotation16, controller16, res1);
				Matrix4x4Multiply(scaleLine16, res1, res2);

				segLocation.x[up] = res2[3 * 4 + 0] + locationAcc[tube].x;
				segLocation.y[up] = res2[3 * 4 + 1] + locationAcc[tube].y;
				segLocation.z[up] = res2[3 * 4 + 2] + locationAcc[tube].z;

				float red = (segColor.x[up] < 0.5f) ? 0 : segColor.x[up];
				hapticRed += (unsigned short)(red*10.0);
			}
			if (hapticRed > 10)
				m_pOpenVRSystem->TriggerHapticPulse(tube + 1, 0, hapticRed);  // made for GetColour(forceSave, -0.04f, 0.4f);
		}
	}
}

//A lennard jones potential interaction computed between two objects
int UpdateClass::LJP3(int segNode, int currentNode, float xLoc, float yLoc, float zLoc, floatArray3 segLoc, float forceRange, float e, float s, bool negForces, XMFLOAT3 &pull_Singletotal, float * forceSave, const UINT * originalIndex) {

	XMFLOAT3 vector = { xLoc - segLoc.x[currentNode], yLoc - segLoc.y[currentNode], zLoc - segLoc.z[currentNode] };
	float dist = sqrtf(vector.x*vector.x + vector.y*vector.y + vector.z*vector.z);
	//Check if the objects are in the interaction range
	if (dist < forceRange) {
		float radius = dist;
		float maxForce = s*3.0f; //Could be anything
								 //Lennard-Jones potential with lower exponents and avoiding the pow() function for performance
		if (dist == 0.0f) { dist = 0.00001f; }
		float innerNum = (e / dist);
		float x2 = innerNum*innerNum;
		float x4 = x2*x2;
		float x8 = x4*x4;
		float force = s*(x8 - x4);

		if (force > maxForce) { force = maxForce; }

		bool negForceDisable = (!negForces && force < 0);
		if (negForceDisable) {
			force = 0.0f;
		}
		else if (negForces && force < 0) {
			force *= param.NEG_F_MULTIPLY;
		}

		UINT segOriginalIndex = originalIndex[segNode];
		UINT targetOriginalIndex = originalIndex[currentNode];
		if (segOriginalIndex > LEAP_SEG_CNT) {
			forceSave[segNode] += force;
		} else if (targetOriginalIndex > LEAP_SEG_CNT) {
			forceSave[segNode] += force;
			
			accumulateHapticSegment(force, segOriginalIndex, targetOriginalIndex); // accumulate number of segments interacting with haptic points in hands.
		}
		
		pull_Singletotal = add(pull_Singletotal, multiply(force, normalize(vector)));

		return 1;
	}
	return 0;
}

void UpdateClass::accumulateHapticSegment(float F, UINT segment, UINT target) {
	if (F > hapticForceThreshold) {
		if (segment >= hd1.getStartSegBone(0, 3) && segment <= hd1.getEndSegBone(0, 3)) {
			hd1.accNumSeg[1][0]++;
			hd1.accAvTemp[1][0] += segTempFactor[target];
		}
		else if (segment >= hd1.getStartSegBone(1, 3) && segment <= hd1.getEndSegBone(1, 3)) {
			hd1.accNumSeg[1][1]++;
			hd1.accAvTemp[1][1] += segTempFactor[target];
		}
		else if (segment >= hd1.getStartSegBone(2, 3) && segment <= hd1.getEndSegBone(2, 3)) {
			hd1.accNumSeg[1][2]++;
			hd1.accAvTemp[1][2] += segTempFactor[target];
		}
		else if (segment >= hd1.getStartSegBone(3, 3) && segment <= hd1.getEndSegBone(3, 3)) {
			hd1.accNumSeg[1][3]++;
			hd1.accAvTemp[1][3] += segTempFactor[target];
		}
		else if (segment >= hd1.getStartSegBone(4, 3) && segment <= hd1.getEndSegBone(4, 3)) {
			hd1.accNumSeg[1][4]++;
			hd1.accAvTemp[1][4] += segTempFactor[target];
		}
		else if (segment >= (hd1.getStartSegBone(0, 3) + hd1.getStartSegRight()) && segment <= (hd1.getEndSegBone(0, 3) + hd1.getStartSegRight())) {
			hd1.accNumSeg[0][0]++;
			hd1.accAvTemp[0][1] += segTempFactor[target];
		}
		else if (segment >= (hd1.getStartSegBone(1, 3) + hd1.getStartSegRight()) && segment <= (hd1.getEndSegBone(1, 3) + hd1.getStartSegRight())) {
			hd1.accNumSeg[0][1]++;
			hd1.accAvTemp[0][1] += segTempFactor[target];
		}
		else if (segment >= (hd1.getStartSegBone(2, 3) + hd1.getStartSegRight()) && segment <= (hd1.getEndSegBone(2, 3) + hd1.getStartSegRight())) {
			hd1.accNumSeg[0][2]++;
			hd1.accAvTemp[0][2] += segTempFactor[target];
		}
		else if (segment >= (hd1.getStartSegBone(3, 3) + hd1.getStartSegRight()) && segment <= (hd1.getEndSegBone(3, 3) + hd1.getStartSegRight())) {
			hd1.accNumSeg[0][3]++;
			hd1.accAvTemp[0][3] += segTempFactor[target];
		}
		else if (segment >= (hd1.getStartSegBone(4, 3) + hd1.getStartSegRight()) && segment <= (hd1.getEndSegBone(4, 3) + hd1.getStartSegRight())) {
			hd1.accNumSeg[0][4]++;
			hd1.accAvTemp[0][4] += segTempFactor[target];
		}
	}
}

int UpdateClass::LJP3_Hand(int segNode, int cNode, UINT segNodeOrig, float xLoc, float yLoc, float zLoc, float segR, floatArray3 segLoc, float radiusCNode, float forceRange,
	float s, UINT negForces, XMFLOAT3 &pull_Singletotal, XMFLOAT3 &pull_Handtotal, float * forceSave, const UINT * originalIndex) {

	XMFLOAT3 vector = { xLoc - segLoc.x[cNode], yLoc - segLoc.y[cNode], zLoc - segLoc.z[cNode] };
	float dist = sqrtf(vector.x*vector.x + vector.y*vector.y + vector.z*vector.z);
	//Check if the objects are in the interaction range
	if (dist < forceRange) {
		float maxForce = s * 3.0f; //Could be anything
		float e = radiusCNode + segR;
		//Lennard-Jones potential with lower exponents and avoiding the pow() function for performance
		if (dist == 0.0f) { dist = 0.00001f; }
		float innerNum = (e / dist);
		float x2 = innerNum * innerNum;
		float x4 = x2 * x2;
		float x8 = x4 * x4;
		float force = s * (x8 - x4);


		if (force > maxForce) { force = maxForce; }

		if (force < 0.0f) {
			force *= negForces;
			force *= param.NEG_F_MULTIPLY;
		}

		UINT segOriginalIndex = originalIndex[segNode];
		UINT targetOriginalIndex = originalIndex[cNode];
		if (segOriginalIndex > LEAP_SEG_CNT) {
			forceSave[segNode] += force;
		}
		else if (targetOriginalIndex > LEAP_SEG_CNT) {
			forceSave[segNode] += force;

			//accumulateHapticSegment(force, segOriginalIndex, targetOriginalIndex); // accumulate number of segments interacting with haptic points in hands.
			//DISABLED TO PREVENT "6 MINUTES" RUNNING PROBLEM
		}

		XMFLOAT3 pull = multiply(force, normalize(vector));
		pull_Singletotal = add(pull_Singletotal, pull);

#if HAND_SPRING_PROPAGATE
		UINT cNodeOrig = originalIndex[cNode];
		if (segNodeOrig >= LEAP_SEG_CNT && cNodeOrig < LEAP_SEG_CNT) {
			pull_Handtotal = add(pull_Handtotal, pull);
			//forceSave[segNode] += force;
		}
#endif
		return 1;
	}
	return 0;
}

float UpdateClass::LJPController(float xStatic, float yStatic, float zStatic, float xDy, float yDy, float zDy,
	float forceRange, float e, float s, bool negForces, XMFLOAT3 &pull_Singletotal) {

	XMFLOAT3 vector = { xDy - xStatic, yDy - yStatic, zDy - zStatic };
	float dist = sqrtf(vector.x*vector.x + vector.y*vector.y + vector.z*vector.z);
	//Check if the objects are in the interaction range
	if (dist < forceRange) {
		float maxForce = s*3.0f; //Could be anything
								 //Lennard-Jones potential with lower exponents and avoiding the pow() function for performance
#if 1
		if (dist == 0.0f) { dist = 0.00001f; }
		float innerNum = (e / dist);
#else
		float LJP_R = ((dist - e) == 0.0f) ? 0.00001f : (dist - e);
		float innerNum = (2.0f / LJP_R);
#endif
		float x2 = innerNum*innerNum;
		float x4 = x2*x2;
		float x8 = x4*x4;
		float force = s*(x8 - x4);

		if (force > maxForce) { force = maxForce; }

		bool negForceDisable = (!negForces && force < 0);
		if (negForceDisable)//
			force = 0.0f;

		pull_Singletotal = add(pull_Singletotal, multiply(force, normalize(vector)));
#if 0
		XMFLOAT3 forW = XMFLOAT3(pull_Singletotal.x, pull_Singletotal.y, pull_Singletotal.z);
		if (phi != phi) {
			phi = 0;
		}
		if (forW.x != forW.x || forW.y != forW.y || forW.z != forW.z) {
			pull_Singletotal.x = 0;
			pull_Singletotal.y = 0;
			pull_Singletotal.z = 0;
		}
#endif
		return force;
	}
	return 0;
}

float min0(float a) {
	return (float)(a > 0.0f)*a;
}
float sgn(float val) {
	return (0.0f < val) - (val < 0.0f);
}
void applyDrag_x0(XMFLOAT3 &forVec, InteractionParameters param) {
	forVec.x = min0(abs(forVec.x) - param.x0Drag)*sgn(forVec.x);
	forVec.y = min0(abs(forVec.y) - param.x0Drag)*sgn(forVec.y);
	forVec.z = min0(abs(forVec.z) - param.x0Drag)*sgn(forVec.z);
}
void applyDrag_x1(XMFLOAT3 &forVec, InteractionParameters param) {
	forVec.x *= (1.0f - param.x1Drag);
	forVec.y *= (1.0f - param.x1Drag);
	forVec.z *= (1.0f - param.x1Drag);
}
void applyDrag_x2(XMFLOAT3 &forVec, InteractionParameters param) {
	forVec.x *= (1.0f - (param.x2Drag*abs(forVec.x)));
	forVec.y *= (1.0f - (param.x2Drag*abs(forVec.y)));
	forVec.z *= (1.0f - (param.x2Drag*abs(forVec.z)));
}
void applyDrag_x3(XMFLOAT3 &forVec, InteractionParameters param) {
	forVec.x *= (1.0f - (param.x3Drag*forVec.x*forVec.x));
	forVec.y *= (1.0f - (param.x3Drag*forVec.y*forVec.y));
	forVec.z *= (1.0f - (param.x3Drag*forVec.z*forVec.z));
}
void applyDrag_x4(XMFLOAT3 &forVec, InteractionParameters param) {
	return;
}
void (*applyDrag_x[5]) (XMFLOAT3 &forVec, InteractionParameters param) { applyDrag_x0, applyDrag_x1, applyDrag_x2, applyDrag_x3, applyDrag_x4};
void UpdateClass::applyDrag(XMFLOAT3 &forVec) {
	(*applyDrag_x[param.dragType]) (forVec, param);
}
void UpdateClass::applyDragIF(XMFLOAT3 &forVec) {
	if (param.dragType == 4) {
		return;
	}
	
	if (param.dragType == 0) {
		forVec.x = min0(abs(forVec.x) - param.x0Drag)*sgn(forVec.x);
		forVec.y = min0(abs(forVec.y) - param.x0Drag)*sgn(forVec.y);
		forVec.z = min0(abs(forVec.z) - param.x0Drag)*sgn(forVec.z);
	}
	else if (param.dragType == 1) {
		forVec.x *= (1.0f - param.x1Drag);
		forVec.y *= (1.0f - param.x1Drag);
		forVec.z *= (1.0f - param.x1Drag);
	}
	else if (param.dragType == 2) {
		forVec.x *= (1.0f - (param.x2Drag*abs(forVec.x)));
		forVec.y *= (1.0f - (param.x2Drag*abs(forVec.y)));
		forVec.z *= (1.0f - (param.x2Drag*abs(forVec.z)));
	}
	else if (param.dragType == 3) {
		forVec.x *= (1.0f - (param.x3Drag*forVec.x*forVec.x));
		forVec.y *= (1.0f - (param.x3Drag*forVec.y*forVec.y));
		forVec.z *= (1.0f - (param.x3Drag*forVec.z*forVec.z));
	}
}
void solidWalls(XMFLOAT3 &locVec, XMFLOAT3 &forVec, InteractionParameters param, float adj_X, float adj_Z, float adj_Ytop, float adj_Ybase) {
	if (param.GRAVITY_ON)
		forVec.y += -0.006f;//gravity
	//Boundary conditions
	float DragVal = (1.0f - param.x1Dampen);
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
void loopingWalls(XMFLOAT3 &locVec, XMFLOAT3 &forVec, InteractionParameters param, float adj_X, float adj_Z, float adj_Ytop, float adj_Ybase) {
	if (param.GRAVITY_ON)
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
//Boundary conditions
void UpdateClass::CheckBoundary(bool bounded, float radius, XMFLOAT3 &locVec, XMFLOAT3 &forVec) {

	//Boundaries - adjusted for radius of object
	float adj_X = (float)WALLx_D2 - radius;
	float adj_Z = (float)WALLz_D2 - radius;
	float adj_Ytop = (float)WALLy - radius;
	float adj_Ybase = (float)0.0f + radius;

	if (bounded) {
		solidWalls(locVec, forVec, param, adj_X, adj_Z, adj_Ytop, adj_Ybase);
	}
	else {
		loopingWalls(locVec, forVec, param, adj_X, adj_Z, adj_Ytop, adj_Ybase);
	}
}

//Controller Interactions for VR - only local 
void UpdateClass::VRControllerInteractions() {

	auto segRI = mOpaqueRitems[getInstIdx("segments")];
	auto controllerRI = mOpaqueRitems[getInstIdx("controller")];
	/*XMFLOAT3 controllerLook[2] = { XMFLOAT3(controllerRI->Instances[0].World._31, controllerRI->Instances[0].World._32, controllerRI->Instances[0].World._33),
								  XMFLOAT3(controllerRI->Instances[1].World._31, controllerRI->Instances[1].World._32, controllerRI->Instances[1].World._33) };
	XMFLOAT3 controllerLoc[2] = { XMFLOAT3(controllerRI->Instances[0].World._41, controllerRI->Instances[0].World._42, controllerRI->Instances[0].World._43),
								  XMFLOAT3(controllerRI->Instances[1].World._41, controllerRI->Instances[1].World._42, controllerRI->Instances[1].World._43) };
	XMFLOAT3 grabLoc[2] = { XMFLOAT3(controllerLoc[0].x + controllerLook[0].x*2.0f, controllerLoc[0].y + controllerLook[0].y*2.0f, controllerLoc[0].z + controllerLook[0].z*2.0f),
							XMFLOAT3(controllerLoc[1].x + controllerLook[1].x*2.0f, controllerLoc[1].y + controllerLook[1].y*2.0f, controllerLoc[1].z + controllerLook[1].z*2.0f) };*/

	//Finding Segments to Grab
	for (int cID = 0; cID < 2; cID++) {
		if (vc.VRTriggerSearching[cID] && vc.VRTriggerPressed[cID]) {
			controller_pickingL(segLocationSorted, segOriginalIndex, segCellStart, segCellEnd, cID, vc.lastGrabLoc[cID], 1.2f, &holdingIdx[cID]);
			if (holdingIdx[cID] != -1)
				vc.VRTriggerSearching[cID] = false;
		}
	}

	//Moving single segments with the Controller
	for (int cID = 0; cID < 2; cID++) {
		if (holdingIdx[cID] != -1 && vc.VRTriggerPressed[cID]) {
			segLocation.x[holdingIdx[cID]] += vc.grabDeltaLoc[cID].x;
			segLocation.y[holdingIdx[cID]] += vc.grabDeltaLoc[cID].y;
			segLocation.z[holdingIdx[cID]] += vc.grabDeltaLoc[cID].z;
			transVelocity.x[holdingIdx[cID]] = 0.0f;
			transVelocity.y[holdingIdx[cID]] = 0.0f;
			transVelocity.z[holdingIdx[cID]] = 0.0f;
			if (PAUSED) {
				//segRI->Instances[holdingIdx[cID]].World._41 += grabDeltaLoc[cID].x;
				//segRI->Instances[holdingIdx[cID]].World._42 += grabDeltaLoc[cID].y;
				//segRI->Instances[holdingIdx[cID]].World._43 += grabDeltaLoc[cID].z;
				segLocation.x[holdingIdx[cID]] += vc.grabDeltaLoc[cID].x;
				segLocation.y[holdingIdx[cID]] += vc.grabDeltaLoc[cID].y;
				segLocation.z[holdingIdx[cID]] += vc.grabDeltaLoc[cID].z;
			}
		}
		else if (holdingIdx[cID] != -1 && !vc.VRTriggerPressed[cID]) {
			float cutNoise = 0.006f; //Uses a noise cut off
			transVelocity.x[holdingIdx[cID]] = (abs(vc.grabDeltaLoc[cID].x) < cutNoise) ? 0.0f : vc.grabDeltaLoc[cID].x;
			transVelocity.y[holdingIdx[cID]] = (abs(vc.grabDeltaLoc[cID].y) < cutNoise) ? 0.0f : vc.grabDeltaLoc[cID].y;
			transVelocity.z[holdingIdx[cID]] = (abs(vc.grabDeltaLoc[cID].z) < cutNoise) ? 0.0f : vc.grabDeltaLoc[cID].z;
			/*transVelocity.x[holdingIdx[cID]] = controllerDeltaLoc[cID].x;
			transVelocity.y[holdingIdx[cID]] = controllerDeltaLoc[cID].y;
			transVelocity.z[holdingIdx[cID]] = controllerDeltaLoc[cID].z;*/
			holdingIdx[cID] = -1;
		}
	}
}

void UpdateClass::VRCollisionInteractions(int s, int e) {
	if (m_pOpenVRSystem) {
		XMFLOAT3 forceLoc[2];
		for (int control = s; control < e; ++control)
		{
			forceLoc[control] = XMFLOAT3(vc.lastGrabLoc[control].x + vc.controllerLook[control].x*vc.grabForOffset, vc.lastGrabLoc[control].y + vc.controllerLook[control].y*vc.grabForOffset, vc.lastGrabLoc[control].z + vc.controllerLook[control].z*vc.grabForOffset);
			XMFLOAT3 facePosition = mCamera.GetPosition3f();

			auto forcePartRI = mOpaqueRitems[getInstIdx("forceParticle")];
			forcePartRI->Instances[control].World._41 = forceLoc[control].x;
			forcePartRI->Instances[control].World._42 = forceLoc[control].y;
			forcePartRI->Instances[control].World._43 = forceLoc[control].z;

			int up; float force = 0;
//#pragma omp parallel for private (up)
			for (up = 0; up < (int)SEGMENT_COUNT; ++up)
			{
				XMFLOAT3 pull_Singletotal = { 0.0f, 0.0f, 0.0f };

				force += LJPController(forceLoc[control].x, forceLoc[control].y, forceLoc[control].z, segLocation.x[up], segLocation.y[up], segLocation.z[up], 26, 6, 0.6f, vc.VRTriggerPressed[control], pull_Singletotal);
				LJPController(facePosition.x, facePosition.y, facePosition.z, segLocation.x[up], segLocation.y[up], segLocation.z[up], 22, 18, 0.05f, 0, pull_Singletotal);

				transVelocity.x[up] += pull_Singletotal.x;
				transVelocity.y[up] += pull_Singletotal.y;
				transVelocity.z[up] += pull_Singletotal.z;
			}
			if (force < -0.1f)
				m_pOpenVRSystem->TriggerHapticPulse(control + 1, 0, (unsigned short)(force*-100.0f)); // made for GetColour(forceSave, -0.04f, 0.4f);
		}
	}
}

//Controller Interactions for Mouse
void UpdateClass::MouseInteractions() {
	if (MouseRightPressed) {
		POINT cursorPos;
		GetCursorPos(&cursorPos);
		ScreenToClient(mhMainWnd, &cursorPos);
		float x = (float)cursorPos.x;
		float y = (float)cursorPos.y;

		if (MouseRightSearching) {
			StartCounter_ms(timer_a);
			//Pick((int)x, (int)y);
			PickOpti((int)x, (int)y);
			timeCumulativePick += GetCounter_ms(timer_a);
			if (frameCount % 40 == 0) {
				timeModPick = timeCumulativePick / 40.f;
				timeCumulativePick = 0;
			}
			if (holdingIdx[0] != -1) {
				MouseRightSearching = false;
				lastMousePos = XMFLOAT2(x, y);
			}
		}

		if (holdingIdx[0] != -1) {
			auto ri = mOpaqueRitems[getInstIdx("segments")];
			float eyeToObjDist = 0.0f; //distance
			XMFLOAT3 camLoc = mCamera.GetPosition3f();
			//XMFLOAT3 objLoc = XMFLOAT3(ri->Instances[holdingIdx[0]].World._41, ri->Instances[holdingIdx[0]].World._42, ri->Instances[holdingIdx[0]].World._43);
			XMFLOAT3 objLoc = XMFLOAT3(segLocation.x[holdingIdx[0]], segLocation.y[holdingIdx[0]], segLocation.z[holdingIdx[0]]);
			XMFLOAT3 dir = XMFLOAT3(0.0f, 0.0f, 0.0f);
			eyeToObjDist = sqrtf(dot(subtract(objLoc, camLoc)));
			float objPlaneX = 2.0f * eyeToObjDist * tanf(0.5f*mCamera.GetFovX());
			float objPlaneY = 2.0f * eyeToObjDist * tanf(0.5f*mCamera.GetFovY());
			float dx = (x - lastMousePos.x) * (objPlaneX / mClientWidth);
			float dy = -(y - lastMousePos.y) * (objPlaneY / mClientHeight);
			dir = add(dir, multiply(dx, mCamera.GetRight3f()));
			dir = add(dir, multiply(dy, mCamera.GetUp3f()));
			objLoc.x += dir.x;
			objLoc.y += dir.y;
			objLoc.z += dir.z;
			if (PAUSED) {
				//ri->Instances[holdingIdx[0]].World._41 = objLoc.x;
				//ri->Instances[holdingIdx[0]].World._42 = objLoc.y;
				//ri->Instances[holdingIdx[0]].World._43 = objLoc.z;
				segLocation.x[holdingIdx[0]] = objLoc.x;
				segLocation.y[holdingIdx[0]] = objLoc.y;
				segLocation.z[holdingIdx[0]] = objLoc.z;
			}
			updateLocation(holdingIdx[0], objLoc.x, objLoc.y, objLoc.z);
			//shiftForward(holdingIdx[0], holdingStrength);
			mouseDeltaLoc = subtract(objLoc, lastMouseLoc);
			lastMouseLoc = objLoc;
			lastMousePos = XMFLOAT2(x, y);
		}
	}
	else if (holdingIdx[0] != -1 && !MouseRightPressed) {
		float cutNoise = 0.000f; //Uses a noise cut off
								 /*UCP->updateForward(holdingIdx[0],
								 (abs(mouseDeltaLoc.x) < cutNoise) ? 0.0f : mouseDeltaLoc.x,
								 (abs(mouseDeltaLoc.y) < cutNoise) ? 0.0f : mouseDeltaLoc.y,
								 (abs(mouseDeltaLoc.z) < cutNoise) ? 0.0f : mouseDeltaLoc.z);*/

		//Mouse Delta seem to be a bit extream so *0.1
		updateForward(holdingIdx[0],
			mouseDeltaLoc.x*0.1f,
			mouseDeltaLoc.y*0.1f,
			mouseDeltaLoc.z*0.1f);
		holdingIdx[0] = -1;
	}
	
}

//Controller Segment search
void UpdateClass::controller_pickingL(floatArray3 segLocSorted, const UINT * originalIndex, const UINT *cellStart, const UINT *cellEnd, int controller, XMFLOAT3 grabLoc, float grabRange, int * selectedSeg) {

	//Local accumulator
	XMFLOAT3 pull_Singletotal = { 0.0f, 0.0f, 0.0f };

	int tubeGrid = hashCPU(grabLoc.x, grabLoc.y, grabLoc.z);

	int foundSeg = -1;
	float distMin = 9000;

	//Loop over the 27 locations in a 3x3x3 box
	for (int t = 0; t < 27; t++) {
		//The below line computes 3D grid locations using a basic incrementing iterator and the hash dimentions 
		unsigned int targetGrid = (CUBE_COUNTx_1D*CUBE_COUNTz_1D*((t / 9) - 1)) +
			((tubeGrid - CUBE_COUNTz_1D + (((t % 9) / 3)*CUBE_COUNTz_1D)) - 1 + ((t) % 3));
		if (targetGrid < HASH_ENTRIE && targetGrid >= 0) {
			int startIndex = cellStart[targetGrid];
			int endIndex = cellEnd[targetGrid];
			if (startIndex != 0xffffffff)          // cell is not empty
			{
				for (int cNode = startIndex; cNode < endIndex; cNode++) {
					XMFLOAT3 vector = { grabLoc.x - segLocSorted.x[cNode], grabLoc.y - segLocSorted.y[cNode], grabLoc.z - segLocSorted.z[cNode] };
					float dist = sqrtf(vector.x*vector.x + vector.y*vector.y + vector.z*vector.z);
					if (dist < grabRange) {
						//Call the interaction algorithm
						if (dist < distMin) {
							UINT segNodeIndex = originalIndex[cNode];
							foundSeg = segNodeIndex;
							distMin = dist;
						}
					}
					
				}
			}
		}
	}

	if (foundSeg != -1) {
		*selectedSeg = foundSeg;
	}
}

//Controller Interactions
int UpdateClass::Controller(int segNode, int currentNode, float xLoc, float yLoc, float zLoc, floatArray3 segLoc, floatArray3 segFor, float forceRange, XMFLOAT3 controllerDelta, floatArray3 colors, bool COLOR_FORCE) {
	XMFLOAT3 vector = { xLoc - segLoc.x[currentNode], yLoc - segLoc.y[currentNode], zLoc - segLoc.z[currentNode] };
	float dist = sqrtf(vector.x*vector.x + vector.y*vector.y + vector.z*vector.z);

	//Check if the objects are in the interaction range
	if (dist < forceRange) {

		segLoc.x[currentNode] += controllerDelta.x;
		segLoc.y[currentNode] += controllerDelta.y;
		segLoc.z[currentNode] += controllerDelta.z;

		if (controllerDelta.x != 0.0f && controllerDelta.y != 0.0f && controllerDelta.z != 0.0f) {
			segFor.x[currentNode] = 0.0f;
			segFor.y[currentNode] = 0.0f;
			segFor.z[currentNode] = 0.0f;
			if (COLOR_FORCE) {
				colors.x[currentNode] = 0.0f;
				colors.y[currentNode] = 0.0f;
				colors.z[currentNode] = 2.0f;
			}
		}
		else {
			if (COLOR_FORCE) {
				colors.x[currentNode] = 0.0f;
				colors.y[currentNode] = 0.0f;
				colors.z[currentNode] = 1.0f;
			}
		}
		return 1;
	}
	if (COLOR_FORCE) {
		XMFLOAT3 color = GetColour(dist, 0.0f, param.FORCE_RANGE);
		colors.x[currentNode] = color.x;
		colors.y[currentNode] = color.y;
		colors.z[currentNode] = color.z;
	}
	return 0;
}

//Experimenting with various things - functions with "wall" in the name are not relevant
void UpdateClass::holdWall() {
	float buffer = 20.0f;
	float spread = spaceDimXD2_sg - buffer;
	int OneDim = 20;
	float sep = 2.0f; // 1.5f;
	for (int i = 0; i < OneDim*OneDim; i++) {
		segLocation.x[i] = 0;
		segLocation.y[i] = (spaceDimXD2_sg + (OneDim*sep / 2.0f)) - (i % OneDim)*sep;
		segLocation.z[i] = (i / OneDim)*sep - (OneDim*sep / 2.0f);
		transVelocity.x[i] = 0.00f;
		transVelocity.y[i] = 0.0f;
		transVelocity.z[i] = 0.0f;
	}
}
void UpdateClass::sendWall(bool left) {
	float buffer = 20.0f;
	float spread = spaceDimXD2_sg - buffer;
	int OneDim = 20;
	float sep = 3.0f; // 1.5f;
	for (int i = 0; i < OneDim*OneDim; i++) {
		if (left)
			segLocation.x[i] = spread;
		else
			segLocation.x[i] = -spread;
		segLocation.y[i] = (spaceDimXD2_sg + (OneDim*sep / 2.0f)) - (i % OneDim)*sep;
		segLocation.z[i] = (i / OneDim)*sep - (OneDim*sep / 2.0f);
		if (left)
			transVelocity.x[i] = -0.1f;
		else
			transVelocity.x[i] = 0.1f;
		transVelocity.y[i] = 0.0f;
		transVelocity.z[i] = 0.0f;
	}
}
void UpdateClass::CubeSetFunc() {
	XMFLOAT4 offf = XMFLOAT4(210.0f, 100.0f, -37.0f, 0);
	int xw = 40;
	int zd = 40;
	int yh = 128;
	float radi = 1.01f;
	float radi2 = 2.0f*radi;
	float jitter = radi*0.001f;
	for (int z = 0; z<zd; z++)
	{
		for (int y = 0; y<yh; y++)
		{
			for (int x = 0; x<xw; x++)
			{
				int i = (z*yh * xw) + (y*xw) + x + offf.w;

				if (i < (int)OBJECT_COUNT)
				{
					segLocation.x[i] = (radi2 * x) + radi - 1.0f + (frand()*2.0f - 1.0f)*jitter + offf.x;
					segLocation.y[i] = (radi2 * y) + radi - 1.0f + (frand()*2.0f - 1.0f)*jitter + offf.y;
					segLocation.z[i] = (radi2 * z) + radi - 1.0f + (frand()*2.0f - 1.0f)*jitter + offf.z;

					transVelocity.x[i] = 0.0f;
					transVelocity.y[i] = 0.0f;
					transVelocity.z[i] = 0.0f;
				}
			}
		}
	}
}

//External Access to data structures
void UpdateClass::updateLocation(UINT i, float x, float y, float z) {
	segLocation.x[i] = x;
	segLocation.y[i] = y; 
	segLocation.z[i] = z;
}
void UpdateClass::shiftLocation(UINT i, float x, float y, float z) {
	segLocation.x[i] += x;
	segLocation.y[i] += y;
	segLocation.z[i] += z;
}
void UpdateClass::updateForward(UINT i, float x, float y, float z) {
	transVelocity.x[i] = x;
	transVelocity.y[i] = y;
	transVelocity.z[i] = z;
}
void UpdateClass::shiftForward(UINT i, float f) {
	transVelocity.x[i] *= f;
	transVelocity.y[i] *= f;
	transVelocity.z[i] *= f;
}
void UpdateClass::shiftColor(UINT i, float f) {
	segColor.x[i] *= f;
	segColor.y[i] *= f;
	segColor.z[i] *= f;
	holdingIdx[0] = ((int)f == 2) ? i : -1;
}

//Hash Sorting Functions/////////////////////////////////////////
UINT UpdateClass::hashCPU(float x, float y, float z) {
	UINT xCube, yCube, zCube;
	xCube = (UINT)(x + GRIDxHALF) / CUBE;
	zCube = (UINT)(z + GRIDzHALF) / CUBE;
	yCube = (UINT)(y + GRIDyDEPTH_BUFFER) / CUBE;
#if _DEBUG
	UINT hashVal = (yCube*CUBE_COUNTz_1D*CUBE_COUNTx_1D) + (zCube + xCube * CUBE_COUNTz_1D);
	if (hashVal < 0) {
		hashVal = 0;
	}
	else if (hashVal >= HASH_ENTRIE) {
		hashVal = HASH_ENTRIE - 1;
	}
	return hashVal;
#else

	UINT hashVal = (yCube*CUBE_COUNTz_1D*CUBE_COUNTx_1D) + (zCube + xCube * CUBE_COUNTz_1D);
	if (hashVal >= HASH_ENTRIE || hashVal < 0)
		return HASH_ENTRIE - 1;

	return hashVal;
#endif
}

void UpdateClass::merge(struct mapH *mapP, int n, int m) {
	int i, j, k;
	struct mapH *x = (struct mapH*) malloc(sizeof(struct mapH) * n);
	for (i = 0, j = m, k = 0; k < n; k++) {
		if (j == n) {
			x[k] = mapP[i++];
		}
		else if (i == m) {
			x[k] = mapP[j++];
		}
		else if (mapP[j].hashVal < mapP[i].hashVal) {
			x[k] = mapP[j++];
		}
		else {
			x[k] = mapP[i++];
		}
		//x[k] = (j == n ? list[i++]: i == m ? list[j++]:list[j] < list[i] ? list[j++]:list[i++]);
	}
	for (i = 0; i < n; i++) {
		mapP[i] = x[i];
	}
	free(x);
}
void UpdateClass::merge_sort(struct mapH *mapP, int n) {
	if (n < 2)
		return;
	int m = n / 2;
	merge_sort(mapP, m);
	merge_sort(mapP + m, n - m);
	merge(mapP, n, m);
}
void UpdateClass::sortListCPU(floatArray3 locations, floatArray3 locationsSorted, UINT * originalIndex, UINT * cellStart, UINT * cellEnd) {

	struct mapH *mapP = (struct mapH*) malloc(sizeof(struct mapH) * OBJECT_COUNT);
	const int ompTh = 8;
#pragma omp parallel for num_threads(ompTh)
	for (int i = 0; i < (int)OBJECT_COUNT; i++) {
#if _DEBUG
		XMFLOAT3 loc = XMFLOAT3(locations.x[i], locations.y[i], locations.z[i]);
		unsigned int hash = hashCPU(loc.x, loc.y, loc.z);
		if (hash < 0 || hash >= HASH_ENTRIE) {
			hash = 0;
		}
		mapP[i].hashVal = hash;
		mapP[i].orig = i;
#else
		mapP[i].hashVal = hashCPU(locations.x[i], locations.y[i], locations.z[i]);
		mapP[i].orig = i;
#endif
	}
	
	merge_sort(mapP, OBJECT_COUNT); //###### SORT ######//

#pragma omp parallel for num_threads(ompTh)
	for (int i = 0; i < (int)OBJECT_COUNT; i++) {
		UINT Original_Index = mapP[i].orig;
		originalIndex[i] = Original_Index;
		locationsSorted.x[i] = locations.x[Original_Index];
		locationsSorted.y[i] = locations.y[Original_Index];
		locationsSorted.z[i] = locations.z[Original_Index];
	}
	memset(cellStart, 0xffffffff, HASH_ENTRIE * sizeof(UINT));
	int current = mapP[0].hashVal;
	cellStart[current] = 0; // 0 = first in sorted array
	for (int u = 0; u < (int)OBJECT_COUNT; u++) {
		UINT iterHash = mapP[u].hashVal;
		if (iterHash != current) {
			cellEnd[current] = u;
			cellStart[iterHash] = u;
			current = iterHash;
		}
	}
	cellEnd[current] = OBJECT_COUNT;
	free(mapP);
}

void UpdateClass::InitRadix(UINT length) {
	KVList = (KeyValue*)malloc(sizeof(KeyValue)*length);
	KVTemp = (KeyValue*)malloc(sizeof(KeyValue)*length);
	flag = (UINT*)malloc(sizeof(UINT)*length);
	memset(KVList, 0, sizeof(KeyValue)*length);
	memset(KVTemp, 0, sizeof(KeyValue)*length);
	memset(flag, 0, sizeof(UINT)*length);
}
void UpdateClass::CloseRadix() {
	free(KVList);
	free(KVTemp);
	free(flag);
}
void UpdateClass::checkCount2(const KeyValue * __restrict list, KeyValue * __restrict temp, UINT * __restrict countBit, UINT bitLoc, const int blockId, const int blockCnt, const int max) {
	int localInc = 0;
	UINT flagS[256];
	int loop;
	int start = blockId * 256;
	int end = start + 256;
	for (loop = start; loop < end; loop++) {
		UINT hash = list[loop].kHash;
		UINT flippedA = ((hash >> bitLoc) & 1);
		UINT flippedB = ((hash >> (bitLoc + 1)) & 1);
		flagS[localInc] = flippedA + (flippedB << 1);
		localInc++;
	}
	localInc = 0;
	UINT countBitLocA[4] = { 0 };
	for (loop = start; loop < end; loop++) {
		countBitLocA[flagS[localInc]]++; //Atomic needed
		localInc++;
	}

	countBit[blockId] = countBitLocA[0];
	countBit[blockId + blockCnt] = countBitLocA[1];
	countBit[blockId + blockCnt * 2] = countBitLocA[2];
	countBit[blockId + blockCnt * 3] = countBitLocA[3];

	localInc = 0;
	UINT inc[4] = { 0, countBitLocA[0], countBitLocA[0] + countBitLocA[1], countBitLocA[0] + countBitLocA[1] + countBitLocA[2] };
	for (loop = start; loop < end; loop++) {
		temp[inc[flagS[localInc]] + start] = list[loop];
		inc[flagS[localInc]]++;
		localInc++;
	}
}
void UpdateClass::pack2(KeyValue * __restrict list, const KeyValue * __restrict temp, const UINT * __restrict countBit, const UINT * __restrict countBitStart, const int blockId, const int blockCount, const int max) {
	int start = blockId * 256;
	int end = start + 256;
	int loop = start;

	int zCnt = countBit[blockId];
	int oCnt = countBit[blockId + blockCount] + zCnt;
	int tCnt = countBit[blockId + blockCount * 2] + oCnt;

	int zStart = countBitStart[blockId];
	int oneStart = countBitStart[blockId + blockCount];
	int twoStart = countBitStart[blockId + blockCount * 2];
	int threeStart = countBitStart[blockId + blockCount * 3];

	for (int thId = 0; thId < 256; thId++) {
		if (thId < zCnt) {
			list[thId + zStart] = temp[loop];
		}
		else if (thId >= zCnt && thId < oCnt) {
			list[(thId - zCnt) + oneStart] = temp[loop];
		}
		else if (thId >= oCnt && thId < tCnt) {
			list[(thId - oCnt) + twoStart] = temp[loop];
		}
		else {
			list[(thId - tCnt) + threeStart] = temp[loop];
		}
		loop++;
	}
}
void UpdateClass::checkKVList(KeyValue * KVli, UINT flag) {
#if _DEBUG
	KeyValue check[20] = { 0 }; UINT indexError[20] = { 0 };  UINT cntError = 0;
	for (int i = 0; i < 20; i++) { check[i].kHash = 0; check[i].vIdx = 0; indexError[i] = 0; }
	for (int i = 0; i < (int)OBJECT_COUNT; i++) {
		if (KVli[i].kHash >= HASH_ENTRIE) {
			UINT dd = KVli[i].kHash;
			if (cntError < 20) {
				check[cntError] = KVli[i];
				indexError[cntError] = i;
				cntError++;
			}
			dd++;
		}
		if (KVli[i].vIdx >= OBJECT_COUNT) {
			UINT dd = KVli[i].vIdx;
			if (cntError < 20) {
				check[cntError] = KVli[i];
				indexError[cntError] = i;
				cntError++;
			}
			dd++;
		}
	}
	if(cntError != 0)
		check[0].kHash = 0;
#endif
}

int UpdateClass::Radix1DSeg2Kv(floatArray3 locations, floatArray3 locationsSorted, UINT * originalIndex, UINT * cellStart, UINT * cellEnd, UINT length, UINT bitCount) {

	const int ompTh = 8; int i;
#pragma omp parallel for num_threads(ompTh)
	for (i = 0; i < (int)OBJECT_COUNT; i++) {
		KVList[i].kHash = hashCPU(locations.x[i], locations.y[i], locations.z[i]);
		KVList[i].vIdx = i;
	}

	checkKVList(KVList, 0);
	//checkKVList(KVTemp, 1);

	for (UINT bitLoc = 0; bitLoc < bitCount; bitLoc += 2) {
		int blockId;
		memset(countBit, 0, sizeof(UINT) * sizeDa);

		int blockCount = (int)ceil(length / 256.0f);
#pragma omp parallel for num_threads(ompTh)
		for (blockId = 0; blockId < blockCount; blockId++) {
			checkCount2(KVList, KVTemp, countBit, bitLoc, blockId, blockCount, length);
		}
		checkKVList(KVList, 2);
		checkKVList(KVTemp, 3);
		int loop;
		UINT zero = 0;
		UINT onee = 0;
		UINT twoo = 0;
		UINT thre = 0;
		for (loop = 0; loop < blockCount; loop++) {
			countBitCounts[loop] = countBit[loop] + zero;
			countBitCounts[loop + blockCount] = countBit[loop + blockCount] + onee;
			countBitCounts[loop + blockCount * 2] = countBit[loop + blockCount * 2] + twoo;
			countBitCounts[loop + blockCount * 3] = countBit[loop + blockCount * 3] + thre;
			zero += countBit[loop];
			onee += countBit[loop + blockCount];
			twoo += countBit[loop + blockCount * 2];
			thre += countBit[loop + blockCount * 3];
		}

		for (loop = 0; loop < blockCount; loop++)
			countBitCounts[loop + blockCount] += countBitCounts[blockCount - 1];
		for (loop = 0; loop < blockCount; loop++)
			countBitCounts[loop + blockCount * 2] += countBitCounts[blockCount * 2 - 1];
		for (loop = 0; loop < blockCount; loop++)
			countBitCounts[loop + blockCount * 3] += countBitCounts[blockCount * 3 - 1];

#pragma omp parallel for num_threads(ompTh)
		for (loop = 1; loop < blockCount * 4; loop++) {
			countBitStarts[loop] = countBitCounts[loop - 1];
		}

#pragma omp parallel for num_threads(ompTh)
		for (blockId = 0; blockId < blockCount; blockId++) {
			pack2(KVList, KVTemp, countBit, countBitStarts, blockId, blockCount, length);
		}
		checkKVList(KVList, 4);
		checkKVList(KVTemp, 5);
	}
	
	checkKVList(KVList, 6);
	checkKVList(KVTemp, 7);

//Using the sort results to reorder data.
#pragma omp parallel for num_threads(ompTh)
	for (i = 0; i < (int)OBJECT_COUNT; i++) {
		UINT Original_Index = KVList[i].vIdx;
		originalIndex[i] = Original_Index;
		locationsSorted.x[i] = locations.x[Original_Index];
		locationsSorted.y[i] = locations.y[Original_Index];
		locationsSorted.z[i] = locations.z[Original_Index];
		segRadiusSorted[i] = segRadius[Original_Index];
	}
	memset(cellStart, 0xffffffff, HASH_ENTRIE * sizeof(UINT));
	int current = KVList[0].kHash;
	cellStart[current] = 0; // 0 = first in sorted array
	for (int u = 0; u < (int)OBJECT_COUNT; u++) {
		UINT iterHash = KVList[u].kHash;
		if (iterHash != current) {
			cellEnd[current] = u;
			cellStart[iterHash] = u;
			current = iterHash;
		}
	}
	cellEnd[current] = OBJECT_COUNT;

	return 0;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
float UpdateClass::dot(XMFLOAT3 a) {
	return ((a.x * a.x) + (a.y * a.y)) + (a.z * a.z);
}
float UpdateClass::dot(XMFLOAT4 a)
{
	return (((a.x * a.x) + (a.y * a.y)) + (a.z * a.z)) + (a.w * a.w);
}
float UpdateClass::dot(float x, float y, float z) {
	return ((x * x) + (y * y)) + (z * z);
}
float UpdateClass::dot(XMFLOAT3 a, XMFLOAT3 b) {
	return ((a.x * b.x) + (a.y * b.y)) + (a.z * b.z);
}
float UpdateClass::return_inverse(float k) {
	if (k == 0.0f)
		return 0.0f;
	else
		return (1.0f / k);
}
XMFLOAT3 UpdateClass::normalize(XMFLOAT3 q)
{
	float num2 = dot(q);
	float num = return_inverse(sqrtf(num2));
	XMFLOAT3 norm;
	norm.x = q.x * num;
	norm.y = q.y * num;
	norm.z = q.z * num;
	return norm;
}
XMFLOAT3 UpdateClass::normalize(XMFLOAT3 q, float m)
{
	float num = return_inverse(m);
	XMFLOAT3 norm;
	norm.x = q.x * num;
	norm.y = q.y * num;
	norm.z = q.z * num;

	return norm;
}
XMFLOAT3 UpdateClass::normalize(float x, float y, float z)
{
	float num2 = dot(x, y, z);
	float num = return_inverse(sqrtf(num2));
	XMFLOAT3 norm;
	norm.x = x * num;
	norm.y = y * num;
	norm.z = z * num;
	return norm;
}
XMFLOAT4 UpdateClass::normalize(XMFLOAT4 q)
{
	float num2 = dot(q);
	float num = return_inverse(sqrtf(num2));
	XMFLOAT4 norm;
	norm.x = q.x * num;
	norm.y = q.y * num;
	norm.z = q.z * num;
	norm.w = q.w * num;
	return norm;
}
XMFLOAT4 UpdateClass::normalize(XMFLOAT4 q, float m)
{
	float num = return_inverse(m);
	XMFLOAT4 norm;
	norm.x = q.x * num;
	norm.y = q.y * num;
	norm.z = q.z * num;
	norm.w = q.w * num;
	return norm;
}
XMFLOAT3 UpdateClass::multiplyF3(XMFLOAT3 a, float b) {
	a.x *= b;
	a.y *= b;
	a.z *= b;
	return a;
}
XMFLOAT4 UpdateClass::multiplyF4(XMFLOAT4 a, float b) {
	a.x *= b;
	a.y *= b;
	a.z *= b;
	a.w *= b;
	return a;
}
void UpdateClass::swap(float *a, float *b) {
	float t = *b;
	*b = *a;
	*a = t;
}
bool UpdateClass::solveQuadratic(const float &a, const float &b, const float &c, float &x0, float &x1)
{
	float discr = b * b - 4.0f * a * c;
	if (discr < 0) return false;
	else if (discr == 0) {
		x0 = x1 = -0.5f * b / a;
	}
	else {
		float q = (b > 0) ?
			-0.5f * (b + sqrtf(discr)) :
			-0.5f * (b - sqrtf(discr));
		x0 = q / a;
		x1 = c / q;
	}

	return true;
}
XMFLOAT3 UpdateClass::multiply(float f, XMFLOAT3 a) {
	return XMFLOAT3(f*a.x, f*a.y, f*a.z);
}
XMFLOAT3 UpdateClass::divide(XMFLOAT3 a, float f) {
	return XMFLOAT3(a.x / f, a.y / f, a.z / f);
}
XMFLOAT3 UpdateClass::add(XMFLOAT3 a, XMFLOAT3 b) {
	return XMFLOAT3(a.x + b.x, a.y + b.y, a.z + b.z);
}
XMFLOAT3 UpdateClass::subtract(XMFLOAT3 a, XMFLOAT3 b) {
	return XMFLOAT3(a.x - b.x, a.y - b.y, a.z - b.z);
}
// Intersects ray r = p + td, |d| = 1, with sphere s and, if intersecting, returns t value of intersection and intersection point q 
int UpdateClass::IntersectRaySphere(XMFLOAT3 origin, XMFLOAT3 ray, XMFLOAT3 s, float radius, float &t, XMFLOAT3 &q)
{
	XMFLOAT3 m = subtract(origin, s);
	float b = dot(m, ray);
	float c = dot(m, m) - radius * radius;

	// Exit if r’s origin outside s (c > 0) and r pointing away from s (b > 0) 
	if (c > 0.0f && b > 0.0f) return 0;
	float discr = b*b - c;

	// A negative discriminant corresponds to ray missing sphere 
	if (discr < 0.0f) return 0;

	// Ray now found to intersect sphere, compute smallest t value of intersection
	t = -b - sqrt(discr);

	// If t is negative, ray started inside sphere so clamp t to zero 
	if (t < 0.0f) t = 0.0f;
	q = add(origin, multiply(t, ray));

	return 1;
}
bool UpdateClass::intersectGeo(const XMFLOAT3 &rayOrig, const XMFLOAT3 &rayDir, const XMFLOAT3 &sphereCentLoc, const float &sphereRadius2, float &t)
{
	float t0, t1; // solutions for t if the ray intersects 
				  // geometric solution
	XMFLOAT3 L = subtract(sphereCentLoc, rayOrig);
	float tca = dot(L, rayDir);
	if (tca < 0) return false;
	float d2 = dot(L) - tca * tca;
	if (d2 > sphereRadius2) return false;
	float thc = sqrt(sphereRadius2 - d2);
	t0 = tca - thc;
	t1 = tca + thc;

	printf("t0:%5.1f, t1:%5.1f\n", t0, t1);

	if (t0 > t1) swap(&t0, &t1);

	if (t0 < 0) {
		t0 = t1; // if t0 is negative, let's use t1 instead 
		if (t0 < 0) return false; // both t0 and t1 are negative 
	}

	t = t0;

	return true;
}
bool UpdateClass::intersectAny(const XMFLOAT3 &rayOrig, const XMFLOAT3 &rayDir, const XMFLOAT3 &sphereCentLoc, const float &sphereRadius2, float &t)
{
	float t0, t1; // solutions for t if the ray intersects 
				  // analytic solution
	XMFLOAT3 L = subtract(rayOrig, sphereCentLoc);
	float a = dot(rayDir);
	float b = 2.0f * dot(rayDir, L);
	float c = dot(L) - (sphereRadius2); //Precompute Sphere radius^2 and save it
	if (!solveQuadratic(a, b, c, t0, t1)) return false;

	printf("t0:%5.1f, t1:%5.1f\n", t0, t1);

	if (t0 > t1) swap(&t0, &t1);

	if (t0 < 0) {
		t0 = t1; // if t0 is negative, let's use t1 instead 
		if (t0 < 0) return false; // both t0 and t1 are negative 
	}

	t = t0;

	return true;
}
void UpdateClass::Pick(int sx, int sy)
{

	auto ri = mOpaqueRitems[getInstIdx("segments")];
	int cnt = ri->InstanceCount;
	XMFLOAT4X4 P = mCamera.GetProj4x4f();

	// Compute picking ray in view space.
	float vx = (+2.0f*sx / mClientWidth - 1.0f) / P(0, 0);
	float vy = (-2.0f*sy / mClientHeight + 1.0f) / P(1, 1);

	// Ray definition in view space.
	XMVECTOR rayOrigin = XMVectorSet(0.0f, 0.0f, 0.0f, 1.0f);//mCamera.GetPosition();//
	XMVECTOR rayDir = XMVectorSet(vx, vy, 1.0f, 0.0f);

	XMMATRIX V = mCamera.GetView();
	XMMATRIX invView = XMMatrixInverse(&XMMatrixDeterminant(V), V);
	rayOrigin = XMVector3TransformCoord(rayOrigin, invView);
	rayDir = XMVector3TransformNormal(rayDir, invView);
	rayDir = XMVector3Normalize(rayDir);
	XMFLOAT3 cameraLoc; XMStoreFloat3(&cameraLoc, rayOrigin);//XMStoreFloat3(&cameraLoc, rayOrigin);
	XMFLOAT3 cameraRay; XMStoreFloat3(&cameraRay, rayDir);

	int index = -1;
	float tMin = FLT_MAX;
	for (int i = 0; i < cnt; i++) {
		const float rSquared = 1;

		float tSolution = 0;
		XMFLOAT3 sphereLoc = XMFLOAT3(ri->Instances[i].World._41, ri->Instances[i].World._42, ri->Instances[i].World._43);
		bool rayImpact = intersectAny(cameraLoc, cameraRay, sphereLoc, rSquared, tSolution);
		printf("%s = intersectGeo({%5.1f, %5.1f, %5.1f}, {%5.1f, %5.1f, %5.1f}, {%5.1f, %5.1f, %5.1f}, %5.1f, %5.2f)\n",
			rayImpact ? "true" : "false", cameraLoc.x, cameraLoc.y, cameraLoc.z, cameraRay.x, cameraRay.y, cameraRay.z, sphereLoc.x, sphereLoc.y, sphereLoc.z, rSquared, tSolution);
		printf("tSolution %5.3f\n\n", tSolution);
		//bool rayImpact = IntersectRaySphere(cameraLoc, cameraRay, sphereLoc, 1, tSolution, pSolution);

		if (rayImpact && tSolution < tMin) {
			tMin = tSolution;
			index = i;
		}
	}
	if (index > -1) {
		//UCP->shiftColor(index, 2.0f);
		holdingIdx[0] = index;
	}
}
void UpdateClass::PickOpti(int sx, int sy)
{
	auto ri = mOpaqueRitems[getInstIdx("segments")];
	int cnt = ri->InstanceCount;
	XMFLOAT4X4 P = mCamera.GetProj4x4f();

	// Compute picking ray in view space.
	float vx = (+2.0f*sx / mClientWidth - 1.0f) / P(0, 0);
	float vy = (-2.0f*sy / mClientHeight + 1.0f) / P(1, 1);

	// Ray definition in view space.
	XMVECTOR rayOrigin = XMVectorSet(0.0f, 0.0f, 0.0f, 1.0f);//mCamera.GetPosition();//
	XMVECTOR rayDir = XMVectorSet(vx, vy, 1.0f, 0.0f);

	XMMATRIX V = mCamera.GetView();
	XMMATRIX invView = XMMatrixInverse(&XMMatrixDeterminant(V), V);
	rayOrigin = XMVector3TransformCoord(rayOrigin, invView);
	rayDir = XMVector3TransformNormal(rayDir, invView);
	rayDir = XMVector3Normalize(rayDir);
	XMFLOAT3 cameraLoc; XMStoreFloat3(&cameraLoc, rayOrigin);
	XMFLOAT3 cameraRay; XMStoreFloat3(&cameraRay, rayDir);
	XMFLOAT3 cameraLocNormalized; XMStoreFloat3(&cameraLocNormalized, rayOrigin); //cameraLoc on the bounding cube

	float _result;

	//Calculating cameraLocation on the bounding cube
	//IntersectRayAABB(XMFLOAT3 origin, XMFLOAT3 rayInv, XMFLOAT3 boxMin, XMFLOAT3 boxMax, float &t)
	if (IntersectRayAABB(cameraLoc,
		XMFLOAT3(1.0f / cameraRay.x, 1.0f / cameraRay.y, 1.0f / cameraRay.z),
		XMFLOAT3(-GRIDxHALF, -GRIDyDEPTH_BUFFER, -GRIDzHALF),
		XMFLOAT3(GRIDxHALF, GRIDyELEVATION_BUFFER, GRIDzHALF),
		_result))
	{
		if (_result > 0)
		{
			cameraLocNormalized.x += cameraRay.x * (_result);
			cameraLocNormalized.y += cameraRay.y * (_result);
			cameraLocNormalized.z += cameraRay.z * (_result);
		}
	}

	int index = -1;
	float tMin = FLT_MAX;

	//BFS visit array
	memset(pickingVisited, 0, sizeof(UINT) * HASH_ENTRIE);

	std::queue<XMINT3> gridPos;

	//Get hashed grid position of the starting grid.
	//try to make changes of hashCPU() also works here. (...
	int grid = hashCPU(cameraLocNormalized.x, cameraLocNormalized.y, cameraLocNormalized.z);
	gridPos.push(XMINT3(
		(grid % (CUBE_COUNTz_1D * CUBE_COUNTz_1D)) / CUBE_COUNTz_1D,
		grid / (CUBE_COUNTz_1D * CUBE_COUNTz_1D),
		grid % CUBE_COUNTz_1D));

	//BFS
	while (!gridPos.empty())
	{
		int targetGrid =
			gridPos.front().y * CUBE_COUNTz_1D * CUBE_COUNTx_1D +
			gridPos.front().x * CUBE_COUNTz_1D +
			gridPos.front().z;

		int _cellStart = segCellStart[targetGrid];
		int _cellEnd = segCellEnd[targetGrid];

		//Check if we picked any objects
		//BFS is not terminated though.
		if (_cellStart > -1)
		{
			for (int i = _cellStart; i < _cellEnd; i++)
			{
				const float rSquared = 1;
				float tSolution = 0;

				int id = (int)segOriginalIndex[i];

				XMFLOAT3 sphereLoc = XMFLOAT3(ri->Instances[id].World._41, ri->Instances[id].World._42, ri->Instances[id].World._43);
				bool rayImpact = intersectAny(cameraLoc, cameraRay, sphereLoc, rSquared, tSolution);

				printf("%s = intersectGeo({%5.1f, %5.1f, %5.1f}, {%5.1f, %5.1f, %5.1f}, {%5.1f, %5.1f, %5.1f}, %5.1f, %5.2f)\n",
					rayImpact ? "true" : "false", cameraLoc.x, cameraLoc.y, cameraLoc.z, cameraRay.x, cameraRay.y, cameraRay.z, sphereLoc.x, sphereLoc.y, sphereLoc.z, rSquared, tSolution);
				printf("tSolution %5.3f\n\n", tSolution);

				if (rayImpact && tSolution < tMin)
				{
					tMin = tSolution;
					index = id;
				}
			}
		}

		//Adding grids into the queue
		for (int sx = -1; sx <= 1; sx++)
		{
			for (int sy = -1; sy <= 1; sy++)
			{
				for (int sz = -1; sz <= 1; sz++)
				{
					XMINT3 newGrid = XMINT3(
						gridPos.front().x + sx,
						gridPos.front().y + sy,
						gridPos.front().z + sz);

					//Edge case check etc.
					if ((sx == 0 && sy == 0 && sz == 0)
						|| newGrid.x > (int)CUBE_COUNTx_1D
						|| newGrid.x < 0
						|| newGrid.y >(int)CUBE_COUNTy_1D
						|| newGrid.y < 0
						|| newGrid.z >(int)CUBE_COUNTz_1D
						|| newGrid.z < 0
						|| pickingVisited[
							newGrid.y * CUBE_COUNTz_1D * CUBE_COUNTx_1D +
								newGrid.x * CUBE_COUNTz_1D +
								newGrid.z] > 0)
					{
						continue;
					}

							//Use a sphere to detect if cameraRay is intersected with a grid.
							//sphere radius = sqrt(3) / 2 = 0.867 + rSegment (segments may on the corner of the cube)

							//However, there are 2x same grid occurs near zero.
							float rSquared = (float)CUBE * 0.867f + 1.0f;
							/*bool boundSphereExtended = false;

							float xPos = (float)CUBE / 2.0f + (float)((int)CUBE * (newGrid.x - (int)CUBE_COUNTx_1D_HALF));
							float zPos = (float)CUBE / 2.0f + (float)((int)CUBE * (newGrid.z - (int)CUBE_COUNTz_1D_HALF));
							if (newGrid.x < CUBE_COUNTx_1D_HALF)
							{
								xPos = (float)CUBE / 2.0f + (float)((int)CUBE * ((newGrid.x - 1) - (int)CUBE_COUNTx_1D_HALF));
							}
							if (newGrid.z < CUBE_COUNTz_1D_HALF)
							{
								zPos = (float)CUBE / 2.0f + (float)((int)CUBE * ((newGrid.z - 1) - (int)CUBE_COUNTz_1D_HALF));
							}
							if (newGrid.x == CUBE_COUNTx_1D_HALF)
							{
								xPos = 0;
								rSquared *= 2;
								boundSphereExtended = true;
							}
							if (newGrid.z == CUBE_COUNTz_1D_HALF)
							{
								zPos = 0;
								if (!boundSphereExtended)
								{
									rSquared *= 2;
								}
							}

							//Compute grid centerPos
							XMFLOAT3 gridLoc = XMFLOAT3(
								xPos,
								(float)CUBE / 2.0f + (float)((int)CUBE * (newGrid.y)),
								zPos);*/

							XMFLOAT3 gridLoc = XMFLOAT3(
								(float)CUBE / 2.0f + (float)(CUBE * newGrid.x) - GRIDxHALF,
								(float)CUBE / 2.0f + (float)(CUBE * newGrid.y) - GRIDyDEPTH_BUFFER,
								(float)CUBE / 2.0f + (float)(CUBE * newGrid.z) - GRIDzHALF);

							rSquared = rSquared * rSquared;
							float tSolution = 0;

							bool rayImpact = intersectAny(cameraLoc, cameraRay, gridLoc, rSquared, tSolution);

							//Add into queue
							if (rayImpact && pickingVisited[
								newGrid.y * CUBE_COUNTz_1D * CUBE_COUNTx_1D +
									newGrid.x * CUBE_COUNTz_1D +
									newGrid.z] == 0)
							{
								pickingVisited[
									newGrid.y * CUBE_COUNTz_1D * CUBE_COUNTx_1D +
										newGrid.x * CUBE_COUNTz_1D +
										newGrid.z] = 1;

								printf("%d %d %d inQueue\n", newGrid.x, newGrid.y, newGrid.z);

								gridPos.push(newGrid);
							}
				}
			}
		}

		gridPos.pop();
	}

	if (index > -1) {
		//UCP->shiftColor(index, 2.0f);
		holdingIdx[0] = index;
	}
}
bool UpdateClass::IntersectRayAABB(XMFLOAT3 origin, XMFLOAT3 rayInv, XMFLOAT3 boxMin, XMFLOAT3 boxMax, float &t)
{
	double t1 = (boxMin.x - origin.x)*rayInv.x;
	double t2 = (boxMax.x - origin.x)*rayInv.x;

	double tmin = min(t1, t2);
	double tmax = max(t1, t2);

	t1 = (boxMin.y - origin.y)*rayInv.y;
	t2 = (boxMax.y - origin.y)*rayInv.y;

	tmin = max(tmin, min(min(t1, t2), tmax));
	tmax = min(tmax, max(max(t1, t2), tmin));

	t1 = (boxMin.z - origin.z)*rayInv.z;
	t2 = (boxMax.z - origin.z)*rayInv.z;

	tmin = max(tmin, min(min(t1, t2), tmax));
	tmax = min(tmax, max(max(t1, t2), tmin));

	t = tmin;

	return tmax > max(tmin, 0.0);
}

void UpdateClass::Matrix4x4Multiply(float src1[4][4], float src2[4][4], float dest[4][4])
{
	dest[0][0] = src1[0][0] * src2[0][0] + src1[0][1] * src2[1][0] + src1[0][2] * src2[2][0] + src1[0][3] * src2[3][0];
	dest[0][1] = src1[0][0] * src2[0][1] + src1[0][1] * src2[1][1] + src1[0][2] * src2[2][1] + src1[0][3] * src2[3][1];
	dest[0][2] = src1[0][0] * src2[0][2] + src1[0][1] * src2[1][2] + src1[0][2] * src2[2][2] + src1[0][3] * src2[3][2];
	dest[0][3] = src1[0][0] * src2[0][3] + src1[0][1] * src2[1][3] + src1[0][2] * src2[2][3] + src1[0][3] * src2[3][3];
	dest[1][0] = src1[1][0] * src2[0][0] + src1[1][1] * src2[1][0] + src1[1][2] * src2[2][0] + src1[1][3] * src2[3][0];
	dest[1][1] = src1[1][0] * src2[0][1] + src1[1][1] * src2[1][1] + src1[1][2] * src2[2][1] + src1[1][3] * src2[3][1];
	dest[1][2] = src1[1][0] * src2[0][2] + src1[1][1] * src2[1][2] + src1[1][2] * src2[2][2] + src1[1][3] * src2[3][2];
	dest[1][3] = src1[1][0] * src2[0][3] + src1[1][1] * src2[1][3] + src1[1][2] * src2[2][3] + src1[1][3] * src2[3][3];
	dest[2][0] = src1[2][0] * src2[0][0] + src1[2][1] * src2[1][0] + src1[2][2] * src2[2][0] + src1[2][3] * src2[3][0];
	dest[2][1] = src1[2][0] * src2[0][1] + src1[2][1] * src2[1][1] + src1[2][2] * src2[2][1] + src1[2][3] * src2[3][1];
	dest[2][2] = src1[2][0] * src2[0][2] + src1[2][1] * src2[1][2] + src1[2][2] * src2[2][2] + src1[2][3] * src2[3][2];
	dest[2][3] = src1[2][0] * src2[0][3] + src1[2][1] * src2[1][3] + src1[2][2] * src2[2][3] + src1[2][3] * src2[3][3];
	dest[3][0] = src1[3][0] * src2[0][0] + src1[3][1] * src2[1][0] + src1[3][2] * src2[2][0] + src1[3][3] * src2[3][0];
	dest[3][1] = src1[3][0] * src2[0][1] + src1[3][1] * src2[1][1] + src1[3][2] * src2[2][1] + src1[3][3] * src2[3][1];
	dest[3][2] = src1[3][0] * src2[0][2] + src1[3][1] * src2[1][2] + src1[3][2] * src2[2][2] + src1[3][3] * src2[3][2];
	dest[3][3] = src1[3][0] * src2[0][3] + src1[3][1] * src2[1][3] + src1[3][2] * src2[2][3] + src1[3][3] * src2[3][3];
}
void UpdateClass::Matrix4x4Multiply(float src1[16], float src2[16], float dest[16])
{
	dest[0 * 4 + 0] = src1[0 * 4 + 0] * src2[0 * 4 + 0] + src1[0 * 4 + 1] * src2[1 * 4 + 0] + src1[0 * 4 + 2] * src2[2 * 4 + 0] + src1[0 * 4 + 3] * src2[3 * 4 + 0];
	dest[0 * 4 + 1] = src1[0 * 4 + 0] * src2[0 * 4 + 1] + src1[0 * 4 + 1] * src2[1 * 4 + 1] + src1[0 * 4 + 2] * src2[2 * 4 + 1] + src1[0 * 4 + 3] * src2[3 * 4 + 1];
	dest[0 * 4 + 2] = src1[0 * 4 + 0] * src2[0 * 4 + 2] + src1[0 * 4 + 1] * src2[1 * 4 + 2] + src1[0 * 4 + 2] * src2[2 * 4 + 2] + src1[0 * 4 + 3] * src2[3 * 4 + 2];
	dest[0 * 4 + 3] = src1[0 * 4 + 0] * src2[0 * 4 + 3] + src1[0 * 4 + 1] * src2[1 * 4 + 3] + src1[0 * 4 + 2] * src2[2 * 4 + 3] + src1[0 * 4 + 3] * src2[3 * 4 + 3];
	dest[1 * 4 + 0] = src1[1 * 4 + 0] * src2[0 * 4 + 0] + src1[1 * 4 + 1] * src2[1 * 4 + 0] + src1[1 * 4 + 2] * src2[2 * 4 + 0] + src1[1 * 4 + 3] * src2[3 * 4 + 0];
	dest[1 * 4 + 1] = src1[1 * 4 + 0] * src2[0 * 4 + 1] + src1[1 * 4 + 1] * src2[1 * 4 + 1] + src1[1 * 4 + 2] * src2[2 * 4 + 1] + src1[1 * 4 + 3] * src2[3 * 4 + 1];
	dest[1 * 4 + 2] = src1[1 * 4 + 0] * src2[0 * 4 + 2] + src1[1 * 4 + 1] * src2[1 * 4 + 2] + src1[1 * 4 + 2] * src2[2 * 4 + 2] + src1[1 * 4 + 3] * src2[3 * 4 + 2];
	dest[1 * 4 + 3] = src1[1 * 4 + 0] * src2[0 * 4 + 3] + src1[1 * 4 + 1] * src2[1 * 4 + 3] + src1[1 * 4 + 2] * src2[2 * 4 + 3] + src1[1 * 4 + 3] * src2[3 * 4 + 3];
	dest[2 * 4 + 0] = src1[2 * 4 + 0] * src2[0 * 4 + 0] + src1[2 * 4 + 1] * src2[1 * 4 + 0] + src1[2 * 4 + 2] * src2[2 * 4 + 0] + src1[2 * 4 + 3] * src2[3 * 4 + 0];
	dest[2 * 4 + 1] = src1[2 * 4 + 0] * src2[0 * 4 + 1] + src1[2 * 4 + 1] * src2[1 * 4 + 1] + src1[2 * 4 + 2] * src2[2 * 4 + 1] + src1[2 * 4 + 3] * src2[3 * 4 + 1];
	dest[2 * 4 + 2] = src1[2 * 4 + 0] * src2[0 * 4 + 2] + src1[2 * 4 + 1] * src2[1 * 4 + 2] + src1[2 * 4 + 2] * src2[2 * 4 + 2] + src1[2 * 4 + 3] * src2[3 * 4 + 2];
	dest[2 * 4 + 3] = src1[2 * 4 + 0] * src2[0 * 4 + 3] + src1[2 * 4 + 1] * src2[1 * 4 + 3] + src1[2 * 4 + 2] * src2[2 * 4 + 3] + src1[2 * 4 + 3] * src2[3 * 4 + 3];
	dest[3 * 4 + 0] = src1[3 * 4 + 0] * src2[0 * 4 + 0] + src1[3 * 4 + 1] * src2[1 * 4 + 0] + src1[3 * 4 + 2] * src2[2 * 4 + 0] + src1[3 * 4 + 3] * src2[3 * 4 + 0];
	dest[3 * 4 + 1] = src1[3 * 4 + 0] * src2[0 * 4 + 1] + src1[3 * 4 + 1] * src2[1 * 4 + 1] + src1[3 * 4 + 2] * src2[2 * 4 + 1] + src1[3 * 4 + 3] * src2[3 * 4 + 1];
	dest[3 * 4 + 2] = src1[3 * 4 + 0] * src2[0 * 4 + 2] + src1[3 * 4 + 1] * src2[1 * 4 + 2] + src1[3 * 4 + 2] * src2[2 * 4 + 2] + src1[3 * 4 + 3] * src2[3 * 4 + 2];
	dest[3 * 4 + 3] = src1[3 * 4 + 0] * src2[0 * 4 + 3] + src1[3 * 4 + 1] * src2[1 * 4 + 3] + src1[3 * 4 + 2] * src2[2 * 4 + 3] + src1[3 * 4 + 3] * src2[3 * 4 + 3];
}
void UpdateClass::Matrix4x4Multiply(float4x4 src1, float4x4 src2, float4x4 dest)
{
	dest.m00 = src1.m00 * src2.m00 + src1.m01 * src2.m10 + src1.m02 * src2.m20 + src1.m03 * src2.m30;
	dest.m01 = src1.m00 * src2.m01 + src1.m01 * src2.m11 + src1.m02 * src2.m21 + src1.m03 * src2.m31;
	dest.m02 = src1.m00 * src2.m02 + src1.m01 * src2.m12 + src1.m02 * src2.m22 + src1.m03 * src2.m32;
	dest.m03 = src1.m00 * src2.m03 + src1.m01 * src2.m13 + src1.m02 * src2.m23 + src1.m03 * src2.m33;
	dest.m10 = src1.m10 * src2.m00 + src1.m11 * src2.m10 + src1.m12 * src2.m20 + src1.m13 * src2.m30;
	dest.m11 = src1.m10 * src2.m01 + src1.m11 * src2.m11 + src1.m12 * src2.m21 + src1.m13 * src2.m31;
	dest.m12 = src1.m10 * src2.m02 + src1.m11 * src2.m12 + src1.m12 * src2.m22 + src1.m13 * src2.m32;
	dest.m13 = src1.m10 * src2.m03 + src1.m11 * src2.m13 + src1.m12 * src2.m23 + src1.m13 * src2.m33;
	dest.m20 = src1.m20 * src2.m00 + src1.m21 * src2.m10 + src1.m22 * src2.m20 + src1.m23 * src2.m30;
	dest.m21 = src1.m20 * src2.m01 + src1.m21 * src2.m11 + src1.m22 * src2.m21 + src1.m23 * src2.m31;
	dest.m22 = src1.m20 * src2.m02 + src1.m21 * src2.m12 + src1.m22 * src2.m22 + src1.m23 * src2.m32;
	dest.m23 = src1.m20 * src2.m03 + src1.m21 * src2.m13 + src1.m22 * src2.m23 + src1.m23 * src2.m33;
	dest.m30 = src1.m30 * src2.m00 + src1.m31 * src2.m10 + src1.m32 * src2.m20 + src1.m33 * src2.m30;
	dest.m31 = src1.m30 * src2.m01 + src1.m31 * src2.m11 + src1.m32 * src2.m21 + src1.m33 * src2.m31;
	dest.m32 = src1.m30 * src2.m02 + src1.m31 * src2.m12 + src1.m32 * src2.m22 + src1.m33 * src2.m32;
	dest.m33 = src1.m30 * src2.m03 + src1.m31 * src2.m13 + src1.m32 * src2.m23 + src1.m33 * src2.m33;
}
void UpdateClass::make16mat(float * out, XMMATRIX in) {
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			out[c * 4 + r] = in.r[c].m128_f32[r];
		}
	}
}

//Matlab like coloring///////////////////////////////////////////
XMFLOAT3 UpdateClass::GetColour(float v, float vmin, float vmax)
{
	XMFLOAT3 c = { 1.0f,1.0f,1.0f }; // white
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
void UpdateClass::timeCumulative(double * cumulative, double * mod) {
	if (frameCount % 40 == 0) {
		*mod = *cumulative / 40.f;
		*cumulative = 0;
	}
}

XMFLOAT3 UpdateClass::moveFVect(float distance, XMFLOAT3 location, XMFLOAT3 forw) {
	XMFLOAT3 ret;
	ret.x = location.x + (forw.x * distance);
	ret.y = location.y + (forw.y * distance);
	ret.z = location.z + (forw.z * distance);
	return ret;
}
XMFLOAT4 UpdateClass::Multiply(XMFLOAT4 quaternion1, XMFLOAT4 quaternion2)
{
	XMFLOAT4 quaternion;
	float x = quaternion1.x;
	float y = quaternion1.y;
	float z = quaternion1.z;
	float w = quaternion1.w;
	float num4 = quaternion2.x;
	float num3 = quaternion2.y;
	float num2 = quaternion2.z;
	float num = quaternion2.w;
	float num12 = (y * num2) - (z * num3);
	float num11 = (z * num4) - (x * num2);
	float num10 = (x * num3) - (y * num4);
	float num9 = ((x * num4) + (y * num3)) + (z * num2);
	quaternion.x = ((x * num) + (num4 * w)) + num12;
	quaternion.y = ((y * num) + (num3 * w)) + num11;
	quaternion.z = ((z * num) + (num2 * w)) + num10;
	quaternion.w = (w * num) - num9;
	return quaternion;
}
XMFLOAT4 UpdateClass::Multiply(XMFLOAT4 quaternion1, float scaleFactor)
{
	XMFLOAT4 quaternion;
	quaternion.x = quaternion1.x * scaleFactor;
	quaternion.y = quaternion1.y * scaleFactor;
	quaternion.z = quaternion1.z * scaleFactor;
	quaternion.w = quaternion1.w * scaleFactor;
	return quaternion;
}
XMFLOAT4 UpdateClass::CreateFromYawPitchRoll(float yaw, float pitch, float roll)
{
	XMFLOAT4 quaternion;
	float num9 = roll * 0.5f;
	float num6 = sin(num9);
	float num5 = cos(num9);
	float num8 = pitch * 0.5f;
	float num4 = sin(num8);
	float num3 = cos(num8);
	float num7 = yaw * 0.5f;
	float num2 = sin(num7);
	float num = cos(num7);
	quaternion.x = ((num * num4) * num5) + ((num2 * num3) * num6);
	quaternion.y = ((num2 * num3) * num5) - ((num * num4) * num6);
	quaternion.z = ((num * num3) * num6) - ((num2 * num4) * num5);
	quaternion.w = ((num * num3) * num5) + ((num2 * num4) * num6);
	return quaternion;
}

XMFLOAT3 UpdateClass::getColourR2G(float v, float max) {
	XMFLOAT3 c = { 1.0f,1.0f,1.0f }; // white
	float dv;

	if (v < -max)
		v = -max;
	if (v > max)
		v = (max);
	
	if (v < (-max / 2.0f)) {
		c.y = (2.0f * v / max) + 2.0f;
		c.z = 0.0f;
	} else if (v < 0.0f) {
		c.x = -2.0f * v / max;
		c.z = 0.0f;
	} else if (v < (max / 2.0f)) {
		c.x = 0.0f;
		c.z = 2.0f * v / max;
	} else {
		c.x = 0.0f;
		c.y = 2.0f - (2.0f * v / max);
	}

	return(c);
}

void UpdateClass::ObjectManager::init(UINT handSegCount) {
	objSegStart.push_back(0);
	objSegCount.push_back(handSegCount);
	objAnchorStart.push_back(0);
	objAnchorCount.push_back(0);
	numObj++;
}

void UpdateClass::ObjectManager::initAnch2Seg() {
	anch2SegmentStart.push_back(0);
	for (int i = 0; i < numAnch - 1; i++) {
		anch2SegmentStart.push_back(anch2SegmentStart[i] + anchors.numConSeg[i]);
	}
}

UINT UpdateClass::ObjectManager::addObject(UINT segCount, UINT anchCount) {
	objSegStart.push_back(objSegStart[numObj - 1] + objSegCount[numObj - 1]);
	objSegCount.push_back(segCount);
	objAnchorStart.push_back(objAnchorStart[numObj - 1] + objAnchorCount[numObj - 1]);
	objAnchorCount.push_back(anchCount);

	for (int i = 0; i < anchCount; i++) {
		anchors.x.push_back(FLT_MAX);
		anchors.y.push_back(FLT_MAX);
		anchors.z.push_back(FLT_MAX);
		anchors.veloX.push_back(0);
		anchors.veloY.push_back(0);
		anchors.veloZ.push_back(0);
		for (int j = 0; j < anch2AnchMax; j++) {
			anch2AnchorID.push_back(UINT_MAX);
			anch2AnchorDistance.push_back(FLT_MAX);
			anch2AnchorSpringConst.push_back(FLT_MAX);
		}
		anchors.connectedSegment.push_back(Anchor2SegmentLinks());
		anchors.numConSeg.push_back(0);
		numAnch++;
	}

	numObj++;
	return numObj - 1;
}

void UpdateClass::ObjectManager::setAnchorPos(UINT anchorID, XMFLOAT3 value) {
	anchors.x[anchorID] = value.x;
	anchors.y[anchorID] = value.y;
	anchors.z[anchorID] = value.z;
}

void UpdateClass::ObjectManager::setAnchorPos(UINT objectID, UINT objAnchorN, XMFLOAT3 value) {
	anchors.x[objAnchorStart[objectID] + objAnchorN] = value.x;
	anchors.y[objAnchorStart[objectID] + objAnchorN] = value.y;
	anchors.z[objAnchorStart[objectID] + objAnchorN] = value.z;
}

void UpdateClass::ObjectManager::addAnchorVelo(UINT anchorID, XMFLOAT3 value) {
	anchors.veloX[anchorID] += value.x;
	anchors.veloY[anchorID] += value.y;
	anchors.veloZ[anchorID] += value.z;
}

void UpdateClass::ObjectManager::addAnchorVelo(UINT objectID, UINT objAnchorN, XMFLOAT3 value) {
	anchors.veloX[objAnchorStart[objectID] + objAnchorN] += value.x;
	anchors.veloY[objAnchorStart[objectID] + objAnchorN] += value.y;
	anchors.veloZ[objAnchorStart[objectID] + objAnchorN] += value.z;
}

void UpdateClass::ObjectManager::addAnchorVelo2Pos(UINT anchorID) {
	anchors.x[anchorID] += anchors.veloX[anchorID];
	anchors.y[anchorID] += anchors.veloY[anchorID];
	anchors.z[anchorID] += anchors.veloZ[anchorID];
}

bool UpdateClass::ObjectManager::setAnchor2Anchor(UINT anchor1, UINT anchor2, float springConstant) {
	XMFLOAT3 vec = XMFLOAT3(anchors.x[anchor1] - anchors.x[anchor2], anchors.y[anchor1] - anchors.y[anchor2], anchors.z[anchor1] - anchors.z[anchor2]);
	float dist = sqrtf(((vec.x * vec.x) + (vec.y * vec.y)) + (vec.z * vec.z));

	bool isConnected = false;
	for (int i = 0; i < anch2AnchMax; i++) {
		if (anch2AnchorID[anchor1 * anch2AnchMax + i] == UINT_MAX) {
			anch2AnchorID[anchor1 * anch2AnchMax + i] = anchor2;
			anch2AnchorDistance[anchor1 * anch2AnchMax + i] = dist;
			anch2AnchorSpringConst[anchor1 * anch2AnchMax + i] = springConstant;
			isConnected = true;
			break;
		}
	}
	if (!isConnected) return isConnected;

	isConnected = false;
	for (int i = 0; i < anch2AnchMax; i++) {
		if (anch2AnchorID[anchor2 * anch2AnchMax + i] == UINT_MAX) {
			anch2AnchorID[anchor2 * anch2AnchMax + i] = anchor1;
			anch2AnchorDistance[anchor2 * anch2AnchMax + i] = dist;
			anch2AnchorSpringConst[anchor2 * anch2AnchMax + i] = springConstant;
			isConnected = true;
			break;
		}
	}
	if (!isConnected) return isConnected;

	return isConnected;
}

void UpdateClass::ObjectManager::set4Anchors(UINT objectID, UINT objSubID, XMFLOAT3 objCenter, float distanceToCenter, bool isReverse, float springConstant) {
	UINT startAnchor = getObjAnchorStart(objectID) + (objSubID * 4);
	int posNeg = isReverse ? -1 : 1;

	anchors.x[startAnchor] = objCenter.x;
	anchors.y[startAnchor] = objCenter.y + distanceToCenter * posNeg;
	anchors.z[startAnchor] = objCenter.z;

	anchors.x[startAnchor + 1] = objCenter.x + 0.94280904 * distanceToCenter * posNeg;
	anchors.y[startAnchor + 1] = objCenter.y - 0.33333333 * distanceToCenter * posNeg;
	anchors.z[startAnchor + 1] = objCenter.z;

	anchors.x[startAnchor + 2] = objCenter.x - 0.47140452 * distanceToCenter * posNeg;
	anchors.y[startAnchor + 2] = objCenter.y - 0.33333333 * distanceToCenter * posNeg;
	anchors.z[startAnchor + 2] = objCenter.z + 0.81649658 * distanceToCenter;

	anchors.x[startAnchor + 3] = objCenter.x - 0.47140452 * distanceToCenter * posNeg;
	anchors.y[startAnchor + 3] = objCenter.y - 0.33333333 * distanceToCenter * posNeg;
	anchors.z[startAnchor + 3] = objCenter.z - 0.81649658 * distanceToCenter;

	//Insert other anchors distances.
	for (int i = 0; i < 3; i++) {
		for (int j = i + 1; j < 4; j++) {
			setAnchor2Anchor(startAnchor + i, startAnchor + j, springConstant);
		}
	}
}

void UpdateClass::ObjectManager::set3Anchors(UINT objectID, UINT anchorN, XMFLOAT3 objCenter, float distanceToCenter, bool isReverse, bool connectedTriangle, float springConstant) {
	UINT startAnchor = getObjAnchorStart(objectID) + anchorN;
	int posNeg = isReverse ? -1 : 1;

	anchors.x[startAnchor] = objCenter.x;
	anchors.y[startAnchor] = objCenter.y + distanceToCenter * posNeg;
	anchors.z[startAnchor] = objCenter.z;

	anchors.x[startAnchor + 1] = objCenter.x;
	anchors.y[startAnchor + 1] = objCenter.y - 0.5 * distanceToCenter * posNeg;
	anchors.z[startAnchor + 1] = objCenter.z + 0.8660254038 * distanceToCenter;

	anchors.x[startAnchor + 2] = objCenter.x;
	anchors.y[startAnchor + 2] = objCenter.y - 0.5 * distanceToCenter * posNeg;
	anchors.z[startAnchor + 2] = objCenter.z - 0.8660254038 * distanceToCenter;

	//Connect all anchors together, if connectedTriangle is true.
	if (connectedTriangle) {
		setAnchor2Anchor(startAnchor, startAnchor + 1, springConstant);
		setAnchor2Anchor(startAnchor, startAnchor + 2, springConstant);
		setAnchor2Anchor(startAnchor + 1, startAnchor + 2, springConstant);
	}
}

void UpdateClass::ObjectManager::set4FlatAnchors(UINT objectID, UINT anchorN, XMFLOAT3 objCenter, float distanceToCenter, bool isDiagonal, float springConstant) {
	UINT startAnchor = getObjAnchorStart(objectID) + anchorN;
	
	if (isDiagonal) {
		anchors.x[startAnchor] = objCenter.x;
		anchors.y[startAnchor] = objCenter.y + distanceToCenter;
		anchors.z[startAnchor] = objCenter.z;

		anchors.x[startAnchor + 1] = objCenter.x;
		anchors.y[startAnchor + 1] = objCenter.y;
		anchors.z[startAnchor + 1] = objCenter.z + distanceToCenter;

		anchors.x[startAnchor + 2] = objCenter.x;
		anchors.y[startAnchor + 2] = objCenter.y - distanceToCenter;
		anchors.z[startAnchor + 2] = objCenter.z;

		anchors.x[startAnchor + 3] = objCenter.x;
		anchors.y[startAnchor + 3] = objCenter.y;
		anchors.z[startAnchor + 3] = objCenter.z - distanceToCenter;
	} else {
		anchors.x[startAnchor] = objCenter.x;
		anchors.y[startAnchor] = objCenter.y + distanceToCenter * 0.7071067812;
		anchors.z[startAnchor] = objCenter.z + distanceToCenter * 0.7071067812;

		anchors.x[startAnchor + 1] = objCenter.x;
		anchors.y[startAnchor + 1] = objCenter.y - distanceToCenter * 0.7071067812;
		anchors.z[startAnchor + 1] = objCenter.z + distanceToCenter * 0.7071067812;

		anchors.x[startAnchor + 2] = objCenter.x;
		anchors.y[startAnchor + 2] = objCenter.y - distanceToCenter * 0.7071067812;
		anchors.z[startAnchor + 2] = objCenter.z - distanceToCenter * 0.7071067812;

		anchors.x[startAnchor + 3] = objCenter.x;
		anchors.y[startAnchor + 3] = objCenter.y + distanceToCenter * 0.7071067812;
		anchors.z[startAnchor + 3] = objCenter.z - distanceToCenter * 0.7071067812;
	}

	//Connect all anchors together.
	setAnchor2Anchor(startAnchor, startAnchor + 1, springConstant);
	setAnchor2Anchor(startAnchor, startAnchor + 2, springConstant);
	setAnchor2Anchor(startAnchor, startAnchor + 3, springConstant);
	setAnchor2Anchor(startAnchor + 1, startAnchor + 2, springConstant);
	setAnchor2Anchor(startAnchor + 1, startAnchor + 3, springConstant);
	setAnchor2Anchor(startAnchor + 2, startAnchor + 3, springConstant);
}

void UpdateClass::ObjectManager::set6Anchors(UINT objectID, UINT objSubID, XMFLOAT3 objCenter, float distanceToCenter, float springConstant) {
	UINT startAnchor = getObjAnchorStart(objectID) + (objSubID * 6);

	anchors.x[startAnchor] = objCenter.x + distanceToCenter;
	anchors.y[startAnchor] = objCenter.y;
	anchors.z[startAnchor] = objCenter.z;

	anchors.x[startAnchor + 1] = objCenter.x - distanceToCenter;
	anchors.y[startAnchor + 1] = objCenter.y;
	anchors.z[startAnchor + 1] = objCenter.z;

	anchors.x[startAnchor + 2] = objCenter.x;
	anchors.y[startAnchor + 2] = objCenter.y + distanceToCenter;
	anchors.z[startAnchor + 2] = objCenter.z;

	anchors.x[startAnchor + 3] = objCenter.x;
	anchors.y[startAnchor + 3] = objCenter.y - distanceToCenter;
	anchors.z[startAnchor + 3] = objCenter.z;

	anchors.x[startAnchor + 4] = objCenter.x;
	anchors.y[startAnchor + 4] = objCenter.y;
	anchors.z[startAnchor + 4] = objCenter.z + distanceToCenter;

	anchors.x[startAnchor + 5] = objCenter.x;
	anchors.y[startAnchor + 5] = objCenter.y;
	anchors.z[startAnchor + 5] = objCenter.z - distanceToCenter;

	//Insert other anchors distances.
	for (int i = 0; i < 5; i++) {
		for (int j = i + 1; j < 6; j++) {
			setAnchor2Anchor(startAnchor + i, startAnchor + j, springConstant);
		}
	}
}

void UpdateClass::ObjectManager::addAnchor2Segment(UINT anchorID, UINT segmentID, float distance, float springConstant) {
	anchor2SegConnect.push_back(anchorID);
	anch2SegmentConnect.push_back(segmentID);
	anchorToSegmentDistance.push_back(distance);
	anchorToSegmentSpringConst.push_back(springConstant);
	anchors.connectedSegment[anchorID].anch2SegLink.push_back(anchor2SegConnect.size() - 1);
	anchors.numConSeg[anchorID]++;
}