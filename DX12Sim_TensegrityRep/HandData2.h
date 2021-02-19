///////////////////////////////////
// Arif Pramudwiatmoko           //
// Konagaya Laboratory           //
// Tokyo Institute fo Technology //
///////////////////////////////////

/*This file contains HandData class for virtual hand model.
This model uses hand paticles with the different sizes.*/

#pragma once
class HandData {
private:
	//The start segment for bones.
	//startSegBone[finger][bone]
	//finger = 0 for thumb, 1 
	//finger = 0 for thumb, 1 for index finger, 2 for middle finger, 3 for ring finger, 4 for pinky finger.
	//bone   = 0 for metacarpal, 1 for proximal, 2 for intermediate, 3 for distal.
	UINT startSegBone[5][4] = {
		0, 1, 5, 8,				// Start of segment index for thumb finger
		10, 16, 20, 23,			// Start of segment index for index finger
		25, 34, 38, 41,			// Start of segment index for middle finger
		43, 52, 56, 59,			// Start of segment index for ring finger
		61, 70, 74, 77 };		// Start of segment index for pinky finger

									//The end segment for bones.
									//endSegBone[finger][bone]
									//finger = 0 for thumb, 1 for index finger, 2 for middle finger, 3 for ring finger, 4 for pinky finger.
									//bone   = 0 for metacarpal, 1 for proximal, 2 for intermediate, 3 for distal.
	UINT endSegBone[5][4] = {
		0, 4, 7, 9,				// End of segment index for thumb finger
		15, 19, 22, 24,			// End of segment index for index finger
		33, 37, 40, 42,			// End of segment index for middle finger
		51, 55, 58, 60,			// End of segment index for ring finger
		69, 73, 76, 78 };		// End of segment index for pinky finger

									//3D coordinates of segments for hands
									//handSegPos[index][xyz]
									//index = segment index.
									//xyz   = 3D coordinates. 0 for x, 1 for y, 2 for z.
	float handSegPos[79][3] = {
		// Segment locations for thumb-metacarpal
		{ 0.0f, 0.0f, 0.0f },

		// Segment locations for thumb-proximal
		{ 0.00000f,  0.00000f, -0.34000f },{ 0.00000f,  0.00000f, -0.12000f },//
		{ 0.00000f,  0.00000f,  0.12000f },{ 0.00000f,  0.00000f,  0.34000f },

		// Segment locations for thumb-intermediate
		{ 0.00000f,  0.00000f, -0.30000f },{ 0.00000f,  0.00000f, 0.00000f },//
		{ 0.00000f,  0.00000f,  0.30000f },

		// Segment locations for thumb-distal
		{ 0.00000f,  0.00000f, -0.30000f },{ 0.00000f,  0.00000f,  0.30000f },//

		// Segment locations for index-metacarpal
		{ 0.00000f,  0.00000f, -0.40000f },{ 0.00000f,  0.00000f, -0.24000f },//
		{ 0.00000f,  0.00000f, -0.08000f },{ 0.00000f,  0.00000f,  0.08000f },
		{ 0.00000f,  0.00000f,  0.24000f },{ 0.00000f,  0.00000f,  0.40000f },

		// Segment locations for index-proximal
		{ 0.00000f,  0.00000f, -0.34000f },{ 0.00000f,  0.00000f, -0.12000f },//
		{ 0.00000f,  0.00000f,  0.12000f },{ 0.00000f,  0.00000f,  0.34000f },

		// Segment locations for index-intermediate
		{ 0.00000f,  0.00000f, -0.30000f },{ 0.00000f,  0.00000f, 0.00000f },//
		{ 0.00000f,  0.00000f,  0.30000f },

		// Segment locations for index-distal
		{ 0.00000f,  0.00000f, -0.30000f },{ 0.00000f,  0.00000f,  0.30000f },//

		// Segment locations for middle-metacarpal
		{ 0.00000f,  0.00000f, -0.40000f },{ 0.00000f,  0.00000f, -0.24000f },//
		{ 0.00000f,  0.00000f, -0.08000f },{ 0.00000f,  0.00000f,  0.08000f },
		{ 0.00000f,  0.00000f,  0.24000f },{ 0.00000f,  0.00000f,  0.40000f },

		//{ 0.50000f,  0.00000f, 0.00000f },{ 0.50000f,  0.00000f, 0.00000f },
		{ 0.50000f,  0.00000f,  0.00000f },
		{ 0.50000f,  0.00000f,  0.16000f },{ 0.50000f,  0.00000f,  0.32000f },

		// Segment locations for middle-proximal
		{ 0.00000f,  0.00000f, -0.34000f },{ 0.00000f,  0.00000f, -0.12000f },//
		{ 0.00000f,  0.00000f,  0.12000f },{ 0.00000f,  0.00000f,  0.34000f },

		// Segment locations for middle-intermediate
		{ 0.00000f,  0.00000f, -0.30000f },{ 0.00000f,  0.00000f, 0.00000f },//
		{ 0.00000f,  0.00000f,  0.30000f },

		// Segment locations for middle-distal
		{ 0.00000f,  0.00000f, -0.30000f },{ 0.00000f,  0.00000f,  0.30000f },//

		// Segment locations for ring-metacarpal
		{ 0.00000f,  0.00000f, -0.40000f },{ 0.00000f,  0.00000f, -0.24000f },//
		{ 0.00000f,  0.00000f, -0.08000f },{ 0.00000f,  0.00000f,  0.08000f },
		{ 0.00000f,  0.00000f,  0.24000f },{ 0.00000f,  0.00000f,  0.40000f },

		//{ 0.50000f,  0.00000f, 0.00000f },{ 0.50000f,  0.00000f, 0.00000f },
		{ 0.50000f,  0.00000f,  0.00000f },
		{ 0.50000f,  0.00000f,  0.16000f },{ 0.50000f,  0.00000f,  0.32000f },

		// Segment locations for ring-proximal
		{ 0.00000f,  0.00000f, -0.34000f },{ 0.00000f,  0.00000f, -0.12000f },//
		{ 0.00000f,  0.00000f,  0.12000f },{ 0.00000f,  0.00000f,  0.34000f },

		// Segment locations for ring-intermediate
		{ 0.00000f,  0.00000f, -0.30000f },{ 0.00000f,  0.00000f, 0.00000f },//
		{ 0.00000f,  0.00000f,  0.30000f },

		// Segment locations for ring-distal
		{ 0.00000f,  0.00000f, -0.30000f },{ 0.00000f,  0.00000f,  0.30000f },//

		// Segment locations for pinky-metacarpal
		{ 0.00000f,  0.00000f, -0.40000f }, { 0.00000f,  0.00000f, -0.24000f },//
		{ 0.00000f,  0.00000f, -0.08000f }, { 0.00000f,  0.00000f,  0.08000f },
		{ 0.00000f,  0.00000f,  0.24000f }, { 0.00000f,  0.00000f,  0.40000f },

		//{ 0.50000f,  0.00000f, 0.00000f }, { 0.50000f,  0.00000f, 0.00000f },
		{ 0.50000f,  0.00000f,  0.00000f },
		{ 0.50000f,  0.00000f,  0.16000f }, { 0.50000f,  0.00000f,  0.32000f },

		// Segment locations for pinky-proximal
		{ 0.00000f,  0.00000f, -0.34000f }, { 0.00000f,  0.00000f, -0.12000f },//
		{ 0.00000f,  0.00000f,  0.12000f }, { 0.00000f,  0.00000f,  0.34000f },

		// Segment locations for pinky-intermediate
		{ 0.00000f,  0.00000f, -0.30000f },{ 0.00000f,  0.00000f, 0.00000f },//
		{ 0.00000f,  0.00000f,  0.30000f },

		// Segment locations for pinky-distal
		{ 0.00000f,  0.00000f, -0.30000f },{ 0.00000f,  0.00000f,  0.30000f },//
		
	};

	//Radius of hand segments.
	float handSegRad[79] = {
		2.7f, 2.7f, 2.6f, 2.4f, 2.2f, 2.0f, 2.0f, 2.0f, 2.0f, 1.9f,//Thumb

		2.6f, 2.5f, 2.5f, 2.4f, 2.3f, 2.2f,//Index
		2.0f, 1.9f, 1.9f, 1.8f,
		1.7f, 1.7f, 1.7f,
		1.6f, 1.6f,

		2.5f, 2.4f, 2.4f, 2.3f, 2.2f, 2.1f,//Middle
		              2.5f, 2.4f, 2.25f,
		2.0f, 1.9f, 1.9f, 1.8f,
		1.7f, 1.7f, 1.7f,
		1.6f, 1.6f,
		
		2.4f, 2.3f, 2.3f, 2.2f, 2.1f, 2.0f,//Ring
		              2.35f, 2.25f, 2.15f,
		1.9f, 1.8f,	1.8f, 1.7f,
		1.7f, 1.7f, 1.6f,
		1.6f, 1.5f,
		
		2.4f, 2.3f, 2.3f, 2.2f, 2.1f, 2.0f,//Pinky
		              2.25f, 2.15f, 2.05f,
		1.8f, 1.7f, 1.6f, 1.55f,
		1.5f, 1.5f, 1.5f,
		1.5f, 1.4f
	};

	//The start segment for other hands.
	UINT startSegRight = endSegBone[4][3] + 1;

public:
	//To get the start segment for bones.
	//finger = 0 for thumb, 1 for index finger, 2 for middle finger, 3 for ring finger, 4 for pinky finger.
	//bone   = 0 for metacarpal, 1 for proximal, 2 for intermediate, 3 for distal.
	UINT getStartSegBone(UINT finger, UINT bone) {
		return startSegBone[finger][bone];
	}

	//To get the end segment for bones.
	//finger = 0 for thumb, 1 for index finger, 2 for middle finger, 3 for ring finger, 4 for pinky finger.
	//bone   = 0 for metacarpal, 1 for proximal, 2 for intermediate, 3 for distal.
	UINT getEndSegBone(UINT finger, UINT bone) {
		return endSegBone[finger][bone];
	}

	//To get the x coordinate of a hand segment.
	float getHandSegPosX(UINT segIdx) {
		return handSegPos[segIdx][0];
	}

	//To get the y coordinate of a hand segment.
	float getHandSegPosY(UINT segIdx) {
		return handSegPos[segIdx][1];
	}

	//To get the z coordinate of a hand segment.
	float getHandSegPosZ(UINT segIdx) {
		return handSegPos[segIdx][2];
	}

	//To get the radius of a hand segment.
	float getHandSegRad(UINT segIdx) {
		return handSegRad[segIdx];
	}

	//To get the start segment for other hands.
	UINT getStartSegRight() {
		return startSegRight;
	}

	//To get number of segments needed to construct these two hands.
	UINT getNumSeg() {
		return 2 * startSegRight;
	}

	//To accumulate the numbers of segmens collided with haptic points of the hands.
	//accXXX[hand][finger]
	//hand   = 0 for right hand, 1 for left hand.
	//finger = 0 for thumb, 1 for index finger, 2 for middle finger, 3 for ring finger, 4 for pinky finger, 5 for palm hand.
	int accNumSeg[2][6];
	float accAvTemp[2][6];

	//To record the velocity of each haptic points.
	//fingerVelocity[hand][finger]
	//hand     = 0 for right hand, 1 for left hand.
	//finger   = 0 for thumb, 1 for index finger, 2 for middle finger, 3 for ring finger, 4 for pinky finger, 5 for palm hand.
	float fingerVelocity[2][6];
};