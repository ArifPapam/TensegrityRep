#include "Render_DX12.h"
//#include "d3dApp.h"
#include "PolynomialRegression.h"

/*#define HAVE_STRUCT_TIMESPEC
#include "Common/pthread.h"
#pragma comment(lib, "pthreadVC2.lib")*/

#define StartCounter t_StartCounter_ms
#define GetCounter t_GetCounter_ms

void t_make16mat(float * out, XMMATRIX in) {
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			out[c * 4 + r] = in.r[c].m128_f32[r];
		}
	}
}
void t_make16mat(float4x4 *out, XMMATRIX in) {
	out->m00 = in.r[0].m128_f32[0];
	out->m01 = in.r[0].m128_f32[1];
	out->m02 = in.r[0].m128_f32[2];
	out->m03 = in.r[0].m128_f32[3];

	out->m10 = in.r[1].m128_f32[0];
	out->m11 = in.r[1].m128_f32[1];
	out->m12 = in.r[1].m128_f32[2];
	out->m13 = in.r[1].m128_f32[3];

	out->m20 = in.r[2].m128_f32[0];
	out->m21 = in.r[2].m128_f32[1];
	out->m22 = in.r[2].m128_f32[2];
	out->m23 = in.r[2].m128_f32[3];

	out->m30 = in.r[3].m128_f32[0];
	out->m31 = in.r[3].m128_f32[1];
	out->m32 = in.r[3].m128_f32[2];
	out->m33 = in.r[3].m128_f32[3];
}
XMFLOAT3 t_subtract(XMFLOAT3 a, XMFLOAT3 b) {
	return XMFLOAT3(a.x - b.x, a.y - b.y, a.z - b.z);
}
void t_StartCounter_ms(timer_ms &t) {
	LARGE_INTEGER li;
	if (!QueryPerformanceFrequency(&li))
		MessageBox(0, L"QueryPerformanceFrequency failed!", 0, 0);

	t.PCFreq = double(li.QuadPart) / 1000.0;

	QueryPerformanceCounter(&li);
	t.CounterStart = li.QuadPart;
}
double t_GetCounter_ms(timer_ms &t)
{
	LARGE_INTEGER li;
	QueryPerformanceCounter(&li);
	return double(li.QuadPart - t.CounterStart) / t.PCFreq;
}

extern "C" void * controllerWork(void * arg)
{
	void ** inputs = (void**)arg;
	//void * inputsB[10];
	//for (int f = 0; f < 8; f++) {
	//	inputsB[f] = inputs[f];
	//	std::wstringstream ss;
	//	ss << inputs[f];
	//	std::wstring name = ss.str();
	//	std::wstring text = L" inputs[" + std::to_wstring(f) + L"] " + name + L"\n"; ::OutputDebugString(text.c_str());
	//}
	bool * vrActivate = (bool*)(inputs[0]);
	Camera * mCamera = (Camera*)(inputs[1]);
	bool * end_CurrentTest = (bool*)(inputs[2]);
	vr::IVRSystem* t_pOpenVRSystem = (vr::IVRSystem*)(inputs[3]);
	VIVEController * vc = (VIVEController*)(inputs[4]);
	RenderItem * contRI = (RenderItem*)(inputs[5]);
	RenderItem * lineRI = (RenderItem*)(inputs[6]);
	vr::TrackedDevicePose_t * t_poseOpenVR = (vr::TrackedDevicePose_t*)(inputs[7]);
	vr::IVRCompositor * t_pOpenVRCompositor = (vr::IVRCompositor*)(inputs[8]);

	///vr::TrackedDevicePose_t				t_poseOpenVR[vr::k_unMaxTrackedDeviceCount];

	FILE * controllerPrint;
	int err = fopen_s(&controllerPrint, "controllerPrint.txt", "w");

	int runIteration = 0;
	double saveTime = 0;
	timer_ms cLoopTime;
	cLoopTime.CounterStart = 0;
	cLoopTime.PCFreq = 0;
	double cDeltaTime = 0;
	uint64_t cIterCount = 0;

	bool o0 = !*end_CurrentTest;
	bool o1 = *vrActivate;
	while (!*end_CurrentTest && *vrActivate) { // Need thread safe exit TODO
		StartCounter(cLoopTime);
		///t_pOpenVRSystem->GetDeviceToAbsoluteTrackingPose(vr::ETrackingUniverseOrigin::TrackingUniverseRawAndUncalibrated, 0, t_poseOpenVR, vr::k_unMaxTrackedDeviceCount);
		for (int controllerID = 1; controllerID < 3; controllerID++) {
			vr::VRControllerState_t state;
			if (t_pOpenVRSystem->GetControllerState(controllerID, &state, sizeof(state)))
			{
				if ((state.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Touchpad)) != 0) {
					const float threshold = 0.7f;
					const float dt = cDeltaTime; // Is float sufficiant? TODO
					const float speed = 40.0f;
					if (controllerID == 1) {
						if (state.rAxis[0].y > threshold) {
							vc->VR_MoveOffset.x += mCamera->mLook.x * speed * dt;
							vc->VR_MoveOffset.y += mCamera->mLook.y * speed * dt;
							vc->VR_MoveOffset.z += mCamera->mLook.z * speed * dt;
						}
						else if (state.rAxis[0].y < -threshold) {
							vc->VR_MoveOffset.x -= mCamera->mLook.x * speed * dt;
							vc->VR_MoveOffset.y -= mCamera->mLook.y * speed * dt;
							vc->VR_MoveOffset.z -= mCamera->mLook.z * speed * dt;
						}
						else if (state.rAxis[0].x > threshold) {
							vc->VR_MoveOffset.x += mCamera->mRight.x * speed * dt;
							vc->VR_MoveOffset.y += mCamera->mRight.y * speed * dt;
							vc->VR_MoveOffset.z += mCamera->mRight.z * speed * dt;
						}
						else if (state.rAxis[0].x < -threshold) {
							vc->VR_MoveOffset.x -= mCamera->mRight.x * speed * dt;
							vc->VR_MoveOffset.y -= mCamera->mRight.y * speed * dt;
							vc->VR_MoveOffset.z -= mCamera->mRight.z * speed * dt;
						}
					}
					else if (controllerID == 2) {
						if (state.rAxis[0].y > threshold) {
							vc->grabForOffset += 0.1f;
							if (vc->grabForOffset > 200.0f) {
								vc->grabForOffset = 200.0f;
							}
						}
						else if (state.rAxis[0].y < -threshold) {
							vc->grabForOffset -= 0.1f;
							if (vc->grabForOffset < 5.0f) {
								vc->grabForOffset = 5.0f;
							}
						}
						else if (state.rAxis[0].x > threshold) {
							vc->VR_MoveOffset.x += mCamera->mUp.x * speed * dt;
							vc->VR_MoveOffset.y += mCamera->mUp.y * speed * dt;
							vc->VR_MoveOffset.z -= mCamera->mUp.z * speed * dt;
						}
						else if (state.rAxis[0].x < -threshold) {
							vc->VR_MoveOffset.x -= mCamera->mUp.x * speed * dt;
							vc->VR_MoveOffset.y -= mCamera->mUp.y * speed * dt;
							vc->VR_MoveOffset.z += mCamera->mUp.z * speed * dt;
						}
					}
				}

				if ((state.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_Grip)) != 0) {
					vc->VR_MoveOffset.x = 0.0f - mCamera->mPosition.x; // Moves To 0 location in Sim Space ( it negates where you are in VR space )
					vc->VR_MoveOffset.y = 50.0f - mCamera->mPosition.y;
					vc->VR_MoveOffset.z = 0.0f - mCamera->mPosition.z;
				}

				//------------------Trigger------------------------------------------------------------------------

				//Tracking controller
				//XMFLOAT3 headVRSpace = { m_poseOpenVR[0].mDeviceToAbsoluteTracking.m[0][3], m_poseOpenVR[0].mDeviceToAbsoluteTracking.m[1][3], -m_poseOpenVR[0].mDeviceToAbsoluteTracking.m[2][3] };
				vr::HmdMatrix34_t controllerPos = t_poseOpenVR[controllerID].mDeviceToAbsoluteTracking;
				if (controllerID == 1) {
					fprintf(controllerPrint, "%8.5f\t%8.5f\t%8.5f\t%8.5f\t%8.5f\n", controllerPos.m[0][3], controllerPos.m[1][3], controllerPos.m[2][3], 0.0f, cDeltaTime);
				}

				XMFLOAT3 locationAcc;
				locationAcc.x = controllerPos.m[0][3] * vc->VIVEtoWorldScaling + vc->VR_MoveOffset.x;
				locationAcc.y = controllerPos.m[1][3] * vc->VIVEtoWorldScaling + vc->VR_MoveOffset.y;
				locationAcc.z = controllerPos.m[2][3] * -vc->VIVEtoWorldScaling + vc->VR_MoveOffset.z;

				//Carefull row vs colum major order
				XMMATRIX ViveMatrix = XMMATRIX(
					controllerPos.m[0][0], controllerPos.m[1][0], controllerPos.m[2][0], 0.0f,
					controllerPos.m[0][1], controllerPos.m[1][1], controllerPos.m[2][1], 0.0f,
					controllerPos.m[0][2], controllerPos.m[1][2], controllerPos.m[2][2], 0.0f,
					0.0f, 0.0f, 0.0f, 1.0f);

				XMMATRIX ZfixRotation = XMMATRIX(
					cos(3.1416f), -sin(3.1416f), 0.0f, 0.0f,
					sin(3.1416f), cos(3.1416f), 0.0f, 0.0f,
					0.0f, 0.0f, 1.0f, 0.0f,
					0.0f, 0.0f, 0.0f, 1.0f);

				XMMATRIX controllerMatrix = XMMatrixMultiply(ViveMatrix, ZfixRotation);

				//Saving Data to Globals
				vc->controllerMatrixSave[controllerID - 1] = controllerMatrix;
				t_make16mat(&vc->controllerMats16[controllerID - 1], controllerMatrix);

				DirectX::XMStoreFloat4x4(&contRI->Instances[controllerID - 1].World, controllerMatrix);
				contRI->Instances[controllerID - 1].World._41 = locationAcc.x;
				contRI->Instances[controllerID - 1].World._42 = locationAcc.y;
				contRI->Instances[controllerID - 1].World._43 = locationAcc.z;

				XMMATRIX scaleLine = XMMATRIX(
					1.0f, 0.0f, 0.0f, 0.0f,
					0.0f, vc->grabForOffset, 0.0f, 0.0f,
					0.0f, 0.0f, 1.0f, 0.0f,
					0.0f, 0.0f, 0.0f, 1.0f);

				XMMATRIX XfixRotation = XMMATRIX(
					1.0f, 0.0f, 0.0f, 0.0f,
					0.0f, cos(1.570796f), sin(1.570796f), 0.0f,
					0.0f, -sin(1.570796f), cos(1.570796f), 0.0f,
					0.0f, 0.0f, 0.0f, 1.0f);

				XMMATRIX lineMatrix = XMMatrixMultiply(scaleLine, XMMatrixMultiply(XfixRotation, controllerMatrix));

				DirectX::XMStoreFloat4x4(&lineRI->Instances[controllerID + 7].World, lineMatrix);
				lineRI->Instances[controllerID + 7].World._41 = locationAcc.x;
				lineRI->Instances[controllerID + 7].World._42 = locationAcc.y;
				lineRI->Instances[controllerID + 7].World._43 = locationAcc.z;

				if ((state.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger)) != 0) {
					if (!vc->VRTriggerPressed[controllerID - 1])
						vc->VRTriggerSearching[controllerID - 1] = true;
					vc->VRTriggerPressed[controllerID - 1] = true;
					contRI->Instances[controllerID - 1].Color.x = 0.0f;
					contRI->Instances[controllerID - 1].Color.y = 1.0f * (1 / controllerID) + 0.3f * (controllerID - 1);
					contRI->Instances[controllerID - 1].Color.z = 1.0f * (controllerID - 1);
				}
				else {
					vc->VRTriggerPressed[controllerID - 1] = false;
					vc->VRTriggerSearching[controllerID - 1] = false;
					contRI->Instances[controllerID - 1].Color.x = 0.0f;
					contRI->Instances[controllerID - 1].Color.y = 0.6f * (1 / controllerID);
					contRI->Instances[controllerID - 1].Color.z = 0.6f * (controllerID - 1);
				}

				vc->controllerLook[controllerID - 1] = XMFLOAT3(contRI->Instances[controllerID - 1].World._31, contRI->Instances[controllerID - 1].World._32, contRI->Instances[controllerID - 1].World._33);
				XMFLOAT3 grabLoc = XMFLOAT3(locationAcc.x + vc->controllerLook[controllerID - 1].x*2.0f, locationAcc.y + vc->controllerLook[controllerID - 1].y*2.0f, locationAcc.z + vc->controllerLook[controllerID - 1].z*2.0f);
				vc->grabDeltaLoc[controllerID - 1] = t_subtract(grabLoc, vc->lastGrabLoc[controllerID - 1]);
				vc->lastGrabLoc[controllerID - 1] = grabLoc;

				//Currently used only to print to the screen
				vc->controllerDeltaLoc[controllerID - 1] = t_subtract(locationAcc, vc->lastControllerLoc[controllerID - 1]);
				vc->lastControllerLoc[controllerID - 1] = locationAcc;				
			}
		}

		cIterCount++; if (cIterCount > 5000000) { cIterCount = 1000; } // Resets to an offset to not mess with futureControllerLoc recording
		//Sleep(3);
		cDeltaTime = GetCounter(cLoopTime);
	}

	fclose(controllerPrint);
	return 0;
}