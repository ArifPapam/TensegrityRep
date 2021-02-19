/**
 * Copyright 1993-2013 NVIDIA Corporation.  All rights reserved.
 *
 * Please refer to the NVIDIA end user license agreement (EULA) associated
 * with this source code for terms and conditions that govern your use of
 * this software. Any use, reproduction, disclosure, or distribution of
 * this software and related documentation outside the terms of the EULA
 * is strictly prohibited.
 *
 */

/*
 *  This file implements common mathematical operations on vector types
 *  (float3, float4 etc.) since these are not provided as standard by CUDA.
 *
 *  The syntax is modeled on the Cg standard library.
 *
 *  This is part of the Helper library includes
 *
 *    Thanks to Linh Hah for additions and fixes.
 */

#ifndef HELPER_MATH_H
#define HELPER_MATH_H


typedef unsigned int uint;
typedef unsigned short ushort;

#ifndef EXIT_WAIVED
#define EXIT_WAIVED 2
#endif

#ifndef __CUDACC__

struct float2 {
	float x;
	float y;
	
};
struct float3 {
	float x;
	float y;
	float z;
	
};
struct float4 {
	float x;
	float y;
	float z;
	float w;
	
};

struct int2 {
	int x;
	int y;
};
struct int3 {
	int x;
	int y;
	int z;
};
struct int4 {
	int x;
	int y;
	int z;
	int w;
};

struct uint2 {
	uint x;
	uint y;
};
struct uint3 {
	uint x;
	uint y;
	uint z;
};
struct uint4 {
	uint x;
	uint y;
	uint z;
	uint w;
};

float2 make_float2(float x, float y) {
		float2 n;
		n.x = x;
		n.y = y;
		return n;
	}
float3 make_float3(float x, float y, float z) {
		float3 n;
		n.x = x;
		n.y = y;
		n.z = z;
		return n;
	}
float4 make_float4(float x, float y, float z, float w) {
		float4 n;
		n.x = x;
		n.y = y;
		n.z = z;
		n.w = w;
		return n;
	}

int2 make_int2(int x, int y) {
	int2 n;
	n.x = x;
	n.y = y;
	return n;
}
int3 make_int3(int x, int y, int z) {
	int3 n;
	n.x = x;
	n.y = y;
	n.z = z;
	return n;
}
int4 make_int4(int x, int y, int z, int w) {
	int4 n;
	n.x = x;
	n.y = y;
	n.z = z;
	n.w = w;
	return n;
}

uint2 make_uint2(uint x, uint y) {
	uint2 n;
	n.x = x;
	n.y = y;
	return n;
}
uint3 make_uint3(uint x, uint y, uint z) {
	uint3 n;
	n.x = x;
	n.y = y;
	n.z = z;
	return n;
}
uint4 make_uint4(uint x, uint y, uint z, uint w) {
	uint4 n;
	n.x = x;
	n.y = y;
	n.z = z;
	n.w = w;
	return n;
}

////////////////////////////////////////////////////////////////////////////////
// host implementations of CUDA functions
////////////////////////////////////////////////////////////////////////////////

inline float fminf(float a, float b)
{
    return a < b ? a : b;
}

inline float fmaxf(float a, float b)
{
    return a > b ? a : b;
}

inline int maxV(int a, int b)
{
    return a > b ? a : b;
}

inline int minV(int a, int b)
{
    return a < b ? a : b;
}

inline float rsqrtf(float x)
{
    return 1.0f / sqrtf(x);
}
#endif

////////////////////////////////////////////////////////////////////////////////
// constructors
////////////////////////////////////////////////////////////////////////////////

inline  float2 make_float2(float s)
{
    return make_float2(s, s);
}
inline  float2 make_float2(float3 a)
{
    return make_float2(a.x, a.y);
}
inline  float2 make_float2(int2 a)
{
    return make_float2(float(a.x), float(a.y));
}
inline  float2 make_float2(uint2 a)
{
    return make_float2(float(a.x), float(a.y));
}

inline  int2 make_int2(int s)
{
    return make_int2(s, s);
}
inline  int2 make_int2(int3 a)
{
    return make_int2(a.x, a.y);
}
inline  int2 make_int2(uint2 a)
{
    return make_int2(int(a.x), int(a.y));
}
inline  int2 make_int2(float2 a)
{
    return make_int2(int(a.x), int(a.y));
}

inline  uint2 make_uint2(uint s)
{
    return make_uint2(s, s);
}
inline  uint2 make_uint2(uint3 a)
{
    return make_uint2(a.x, a.y);
}
inline  uint2 make_uint2(int2 a)
{
    return make_uint2(uint(a.x), uint(a.y));
}

inline  float3 make_float3(float s)
{
    return make_float3(s, s, s);
}
inline  float3 make_float3(float2 a)
{
    return make_float3(a.x, a.y, 0.0f);
}
inline  float3 make_float3(float2 a, float s)
{
    return make_float3(a.x, a.y, s);
}
inline  float3 make_float3(float4 a)
{
    return make_float3(a.x, a.y, a.z);
}
inline  float3 make_float3(int3 a)
{
    return make_float3(float(a.x), float(a.y), float(a.z));
}
inline  float3 make_float3(uint3 a)
{
    return make_float3(float(a.x), float(a.y), float(a.z));
}

inline  int3 make_int3(int s)
{
    return make_int3(s, s, s);
}
inline  int3 make_int3(int2 a)
{
    return make_int3(a.x, a.y, 0);
}
inline  int3 make_int3(int2 a, int s)
{
    return make_int3(a.x, a.y, s);
}
inline  int3 make_int3(uint3 a)
{
    return make_int3(int(a.x), int(a.y), int(a.z));
}
inline  int3 make_int3(float3 a)
{
    return make_int3(int(a.x), int(a.y), int(a.z));
}

inline  uint3 make_uint3(uint s)
{
    return make_uint3(s, s, s);
}
inline  uint3 make_uint3(uint2 a)
{
    return make_uint3(a.x, a.y, 0);
}
inline  uint3 make_uint3(uint2 a, uint s)
{
    return make_uint3(a.x, a.y, s);
}
inline  uint3 make_uint3(uint4 a)
{
    return make_uint3(a.x, a.y, a.z);
}
inline  uint3 make_uint3(int3 a)
{
    return make_uint3(uint(a.x), uint(a.y), uint(a.z));
}

inline  float4 make_float4(float s)
{
    return make_float4(s, s, s, s);
}
inline  float4 make_float4(float3 a)
{
    return make_float4(a.x, a.y, a.z, 0.0f);
}
inline  float4 make_float4(float3 a, float w)
{
    return make_float4(a.x, a.y, a.z, w);
}
inline  float4 make_float4(int4 a)
{
    return make_float4(float(a.x), float(a.y), float(a.z), float(a.w));
}
inline  float4 make_float4(uint4 a)
{
    return make_float4(float(a.x), float(a.y), float(a.z), float(a.w));
}

inline  int4 make_int4(int s)
{
    return make_int4(s, s, s, s);
}
inline  int4 make_int4(int3 a)
{
    return make_int4(a.x, a.y, a.z, 0);
}
inline  int4 make_int4(int3 a, int w)
{
    return make_int4(a.x, a.y, a.z, w);
}
inline  int4 make_int4(uint4 a)
{
    return make_int4(int(a.x), int(a.y), int(a.z), int(a.w));
}
inline  int4 make_int4(float4 a)
{
    return make_int4(int(a.x), int(a.y), int(a.z), int(a.w));
}


inline  uint4 make_uint4(uint s)
{
    return make_uint4(s, s, s, s);
}
inline  uint4 make_uint4(uint3 a)
{
    return make_uint4(a.x, a.y, a.z, 0);
}
inline  uint4 make_uint4(uint3 a, uint w)
{
    return make_uint4(a.x, a.y, a.z, w);
}
inline  uint4 make_uint4(int4 a)
{
    return make_uint4(uint(a.x), uint(a.y), uint(a.z), uint(a.w));
}

////////////////////////////////////////////////////////////////////////////////
// negate
////////////////////////////////////////////////////////////////////////////////

inline  float2 operator-(float2 &a)
{
    return make_float2(-a.x, -a.y);
}
inline  int2 operator-(int2 &a)
{
    return make_int2(-a.x, -a.y);
}
inline  float3 operator-(float3 &a)
{
    return make_float3(-a.x, -a.y, -a.z);
}
inline  int3 operator-(int3 &a)
{
    return make_int3(-a.x, -a.y, -a.z);
}
inline  float4 operator-(float4 &a)
{
    return make_float4(-a.x, -a.y, -a.z, -a.w);
}
inline  int4 operator-(int4 &a)
{
    return make_int4(-a.x, -a.y, -a.z, -a.w);
}

////////////////////////////////////////////////////////////////////////////////
// addition
////////////////////////////////////////////////////////////////////////////////

inline  float2 operator+(float2 a, float2 b)
{
    return make_float2(a.x + b.x, a.y + b.y);
}
inline  void operator+=(float2 &a, float2 b)
{
    a.x += b.x;
    a.y += b.y;
}
inline  float2 operator+(float2 a, float b)
{
    return make_float2(a.x + b, a.y + b);
}
inline  float2 operator+(float b, float2 a)
{
    return make_float2(a.x + b, a.y + b);
}
inline  void operator+=(float2 &a, float b)
{
    a.x += b;
    a.y += b;
}

inline  int2 operator+(int2 a, int2 b)
{
    return make_int2(a.x + b.x, a.y + b.y);
}
inline  void operator+=(int2 &a, int2 b)
{
    a.x += b.x;
    a.y += b.y;
}
inline  int2 operator+(int2 a, int b)
{
    return make_int2(a.x + b, a.y + b);
}
inline  int2 operator+(int b, int2 a)
{
    return make_int2(a.x + b, a.y + b);
}
inline  void operator+=(int2 &a, int b)
{
    a.x += b;
    a.y += b;
}

inline  uint2 operator+(uint2 a, uint2 b)
{
    return make_uint2(a.x + b.x, a.y + b.y);
}
inline  void operator+=(uint2 &a, uint2 b)
{
    a.x += b.x;
    a.y += b.y;
}
inline  uint2 operator+(uint2 a, uint b)
{
    return make_uint2(a.x + b, a.y + b);
}
inline  uint2 operator+(uint b, uint2 a)
{
    return make_uint2(a.x + b, a.y + b);
}
inline  void operator+=(uint2 &a, uint b)
{
    a.x += b;
    a.y += b;
}


inline  float3 operator+(float3 a, float3 b)
{
    return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}
inline  void operator+=(float3 &a, float3 b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
}
inline  float3 operator+(float3 a, float b)
{
    return make_float3(a.x + b, a.y + b, a.z + b);
}
inline  void operator+=(float3 &a, float b)
{
    a.x += b;
    a.y += b;
    a.z += b;
}

inline  int3 operator+(int3 a, int3 b)
{
    return make_int3(a.x + b.x, a.y + b.y, a.z + b.z);
}
inline  void operator+=(int3 &a, int3 b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
}
inline  int3 operator+(int3 a, int b)
{
    return make_int3(a.x + b, a.y + b, a.z + b);
}
inline  void operator+=(int3 &a, int b)
{
    a.x += b;
    a.y += b;
    a.z += b;
}

inline  uint3 operator+(uint3 a, uint3 b)
{
    return make_uint3(a.x + b.x, a.y + b.y, a.z + b.z);
}
inline  void operator+=(uint3 &a, uint3 b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
}
inline  uint3 operator+(uint3 a, uint b)
{
    return make_uint3(a.x + b, a.y + b, a.z + b);
}
inline  void operator+=(uint3 &a, uint b)
{
    a.x += b;
    a.y += b;
    a.z += b;
}

inline  int3 operator+(int b, int3 a)
{
    return make_int3(a.x + b, a.y + b, a.z + b);
}
inline  uint3 operator+(uint b, uint3 a)
{
    return make_uint3(a.x + b, a.y + b, a.z + b);
}
inline  float3 operator+(float b, float3 a)
{
    return make_float3(a.x + b, a.y + b, a.z + b);
}

inline  float4 operator+(float4 a, float4 b)
{
    return make_float4(a.x + b.x, a.y + b.y, a.z + b.z,  a.w + b.w);
}
inline  void operator+=(float4 &a, float4 b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    a.w += b.w;
}
inline  float4 operator+(float4 a, float b)
{
    return make_float4(a.x + b, a.y + b, a.z + b, a.w + b);
}
inline  float4 operator+(float b, float4 a)
{
    return make_float4(a.x + b, a.y + b, a.z + b, a.w + b);
}
inline  void operator+=(float4 &a, float b)
{
    a.x += b;
    a.y += b;
    a.z += b;
    a.w += b;
}

inline  int4 operator+(int4 a, int4 b)
{
    return make_int4(a.x + b.x, a.y + b.y, a.z + b.z,  a.w + b.w);
}
inline  void operator+=(int4 &a, int4 b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    a.w += b.w;
}
inline  int4 operator+(int4 a, int b)
{
    return make_int4(a.x + b, a.y + b, a.z + b,  a.w + b);
}
inline  int4 operator+(int b, int4 a)
{
    return make_int4(a.x + b, a.y + b, a.z + b,  a.w + b);
}
inline  void operator+=(int4 &a, int b)
{
    a.x += b;
    a.y += b;
    a.z += b;
    a.w += b;
}

inline  uint4 operator+(uint4 a, uint4 b)
{
    return make_uint4(a.x + b.x, a.y + b.y, a.z + b.z,  a.w + b.w);
}
inline  void operator+=(uint4 &a, uint4 b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    a.w += b.w;
}
inline  uint4 operator+(uint4 a, uint b)
{
    return make_uint4(a.x + b, a.y + b, a.z + b,  a.w + b);
}
inline  uint4 operator+(uint b, uint4 a)
{
    return make_uint4(a.x + b, a.y + b, a.z + b,  a.w + b);
}
inline  void operator+=(uint4 &a, uint b)
{
    a.x += b;
    a.y += b;
    a.z += b;
    a.w += b;
}

////////////////////////////////////////////////////////////////////////////////
// subtract
////////////////////////////////////////////////////////////////////////////////

inline  float2 operator-(float2 a, float2 b)
{
    return make_float2(a.x - b.x, a.y - b.y);
}
inline  void operator-=(float2 &a, float2 b)
{
    a.x -= b.x;
    a.y -= b.y;
}
inline  float2 operator-(float2 a, float b)
{
    return make_float2(a.x - b, a.y - b);
}
inline  float2 operator-(float b, float2 a)
{
    return make_float2(b - a.x, b - a.y);
}
inline  void operator-=(float2 &a, float b)
{
    a.x -= b;
    a.y -= b;
}

inline  int2 operator-(int2 a, int2 b)
{
    return make_int2(a.x - b.x, a.y - b.y);
}
inline  void operator-=(int2 &a, int2 b)
{
    a.x -= b.x;
    a.y -= b.y;
}
inline  int2 operator-(int2 a, int b)
{
    return make_int2(a.x - b, a.y - b);
}
inline  int2 operator-(int b, int2 a)
{
    return make_int2(b - a.x, b - a.y);
}
inline  void operator-=(int2 &a, int b)
{
    a.x -= b;
    a.y -= b;
}

inline  uint2 operator-(uint2 a, uint2 b)
{
    return make_uint2(a.x - b.x, a.y - b.y);
}
inline  void operator-=(uint2 &a, uint2 b)
{
    a.x -= b.x;
    a.y -= b.y;
}
inline  uint2 operator-(uint2 a, uint b)
{
    return make_uint2(a.x - b, a.y - b);
}
inline  uint2 operator-(uint b, uint2 a)
{
    return make_uint2(b - a.x, b - a.y);
}
inline  void operator-=(uint2 &a, uint b)
{
    a.x -= b;
    a.y -= b;
}

inline  float3 operator-(float3 a, float3 b)
{
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline  void operator-=(float3 &a, float3 b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
}
inline  float3 operator-(float3 a, float b)
{
    return make_float3(a.x - b, a.y - b, a.z - b);
}
inline  float3 operator-(float b, float3 a)
{
    return make_float3(b - a.x, b - a.y, b - a.z);
}
inline  void operator-=(float3 &a, float b)
{
    a.x -= b;
    a.y -= b;
    a.z -= b;
}

inline  int3 operator-(int3 a, int3 b)
{
    return make_int3(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline  void operator-=(int3 &a, int3 b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
}
inline  int3 operator-(int3 a, int b)
{
    return make_int3(a.x - b, a.y - b, a.z - b);
}
inline  int3 operator-(int b, int3 a)
{
    return make_int3(b - a.x, b - a.y, b - a.z);
}
inline  void operator-=(int3 &a, int b)
{
    a.x -= b;
    a.y -= b;
    a.z -= b;
}

inline  uint3 operator-(uint3 a, uint3 b)
{
    return make_uint3(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline  void operator-=(uint3 &a, uint3 b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
}
inline  uint3 operator-(uint3 a, uint b)
{
    return make_uint3(a.x - b, a.y - b, a.z - b);
}
inline  uint3 operator-(uint b, uint3 a)
{
    return make_uint3(b - a.x, b - a.y, b - a.z);
}
inline  void operator-=(uint3 &a, uint b)
{
    a.x -= b;
    a.y -= b;
    a.z -= b;
}

inline  float4 operator-(float4 a, float4 b)
{
    return make_float4(a.x - b.x, a.y - b.y, a.z - b.z,  a.w - b.w);
}
inline  void operator-=(float4 &a, float4 b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    a.w -= b.w;
}
inline  float4 operator-(float4 a, float b)
{
    return make_float4(a.x - b, a.y - b, a.z - b,  a.w - b);
}
inline  void operator-=(float4 &a, float b)
{
    a.x -= b;
    a.y -= b;
    a.z -= b;
    a.w -= b;
}

inline  int4 operator-(int4 a, int4 b)
{
    return make_int4(a.x - b.x, a.y - b.y, a.z - b.z,  a.w - b.w);
}
inline  void operator-=(int4 &a, int4 b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    a.w -= b.w;
}
inline  int4 operator-(int4 a, int b)
{
    return make_int4(a.x - b, a.y - b, a.z - b,  a.w - b);
}
inline  int4 operator-(int b, int4 a)
{
    return make_int4(b - a.x, b - a.y, b - a.z, b - a.w);
}
inline  void operator-=(int4 &a, int b)
{
    a.x -= b;
    a.y -= b;
    a.z -= b;
    a.w -= b;
}

inline  uint4 operator-(uint4 a, uint4 b)
{
    return make_uint4(a.x - b.x, a.y - b.y, a.z - b.z,  a.w - b.w);
}
inline  void operator-=(uint4 &a, uint4 b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    a.w -= b.w;
}
inline  uint4 operator-(uint4 a, uint b)
{
    return make_uint4(a.x - b, a.y - b, a.z - b,  a.w - b);
}
inline  uint4 operator-(uint b, uint4 a)
{
    return make_uint4(b - a.x, b - a.y, b - a.z, b - a.w);
}
inline  void operator-=(uint4 &a, uint b)
{
    a.x -= b;
    a.y -= b;
    a.z -= b;
    a.w -= b;
}

////////////////////////////////////////////////////////////////////////////////
// multiply
////////////////////////////////////////////////////////////////////////////////

inline  float2 operator*(float2 a, float2 b)
{
    return make_float2(a.x * b.x, a.y * b.y);
}
inline  void operator*=(float2 &a, float2 b)
{
    a.x *= b.x;
    a.y *= b.y;
}
inline  float2 operator*(float2 a, float b)
{
    return make_float2(a.x * b, a.y * b);
}
inline  float2 operator*(float b, float2 a)
{
    return make_float2(b * a.x, b * a.y);
}
inline  void operator*=(float2 &a, float b)
{
    a.x *= b;
    a.y *= b;
}

inline  int2 operator*(int2 a, int2 b)
{
    return make_int2(a.x * b.x, a.y * b.y);
}
inline  void operator*=(int2 &a, int2 b)
{
    a.x *= b.x;
    a.y *= b.y;
}
inline  int2 operator*(int2 a, int b)
{
    return make_int2(a.x * b, a.y * b);
}
inline  int2 operator*(int b, int2 a)
{
    return make_int2(b * a.x, b * a.y);
}
inline  void operator*=(int2 &a, int b)
{
    a.x *= b;
    a.y *= b;
}

inline  uint2 operator*(uint2 a, uint2 b)
{
    return make_uint2(a.x * b.x, a.y * b.y);
}
inline  void operator*=(uint2 &a, uint2 b)
{
    a.x *= b.x;
    a.y *= b.y;
}
inline  uint2 operator*(uint2 a, uint b)
{
    return make_uint2(a.x * b, a.y * b);
}
inline  uint2 operator*(uint b, uint2 a)
{
    return make_uint2(b * a.x, b * a.y);
}
inline  void operator*=(uint2 &a, uint b)
{
    a.x *= b;
    a.y *= b;
}

inline  float3 operator*(float3 a, float3 b)
{
    return make_float3(a.x * b.x, a.y * b.y, a.z * b.z);
}
inline  void operator*=(float3 &a, float3 b)
{
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
}
inline  float3 operator*(float3 a, float b)
{
    return make_float3(a.x * b, a.y * b, a.z * b);
}
inline  float3 operator*(float b, float3 a)
{
    return make_float3(b * a.x, b * a.y, b * a.z);
}
inline  void operator*=(float3 &a, float b)
{
    a.x *= b;
    a.y *= b;
    a.z *= b;
}

inline  int3 operator*(int3 a, int3 b)
{
    return make_int3(a.x * b.x, a.y * b.y, a.z * b.z);
}
inline  void operator*=(int3 &a, int3 b)
{
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
}
inline  int3 operator*(int3 a, int b)
{
    return make_int3(a.x * b, a.y * b, a.z * b);
}
inline  int3 operator*(int b, int3 a)
{
    return make_int3(b * a.x, b * a.y, b * a.z);
}
inline  void operator*=(int3 &a, int b)
{
    a.x *= b;
    a.y *= b;
    a.z *= b;
}

inline  uint3 operator*(uint3 a, uint3 b)
{
    return make_uint3(a.x * b.x, a.y * b.y, a.z * b.z);
}
inline  void operator*=(uint3 &a, uint3 b)
{
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
}
inline  uint3 operator*(uint3 a, uint b)
{
    return make_uint3(a.x * b, a.y * b, a.z * b);
}
inline  uint3 operator*(uint b, uint3 a)
{
    return make_uint3(b * a.x, b * a.y, b * a.z);
}
inline  void operator*=(uint3 &a, uint b)
{
    a.x *= b;
    a.y *= b;
    a.z *= b;
}

inline  float4 operator*(float4 a, float4 b)
{
    return make_float4(a.x * b.x, a.y * b.y, a.z * b.z,  a.w * b.w);
}
inline  void operator*=(float4 &a, float4 b)
{
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
    a.w *= b.w;
}
inline  float4 operator*(float4 a, float b)
{
    return make_float4(a.x * b, a.y * b, a.z * b,  a.w * b);
}
inline  float4 operator*(float b, float4 a)
{
    return make_float4(b * a.x, b * a.y, b * a.z, b * a.w);
}
inline  void operator*=(float4 &a, float b)
{
    a.x *= b;
    a.y *= b;
    a.z *= b;
    a.w *= b;
}

inline  int4 operator*(int4 a, int4 b)
{
    return make_int4(a.x * b.x, a.y * b.y, a.z * b.z,  a.w * b.w);
}
inline  void operator*=(int4 &a, int4 b)
{
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
    a.w *= b.w;
}
inline  int4 operator*(int4 a, int b)
{
    return make_int4(a.x * b, a.y * b, a.z * b,  a.w * b);
}
inline  int4 operator*(int b, int4 a)
{
    return make_int4(b * a.x, b * a.y, b * a.z, b * a.w);
}
inline  void operator*=(int4 &a, int b)
{
    a.x *= b;
    a.y *= b;
    a.z *= b;
    a.w *= b;
}

inline  uint4 operator*(uint4 a, uint4 b)
{
    return make_uint4(a.x * b.x, a.y * b.y, a.z * b.z,  a.w * b.w);
}
inline  void operator*=(uint4 &a, uint4 b)
{
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
    a.w *= b.w;
}
inline  uint4 operator*(uint4 a, uint b)
{
    return make_uint4(a.x * b, a.y * b, a.z * b,  a.w * b);
}
inline  uint4 operator*(uint b, uint4 a)
{
    return make_uint4(b * a.x, b * a.y, b * a.z, b * a.w);
}
inline  void operator*=(uint4 &a, uint b)
{
    a.x *= b;
    a.y *= b;
    a.z *= b;
    a.w *= b;
}

////////////////////////////////////////////////////////////////////////////////
// divide
////////////////////////////////////////////////////////////////////////////////

inline  float2 operator/(float2 a, float2 b)
{
    return make_float2(a.x / b.x, a.y / b.y);
}
inline  void operator/=(float2 &a, float2 b)
{
    a.x /= b.x;
    a.y /= b.y;
}
inline  float2 operator/(float2 a, float b)
{
    return make_float2(a.x / b, a.y / b);
}
inline  void operator/=(float2 &a, float b)
{
    a.x /= b;
    a.y /= b;
}
inline  float2 operator/(float b, float2 a)
{
    return make_float2(b / a.x, b / a.y);
}

inline  float3 operator/(float3 a, float3 b)
{
    return make_float3(a.x / b.x, a.y / b.y, a.z / b.z);
}
inline  void operator/=(float3 &a, float3 b)
{
    a.x /= b.x;
    a.y /= b.y;
    a.z /= b.z;
}
inline  float3 operator/(float3 a, float b)
{
    return make_float3(a.x / b, a.y / b, a.z / b);
}
inline  void operator/=(float3 &a, float b)
{
    a.x /= b;
    a.y /= b;
    a.z /= b;
}
inline  float3 operator/(float b, float3 a)
{
    return make_float3(b / a.x, b / a.y, b / a.z);
}

inline  float4 operator/(float4 a, float4 b)
{
    return make_float4(a.x / b.x, a.y / b.y, a.z / b.z,  a.w / b.w);
}
inline  void operator/=(float4 &a, float4 b)
{
    a.x /= b.x;
    a.y /= b.y;
    a.z /= b.z;
    a.w /= b.w;
}
inline  float4 operator/(float4 a, float b)
{
    return make_float4(a.x / b, a.y / b, a.z / b,  a.w / b);
}
inline  void operator/=(float4 &a, float b)
{
    a.x /= b;
    a.y /= b;
    a.z /= b;
    a.w /= b;
}
inline  float4 operator/(float b, float4 a)
{
    return make_float4(b / a.x, b / a.y, b / a.z, b / a.w);
}

////////////////////////////////////////////////////////////////////////////////
// min
////////////////////////////////////////////////////////////////////////////////

inline   float2 fminf(float2 a, float2 b)
{
    return make_float2(fminf(a.x,b.x), fminf(a.y,b.y));
}
inline  float3 fminf(float3 a, float3 b)
{
    return make_float3(fminf(a.x,b.x), fminf(a.y,b.y), fminf(a.z,b.z));
}
inline   float4 fminf(float4 a, float4 b)
{
    return make_float4(fminf(a.x,b.x), fminf(a.y,b.y), fminf(a.z,b.z), fminf(a.w,b.w));
}

int2 minV(int2 a, int2 b)
{
    return make_int2(minV(a.x,b.x), minV(a.y,b.y));
}
inline  int3 minV(int3 a, int3 b)
{
    return make_int3(minV(a.x,b.x), minV(a.y,b.y), minV(a.z,b.z));
}
inline  int4 minV(int4 a, int4 b)
{
    return make_int4(minV(a.x,b.x), minV(a.y,b.y), minV(a.z,b.z), minV(a.w,b.w));
}

inline  uint2 minV(uint2 a, uint2 b)
{
    return make_uint2(minV(a.x,b.x), minV(a.y,b.y));
}
inline  uint3 minV(uint3 a, uint3 b)
{
    return make_uint3(minV(a.x,b.x), minV(a.y,b.y), minV(a.z,b.z));
}
inline  uint4 minV(uint4 a, uint4 b)
{
    return make_uint4(minV(a.x,b.x), minV(a.y,b.y), minV(a.z,b.z), minV(a.w,b.w));
}

////////////////////////////////////////////////////////////////////////////////
// max
////////////////////////////////////////////////////////////////////////////////

inline  float2 fmaxf(float2 a, float2 b)
{
    return make_float2(fmaxf(a.x,b.x), fmaxf(a.y,b.y));
}
inline  float3 fmaxf(float3 a, float3 b)
{
    return make_float3(fmaxf(a.x,b.x), fmaxf(a.y,b.y), fmaxf(a.z,b.z));
}
inline  float4 fmaxf(float4 a, float4 b)
{
    return make_float4(fmaxf(a.x,b.x), fmaxf(a.y,b.y), fmaxf(a.z,b.z), fmaxf(a.w,b.w));
}

inline  int2 maxV(int2 a, int2 b)
{
    return make_int2(maxV(a.x,b.x), maxV(a.y,b.y));
}
inline  int3 maxV(int3 a, int3 b)
{
    return make_int3(maxV(a.x,b.x), maxV(a.y,b.y), maxV(a.z,b.z));
}
inline  int4 maxV(int4 a, int4 b)
{
    return make_int4(maxV(a.x,b.x), maxV(a.y,b.y), maxV(a.z,b.z), maxV(a.w,b.w));
}

inline  uint2 maxV(uint2 a, uint2 b)
{
    return make_uint2(maxV(a.x,b.x), maxV(a.y,b.y));
}
inline  uint3 maxV(uint3 a, uint3 b)
{
    return make_uint3(maxV(a.x,b.x), maxV(a.y,b.y), maxV(a.z,b.z));
}
inline  uint4 maxV(uint4 a, uint4 b)
{
    return make_uint4(maxV(a.x,b.x), maxV(a.y,b.y), maxV(a.z,b.z), maxV(a.w,b.w));
}

////////////////////////////////////////////////////////////////////////////////
// lerp
// - linear interpolation between a and b, based on value t in [0, 1] range
////////////////////////////////////////////////////////////////////////////////

inline float lerp(float a, float b, float t)
{
    return a + t*(b-a);
}
inline float2 lerp(float2 a, float2 b, float t)
{
    return a + t*(b-a);
}
inline float3 lerp(float3 a, float3 b, float t)
{
    return a + t*(b-a);
}
inline float4 lerp(float4 a, float4 b, float t)
{
    return a + t*(b-a);
}

////////////////////////////////////////////////////////////////////////////////
// clamp
// - clamp the value v to be in the range [a, b]
////////////////////////////////////////////////////////////////////////////////

inline float clamp(float f, float a, float b)
{
    return fmaxf(a, fminf(f, b));
}
inline int clamp(int f, int a, int b)
{
    return maxV(a, minV(f, b));
}
inline uint clamp(uint f, uint a, uint b)
{
    return maxV(a, minV(f, b));
}

inline float2 clamp(float2 v, float a, float b)
{
    return make_float2(clamp(v.x, a, b), clamp(v.y, a, b));
}
inline float2 clamp(float2 v, float2 a, float2 b)
{
    return make_float2(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y));
}
inline float3 clamp(float3 v, float a, float b)
{
    return make_float3(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b));
}
inline float3 clamp(float3 v, float3 a, float3 b)
{
    return make_float3(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z));
}
inline float4 clamp(float4 v, float a, float b)
{
    return make_float4(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b), clamp(v.w, a, b));
}
inline float4 clamp(float4 v, float4 a, float4 b)
{
    return make_float4(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z), clamp(v.w, a.w, b.w));
}

inline int2 clamp(int2 v, int a, int b)
{
    return make_int2(clamp(v.x, a, b), clamp(v.y, a, b));
}
inline int2 clamp(int2 v, int2 a, int2 b)
{
    return make_int2(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y));
}
inline int3 clamp(int3 v, int a, int b)
{
    return make_int3(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b));
}
inline int3 clamp(int3 v, int3 a, int3 b)
{
    return make_int3(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z));
}
inline int4 clamp(int4 v, int a, int b)
{
    return make_int4(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b), clamp(v.w, a, b));
}
inline int4 clamp(int4 v, int4 a, int4 b)
{
    return make_int4(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z), clamp(v.w, a.w, b.w));
}

inline uint2 clamp(uint2 v, uint a, uint b)
{
    return make_uint2(clamp(v.x, a, b), clamp(v.y, a, b));
}
inline uint2 clamp(uint2 v, uint2 a, uint2 b)
{
    return make_uint2(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y));
}
inline uint3 clamp(uint3 v, uint a, uint b)
{
    return make_uint3(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b));
}
inline uint3 clamp(uint3 v, uint3 a, uint3 b)
{
    return make_uint3(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z));
}
inline uint4 clamp(uint4 v, uint a, uint b)
{
    return make_uint4(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b), clamp(v.w, a, b));
}
inline uint4 clamp(uint4 v, uint4 a, uint4 b)
{
    return make_uint4(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z), clamp(v.w, a.w, b.w));
}

////////////////////////////////////////////////////////////////////////////////
// dot product
////////////////////////////////////////////////////////////////////////////////

inline  float dot(float2 a, float2 b)
{
    return a.x * b.x + a.y * b.y;
}
inline  float dot(float3 a, float3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline  float dot(float4 a, float4 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline  int dot(int2 a, int2 b)
{
    return a.x * b.x + a.y * b.y;
}
inline  int dot(int3 a, int3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline  int dot(int4 a, int4 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline  uint dot(uint2 a, uint2 b)
{
    return a.x * b.x + a.y * b.y;
}
inline  uint dot(uint3 a, uint3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline  uint dot(uint4 a, uint4 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

////////////////////////////////////////////////////////////////////////////////
// length
////////////////////////////////////////////////////////////////////////////////

inline  float length(float2 v)
{
    return sqrtf(dot(v, v));
}
inline  float length(float3 v)
{
    return sqrtf(dot(v, v));
}
inline  float length(float4 v)
{
    return sqrtf(dot(v, v));
}

////////////////////////////////////////////////////////////////////////////////
// normalize
////////////////////////////////////////////////////////////////////////////////

inline  float2 normalize(float2 v)
{
    float invLen = rsqrtf(dot(v, v));
    return v * invLen;
}
inline  float3 normalize(float3 v)
{
    float invLen = rsqrtf(dot(v, v));
    return v * invLen;
}
inline  float4 normalize(float4 v)
{
    float invLen = rsqrtf(dot(v, v));
    return v * invLen;
}

////////////////////////////////////////////////////////////////////////////////
// floor
////////////////////////////////////////////////////////////////////////////////

inline  float2 floorf(float2 v)
{
    return make_float2(floorf(v.x), floorf(v.y));
}
inline  float3 floorf(float3 v)
{
    return make_float3(floorf(v.x), floorf(v.y), floorf(v.z));
}
inline  float4 floorf(float4 v)
{
    return make_float4(floorf(v.x), floorf(v.y), floorf(v.z), floorf(v.w));
}

////////////////////////////////////////////////////////////////////////////////
// frac - returns the fractional portion of a scalar or each vector component
////////////////////////////////////////////////////////////////////////////////

inline  float fracf(float v)
{
    return v - floorf(v);
}
inline  float2 fracf(float2 v)
{
    return make_float2(fracf(v.x), fracf(v.y));
}
inline  float3 fracf(float3 v)
{
    return make_float3(fracf(v.x), fracf(v.y), fracf(v.z));
}
inline  float4 fracf(float4 v)
{
    return make_float4(fracf(v.x), fracf(v.y), fracf(v.z), fracf(v.w));
}

////////////////////////////////////////////////////////////////////////////////
// fmod
////////////////////////////////////////////////////////////////////////////////

inline  float2 fmodf(float2 a, float2 b)
{
    return make_float2(fmodf(a.x, b.x), fmodf(a.y, b.y));
}
inline  float3 fmodf(float3 a, float3 b)
{
    return make_float3(fmodf(a.x, b.x), fmodf(a.y, b.y), fmodf(a.z, b.z));
}
inline  float4 fmodf(float4 a, float4 b)
{
    return make_float4(fmodf(a.x, b.x), fmodf(a.y, b.y), fmodf(a.z, b.z), fmodf(a.w, b.w));
}

////////////////////////////////////////////////////////////////////////////////
// absolute value
////////////////////////////////////////////////////////////////////////////////

inline  float2 fabs(float2 v)
{
    return make_float2(fabs(v.x), fabs(v.y));
}
inline  float3 fabs(float3 v)
{
    return make_float3(fabs(v.x), fabs(v.y), fabs(v.z));
}
inline  float4 fabs(float4 v)
{
    return make_float4(fabs(v.x), fabs(v.y), fabs(v.z), fabs(v.w));
}

inline  int2 abs(int2 v)
{
    return make_int2(abs(v.x), abs(v.y));
}
inline  int3 abs(int3 v)
{
    return make_int3(abs(v.x), abs(v.y), abs(v.z));
}
inline  int4 abs(int4 v)
{
    return make_int4(abs(v.x), abs(v.y), abs(v.z), abs(v.w));
}

////////////////////////////////////////////////////////////////////////////////
// reflect
// - returns reflection of incident ray I around surface normal N
// - N should be normalized, reflected vector's length is equal to length of I
////////////////////////////////////////////////////////////////////////////////

inline  float3 reflect(float3 i, float3 n)
{
    return i - 2.0f * n * dot(n,i);
}

////////////////////////////////////////////////////////////////////////////////
// cross product
////////////////////////////////////////////////////////////////////////////////

inline  float3 cross(float3 a, float3 b)
{
    return make_float3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

////////////////////////////////////////////////////////////////////////////////
// smoothstep
// - returns 0 if x < a
// - returns 1 if x > b
// - otherwise returns smooth interpolation between 0 and 1 based on x
////////////////////////////////////////////////////////////////////////////////

inline float smoothstep(float a, float b, float x)
{
    float y = clamp((x - a) / (b - a), 0.0f, 1.0f);
    return (y*y*(3.0f - (2.0f*y)));
}
inline float2 smoothstep(float2 a, float2 b, float2 x)
{
    float2 y = clamp((x - a) / (b - a), 0.0f, 1.0f);
    return (y*y*(make_float2(3.0f) - (make_float2(2.0f)*y)));
}
inline float3 smoothstep(float3 a, float3 b, float3 x)
{
    float3 y = clamp((x - a) / (b - a), 0.0f, 1.0f);
    return (y*y*(make_float3(3.0f) - (make_float3(2.0f)*y)));
}
inline float4 smoothstep(float4 a, float4 b, float4 x)
{
    float4 y = clamp((x - a) / (b - a), 0.0f, 1.0f);
    return (y*y*(make_float4(3.0f) - (make_float4(2.0f)*y)));
}

#endif
