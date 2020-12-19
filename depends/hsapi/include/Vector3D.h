/*
 * @Author: Han Hu
 * @Date: 2020-12-19 12:40:48
 * @Description: 
 */
#pragma once
#include "math.h"

template <typename T>
class Vector3D
{
public:
	Vector3D(void)
	{
		vx = vy = vz = T(0);
	}

	Vector3D( T _vx, T _vy, T _vz )
	{
		vx = _vx;
		vy = _vy;
		vz = _vz;
	}

	Vector3D( const Vector3D& other )
	{
		vx = other.vx;
		vy = other.vy;
		vz = other.vz;
	}

	Vector3D & operator = (const Vector3D& other)
	{
		vx = other.vx;
		vy = other.vy;
		vz = other.vz;
		return (*this);
	}


	~Vector3D(void)
	{
	}

	T mod() const	//返回向量的长度 
	{
		return sqrt(vx*vx + vy*vy + vz*vz);
	}

	T normalize()// 归一化 转换为单位向量 返回值为归一化之前的向量模
	{
		T mod = sqrt(vx*vx+vy*vy+vz*vz);
		if (mod<1e-8)
		{
			return mod;
		}

		vx /= mod;
		vy /= mod;
		vz /= mod;

		return mod;
	}
	
	// 向量点乘
	T dotProduct( const Vector3D& other ) const
	{
		return (other.vx*vx + other.vy*vy + other.vz*vz);
	}

	//向量叉乘  
	Vector3D<T> crossProduct( const Vector3D& v2 ) const
	{
		return Vector3D<T>(vy * v2.vz - v2.vy * vz, 
			vz * v2.vx - v2.vz * vx, vx * v2.vy - v2.vx * vy);
	}

public:
	T vx;
	T vy;
	T vz;
};

