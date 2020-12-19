/*
 * @Author: Han Hu
 * @Date: 2020-12-19 12:40:48
 * @Description: 
 */
#pragma once
#include "Vector3D.h"

template <typename T>
class Point3D
{
public:
	Point3D(void)
	{
		x = y = z = T(0);
	}

	Point3D( T _x, T _y, T _z )
	{
		x = _x;
		y = _y;
		z = _z;
	}

	Point3D( const Point3D& other )
	{
		x = other.x;
		y = other.y;
		z = other.z;
	}

	Point3D & operator = (const Point3D& other)
	{
		x = other.x;
		y = other.y;
		z = other.z;
		return (*this);
	}

	~Point3D(void){};

public:
	// 求两点之间的距离
	T Distance2Point(const Point3D& otherPoint) const
	{
		return sqrt( (this->x - otherPoint.x)*(this->x - otherPoint.x) + (this->y - otherPoint.y)*(this->y - otherPoint.y) + (this->z - otherPoint.z)*(this->z - otherPoint.z) );
	}

	// 求两点构成的向量 以当前点为起点
	Vector3D<T> GetVector( const Point3D& otherPoint ) const
	{
		//求空间向量
		Vector3D<T> retVal = Vector3D<T>(otherPoint.x - x, otherPoint.y - y, otherPoint.z - z);

		//标准化
		retVal.normalize();

		return retVal;
	}

	void GetVector( const Point3D& otherPoint, Vector3D<T>&  retVal) const
	{
		//求空间向量
		retVal = Vector3D<T>(otherPoint.x - x, otherPoint.y - y, otherPoint.z - z);

		//标准化
		retVal.normalize();
		
	}

	// 求当前点指向另一点构成的向量(保留长度信息) 
	Vector3D<T> VectorTo( const Point3D& otherPoint ) const
	{
		return Vector3D<T>(otherPoint.x - x, otherPoint.y - y, otherPoint.z - z);
	}

public:
	T x;
	T y;
	T z;
};

