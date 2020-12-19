/*
 * @Author: Han Hu
 * @Date: 2020-12-19 12:40:48
 * @Description: 
 */
#pragma once
#include "Point3d.h"

typedef unsigned char byte;
typedef unsigned __int16 ui16;
typedef unsigned __int32 ui32;
typedef unsigned __int64 ui64;
typedef __int16 i16;
typedef __int32 i32;
typedef __int64 i64;
typedef int BOOL;

#define TRUE 1
#define FALSE 0


//点云信息结构体
struct HSPoint
{
	Point3D<double> pt;		// 点坐标
	double lfRange;			// 激光测量距离 （经过延时改正）
	double vAng;			// 垂直转镜旋转角 单位：弧度  // （经过垂直角改正）
	double hAng;			// 水平转台旋转角 单位：弧度
	double xAngle;			// 水平倾角x
	double yAngle;			// 水平倾角y
	int nMirror;			// 棱镜面编号
	int nChannel;			// 通道号
	int nWave;				// 回波序号
	unsigned int nReflection;		// 反射强度
	unsigned char pixelGrey; // 颜色灰度值
	double lfGpsSec;// 天内秒
//	int vcode;       //垂直码盘编码
	HSPoint()
	{
//		vcode =0;
		lfRange = 0.0;
		hAng = 0.0;
		vAng = 0.0;
		xAngle = 0.0;
		yAngle = 0.0;
		nMirror = 0;			
		nChannel = 0;
		nWave = 0;
		nReflection = 0;
		pixelGrey = 0; 
		lfGpsSec = 0.0;
	}
};

struct TIME
{
	ui16 Year;
	ui16 Month;
	ui16 Day;
	ui16 Hour;
	ui16 Minute;
	ui16 Second;
	ui16 Milliseconds;

	TIME()
	{
		Year = 1900;
		Month = 1;
		Day = 1;
		Hour = 0;
		Minute = 0;
		Second = 0;
		Milliseconds = 0;
	}
};