/*
 * @Author: Han Hu
 * @Date: 2020-12-19 12:40:48
 * @Description: 
 */
#pragma once
#ifdef HSRAW_EXPORTS
#define HSRAW_API __declspec(dllexport)
#else
#define HSRAW_API __declspec(dllimport)
#endif
#include <string>
#include <vector>
using namespace std;
#include "UserStruct.h"

// 回调函数定义
typedef void (*LPHsrCallback)(float, const char*);

class CHsRawCore;

class HSRAW_API HsRawiLSP 
{
public:
	HsRawiLSP(void);
	~HsRawiLSP();

	//打开hsr文件
	bool openFile(string strHsrFile);//hsr文件路径

	// 从一段内存读文件头
	void readFileHeader(const byte* pHeaderBuffer);//文件头二进制流

	// 关闭文件
	void closeFile();

	// 得到文件大小
	i64 getFileSize();

	// 得到文件时间
	void getFileTime(TIME& t);

	// 得到转速
	int getRotationalSpeed();

	// 得到激光频率
	int getLaserFreqkHz();

	// 得到一桢数据的字节数
	int getFrameSize();

	// 读一圈数据 正常返回值为0 读到文件末尾为-1
	int getRingData( vector<vector<HSPoint>>& retVecHsPoints, // 返回一圈点云数据
		LPHsrCallback callbackFun = NULL);		              // 回调函数

	// 获取一帧线扫点云
	void getFrameData( const byte* pFrameBuffer,     // 输入一帧原始数据
		vector<HSPoint>& retVecHsPoints                   // 返回一帧点云坐标
		); 

	//设置是否进行过滤
	void setFilter(bool bDoFilter);

private:
	CHsRawCore* m_hsRaw; 
};
