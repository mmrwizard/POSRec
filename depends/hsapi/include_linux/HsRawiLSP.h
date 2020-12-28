#ifndef HSRAWAPI_H
#define HSRAWAPI_H

#if defined _WIN32 || defined _WIN64 || defined WIN32 || defined WIN64
#define MY_DECL_EXPORT __declspec(dllexport)
#define MY_DECL_IMPORT __declspec(dllimport)
#else
#define MY_DECL_EXPORT __attribute__((visibility("default")))
#define MY_DECL_IMPORT __attribute__((visibility("default")))
#endif

#if defined(HSRAW_EXPORTS)
#  define HSRAW_API MY_DECL_EXPORT
#else
#  define HSRAW_API MY_DECL_IMPORT
#endif

#include <string>
#include <vector>
#include <fstream>
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
    bool openFile(std::string strHsrFile);//hsr文件路径

    // 从文件输入流读文件头
	// ifstream打开文件后，文件指针必须在文件起始位置，再调用本函数
	// 返回值为文件头的大小： 旧版本文件头大小为1024字节，新版本文件头大小为8192字节
	int readFileHeader(std::ifstream& fileStream);

    // 从一段内存读文件头
    void readFileHeader(byte* pHeaderBuffer);//文件头二进制流	

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
    int getRingData( std::vector<std::vector<HSPoint> >& retVecHsPoints, // 返回一圈点云数据
        LPHsrCallback callbackFun = NULL);		              // 回调函数

	void getFrameData( const byte* pFrameBuffer,     // 输入一帧原始数据
        std::vector<HSPoint>& retVecHsPoints,                   // 返回一帧点云坐标
        int resample = 0
        ,float minRange=0,float maxRange=1500,float minANg=0,float maxANg=360);

    //设置是否进行过滤
    void setFilter(bool bDoFilter);

private:
    CHsRawCore* m_hsRaw;
};

#endif
