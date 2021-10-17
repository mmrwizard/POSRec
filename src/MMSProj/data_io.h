#ifndef DATAIO_H
#define DATAIO_H

#include "utility.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <liblas/liblas.hpp>
#include <liblas/version.hpp>
#include <liblas/point.hpp>

//#include <LASlib/lasreader.hpp>
//#include <LASlib/laswriter.hpp>

#include <boost/filesystem.hpp>
#include <boost/function.hpp>

// #include <gdal.h>
// #include <gdal_alg.h>
// #include <gdal_priv.h>
// #include <ogrsf_frmts.h>

#include <strstream>
#include <fstream>
#include <vector>
//#include <io.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

namespace roadmarking
{
	class DataIo
	{
	public:
		//pcd file
		bool readPcdFile(const string &fileName, const pcXYZIPtr &pointCloud);
		bool readPcdFile(const string &fileName, const pcXYZIPtr &pointCloud, Bounds &bound_3d);
		bool writePcdFile(const string &fileName, const pcXYZIPtr &pointCloud);
		bool writePcdFile(const string &fileName, const pcXYZRGBPtr &pointCloud);
		bool writePcdAll(const string &foldername, const string &fileName, const vector<pcXYZI> &pointClouds);													

		//las file
		//using liblas (used)
		bool readLasFileHeader(const std::string &fileName, liblas::Header &header);
		bool readLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud);
		//bool readLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud, Bounds &bound_3d);
		bool readLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud, vector<double> &gps_time,Bounds &bound_3d);
		//mrf:read guardcloud
		bool readGuardLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud, vector<double> &gps_time, Bounds &bound_3d);

		bool writeLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud, liblas::Color lasColor, double minX, double minY); 
		
		//export ground pointcloud
		bool writeLasGcloud(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud, double minX, double minY);

	private:
		void getCloudBound(const pcXYZIPtr &cloud, Bounds &bound);
	};
}

#endif
