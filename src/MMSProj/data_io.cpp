#include "data_io.h"
#include "utility.h"

// pcl io
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

// *.shp vectorization
// #include <gdal.h>
// #include <gdal_alg.h>
// #include <gdal_priv.h>
// #include <ogrsf_frmts.h>

// *.las io
#include <liblas/liblas.hpp>
#include <liblas/version.hpp>
#include <liblas/point.hpp>

#include <iosfwd>
#include <fstream>

//#if GDAL_VERSION_MAJOR >= 2
//#define GDAL2
//#endif

using namespace std;
using namespace boost::filesystem;

namespace roadmarking
{   
	bool DataIo::readPcdFile(const std::string &fileName, const pcXYZIPtr &pointCloud)
	{
		pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); //Ban pcl warnings

		if (pcl::io::loadPCDFile<pcl::PointXYZI>(fileName, *pointCloud) == -1) //* load the file
		{
			PCL_ERROR("Couldn't read file\n");
			return false;
		}

		return true;
	}

	bool DataIo::readPcdFile(const std::string &fileName, const pcXYZIPtr &pointCloud, Bounds &bound_3d)
	{
		pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); //Ban pcl warnings
		//intensity should be stored as 'intensity' (lower case i) field instead of 'Intensity' in the pcd file
		//reference: http://www.danielgm.net/cc/forum/viewtopic.php?t=4540
		if (pcl::io::loadPCDFile<pcl::PointXYZI>(fileName, *pointCloud) == -1) //* load the file
		{
			PCL_ERROR("Couldn't read file\n");
			return false;
		}
		if (pointCloud->points[0].intensity==0) //check if the intensity exsit
			printf("Warning! Point cloud intensity may not be imported properly, check the scalar field's name.\n");
		getCloudBound(pointCloud, bound_3d);
		for (int i=0; i<pointCloud->size(); i++) //shift the center point to the origin of the coordinate system
	    {
			pointCloud->points[i].x -= (0.5*(bound_3d.min_x+bound_3d.max_x));
			pointCloud->points[i].y -= (0.5*(bound_3d.min_y+bound_3d.max_y));
		}
		return true;
	}

	bool DataIo::writePcdFile(const std::string &fileName, const pcXYZIPtr &pointCloud)
	{
		pointCloud->width = pointCloud->points.size();
		pointCloud->height = 1;
		if (pcl::io::savePCDFileBinary<pcl::PointXYZI>(fileName, *pointCloud) == -1) //* load the file
		{
			PCL_ERROR("Couldn't read file\n");
			return false;
		}
		return true;
	}

	bool DataIo::writePcdFile(const string &fileName, const pcXYZRGBPtr &pointCloud)
	{
		pointCloud->width = pointCloud->points.size();
		pointCloud->height = 1;
		if (pcl::io::savePCDFileBinary<pcl::PointXYZRGB>(fileName, *pointCloud) == -1) //* load the file
		{
			PCL_ERROR("Couldn't read file\n");
			return false;
		}
		return true;
	}


	
	bool DataIo::writePcdAll(const string &folderName, const string &fileName, const vector<pcXYZI> &pointClouds)
	{

		if (!boost::filesystem::exists(folderName))
		{
			boost::filesystem::create_directory(folderName);
		}

		for (int i = 0; i < pointClouds.size(); i++)
		{
			string outputFileName;
			ostringstream oss;
			oss << setw(4) << setfill('0') << i << "_" << fileName;
			// outputFileName = folderName + "\\" + oss.str(); // Windows
			outputFileName = folderName + "/" + oss.str(); // Ubuntu
			if (pointClouds[i].makeShared()->points.size() < 100)
			{
				//cout << "Warning: the" << i << "th point cloud contain point less than the threshold. Do not save it." << endl;
				continue;
			}
			writePcdFile(outputFileName, pointClouds[i].makeShared());
		}

		//cout << "Output Done" << endl;
		return true;
	}

	

	bool DataIo::readLasFileHeader(const std::string &fileName, liblas::Header &header)
	{
		if (fileName.substr(fileName.rfind('.')).compare(".las"))
		{
			return 0;
		}
		else
		{
			std::ifstream ifs;
			ifs.open(fileName, std::ios::in | std::ios::binary);
			if (ifs.bad())
			{
				return 0;
			}

			liblas::ReaderFactory f;
			liblas::Reader reader = f.CreateWithStream(ifs);

			header = reader.GetHeader();
		}

		return 1;
	}

	bool DataIo::readLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud)
	{
		if (fileName.substr(fileName.rfind('.')).compare(".las"))
		{
			return 0;
		}

		std::ifstream ifs;
		ifs.open(fileName, std::ios::in | std::ios::binary);
		if (ifs.bad())
		{
			cout << "Failed" << endl;
		}
		liblas::ReaderFactory f;
		liblas::Reader reader = f.CreateWithStream(ifs);
		liblas::Header const &header = reader.GetHeader();

		double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
		Xmin = header.GetMinX();
		Ymin = header.GetMinY();
		Zmin = header.GetMinZ();
		Xmax = header.GetMaxX();
		Ymax = header.GetMaxY();
		Zmax = header.GetMaxZ();

		while (reader.ReadNextPoint())
		{
			const liblas::Point &p = reader.GetPoint();
			//i++;
			pcl::PointXYZI pt;
			pt.x = p.GetX() - Xmin;
			pt.y = p.GetY() - Ymin;
			pt.z = p.GetZ();
			pt.intensity = p.GetIntensity();
			pointCloud.points.push_back(pt);
		}

		return 1;
	}
	bool DataIo::readLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud, vector<double> &gps_time,Bounds &bound_3d)
	{
		if (fileName.substr(fileName.rfind('.')).compare(".las"))
		{
			return 0;
		}

		std::ifstream ifs;
		ifs.open(fileName, std::ios::in | std::ios::binary);
		if (ifs.bad())
		{
			cout << "Failed" << endl;
		}
		liblas::ReaderFactory f;
		liblas::Reader reader = f.CreateWithStream(ifs);
		liblas::Header const &header = reader.GetHeader();
		double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
		Xmin = header.GetMinX();
		Ymin = header.GetMinY();
		Zmin = header.GetMinZ();
		Xmax = header.GetMaxX();
		Ymax = header.GetMaxY();
		Zmax = header.GetMaxZ();

		bound_3d.min_x = Xmin;
		bound_3d.min_y = Ymin;
		bound_3d.min_z = Zmin;
		bound_3d.max_x = Xmax;
		bound_3d.max_y = Ymax;
		bound_3d.max_z = Zmax;

		float center_x = bound_3d.min_x / 2.0f + bound_3d.max_x / 2.0f;
		float center_y = bound_3d.min_y / 2.0f + bound_3d.max_y / 2.0f;
		float center_z = bound_3d.min_z / 2.0f + bound_3d.max_z / 2.0f;

		while (reader.ReadNextPoint())
		{
			const liblas::Point &p = reader.GetPoint();
			//i++;
			pcl::PointXYZI pt;

			pt.x = p.GetX() - center_x;
			pt.y = p.GetY() - center_y;
			//pt.x = p.GetX() ;
			//pt.y = p.GetY() ;
			pt.z = p.GetZ();
			pt.intensity = p.GetIntensity(); //The scalar field should be stored as exactly "Intensity" (captial I) in the las file
			gps_time.push_back(p.GetTime());
			pointCloud.points.push_back(pt);
		}
		//获取最小gps时间
		//double Min_gps_time = *min_element(gps_time.begin(), gps_time.end());
		//cout << Min_gps_time << endl;
		if (pointCloud.points[0].intensity==0) //check if the intensity exsit
			printf("Warning! Point cloud intensity may not be imported properly, check the scalar field's name.\n");

		return 1;
	}

	void DataIo::getCloudBound(const pcXYZIPtr &cloud, Bounds &bound)
	{
		double min_x = cloud->points[0].x;
		double min_y = cloud->points[0].y;
		double min_z = cloud->points[0].z;
		double max_x = cloud->points[0].x;
		double max_y = cloud->points[0].y;
		double max_z = cloud->points[0].z;

		for (int i = 0; i < cloud->size(); i++)
		{
			if (min_x > cloud->points[i].x)
				min_x = cloud->points[i].x;
			if (min_y > cloud->points[i].y)
				min_y = cloud->points[i].y;
			if (min_z > cloud->points[i].z)
				min_z = cloud->points[i].z;
			if (max_x < cloud->points[i].x)
				max_x = cloud->points[i].x;
			if (max_y < cloud->points[i].y)
				max_y = cloud->points[i].y;
			if (max_z < cloud->points[i].z)
				max_z = cloud->points[i].z;
		}
		bound.min_x = min_x;
		bound.max_x = max_x;
		bound.min_y = min_y;
		bound.max_y = max_y;
		bound.min_z = min_z;
		bound.max_z = max_z;
	}
}
