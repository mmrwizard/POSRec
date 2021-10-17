#include "data_io.h"
#include "utility.h"
#include "imageprocess.h"

#include <chrono>

using namespace std;
using namespace roadmarking;
using namespace cv;

int main(int argc, char *argv[])
{
	std::string inputFilePath = argv[1];
	std::string outputFolderPath = argv[2];

	DataIo io;
	string filename = inputFilePath.substr(inputFilePath.rfind('/'));

	pcXYZIPtr cloud(new pcXYZI());
	vector<double> gps_time;
	vector<double> CloudBoundingBox;
	Bounds bound_3d_temp;
	double X_origin = 0.0, Y_origin = 0.0;

	Mat imgZ;

	//step 1: data import
	string extension = inputFilePath.substr(inputFilePath.find_last_of('.') + 1); //Get the suffix of the file;
	if (!strcmp(extension.c_str(), "las"))
		io.readLasFile(inputFilePath, *cloud, gps_time, bound_3d_temp);
	else if (!strcmp(extension.c_str(), "pcd"))
		io.readPcdFile(inputFilePath, cloud, bound_3d_temp);
	else
		printf("Unrecognized data format. Please use *.pcd or *.las format point cloud.\n");

	X_origin = 0.5 * (bound_3d_temp.min_x + bound_3d_temp.max_x);
	Y_origin = 0.5 * (bound_3d_temp.min_y + bound_3d_temp.max_y);
	cout << "Import [" << cloud->size() << "] points." << endl;
	printf("Global shift (X: %8.3f m, Y: %8.3f m)\n", -X_origin, -Y_origin);

	std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

	StructOperator so;
	Bounds bounds;
	CenterPoint center;
	so.getBoundAndCenter(*cloud, bounds, center);

	//step 2: 
	//2.1 : 3d->2d projection, generating projection image
	Imageprocess ip;
	float resolution = 0.08;//gird size
	ip.savepcgrid(bound_3d_temp, resolution, cloud); //get image size and save point indices in each pixel
	ip.pc2imgZ(cloud, 2, imgZ);


	//2.2 : 2d->3d back to point cloud
	//pcXYZIPtr outcloud(new pcXYZI());
	//ip.img2pc_c(imgZ, cloud, outcloud);

	//step 3: 
	//3.1: output image
	if (!boost::filesystem::exists(outputFolderPath))
		boost::filesystem::create_directory(outputFolderPath);

	string output_sub_folder = outputFolderPath + filename + "_out";

	if (!boost::filesystem::exists(output_sub_folder))
		boost::filesystem::create_directory(output_sub_folder);

	ip.saveimg(output_sub_folder + "/Geo-referenced_Image", 0, imgZ);

	//3.2 : output point by image project
	//io.writeLasGcloud(output_sub_folder + "/outcloud.las", *outcloud, X_origin, Y_origin);

}