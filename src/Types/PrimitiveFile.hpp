/*
 * PrimitiveFile.hpp
 *
 *  Created on: Nov 20, 2014
 *      Author: lzmuda
 */

#ifndef PRIMITIVEFILE_HPP_
#define PRIMITIVEFILE_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/variant.hpp>

#include "Logger.hpp"

#include <cstdlib>
#include <iostream>
#include <glob.h>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "Logger.hpp"
#include "mongo/client/dbclient.h"
#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include <dirent.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZRGBSIFT.hpp>
#include <Types/SIFTObjectModelFactory.hpp>
#include <Types/MongoBase.hpp>

#include <vector>
#include <list>
#include <iostream>
#include <boost/variant.hpp>
#include <boost/lexical_cast.hpp>


namespace PrimitiveFile{
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;

class PrimitiveFile
{
private:
	string mongoFileName;	// file name in mongo base
	string pcdCloudType;	// sift, shot, xyz,  itd... itp...
	string extension;		// png, jpg, yaml, xml
	string place;			// {grid, document}
	//TODO make ENUM fileType
	string fileType; 		// mask, rgb, depth, I, XYZ, cameraInfo, PCL
	string PCLType;			// SIFT, SHOT, XYZ, ORB, NORMALS
	// LR
	struct notTextured
	{
		cv::Mat leftImage;
		cv::Mat rightImage;
	};

	// LR + LR textured
	struct textured
	{
		struct notTextured notTextur;
		cv::Mat leftImageTextured;
		cv::Mat rightImageTextured;
	};
	static struct textured* texturedPtr;
	static struct nontextured * nontexturedPtr;
	boost::variant<std::vector<double>, cv::Mat, string,  pcl::PointCloud<pcl::PointXYZ>::Ptr,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<PointXYZSIFT>::Ptr,
	pcl::PointCloud<PointXYZRGBSIFT>::Ptr,
	struct textured*,
	struct notTextured*> dataType;

	//TODO add Normals, orb
	//};
public:

	void getSize();
	void saveIntoDisc();
	void saveIntoDucument();
	void convertToBuffer();
	void saveIntoGrid();
	void getMime();
	void readFromMongoDB();
	void readFromGrid();
	void readFromDocument();
	void test()
	{
		cv::Mat b;
		dataType = b;
		struct textured tex;
		dataType = &tex;
		pcl::PointCloud<PointXYZSIFT>::Ptr cloudSift;
		dataType = cloudSift;
	};
};
}//namespace File


#endif /* PRIMITIVEFILE_HPP_ */
