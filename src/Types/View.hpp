/*
 * View.hpp
 *
 *  Created on: Nov 20, 2014
 *      Author: lzmuda
 */

#ifndef VIEW_HPP_
#define VIEW_HPP_

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
#include <Types/PrimitiveFile.hpp>

#include <vector>
#include <list>
#include <iostream>
#include <boost/variant.hpp>
#include <boost/lexical_cast.hpp>

namespace MongoBase{
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace PrimitiveFile;
class View : public MongoBase::MongoBase
{
private:
	string ViewName;						// lab012
	string SensorType;						// Stereo, ToF...
	string dateOfInsert;					// 02042013
	std::vector<PrimitiveFile::PrimitiveFile> files;
	std::vector<string>	fileTypes;			// [MASK, IMAGE, …, IMAGE3D]

public:
	Base::DataStreamIn <cv::Mat> in_img;
	void addFile();
	void getAllFiles();
	void saveAllFiles();
	void removeFile();
	void removeAllFiles();
	void setViewName();
	void getViewName();
	void setSensorType();
	void getSensorType();
	void setDateOfInsert();
	void getDateOfInsert()
	{
		in_img.read();
	};
};// class View
}//namespace MongoBase




#endif /* VIEW_HPP_ */
