/*
 * Model.hpp
 *
 *  Created on: Nov 20, 2014
 *      Author: lzmuda
 */

#ifndef MODEL_HPP_
#define MODEL_HPP_

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

namespace MongoBase {
using namespace PrimitiveFile;
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;

class Model : public MongoBase::MongoBase
{
private:
	std::string ModelName;					// model1
	std::vector<PrimitiveFile::PrimitiveFile> files;
	std::vector<std::string>	fileTypes;		// [MASK, IMAGE, …, IMAGE3D]
	std::vector<AbstractObject*>& models;

public:
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
	void getDateOfInsert();
	void loadModel();
	std::vector<AbstractObject*> getModels();
};// class Model
}//namespace MongoBase

#endif /* MODEL_HPP_ */
