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

namespace MongoBase{
namespace PrimitiveFile{
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;

class setSizeVisitor : public boost::static_visitor<float>
{
public:
	float operator()(const std::string & str) const
	{
		return str.length();
	}

	float operator()(const cv::Mat & img) const
	{
		LOG(LNOTICE)<<"operator(): cv::Mat";
		return (float)img.elemSize1()*(float)img.total();
	}

	float operator()(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudXYZ) const
	{
		LOG(LNOTICE)<<"operator(): PointXYZ";
		float sizeBytes;
		std::vector< pcl::PCLPointField> fields;
		pcl::getFields<pcl::PointXYZ>(fields);
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeBytes+=(float)pcl::getFieldSize(it->datatype)*cloudXYZ->size();
		return sizeBytes;
	}

	float operator()(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudXYZRGB) const
	{
		LOG(LNOTICE)<<"operator(): PointXYZRGB";
		float sizeBytes;
		std::vector< pcl::PCLPointField> fields;
		pcl::getFields<pcl::PointXYZRGB>(fields);
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeBytes+=(float)pcl::getFieldSize(it->datatype)*cloudXYZRGB->size();
		return sizeBytes;
	}

	float operator()(const pcl::PointCloud<PointXYZSIFT>::Ptr& cloudXYZSIFT) const
    {
		LOG(LNOTICE)<<"operator(): PointXYZSIFT";
		float sizeBytes;
		std::vector< pcl::PCLPointField> fields;
		pcl::getFields<PointXYZSIFT>(fields);
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeBytes+=(float)pcl::getFieldSize(it->datatype)*cloudXYZSIFT->size();
		return sizeBytes;
    }

	float operator()(const pcl::PointCloud<PointXYZRGBSIFT>::Ptr& cloudXYZRGBSIFT) const
    {
		LOG(LNOTICE)<<"operator(): PointXYZRGBSIFT";
		float sizeBytes;
		std::vector< pcl::PCLPointField> fields;
		pcl::getFields<PointXYZRGBSIFT>(fields);
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeBytes+=(float)pcl::getFieldSize(it->datatype)*cloudXYZRGBSIFT->size();
		return sizeBytes;
    }

	float operator()(const pcl::PointCloud<PointXYZSHOT>::Ptr& cloudXYZSHOT) const
    {
		LOG(LNOTICE)<<"operator(): PointXYZSHOT";
		float sizeBytes;
		std::vector< pcl::PCLPointField> fields;
		pcl::getFields<PointXYZSHOT>(fields);
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeBytes+=(float)pcl::getFieldSize(it->datatype)*cloudXYZSHOT->size();
		return sizeBytes;
    }

	float operator()(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloudXYZNormal) const
    {
		LOG(LNOTICE)<<"operator(): PointXYZRGBNormal";
		float sizeBytes;
		std::vector< pcl::PCLPointField> fields;
		pcl::getFields<PointXYZSHOT>(fields);
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeBytes+=(float)pcl::getFieldSize(it->datatype)*cloudXYZNormal->size();
		return sizeBytes;
    }
};

class PrimitiveFile
{
private:
	string mongoFileName;	// file name in mongo base
	string pcdCloudType;	// sift, shot, xyz,  itd... itp...
	string extension;		// png, jpg, yaml, xml
	string place;			// {grid, document}
	float sizeBytes;
	float sizeMBytes;
	//TODO make ENUM fileType
	keyTypes fileType; 		// mask, rgb, depth, I, XYZ, cameraInfo, PCL
	string PCLType;			// SIFT, SHOT, XYZ, ORB, NORMALS

	boost::variant<	cv::Mat,
	string,
	pcl::PointCloud<pcl::PointXYZ>::Ptr,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
	pcl::PointCloud<PointXYZSIFT>::Ptr,
	pcl::PointCloud<PointXYZRGBSIFT>::Ptr,
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr,
	pcl::PointCloud<PointXYZSHOT>::Ptr
	> data;

public:
	PrimitiveFile(const cv::Mat& img, keyTypes& key) : data(img), fileType(key), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor cv::Mat";
		this->setSize();
	};
	PrimitiveFile(const std::string& str, keyTypes& key) : data(str), fileType(key), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor std::string";
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,  keyTypes& key) : data(cloud), fileType(key), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor <pcl::PointXYZ>";
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, keyTypes& key) : data(cloud), fileType(key), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor <pcl::PointXYZRGB>";
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<PointXYZSIFT>::Ptr& cloud, keyTypes& key) : data(cloud), fileType(key), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor <PointXYZSIFT>";
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<PointXYZRGBSIFT>::Ptr& cloud, keyTypes& key) : data(cloud), fileType(key), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor <PointXYZRGBSIFT>";
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<PointXYZSHOT>::Ptr& cloud, keyTypes& key) : data(cloud), fileType(key), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor <PointXYZSHOT>";
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, keyTypes& key) : data(cloud), fileType(key), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor <pcl::PointXYZRGBNormal>";
		this->setSize();
	};

	void saveIntoDisc();
	void saveIntoDucument();
	void convertToBuffer();
	void saveIntoGrid();
	void setMime();
	void getMime();
	void readFromMongoDB();
	void readFromGrid();
	void readFromDocument();

	void setType(keyTypes key)
	{
		fileType = key;
	};

	keyTypes getType()
	{
		return fileType;
	};

	void getSize();
	void setSize()
	{
		// get size of data in Bytes
		sizeBytes = boost::apply_visitor(setSizeVisitor(), data);
		// convert to MBytes
		sizeMBytes = sizeBytes/(1024*1024);

		LOG(LNOTICE)<< "sizeBytes: "<<sizeBytes;
		LOG(LNOTICE)<< "sizeMBytes: "<<sizeMBytes;

	};
};

}//namespace File
}//MongoBase


#endif /* PRIMITIVEFILE_HPP_ */
