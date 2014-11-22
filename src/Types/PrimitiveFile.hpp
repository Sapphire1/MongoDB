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
	// LR
	/*
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
	*/
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

	void saveIntoDisc();
	void saveIntoDucument();
	void convertToBuffer();
	void saveIntoGrid();
	void setMime();
	void getMime();

	void putString(const std::string& str, keyTypes key);
	void putMat(const cv::Mat& image, keyTypes key);
	void putPCxyz(const pcl::PointCloud<pcl::PointXYZ>::Ptr&, keyTypes key);
	void putPCyxzrgb(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, keyTypes key);
	void putPCxyzsift(const pcl::PointCloud<PointXYZSIFT>::Ptr&, keyTypes key);
	void putPCxyzrgbsift(const pcl::PointCloud<PointXYZRGBSIFT>::Ptr&, keyTypes key);
	void putPCxyzshot(const pcl::PointCloud<PointXYZSHOT>::Ptr&, keyTypes key);
	void putPCxyzrgbNormal(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr&, keyTypes key);

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
};

void PrimitiveFile::putString(const std::string& str, keyTypes key)
{
	LOG(LNOTICE)<< "File::putString";
	LOG(LNOTICE)<< "key: "<<key;
	data = str;
	// use boost variant to do this
	setType(key);

	// get size of data in Bytes
	sizeBytes = boost::apply_visitor( setSizeVisitor(), data);

	// convert to MBytes
	sizeMBytes = sizeBytes/(1024*1024);

	LOG(LNOTICE)<<"sizeBytes: " << sizeBytes;
	LOG(LNOTICE)<<"sizeMBytes: " << sizeMBytes;

}

void PrimitiveFile::putMat(const cv::Mat& img, keyTypes key)
{
	LOG(LNOTICE)<< "PrimitiveFile::putMat";
	LOG(LNOTICE)<< "key: "<<key;

	data = img;

	setType(key);

	// get size of data in Bytes
	sizeBytes = boost::apply_visitor(setSizeVisitor(), data);

	// convert to MBytes
	sizeMBytes = sizeBytes/(1024*1024);

	LOG(LNOTICE)<<"sizeBytes: " << sizeBytes;
	LOG(LNOTICE)<<"sizeMBytes: " << sizeMBytes;

}

void PrimitiveFile::putPCxyz(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudXYZ, keyTypes key)
{
	LOG(LNOTICE)<< "PrimitiveFile::putPCxyz";
	LOG(LNOTICE)<< "key: "<<key;

	data = cloudXYZ;

	setType(key);

	// get size of data in Bytes
	sizeBytes = boost::apply_visitor( setSizeVisitor(), data);

	// convert to MBytes
	sizeMBytes = sizeBytes/(1024*1024);

	LOG(LNOTICE)<<"sizeBytes: " << sizeBytes;
	LOG(LNOTICE)<<"sizeMBytes: " << sizeMBytes;

}

void PrimitiveFile::putPCyxzrgb(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudXYZRGB, keyTypes key)
{
	LOG(LNOTICE)<< "PrimitiveFile::putPCyxzrgb";
	LOG(LNOTICE)<< "key: "<<key;

	data = cloudXYZRGB;

	setType(key);

	// get size of data in Bytes
	sizeBytes = boost::apply_visitor( setSizeVisitor(), data);

	// convert to MBytes
	sizeMBytes = sizeBytes/(1024*1024);

	LOG(LNOTICE)<<"sizeBytes: " << sizeBytes;
	LOG(LNOTICE)<<"sizeMBytes: " << sizeMBytes;
}

void PrimitiveFile::putPCxyzsift(const pcl::PointCloud<PointXYZSIFT>::Ptr& cloudXYZSIFT, keyTypes key)
{
	LOG(LNOTICE)<< "PrimitiveFile::putPCxyzsift";
	LOG(LNOTICE)<< "key: "<<key;

	data = cloudXYZSIFT;

	setType(key);

	// get size of data in Bytes
	sizeBytes = boost::apply_visitor( setSizeVisitor(), data);

	// convert to MBytes
	sizeMBytes = sizeBytes/(1024*1024);

	LOG(LNOTICE)<<"sizeBytes: " << sizeBytes;
	LOG(LNOTICE)<<"sizeMBytes: " << sizeMBytes;
}

void PrimitiveFile::putPCxyzrgbsift(const pcl::PointCloud<PointXYZRGBSIFT>::Ptr& cloudXYZRGBSIFT, keyTypes key)
{
	LOG(LNOTICE)<< "PrimitiveFile::putPCxyzrgbsift";
	LOG(LNOTICE)<< "key: "<<key;

	data = cloudXYZRGBSIFT;

	setType(key);

	// get size of data in Bytes
	sizeBytes = boost::apply_visitor( setSizeVisitor(), data);

	// convert to MBytes
	sizeMBytes = sizeBytes/(1024*1024);

	LOG(LNOTICE)<<"sizeBytes: " << sizeBytes;
	LOG(LNOTICE)<<"sizeMBytes: " << sizeMBytes;
}

void PrimitiveFile::putPCxyzshot(const pcl::PointCloud<PointXYZSHOT>::Ptr& cloudXYZSHOT, keyTypes key)
{
	LOG(LNOTICE)<< "PrimitiveFile::putPCxyzshot";
	LOG(LNOTICE)<< "key: "<<key;

	data = cloudXYZSHOT;

	setType(key);

	// get size of data in Bytes
	sizeBytes = boost::apply_visitor( setSizeVisitor(), data);

	// convert to MBytes
	sizeMBytes = sizeBytes/(1024*1024);

	LOG(LNOTICE)<<"sizeBytes: " << sizeBytes;
	LOG(LNOTICE)<<"sizeMBytes: " << sizeMBytes;
}

void PrimitiveFile::putPCxyzrgbNormal(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloudXYZRGBNormal, keyTypes key)
{
	LOG(LNOTICE)<< "PrimitiveFile::putPCxyzrgbNormal";
	LOG(LNOTICE)<< "key: "<<key;

	data = cloudXYZRGBNormal;

	setType(key);

	// get size of data in Bytes
	sizeBytes = boost::apply_visitor( setSizeVisitor(), data);

	// convert to MBytes
	sizeMBytes = sizeBytes/(1024*1024);

	LOG(LNOTICE)<<"sizeBytes: " << sizeBytes;
	LOG(LNOTICE)<<"sizeMBytes: " << sizeMBytes;
}

}//namespace File
}//MongoBase


#endif /* PRIMITIVEFILE_HPP_ */
