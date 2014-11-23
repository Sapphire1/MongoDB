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

class PrimitiveFile : public MongoBase
{
private:
	string mongoFileName;	// file name in mongo base
	string pcdCloudType;	// sift, shot, xyz,  itd... itp...
	string extension;		// png, jpg, yaml, xml
	string place;			// {grid, document}
	float sizeBytes;
	float sizeMBytes;
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
		//TODO change it - read from component
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
	void saveIntoMongoBase();
	void insertFileIntoGrid();
	void insertFileIntoDocument();
	void convertToBuffer();
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

void PrimitiveFile::saveIntoMongoBase()
{
	//if(sizeMBytes<15)
		insertFileIntoDocument();
	//else if(sizeMBytes>=15)
	//	insertFileIntoGrid();
}

void PrimitiveFile::insertFileIntoDocument()
{
	// use variant here...
	LOG(LNOTICE)<<"PrimitiveFile::insertFileIntoCollection";
	BSONObjBuilder builder;
	BSONObj b;
	BSONElement bsonElement;
	OID oid;
	switch(fileType)
	{
		case xml:
		{
			break;
		}
		case rgb:
		{
			LOG(LNOTICE)<<"Insert RGB";
			cv::Mat* img =  boost::get<cv::Mat>(&data);

			std::vector<uchar> buf;
			std::vector<int> params(2);
			params[0] = CV_IMWRITE_JPEG_QUALITY;
			params[1] = 95;
			cv::imencode(".jpg", *img, buf, params);
			string tempFileName = "temp";
			// TODO add proper tempFileName
			BSONObj b=BSONObjBuilder().genOID().appendBinData(tempFileName, buf.size(), mongo::BinDataGeneral, &buf[0]).append("filename", tempFileName).append("size", 12).append("place", "document").append("extension", fileType).obj();
			BSONElement bsonElement;
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			// niech view ma klienta i podczas tworzenia pliku niech przekazuje na niego referencję
			// a widok w konstruktorze tworzy sobie klienta i się z nim łączy
			// usunąć z komponentu tworzenie i łączenie się !!!
			// todo przeniesc to do konstruktora!!!
			c->connect("localhost");
			dbCollectionPath = "images.containers";
			/////////////////////////////////////
			c->insert(dbCollectionPath, b);
			break;
		}
		case density:
		{
			break;
		}
		case intensity:
		{
			break;
		}
		case mask:
		{
			break;
		}
		case stereoL:
		{
			break;
		}
		case stereoR:
		{
			break;
		}
		case stereoLTextured:
		{
			break;
		}
		case stereoRTextured:
		{
			break;
		}
		case pc_xyz:
		{
			break;
		}
		case pc_xyzrgb:
		{
			break;
		}
		case pc_xyzsift:
		{
			break;
		}
		case pc_xyzrgbsift:
		{
			break;
		}
		case pc_xyzshot:
		{
			break;
		}
		case pc_xyzrgbnormal:
		{
			break;
		}
	}

	/*
	if (fileType=="png" || fileType=="jpg")	// save image
	{

	}
	else if(fileType=="pcd")	// save cloud
	{
		if(cloudType=="xyzrgb")
		{
			int cloudSize = cloudXYZRGB->size();
			int fieldsNr = xyzrgbPointSize*cloudSize;
			int totalSize = fieldsNr*sizeof(float);
			float buff[fieldsNr];

			// copy all points to memory
			for(int iter=0; iter<cloudSize; iter++)
			{
				const pcl::PointXYZRGB p = cloudXYZRGB->points[iter];
				copyXYZRGBPointToFloatArray (p, &buff[xyzrgbPointSize*iter]);
			}
			b=BSONObjBuilder().genOID().appendBinData(tempFileName, totalSize, mongo::BinDataGeneral, &buff[0]).append("filename", tempFileName).append("size", totalSize).append("place", "document").append("extension", fileType).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			c->insert(dbCollectionPath, b);
		}
		else if(cloudType=="xyz")
		{
			int cloudSize = cloudXYZ->size();
			int fieldsNr = xyzPointSize*cloudSize;
			int totalSize = fieldsNr*sizeof(float);
			float buff[fieldsNr];

			// copy all points to memory
			for(int iter=0; iter<cloudSize; iter++)
			{
				const pcl::PointXYZ p = cloudXYZ->points[iter];
				copyXYZPointToFloatArray (p, &buff[xyzPointSize*iter]);
			}
			b=BSONObjBuilder().genOID().appendBinData(tempFileName, totalSize, mongo::BinDataGeneral, &buff[0]).append("filename", tempFileName).append("size", totalSize).append("place", "document").append("extension", fileType).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			c->insert(dbCollectionPath, b);
		}
		//TODO dodac SHOT'Y
		else if(cloudType=="xyzsift")
		{
			int cloudSize = cloudXYZSIFT->size();
			int fieldsNr = siftPointSize*cloudSize;
			int totalSize = fieldsNr*sizeof(float);
			float buff[fieldsNr];

			// copy all points to memory
			for(int iter=0; iter<cloudSize; iter++)
			{
				const PointXYZSIFT p = cloudXYZSIFT->points[iter];
				copyXYZSiftPointToFloatArray (p, &buff[siftPointSize*iter]);
			}
			b=BSONObjBuilder().genOID().appendBinData(tempFileName, totalSize, mongo::BinDataGeneral, &buff[0]).append("filename", tempFileName).append("size", totalSize).append("place", "document").append("extension", fileType).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			c->insert(dbCollectionPath, b);
		}
		LOG(LNOTICE)<<"ViewWriter::insertFileIntoCollection, PCD file end";
	}
	else if (fileType=="yaml" || fileType=="yml")	// save image
	{
		LOG(LNOTICE)<<xyzimage.size();
		LOG(LNOTICE)<<"ViewWriter::insertFileIntoCollection, cv::Mat - XYZRGB";
		int bufSize = xyzimage.total()*xyzimage.channels()*4;
		int width = xyzimage.size().width;
		int height = xyzimage.size().height;
		int channels = xyzimage.channels();
		float* buf;
		buf = (float*)xyzimage.data;
		LOG(LNOTICE)<<"Set buffer YAML";
		b=BSONObjBuilder().genOID().appendBinData(tempFileName, bufSize, mongo::BinDataGeneral, &buf[0]).append("filename", tempFileName).append("size", size).append("place", "document").append("extension", fileType).append("width", width).append("height", height).append("channels", channels).obj();
		LOG(LNOTICE)<<"YAML inserted";
		b.getObjectID(bsonElement);
		oid=bsonElement.__oid();
		c->insert(dbCollectionPath, b);
	}
	else if(fileType=="xml")
	{
		LOG(LNOTICE)<<"ViewWriter::insertFileIntoCollection, xml file";
		string input;

		// read string from input
		//input =cipFileIn.read()+" ";
		LOG(LNOTICE)<<"Input:"<< input;
		input="";
		// convert to char*
		char const *cipCharTable = input.c_str();
		int size = strlen(cipCharTable);
		LOG(LNOTICE)<<string(cipCharTable);
		LOG(LNOTICE)<<"Size: "<<size;
		// create bson object
		b = BSONObjBuilder().genOID().appendBinData(tempFileName, size, BinDataGeneral,  cipCharTable).append("filename", tempFileName).append("size", size).append("place", "document").append("extension", fileType).obj();

		// insert object into collection
		c->insert(dbCollectionPath, b);

		b.getObjectID(bsonElement);
		oid=bsonElement.__oid();
	}
	cloudType="";
	*/
}

}//namespace File
}//MongoBase


#endif /* PRIMITIVEFILE_HPP_ */
