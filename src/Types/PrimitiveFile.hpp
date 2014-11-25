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

class PrimitiveFile //: public MongoBase
{
private:
	//string mongoFileName;	// file name in mongo base
	string fileName;
	string pcdCloudType;	// sift, shot, xyz,  itd... itp...
	string place;			// {grid, document}
	float sizeBytes;
	float sizeMBytes;
	keyTypes fileType; 		// mask, rgb, depth, I, XYZ, cameraInfo, PCL
	string PCLType;			// SIFT, SHOT, XYZ, ORB, NORMALS
	string viewName;
	string modelName;
	string collectionName;
	string documentType;	// View, Model, Object, file

	boost::variant<	cv::Mat,
	string,
	pcl::PointCloud<pcl::PointXYZ>::Ptr,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
	pcl::PointCloud<PointXYZSIFT>::Ptr,
	pcl::PointCloud<PointXYZRGBSIFT>::Ptr,
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr,
	pcl::PointCloud<PointXYZSHOT>::Ptr
	> data;

	boost::shared_ptr<MongoBase> basePtr;

public:
	PrimitiveFile(const cv::Mat& img, keyTypes& key, string& name, string& hostname) : data(img), fileType(key), fileName(name), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor cv::Mat";
		basePtr = boost::shared_ptr<MongoBase>(new MongoBase(hostname));
		this->setSize();
	};
	PrimitiveFile(const std::string& str, keyTypes& key, string& name, string& hostname) : data(str), fileType(key), fileName(name), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor std::string";
		basePtr = boost::shared_ptr<MongoBase>(new MongoBase(hostname));
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,  keyTypes& key, string& name, string& hostname) : data(cloud), fileType(key), fileName(name), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor <pcl::PointXYZ>";
		basePtr = boost::shared_ptr<MongoBase>(new MongoBase(hostname));
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, keyTypes& key, string& name, string& hostname) : data(cloud), fileType(key), fileName(name), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor <pcl::PointXYZRGB>";
		basePtr = boost::shared_ptr<MongoBase>(new MongoBase(hostname));
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<PointXYZSIFT>::Ptr& cloud, keyTypes& key, string& name, string& hostname) : data(cloud), fileType(key), fileName(name), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor <PointXYZSIFT>";
		basePtr = boost::shared_ptr<MongoBase>(new MongoBase(hostname));
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<PointXYZRGBSIFT>::Ptr& cloud, keyTypes& key, string& name, string& hostname) : data(cloud), fileType(key), fileName(name), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor <PointXYZRGBSIFT>";
		basePtr = boost::shared_ptr<MongoBase>(new MongoBase(hostname));
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<PointXYZSHOT>::Ptr& cloud, keyTypes& key, string& name, string& hostname) : data(cloud), fileType(key), fileName(name), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor <PointXYZSHOT>";
		basePtr = boost::shared_ptr<MongoBase>(new MongoBase(hostname));
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, keyTypes& key, string& name, string& hostname) : data(cloud), fileType(key), fileName(name), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor <pcl::PointXYZRGBNormal>";
		basePtr = boost::shared_ptr<MongoBase>(new MongoBase(hostname));
		this->setSize();
	};
	string getFileName(){return fileName;}
	void saveIntoDisc();
	void saveIntoMongoBase();
	void insertFileIntoGrid(OID& oid);
	void insertFileIntoDocument(OID& oid);
	void convertToBuffer();
	//void setMime();
	void getMime();
	void readFromMongoDB();
	void readFromGrid();
	void readFromDocument();
	void saveImageOnDisc();
	void saveToDisc(bool suffix, bool binary);

	void copyXYZPointToFloatArray (const pcl::PointXYZ &p, float * out) const;
	void copyXYZSiftPointToFloatArray (const PointXYZSIFT &p, float * out) const;
	void copyXYZRGBPointToFloatArray (const pcl::PointXYZRGB &p, float * out) const;
	void copyXYZRGBNormalPointToFloatArray (const pcl::PointXYZRGBNormal &p, float * out) const;
	void copyXYZSHOTPointToFloatArray (const PointXYZSHOT &p, float * out) const;

	void savePCxyzFileToDisc(bool suffix, bool binary, std::string& fn);
	void savePCxyzRGBFileToDisc(bool suffix, bool binary, std::string& fn);
	void savePCxyzSIFTFileToDisc(bool suffix, bool binary, std::string& fn);
	void savePCxyzRGBNormalFileToDisc(bool suffix, bool binary, std::string& fn);
	void savePCxyzSHOTFileToDisc(bool suffix, bool binary, std::string& fn);

	void setViewName(string& ViewName)
	{
		this->viewName = ViewName;
	};

	void setModelName(string& ModelName)
	{
		this->modelName = ModelName;
	};

	void setType(keyTypes key)
	{
		fileType = key;
	};

	keyTypes getType()
	{
		return fileType;
	};

	void getSize();
	void saveImage(OID& oid);
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
	OID oid;
	//if(sizeMBytes<15)
		insertFileIntoDocument(oid);
	//else if(sizeMBytes>=15)
	//	insertFileIntoGrid(oid);

	BSONObj query;
	// update document
	if (viewName!="")
	{
		query = BSON("ViewName"<<viewName<<"DocumentType"<<"View");
	}
	else if (modelName!="")
	{
		query = BSON("ModelName"<<modelName<<"DocumentType"<<"Model");
	}
	else
	{
		LOG(LERROR)<<"NO VIEW OR MODEL NAME!!!";
	}
	BSONObj update = BSON("$addToSet"<<BSON("fileOIDs"<<BSON("fileOID"<<oid.toString())));
	basePtr->update(query, update);

	update = BSON("$addToSet"<<BSON("FileTypes"<<BSON("Type"<<fileType)));
	basePtr->update(query, update);
}
void PrimitiveFile::saveImage(OID& oid)
{
	LOG(LNOTICE)<<"Insert image";
	cv::Mat* img =  boost::get<cv::Mat>(&data);
	std::vector<uchar> buf;
	std::vector<int> params(2);
	params[0] = CV_IMWRITE_JPEG_QUALITY;
	params[1] = 95;
	cv::imencode(".jpg", *img, buf, params);
	BSONObj b=BSONObjBuilder().genOID().appendBinData(fileName, buf.size(), mongo::BinDataGeneral, &buf[0]).append("filename", fileName).append("DocumentType", "file").append("size", sizeBytes).append("place", "document").append("fileType", fileType).obj();
	BSONElement bsonElement;
	b.getObjectID(bsonElement);
	oid=bsonElement.__oid();
	basePtr->insert(b);
}
void PrimitiveFile::copyXYZPointToFloatArray (const pcl::PointXYZ &p, float * out) const
{
	out[0] = p.x;	// 4 bytes
	out[1] = p.y;	// 4 bytes
	out[2] = p.z;	// 4 bytes
}

void PrimitiveFile::copyXYZRGBPointToFloatArray (const pcl::PointXYZRGB &p, float * out) const
{
	out[0] = p.x;	// 4 bytes
	out[1] = p.y;	// 4 bytes
	out[2] = p.z;	// 4 bytes
	out[3] = p.rgb;	// 4 bytes
}

void PrimitiveFile::copyXYZSiftPointToFloatArray (const PointXYZSIFT &p, float * out) const
{
	Eigen::Vector3f outCoordinates = p.getArray3fMap();
	out[0] = outCoordinates[0];	// 4 bytes
	out[1] = outCoordinates[1];	// 4 bytes
	out[2] = outCoordinates[2];	// 4 bytes
	memcpy(&out[3], &p.multiplicity, sizeof(int)); // 4 bytes
	memcpy(&out[4], &p.pointId, sizeof(int));	// 4 bytes
	memcpy(&out[5], &p.descriptor, 128*sizeof(float)); // 128 * 4 bytes = 512 bytes
}

//TODO check this method
void PrimitiveFile::copyXYZRGBNormalPointToFloatArray (const pcl::PointXYZRGBNormal &p, float * out) const
{
	out[0] = p.x;	// 4 bytes
	out[1] = p.y;	// 4 bytes
	out[2] = p.z;	// 4 bytes
	out[3] = p.data[3];	// 4 bytes
	/*
	out[4] = p.normal_x;	// 4 bytes
	out[5] = p.normal_y;	// 4 bytes
	out[6] = p.normal_z;	// 4 bytes
	out[7] = p.data_n[3];	// 4 bytes
	out[8] = p.curvature;	// 4 bytes
	out[9] = p.rgba;	// 4 bytes
	*/
}

void PrimitiveFile::copyXYZSHOTPointToFloatArray (const PointXYZSHOT &p, float * out) const
{
	Eigen::Vector3f outCoordinates = p.getArray3fMap();
	out[0] = outCoordinates[0];	// 4 bytes
	out[1] = outCoordinates[1];	// 4 bytes
	out[2] = outCoordinates[2];	// 4 bytes
	memcpy(&out[3], &p.descriptor, 352*sizeof(float)); // 352 * 4 bytes = 1408 bytes
	memcpy(&out[4], &p.rf, 9*sizeof(float));//9*4=36 bytes
	memcpy(&out[5], &p.multiplicity, sizeof(int)); // 4 bytes
	memcpy(&out[6], &p.pointId, sizeof(int));	// 4 bytes

}

void PrimitiveFile::insertFileIntoDocument(OID& oid)
{
	// use variant here...
	LOG(LNOTICE)<<"PrimitiveFile::insertFileIntoCollection";
	BSONObjBuilder builder;
	BSONObj b;
	BSONElement bsonElement;

	switch(fileType)
	{
		case xml:
		{
			LOG(LNOTICE)<<"PrimitiveFile::insertFileIntoCollection, xml file";
			string* input =  boost::get<std::string>(&data);
			LOG(LNOTICE)<<"Input:"<< input;
			// convert to char*
			char const *cipCharTable = input->c_str();
			LOG(LNOTICE)<<string(cipCharTable);
			// create bson object
			b = BSONObjBuilder().genOID().appendBinData(fileName, sizeBytes, BinDataGeneral,  cipCharTable).append("filename", fileName).append("DocumentType", "file").append("size", sizeBytes).append("place", "document").append("fileType", fileType).obj();

			// insert object into collection
			basePtr->insert(b);

			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			break;
		}
		case rgb:
		{
			saveImage(oid);
			break;
		}
		case density:
		{
			saveImage(oid);
			break;
		}
		case intensity:
		{
			saveImage(oid);
			break;
		}
		case mask:
		{
			saveImage(oid);
			break;
		}
		case stereoL:
		{
			saveImage(oid);
			break;
		}
		case stereoR:
		{
			saveImage(oid);
			break;
		}
		case stereoLTextured:
		{
			saveImage(oid);
			break;
		}
		case stereoRTextured:
		{
			saveImage(oid);
			break;
		}
		case pc_xyz:
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ =  boost::get<pcl::PointCloud<pcl::PointXYZ>::Ptr >(data);
			int cloudSize = cloudXYZ->size();
			int fieldsNr = xyzPointSize*cloudSize;
			//int totalSize = fieldsNr*sizeof(float);
			float buff[fieldsNr];

			// copy all points to memory
			for(int iter=0; iter<cloudSize; iter++)
			{
				const pcl::PointXYZ p = cloudXYZ->points[iter];
				copyXYZPointToFloatArray (p, &buff[xyzPointSize*iter]);
			}
			b=BSONObjBuilder().genOID().appendBinData(fileName, sizeBytes, mongo::BinDataGeneral, &buff[0]).append("filename", fileName).append("DocumentType", "file").append("size", sizeBytes).append("place", "document").append("fileType", fileType).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			basePtr->insert(b);
			break;
		}
		case pc_xyzrgb:
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB =  boost::get<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(data);
			int cloudSize = cloudXYZRGB->size();
			int fieldsNr = xyzrgbPointSize*cloudSize;
			//int totalSize = fieldsNr*sizeof(float);
			float buff[fieldsNr];

			// copy all points to memory
			for(int iter=0; iter<cloudSize; iter++)
			{
				const pcl::PointXYZRGB p = cloudXYZRGB->points[iter];
				copyXYZRGBPointToFloatArray (p, &buff[xyzrgbPointSize*iter]);
			}
			b=BSONObjBuilder().genOID().appendBinData(fileName, sizeBytes, mongo::BinDataGeneral, &buff[0]).append("filename", fileName).append("DocumentType", "file").append("size", sizeBytes).append("place", "document").append("fileType", fileType).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			basePtr->insert(b);
			break;
		}
		case pc_xyzsift:
		{
			pcl::PointCloud<PointXYZSIFT>::Ptr cloudXYZSIFT = boost::get<pcl::PointCloud<PointXYZSIFT>::Ptr>(data);

			int cloudSize = cloudXYZSIFT->size();
			int fieldsNr = siftPointSize*cloudSize;
			//int totalSize = fieldsNr*sizeof(float);
			float buff[fieldsNr];

			// copy all points to memory
			for(int iter=0; iter<cloudSize; iter++)
			{
				const PointXYZSIFT p = cloudXYZSIFT->points[iter];
				copyXYZSiftPointToFloatArray (p, &buff[siftPointSize*iter]);
			}
			b=BSONObjBuilder().genOID().appendBinData(fileName, sizeBytes, mongo::BinDataGeneral, &buff[0]).append("filename", fileName).append("DocumentType", "file").append("size", sizeBytes).append("place", "document").append("fileType", fileType).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			basePtr->insert(b);
			break;
		}
		case pc_xyzrgbsift:
		{
			LOG(LNOTICE)<<"ERROR: I don't know what to do with pc_xyzrgbsift";
			pcl::PointCloud<PointXYZRGBSIFT>::Ptr cloudXYZRGBSIFT = boost::get<pcl::PointCloud<PointXYZRGBSIFT>::Ptr>(data);;
			break;
		}
		case pc_xyzshot:
		{
			LOG(LNOTICE)<<"ERROR: I don't know what to do with pc_xyzshot";
			break;
		}
		case pc_xyzrgbnormal:
		{
			LOG(LNOTICE)<<"ERROR: I don't know what to do with pc_xyzrgbnormal";
			break;
		}
		case xyz:
		{
			cv::Mat* xyzimage = boost::get<cv::Mat>(&data);
			//int bufSize = xyzimage.total()*xyzimage.channels()*4;
			int width = xyzimage->size().width;
			int height = xyzimage->size().height;
			int channels = xyzimage->channels();
			float* buf;
			buf = (float*)xyzimage->data;
			LOG(LNOTICE)<<"Set buffer YAML";
			b=BSONObjBuilder().genOID().appendBinData(fileName, sizeBytes, mongo::BinDataGeneral, &buf[0]).append("filename", fileName).append("DocumentType", "file").append("size", sizeBytes).append("place", "document").append("fileType", fileType).append("width", width).append("height", height).append("channels", channels).obj();
			LOG(LNOTICE)<<"YAML inserted";
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			basePtr->insert(b);
			break;
		}
	}
}

void PrimitiveFile::savePCxyzRGBNormalFileToDisc(bool suffix, bool binary, std::string& fn)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudXYZNormal = boost::get<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>(data);
	if(suffix)
	{
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyzrgbNormal.pcd");
	}
	LOG(LNOTICE) <<"FileName:"<<fn;
	pcl::io::savePCDFile(fn, *cloudXYZNormal, binary);
}


void PrimitiveFile::savePCxyzSHOTFileToDisc(bool suffix, bool binary, std::string& fn)
{
	pcl::PointCloud<PointXYZSHOT>::Ptr cloudXYZSHOT = boost::get<pcl::PointCloud<PointXYZSHOT>::Ptr>(data);
	if(suffix)
	{
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyzrgbshot.pcd");
	}
	LOG(LNOTICE) <<"FileName:"<<fn;
	pcl::io::savePCDFile(fn, *cloudXYZSHOT, binary);
}

void PrimitiveFile::savePCxyzFileToDisc(bool suffix, bool binary, std::string& fn)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ = boost::get<pcl::PointCloud<pcl::PointXYZ>::Ptr>(data);
	if(suffix)
	{
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyz.pcd");
	}
	LOG(LNOTICE) <<"FileName:"<<fn;
	pcl::io::savePCDFile(fn, *cloudXYZ, binary);
}

void PrimitiveFile::savePCxyzRGBFileToDisc(bool suffix, bool binary, std::string& fn)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB = boost::get<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(data);
	if(suffix)
	{
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyzrgb.pcd");
	}
	LOG(LNOTICE) <<"FileName:"<<fn;
	pcl::io::savePCDFile(fn, *cloudXYZRGB, binary);
}

void PrimitiveFile::savePCxyzSIFTFileToDisc(bool suffix, bool binary, std::string& fn)
{
	pcl::PointCloud<PointXYZSIFT>::Ptr cloudXYZSIFT = boost::get<pcl::PointCloud<PointXYZSIFT>::Ptr>(data);
	if(suffix)
	{
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyzsift.pcd");
	}
	LOG(LNOTICE) <<"FileName:"<<fn;
	pcl::io::savePCDFile(fn, *cloudXYZSIFT, binary);
}

void PrimitiveFile::saveImageOnDisc()
{
	LOG(LTRACE)<<"Save image to disc";
	cv::Mat* img = boost::get<cv::Mat>(&data);
	cv::imwrite(fileName, *img);
}

void PrimitiveFile::saveToDisc(bool suffix, bool binary)
{
	switch(fileType)
	{
		case xml:
		{
			std::string* cameraMatrix = boost::get<std::string>(&data);
			char const* ca = fileName.c_str();
			std::ofstream out(ca);
			out<<*cameraMatrix;
			out.close();
			break;
		}
		case rgb:
		{
			LOG(LTRACE)<<"RGB";
			saveImageOnDisc();
			break;
		}
		case density:
		{
			saveImageOnDisc();
			break;
		}
		case intensity:
		{
			saveImageOnDisc();
			break;
		}
		case mask:
		{
			saveImageOnDisc();
			break;
		}
		case stereoL:
		{
			saveImageOnDisc();
			break;
		}
		case stereoR:
		{
			saveImageOnDisc();
			break;
		}
		case stereoLTextured:
		{
			saveImageOnDisc();
			break;
		}
		case stereoRTextured:
		{
			saveImageOnDisc();
			break;
		}
		case pc_xyz:
		{
			savePCxyzFileToDisc(suffix, binary, fileName);
			break;
		}
		case pc_xyzrgb:
		{
			savePCxyzRGBFileToDisc(suffix, binary, fileName);
			break;
		}
		case pc_xyzsift:
		{
			savePCxyzSIFTFileToDisc(suffix, binary, fileName);
			break;
		}
		case pc_xyzrgbsift:
		{
			//TODO add method !!!
			//savePCxyzRGBSIFTFileToDisc(suffix, binary, fileName);
			break;
		}
		case pc_xyzshot:
		{
			savePCxyzSHOTFileToDisc(suffix, binary, fileName);
			break;
		}
		case pc_xyzrgbnormal:
		{
			savePCxyzRGBNormalFileToDisc(suffix, binary, fileName);
			break;
		}
		case xyz:
		{
			cv::Mat* xyzimage = boost::get<cv::Mat>(&data);
			cv::FileStorage fs(fileName, cv::FileStorage::WRITE);
			fs << "img" << *xyzimage;
			fs.release();
			break;
		}
		default:
		{
			LOG(LNOTICE)<<"I don't know such file type :( \nBye!";
			exit(1);
		}
	}
}

void PrimitiveFile::insertFileIntoGrid(OID& oid)
{
	LOG(LNOTICE)<<"Writting to GRIDFS!";
	try{
		BSONObj object;
		BSONElement bsonElement;
		string mime="";
		basePtr->setMime(fileType, mime);
		std::stringstream time;
		bool suffix = true;
		bool binary = false;
		saveToDisc(suffix, binary);

		boost::shared_ptr<DBClientConnection>  c = basePtr->getClient();
		// create GridFS client
		GridFS fs(*c, basePtr->collectionName);

		// save in grid
		object = fs.storeFile(fileName, fileName, mime);

		//	if(cloudType=="xyzsift")
		//		b = BSONObjBuilder().appendElements(object).append("ObjectName", objectName).append("size", totalSize).append("place", "grid").append("mean_viewpoint_features_number", mean_viewpoint_features_number).obj();
		//	else
		//		b = BSONObjBuilder().appendElements(object).append("ObjectName", objectName).append("size", totalSize).append("place", "grid").obj();
		BSONObj b;
		if(viewName!="")
			b = BSONObjBuilder().appendElements(object).append("ViewName", viewName).append("FileType", fileType).append("DocumentType", "file").append("size", sizeBytes).append("place", "grid").obj();
		else if(modelName!="")
			b = BSONObjBuilder().appendElements(object).append("modelName", modelName).append("FileType", fileType).append("DocumentType", "file").append("size", sizeBytes).append("place", "grid").obj();

		basePtr->insert(b);
		b.getObjectID(bsonElement);
		oid=bsonElement.__oid();
		string field = "filename";
		basePtr->index(field);
	}catch(DBException &e)
	{
		LOG(LNOTICE) <<"Something goes wrong... :<\nBye!";
		//LOG(LNOTICE) <<c->getLastError();
		LOG(LNOTICE) << e.what();
	}
}

}//namespace File
}//MongoBase


#endif /* PRIMITIVEFILE_HPP_ */
