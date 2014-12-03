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
#include <Types/MongoProxy.hpp>

#include <vector>
#include <list>
#include <iostream>
#include <boost/variant.hpp>
#include <boost/lexical_cast.hpp>

#define siftPointSize 133
#define xyzPointSize 3
#define xyzrgbPointSize 4

enum fileTypes {ImageRgb, FileCameraInfo, ImageXyz,  ImageDepth, ImageIntensity, ImageMask, StereoLeft, StereoRight, StereoLeftTextured, StereoRightTextured,
PCXyz, PCXyzRgb, PCXyzSift, PCXyzRgbSift, PCXyzShot, PCXyzRgbNormal, Stereo, StereoTextured};

const char* FTypes[] = {"ImageRgb", "FileCameraInfo", "ImageXyz",  "ImageDepth", "ImageIntensity", "ImageMask", "StereoLeft", "StereoRight", "StereoLeftTextured", "StereoRightTextured",
"PCXyz", "PCXyzRgb", "PCXyzSift", "PCXyzRgbSift", "PCXyzShot", "PCXyzRgbNormal", "Stereo", "StereoTextured"};


namespace MongoDB{
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
	//string mongoFileName;	// file name in mongo base
	string fileName;
	string pcdCloudType;	// sift, shot, xyz,  itd... itp...
	string place;			// {grid, document}
	float sizeBytes;
	float sizeMBytes;
	fileTypes fileType; 		// mask, rgb, depth, I, XYZ, cameraInfo, PCL
	string PCLType;			// SIFT, SHOT, XYZ, ORB, NORMALS
	//TODO remove view and model name!!!
	string viewName;
	string modelName;
	string collectionName;
	string documentType;	// View, Model, Object, file
	OID fileOID;
	BSONObj fileDocument;
	boost::variant<	cv::Mat,
	string,
	pcl::PointCloud<pcl::PointXYZ>::Ptr,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
	pcl::PointCloud<PointXYZSIFT>::Ptr,
	pcl::PointCloud<PointXYZRGBSIFT>::Ptr,
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr,
	pcl::PointCloud<PointXYZSHOT>::Ptr
	> data;
	string hostname;

//	boost::shared_ptr<MongoBase> basePtr;

public:
	PrimitiveFile(const cv::Mat& img, fileTypes& type, string& name, string& viewName, string& hostname) : data(img), fileType(type), fileName(name), viewName(viewName), hostname(hostname), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor cv::Mat";
		LOG(LNOTICE)<<"fileType :" <<fileType;
		this->setSize();
	};
	PrimitiveFile(const std::string& str, fileTypes& type, string& name, string& viewName, string& hostname) : data(str), fileType(type), fileName(name), viewName(viewName), hostname(hostname), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor std::string";
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,  fileTypes& type, string& name, string& viewName, string& hostname) : data(cloud), fileType(type), fileName(name), viewName(viewName), hostname(hostname), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor <pcl::PointXYZ>";
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, fileTypes& type, string& name, string& viewName, string& hostname) : data(cloud), fileType(type), fileName(name), viewName(viewName), hostname(hostname), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor <pcl::PointXYZRGB>";
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<PointXYZSIFT>::Ptr& cloud, fileTypes& type, string& name, string& viewName, string& hostname) : data(cloud), fileType(type), fileName(name), viewName(viewName), hostname(hostname), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor <PointXYZSIFT>";
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<PointXYZRGBSIFT>::Ptr& cloud, fileTypes& type, string& name, string& viewName, string& hostname) : data(cloud), fileType(type), fileName(name), viewName(viewName), hostname(hostname), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor <PointXYZRGBSIFT>";
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<PointXYZSHOT>::Ptr& cloud, fileTypes& type, string& name, string& viewName, string& hostname) : data(cloud), fileType(type), fileName(name), viewName(viewName), hostname(hostname), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor <PointXYZSHOT>";
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, fileTypes& type, string& name, string& viewName, string& hostname) : data(cloud), fileType(type), fileName(name), viewName(viewName), hostname(hostname), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor <pcl::PointXYZRGBNormal>";
		this->setSize();
	};

	///////////////////////////////////////////////////
	PrimitiveFile(fileTypes& type, string& hostname, mongo::OID& fileOID) : fileType(type), hostname(hostname), fileOID(fileOID), sizeMBytes(0), fileName("NoName"), sizeBytes(0)
	{
		LOG(LNOTICE)<<"Constructor PrimitiveFile";
		LOG(LNOTICE)<<"fileType :" <<fileType;
	};

	/////////////////////////////////////////////////////////
	string getFileName(){return fileName;}
	void saveIntoDisc();
	void saveIntoMongoBase();
	void insertFileIntoGrid(OID& oid);
	void insertFileIntoDocument(OID& oid);
	void convertToBuffer();
	void getMime();
	void readFromMongoDB();
	void readFromGrid();
	void readFromDocument();
	void saveImageOnDisc();
	void getCVMatData(cv::Mat&);
	void saveToDisc(bool suffix, bool binary);
	void readXYZMatFromDocument();
	void getFileFromGrid(const GridFile& file);
	void writeToSinkFromFile(string& mime);
	void copyXYZPointToFloatArray (const pcl::PointXYZ &p, float * out) const;
	void copyXYZSiftPointToFloatArray (const PointXYZSIFT &p, float * out) const;
	void copyXYZRGBPointToFloatArray (const pcl::PointXYZRGB &p, float * out) const;
	void copyXYZRGBNormalPointToFloatArray (const pcl::PointXYZRGBNormal &p, float * out) const;
	void copyXYZSHOTPointToFloatArray (const PointXYZSHOT &p, float * out) const;
	void readPointCloudFromDocument();
	void savePCxyzFileToDisc(bool suffix, bool binary, std::string& fn);
	void savePCxyzRGBFileToDisc(bool suffix, bool binary, std::string& fn);
	void savePCxyzSIFTFileToDisc(bool suffix, bool binary, std::string& fn);
	void savePCxyzRGBNormalFileToDisc(bool suffix, bool binary, std::string& fn);
	void savePCxyzSHOTFileToDisc(bool suffix, bool binary, std::string& fn);
	void setMime(const fileTypes type,  string& mime);
	void readTextFileFromDocument();
	void readFile();
	void readImageFromDocument();
	void ReadPCDCloud(const string& filename);
	void getStringData(std::string& str);
	void getXYZData(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudXYZ);
	void getXYZRGBData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudXYZRGB);
	void getXYZSIFTData(pcl::PointCloud<PointXYZSIFT>::Ptr& cloudXYZSIFT);
	void getXYZRGBSIFTData(pcl::PointCloud<PointXYZRGBSIFT>::Ptr& cloudXYZRGBSIFT);
	void getXYZSHOTData(pcl::PointCloud<PointXYZSHOT>::Ptr& cloudXYZSHOT);
	void getXYZRGBNormalData(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloudXYZNormal);

	void setViewName(string& ViewName)
	{
		this->viewName = ViewName;
	};

	void setModelName(string& ModelName)
	{
		this->modelName = ModelName;
	};

	void setType(fileTypes type)
	{
		fileType = type;
	};

	fileTypes getType()
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

void PrimitiveFile::readTextFileFromDocument()
{
	LOG(LERROR)<<"Read text\n";

	const char *buffer;
	int size = (int)sizeBytes;
	// get data to buffer
	buffer = fileDocument[fileName].binData(size);
	for (int i=0; i<size;i++)
		LOG(LERROR)<<buffer[i];
	LOG(LERROR)<<*buffer;
	LOG(LERROR)<<"size: "<<size;
	string fromDB(buffer,size-1);
	LOG(LERROR)<<"ReadedFile: \n"<<fromDB;
	LOG(LERROR)<<"Save to sink";
	data = fromDB;
	//cipFileOut.write(fromDB);
}

void PrimitiveFile::readXYZMatFromDocument()
{
	LOG(LERROR)<<"Read XYZ Mat\n";
	int len;
	float *dataXYZ = (float*)fileDocument[fileName].binData(len);
	LOG(LNOTICE)<<"len : "<<len;
	int width = fileDocument.getField("width").Int();//480
	int height = fileDocument.getField("height").Int();//640
	int channels = fileDocument.getField("channels").Int();
	// if(channels==3)
	//TODO dodać jeszcze inne typy CV_32FC4 na przyklad
	cv::Mat imageXYZRGB(width, height, CV_32FC3);
	//imageXYZRGB.convertTo(imageXYZRGB, CV_32FC4);
	vector<float> img;
	//rows = rows*3;
	float img_ptr[channels];
	for (int i = 0; i < width; ++i)
	{
		LOG(LERROR)<<"i : "<<i;
		float* xyz_p = imageXYZRGB.ptr <float> (i);
		for (int j = 0; j < height*channels; j+=channels)
		{
			//TODO dodać jeszcze kilka wierszy w zależności od tego jaki plik czytamy
			//TODO teraz wywali się dla XYZ-RGB
			LOG(LERROR)<<"i*height*channels+j: " <<i*height*channels+j;
			xyz_p[0+j]=dataXYZ[i*height*channels+j];
			xyz_p[1+j]=dataXYZ[i*height*channels+j+1];
			xyz_p[2+j]=dataXYZ[i*height*channels+j+2];
		}
	}
	data = imageXYZRGB;
	cv::FileStorage fs("xyzTest.yaml", cv::FileStorage::WRITE);
	fs << "img" << imageXYZRGB;

	fs.release();
	//out_yaml.write(imageXYZRGB);
}


void PrimitiveFile::saveIntoMongoBase()
{
	OID oid;
//	if(sizeMBytes<15)
		insertFileIntoDocument(oid);
//	else if(sizeMBytes>=15)
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
	MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);

	update = BSON("$addToSet"<<BSON("FileTypes"<<BSON("Type"<<FTypes[fileType])));
	MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);
}

void PrimitiveFile::readImageFromDocument()
{
	LOG(LERROR)<<"Read image\n";
	int len;
	uchar *dataImage = (uchar*)fileDocument[fileName].binData(len);
	std::vector<uchar> v(dataImage, dataImage+len);
	cv::Mat image = cv::imdecode(cv::Mat(v), -1);
	//out_img.write(image);
	data = image;
	// only in test purposes, it's to remove
	imwrite(fileName, image );
}

void PrimitiveFile::readFile()
{
	LOG(LTRACE)<<"MongoProxy::readFile";
	// get bson object from collection
	BSONObj query = BSON("_id" << fileOID);
	BSONObj obj = MongoProxy::MongoProxy::getSingleton(hostname).findOne(query);
	LOG(LNOTICE)<<"obj: "<<obj<<", fileOID: "<<fileOID.toString();
	place = obj.getField("place").str();

	sizeBytes = obj.getField("size").Double();
	fileName = obj.getField("filename").str();
	LOG(LNOTICE)<<"place: "<<place<<" size: "<<sizeBytes<<", fileName: "<<fileName<< ", fileType: " << fileType;

	if(place=="document")
	{
		LOG(LNOTICE)<<"Read from document";

		BSONObj query = BSON("_id" << fileOID);
		fileDocument = MongoProxy::MongoProxy::getSingleton(hostname).findOne(query);

		if(fileType==ImageRgb || fileType==ImageDepth || fileType==ImageIntensity || fileType==ImageMask
				|| fileType==StereoLeft || fileType==StereoRight || fileType==StereoLeftTextured
				|| fileType==StereoRightTextured)
		{
			readImageFromDocument();
		}

		else if(fileType==PCXyz || fileType==PCXyzRgb || fileType==PCXyzSift || fileType==PCXyzRgbSift
				|| fileType==PCXyzShot || fileType==PCXyzRgbNormal)
		{
			readPointCloudFromDocument();
		}
		else if(fileType==ImageXyz)
		{
			readXYZMatFromDocument();
		}
		else if(fileType==FileCameraInfo)
		{
			readTextFileFromDocument();
		}
	}
	else
	{
		//TODO add to readed file to data in writeToSinkFromFile method!!!

		LOG(LNOTICE)<<"Read from GRIDFS!";
		// if saved in grid
		boost::shared_ptr<DBClientConnection>  c = MongoProxy::MongoProxy::getSingleton(hostname).getClient();
		// create GridFS client
		GridFS fs(*c, MongoProxy::MongoProxy::getSingleton(hostname).collectionName);

		LOG(LTRACE)<<"_id"<<fileOID;
		GridFile file = fs.findFile(Query(BSON("_id" << fileOID)));

		if (!file.exists())
		{
			LOG(LERROR) << "File not found in grid";
		}
		else
		{
			// get filename
			string filename = file.getFileField("filename").str();
			// get mime from file
			string mime = file.getContentType();

			getFileFromGrid(file);
			writeToSinkFromFile(mime);
		}
	}
}

void PrimitiveFile::ReadPCDCloud(const string& filename)
{
	//TODO ADD ALL PCD TYPES!!!
	LOG(LTRACE) << "ViewReader::ReadPCDCloud";
	// Try to read the cloud of XYZRGB points.
	if(filename.find("xyzrgb")!=string::npos)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fileName, *cloud_xyzrgb) == -1){
			LOG(LWARNING) <<"Cannot read PointXYZRGB cloud from "<<fileName;
			return;
		}else{
		//	out_cloud_xyzrgb.write(cloud_xyzrgb);
			data = cloud_xyzrgb;
			LOG(LINFO) <<"PointXYZRGB cloud loaded properly from "<<fileName;
		}
	}
	if(filename.find("xyzsift.pcd")!=string::npos)
	{
		pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift (new pcl::PointCloud<PointXYZSIFT>);
		if (pcl::io::loadPCDFile<PointXYZSIFT> (fileName, *cloud_xyzsift) == -1){
			LOG(LWARNING) <<"Cannot read PointXYZSIFT cloud from "<<fileName;
			return;
		}else{
			data = cloud_xyzsift;
		//	out_cloud_xyzsift.write(cloud_xyzsift);
			LOG(LINFO) <<"PointXYZSIFT cloud loaded properly from "<<fileName;
		}
	}

	else if(filename.find("xyz")!=string::npos)
	{
		// Try to read the cloud of XYZ points.
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud_xyz) == -1){
			LOG(LWARNING) <<"Cannot read PointXYZ cloud from "<<fileName;
			return;
		}else{
			data = cloud_xyz;
		//	out_cloud_xyz.write(cloud_xyz);
			LOG(LINFO) <<"PointXYZ cloud loaded properly from "<<fileName;
		}
	}
}

void PrimitiveFile::writeToSinkFromFile(string& mime)
{
	LOG(LNOTICE)<<"PrimitiveFile::writeToSink";
	if(fileType==ImageRgb || fileType==ImageDepth || fileType==ImageIntensity || fileType==ImageMask
		|| fileType==StereoLeft || fileType==StereoRight || fileType==StereoLeftTextured
		|| fileType==StereoRightTextured)
	{
		cv::Mat image = imread(fileName, CV_LOAD_IMAGE_UNCHANGED);
		data = image;
		//out_img.write(image);
	}
	else if(fileType==PCXyz || fileType==PCXyzRgb || fileType==PCXyzSift || fileType==PCXyzRgbSift
				|| fileType==PCXyzShot || fileType==PCXyzRgbNormal)
	{
		ReadPCDCloud(fileName);

	}
	else if(fileType==ImageXyz)
	{
		FileStorage fs2(fileName, FileStorage::READ);
		cv::Mat imageXYZ;
		fs2["img"] >> imageXYZ;
		data = imageXYZ;
	}
	else if(fileType==FileCameraInfo)
	{
		LOG(LINFO)<<"xml :)";
		string CIPFile;
		char const* charFileName = fileName.c_str();
		LOG(LINFO)<<fileName;
		std::ifstream t(charFileName);
		std::stringstream buffer;
		buffer << t.rdbuf();
		CIPFile = buffer.str();
	//	cipFileOut.write(CIPFile);
		data = CIPFile;
		LOG(LINFO)<<CIPFile;
	}
}

void PrimitiveFile::getFileFromGrid(const GridFile& file)
{
	LOG(LTRACE)<<"MongoProxy::getFileFromGrid";
	stringstream ss;
	string str = ss.str();
	char *tempFilename = (char*)fileName.c_str();
	LOG(LNOTICE)<<"\n\ntempFilename: "<<tempFilename<<"\n";
	ofstream ofs(tempFilename);
	gridfs_offset off = file.write(ofs);
	if (off != file.getContentLength())
	{
		LOG(LNOTICE) << "\nFailed to read a file from mongoDB\n";
	}
	else
	{
		LOG(LNOTICE) << "\nSuccess read a file from mongoDB\n";
	}
}

void PrimitiveFile::readPointCloudFromDocument()
{
	LOG(LNOTICE)<<"Read Point Cloud";
	try
	{
		if(fileType==PCXyz)
		{
			// read data to buffer
			int totalSize;
			float* buffer = (float*)fileDocument[fileName].binData(totalSize);
			int bufferSize = totalSize/sizeof(float);
			LOG(LERROR)<<"bufferSize: "<<bufferSize;
			float newBuffer[bufferSize];
			memcpy(newBuffer, buffer, totalSize);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointXYZ pt;
			for(int i=0; i<totalSize/(xyzPointSize*sizeof(float)); i++) // now it should be 10 iterations
			{
				pt.x=newBuffer[i*xyzPointSize];
				pt.y=newBuffer[i*xyzPointSize+1];
				pt.z=newBuffer[i*xyzPointSize+2];
				cloudXYZ->push_back(pt);
			}
			data = cloudXYZ;
			// save to file, only in test purposes
			pcl::io::savePCDFile("newCloudXYZ.pcd", *cloudXYZ, false);
		}
		else if(fileType==PCXyzRgb)
		{
			// read data to buffer
			int totalSize;
			float* buffer = (float*)fileDocument[fileName].binData(totalSize);
			int bufferSize = totalSize/sizeof(float);
			LOG(LERROR)<<"bufferSize: "<<bufferSize;
			float newBuffer[bufferSize];
			memcpy(newBuffer, buffer, totalSize);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointXYZRGB pt;
			for(int i=0; i<totalSize/(xyzrgbPointSize*sizeof(float)); i++) // now it should be 10 iterations
			{
				pt.x=newBuffer[i*xyzrgbPointSize];
				pt.y=newBuffer[i*xyzrgbPointSize+1];
				pt.z=newBuffer[i*xyzrgbPointSize+2];
				pt.rgb=newBuffer[i*xyzrgbPointSize+3];
				cloudXYZRGB->push_back(pt);
			}
			data = cloudXYZRGB;
			// save to file, only in test purposes
			pcl::io::savePCDFile("newCloudXYZRGB.pcd", *cloudXYZRGB, false);
		}
		else if(fileType==PCXyzSift)
		{
			// read data to buffer
			int totalSize;
			float* buffer = (float*)fileDocument[fileName].binData(totalSize);
			int bufferSize = totalSize/sizeof(float);
			LOG(LERROR)<<"bufferSize: "<<bufferSize;
			float newBuffer[bufferSize];
			memcpy(newBuffer, buffer, totalSize);
			// for sift row size in float is equal 128+5 =133  floats
			pcl::PointCloud<PointXYZSIFT>::Ptr cloudXYZSIFT (new pcl::PointCloud<PointXYZSIFT>);
			PointXYZSIFT pt;
			for(int i=0; i<totalSize/(siftPointSize*sizeof(float)); i++) // now it should be 10 iterations
			{
				Eigen::Vector3f pointCoordinates;
				pointCoordinates[0]=newBuffer[i*siftPointSize];
				pointCoordinates[1]=newBuffer[i*siftPointSize+1];
				pointCoordinates[2]=newBuffer[i*siftPointSize+2];
				memcpy(&pt.multiplicity, &newBuffer[3+i*siftPointSize], sizeof(int)); // 4 bytes
				memcpy(&pt.pointId, &newBuffer[4+i*siftPointSize], sizeof(int));	// 4 bytes
				memcpy(&pt.descriptor, &newBuffer[5+i*siftPointSize], 128*sizeof(float)); // 128 * 4 bytes = 512 bytes

				pt.getVector3fMap() = pointCoordinates;
				cloudXYZSIFT->push_back(pt);
			}
			data = cloudXYZSIFT;
			// save to file, only in test purposes
			pcl::io::savePCDFile("newCloudSIFT.pcd", *cloudXYZSIFT, false);
		}
		else if(fileType==PCXyzRgbSift)
		{
		}
		else if(fileType==PCXyzShot)
		{
		}
		else if(fileType==PCXyzRgbNormal)
		{
		}
		LOG(LERROR)<<"ViewWriter::insertFileIntoCollection: END";
	}catch(Exception &ex)
	{
		LOG(LERROR)<<ex.what();
	}
}


void PrimitiveFile::setMime( const fileTypes fileType,  string& mime)
{
	LOG(LNOTICE)<<"fileType : "<<fileType<<std::endl;

	if (fileType==ImageRgb || fileType==ImageDepth || fileType==ImageIntensity || fileType==ImageMask || fileType==StereoLeft || fileType==StereoRight || fileType==StereoLeftTextured || fileType==StereoRightTextured)
		mime="image/png";
	else if(fileType==FileCameraInfo || fileType==PCXyz || fileType==PCXyzRgb || fileType==PCXyzSift || fileType==PCXyzRgbSift || fileType==PCXyzShot ||fileType==PCXyzRgbNormal)
		mime="text/plain";
	else if(fileType==ImageXyz )
			mime="text/plain";
	else
	{
		LOG(LNOTICE) <<"I don't know such typeFile! Please add extension to the `if` statement from http://www.sitepoint.com/web-foundations/mime-types-complete-list/";
		return;
	}
}

void PrimitiveFile::getCVMatData(cv::Mat& img)
{
	img =  boost::get<cv::Mat>(data);
}

void PrimitiveFile::getStringData(std::string& str)
{
	str =  boost::get<std::string>(data);
}

void PrimitiveFile::getXYZData(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudXYZ)
{
	cloudXYZ = boost::get<pcl::PointCloud<pcl::PointXYZ>::Ptr>(data);
}

void PrimitiveFile::getXYZRGBData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudXYZRGB)
{
	cloudXYZRGB = boost::get<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(data);
}

void PrimitiveFile::getXYZSIFTData(pcl::PointCloud<PointXYZSIFT>::Ptr& cloudXYZSIFT)
{
	cloudXYZSIFT = boost::get<pcl::PointCloud<PointXYZSIFT>::Ptr>(data);
}

void PrimitiveFile::getXYZRGBSIFTData(pcl::PointCloud<PointXYZRGBSIFT>::Ptr& cloudXYZRGBSIFT)
{
	cloudXYZRGBSIFT = boost::get<pcl::PointCloud<PointXYZRGBSIFT>::Ptr>(data);
}

void PrimitiveFile::getXYZSHOTData(pcl::PointCloud<PointXYZSHOT>::Ptr& cloudXYZSHOT)
{
	cloudXYZSHOT = boost::get<pcl::PointCloud<PointXYZSHOT>::Ptr>(data);
}

void PrimitiveFile::getXYZRGBNormalData(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloudXYZNormal)
{
	cloudXYZNormal = boost::get<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>(data);
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
	BSONObj b=BSONObjBuilder().genOID().appendBinData(fileName, buf.size(), mongo::BinDataGeneral, &buf[0]).append("filename", fileName).append("DocumentType", "file").append("size", sizeBytes).append("place", "document").append("fileType", FTypes[fileType]).obj();
	BSONElement bsonElement;
	b.getObjectID(bsonElement);
	oid=bsonElement.__oid();
	MongoProxy::MongoProxy::getSingleton(hostname).insert(b);
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
		case FileCameraInfo:
		{
			LOG(LNOTICE)<<"PrimitiveFile::insertFileIntoCollection, xml file";
			string* input =  boost::get<std::string>(&data);
			LOG(LNOTICE)<<"Input:"<< input;
			// convert to char*
			char const *cipCharTable = input->c_str();
			LOG(LNOTICE)<<string(cipCharTable);
			// create bson object
			b = BSONObjBuilder().genOID().appendBinData(fileName, sizeBytes, BinDataGeneral,  cipCharTable).append("filename", fileName).append("DocumentType", "file").append("size", sizeBytes).append("place", "document").append("fileType", FTypes[fileType]).obj();

			// insert object into collection
			MongoProxy::MongoProxy::getSingleton(hostname).insert(b);

			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			break;
		}
		case ImageRgb:
		{
			saveImage(oid);
			break;
		}
		case ImageDepth:
		{
			saveImage(oid);
			break;
		}
		case ImageIntensity:
		{
			saveImage(oid);
			break;
		}
		case ImageMask:
		{
			saveImage(oid);
			break;
		}
		case StereoLeft:
		{
			saveImage(oid);
			break;
		}
		case StereoRight:
		{
			saveImage(oid);
			break;
		}
		case StereoLeftTextured:
		{
			saveImage(oid);
			break;
		}
		case StereoRightTextured:
		{
			saveImage(oid);
			break;
		}
		case PCXyz:
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
			b=BSONObjBuilder().genOID().appendBinData(fileName, sizeBytes, mongo::BinDataGeneral, &buff[0]).append("filename", fileName).append("DocumentType", "file").append("size", sizeBytes).append("place", "document").append("fileType", FTypes[fileType]).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			MongoProxy::MongoProxy::getSingleton(hostname).insert(b);
			break;
		}
		case PCXyzRgb:
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
			b=BSONObjBuilder().genOID().appendBinData(fileName, sizeBytes, mongo::BinDataGeneral, &buff[0]).append("filename", fileName).append("DocumentType", "file").append("size", sizeBytes).append("place", "document").append("fileType", FTypes[fileType]).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			MongoProxy::MongoProxy::getSingleton(hostname).insert(b);
			break;
		}
		case PCXyzSift:
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
			b=BSONObjBuilder().genOID().appendBinData(fileName, sizeBytes, mongo::BinDataGeneral, &buff[0]).append("filename", fileName).append("DocumentType", "file").append("size", sizeBytes).append("place", "document").append("fileType", FTypes[fileType]).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			MongoProxy::MongoProxy::getSingleton(hostname).insert(b);
			break;
		}
		case PCXyzRgbSift:
		{
			LOG(LNOTICE)<<"ERROR: I don't know what to do with pc_xyzrgbsift";
			pcl::PointCloud<PointXYZRGBSIFT>::Ptr cloudXYZRGBSIFT = boost::get<pcl::PointCloud<PointXYZRGBSIFT>::Ptr>(data);;
			break;
		}
		case PCXyzShot:
		{
			LOG(LNOTICE)<<"ERROR: I don't know what to do with pc_xyzshot";
			break;
		}
		case PCXyzRgbNormal:
		{
			LOG(LNOTICE)<<"ERROR: I don't know what to do with pc_xyzrgbnormal";
			break;
		}
		case ImageXyz:
		{
			cv::Mat* xyzimage = boost::get<cv::Mat>(&data);
			//int bufSize = xyzimage.total()*xyzimage.channels()*4;
			int width = xyzimage->size().width;
			int height = xyzimage->size().height;
			int channels = xyzimage->channels();
			float* buf;
			buf = (float*)xyzimage->data;
			LOG(LNOTICE)<<"Set buffer YAML";
			b=BSONObjBuilder().genOID().appendBinData(fileName, sizeBytes, mongo::BinDataGeneral, &buf[0]).append("filename", fileName).append("DocumentType", "file").append("size", sizeBytes).append("place", "document").append("fileType", FTypes[fileType]).append("width", width).append("height", height).append("channels", channels).obj();
			LOG(LNOTICE)<<"YAML inserted";
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			MongoProxy::MongoProxy::getSingleton(hostname).insert(b);
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
		case FileCameraInfo:
		{
			std::string* cameraMatrix = boost::get<std::string>(&data);
			char const* ca = fileName.c_str();
			std::ofstream out(ca);
			out<<*cameraMatrix;
			out.close();
			break;
		}
		case ImageRgb:
		{
			LOG(LTRACE)<<"RGB";
			saveImageOnDisc();
			break;
		}
		case ImageDepth:
		{
			saveImageOnDisc();
			break;
		}
		case ImageIntensity:
		{
			saveImageOnDisc();
			break;
		}
		case ImageMask:
		{
			saveImageOnDisc();
			break;
		}
		case StereoLeft:
		{
			saveImageOnDisc();
			break;
		}
		case StereoRight:
		{
			saveImageOnDisc();
			break;
		}
		case StereoLeftTextured:
		{
			saveImageOnDisc();
			break;
		}
		case StereoRightTextured:
		{
			saveImageOnDisc();
			break;
		}
		case PCXyz:
		{
			savePCxyzFileToDisc(suffix, binary, fileName);
			break;
		}
		case PCXyzRgb:
		{
			savePCxyzRGBFileToDisc(suffix, binary, fileName);
			break;
		}
		case PCXyzSift:
		{
			savePCxyzSIFTFileToDisc(suffix, binary, fileName);
			break;
		}
		case PCXyzRgbSift:
		{
			//TODO add method !!!
			//savePCxyzRGBSIFTFileToDisc(suffix, binary, fileName);
			break;
		}
		case PCXyzShot:
		{
			savePCxyzSHOTFileToDisc(suffix, binary, fileName);
			break;
		}
		case PCXyzRgbNormal:
		{
			savePCxyzRGBNormalFileToDisc(suffix, binary, fileName);
			break;
		}
		case ImageXyz:
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
		setMime(fileType, mime);
		std::stringstream time;
		bool suffix = true;
		bool binary = false;
		saveToDisc(suffix, binary);

		boost::shared_ptr<DBClientConnection>  c = MongoProxy::MongoProxy::getSingleton(hostname).getClient();
		// create GridFS client
		GridFS fs(*c, MongoProxy::MongoProxy::getSingleton(hostname).collectionName);

		// save in grid
		object = fs.storeFile(fileName, fileName, mime);

		//	if(cloudType=="xyzsift")
		//		b = BSONObjBuilder().appendElements(object).append("ObjectName", objectName).append("size", totalSize).append("place", "grid").append("mean_viewpoint_features_number", mean_viewpoint_features_number).obj();
		//	else
		//		b = BSONObjBuilder().appendElements(object).append("ObjectName", objectName).append("size", totalSize).append("place", "grid").obj();
		BSONObj b;
		LOG(LNOTICE)<<"fileType: "<<fileType;
		if(viewName!="")
			b = BSONObjBuilder().appendElements(object).append("ViewName", viewName).append("fileType", FTypes[fileType]).append("DocumentType", "file").append("size", sizeBytes).append("place", "grid").obj();
		else if(modelName!="")
			b = BSONObjBuilder().appendElements(object).append("ModelName", modelName).append("fileType", FTypes[fileType]).append("DocumentType", "file").append("size", sizeBytes).append("place", "grid").obj();

		MongoProxy::MongoProxy::getSingleton(hostname).insert(b);
		b.getObjectID(bsonElement);
		oid=bsonElement.__oid();
		string field = "filename";
		MongoProxy::MongoProxy::getSingleton(hostname).index(field);
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
