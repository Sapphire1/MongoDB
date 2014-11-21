/*
 * MongoBase.hpp
 *
 *  Created on: Sep 23, 2014
 *      Author: lzmuda
 */

#ifndef MONGOBASE_HPP_
#define MONGOBASE_HPP_

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

#include <vector>
#include <list>
#include <iostream>
#include <boost/variant.hpp>
#include <boost/lexical_cast.hpp>

#define siftPointSize 133
#define xyzPointSize 3
#define xyzrgbPointSize 4

namespace MongoBase {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;

enum keyTypes {	xml, xyz, rgb, density, intensity, mask, stereoL, stereoR, stereoLTextured, stereoRTextured,
pc_xyz, pc_xyzrgb, pc_xyzsift, pc_xyzrgbsift, pc_xyzshot, pc_xyzrgbnormal};

class MongoBase {
public:

    boost::shared_ptr<DBClientConnection> c;
	vector<string>  docViewsNames;
	vector<string>  docModelsNames;
	vector<string> splitedSceneNames;
	string dbCollectionPath;

	 /// Input data stream
//	Base::DataStreamIn <cv::Mat> in_img;

	/// Output data stream - processed image
//	Base::DataStreamOut <cv::Mat> out_img;

	/// Matrixes of projection, distortion etc.
//	Base::DataStreamIn <std::string> cipFileIn;

	/// XYZ cv::Mat input
//	Base::DataStreamIn <cv::Mat> in_yaml;

	/// XYZ cv::Mat output
//	Base::DataStreamOut <cv::Mat> out_yaml;

	/// Matrixes of projection, distortion etc.
//	Base::DataStreamOut <std::string> cipFileOut;

	/// Cloud containing points with Cartesian coordinates (XYZ).
//	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr > out_cloud_xyz;

	/// Cloud containing points with Cartesian coordinates and colour (XYZ + RGB).
//	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > out_cloud_xyzrgb;

	/// Cloud containing points with Cartesian coordinates and SIFT descriptor (XYZ + 128).
//	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr > out_cloud_xyzsift;

	/// Cloud containing points with Cartesian coordinates (XYZ).
//	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr > in_cloud_xyz;

	/// Cloud containing points with Cartesian coordinates and colour (XYZ + RGB).
//	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > in_cloud_xyzrgb;

	/// Cloud containing points with Cartesian coordinates and SIFT descriptor (XYZ + 128).
//	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift;

	/// Cloud containing points with Cartesian coordinates, RGB Color and SIFT descriptor (XYZ+ RGB + 128).
//	Base::DataStreamIn<pcl::PointCloud<PointXYZRGBSIFT>::Ptr> in_cloud_xyzrgbsift;

//	Base::DataStreamOut<std::vector<AbstractObject*> > out_models;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB;
	pcl::PointCloud<PointXYZSIFT>::Ptr cloudXYZSIFT;
	pcl::PointCloud<PointXYZRGBSIFT>::Ptr cloudXYZRGBSIFT;
    float sizeOfCloud;
	string cloudType;
	cv::Mat tempImg;
	cv::Mat xyzimage;
	std::string tempFileName;
	std::string tempFileOnDisc;
	OID fileOID;


	MongoBase();
	virtual ~MongoBase();
	vector<string> getAllFiles(const string& pattern);
	vector<string> getAllFolders(const string& pattern);
	int getChildOIDS(BSONObj &obj, const string&,  const string&, vector<OID>&);
	bool isModelLastLeaf(const string&);
	bool isViewLastLeaf(const string&);
	void findDocumentInCollection(DBClientConnection&, string&, Base::Property<string> &, const string &, auto_ptr<DBClientCursor> &, const string &, const string & , int&);
	void initViewNames();
	void initModelNames();
	void setMime(const string& extension,  string& mime);
	void connectToMongoDB(string&);
	void setModelOrViewName(const string& childNodeName, const BSONObj& childObj, string& newName);
    void getFileFromGrid(const GridFile &);
    void addScenes(BSONObj& object, Base::Property<string>& objectName);
    void initModel(const string & modelName, bool addToModelFlag, Base::Property<string>& nodeNameProp, Base::Property<string>& objectName, Base::Property<string>& description);
    void addToObject(const Base::Property<string>& nodeNameProp,const string & name, Base::Property<string>& objectName, Base::Property<string>& description);
    void initView(const string & viewName, bool addToObjectFlag, Base::Property<string>& nodeNameProp, Base::Property<string>& objectName, Base::Property<string>& description);
    void cloudEncoding(OID& oid, string& tempFileName, string & cloudType);
    void saveXYZFileOnDisc(Base::Property<bool>& suffix, Base::Property<bool> binary, std::string& fn);
    void saveXYZRGBFileOnDisc(Base::Property<bool>& suffix, Base::Property<bool> binary, std::string& fn);
    void saveXYZSIFTFileOnDisc(Base::Property<bool>& suffix, Base::Property<bool> binary, std::string& fn);
    void ReadPCDCloud(const string& filename);
    void writeToSinkFromFile(string& mime, string& fileName);
    void readTextFileFromDocument(mongo::OID& childOID, int size);
    void readXYZMatFromDocument(mongo::OID& childOID, int size);
    void readPointCloudFromDocument(mongo::OID& childOID, int size);
    void readImageFromDocument(mongo::OID& childOID, int size);
    void readFile();
};

MongoBase::MongoBase() {
	DBClientConnection *c_ptr = new DBClientConnection();
	c = boost::shared_ptr<DBClientConnection>(c_ptr);
	mongo::client::initialize();
	dbCollectionPath="";
	sizeOfCloud=0;
	tempFileOnDisc="tempFile";
	fileOID = OID("000000000000000000000000");
}

MongoBase::~MongoBase() {
}

void MongoBase::readTextFileFromDocument(mongo::OID& fileOID, int size)
{
	LOG(LERROR)<<"Read text\n";
	const BSONObj *fieldsToReturn = 0;
	int queryOptions = 0;
	// get bson object
	BSONObj obj = c->findOne(dbCollectionPath, Query(BSON("_id" << fileOID)), fieldsToReturn, queryOptions);
	const char *buffer;
	// get data to buffer
	buffer = obj[tempFileName].binData(size);
	for (int i=0; i<size;i++)
		LOG(LERROR)<<buffer[i];
	LOG(LERROR)<<*buffer;
	LOG(LERROR)<<"size: "<<size;
	string fromDB(buffer,size-1);
	LOG(LERROR)<<"ReadedFile: \n"<<fromDB;
	LOG(LERROR)<<"Save to sink";
	//cipFileOut.write(fromDB);
}
void MongoBase::readXYZMatFromDocument(mongo::OID& fileOID, int size)
{
	LOG(LERROR)<<"Read XYZ Mat\n";
	const BSONObj *fieldsToReturn = 0;
	int queryOptions = 0;
	BSONObj obj = c->findOne(dbCollectionPath, Query(BSON("_id" << fileOID)), fieldsToReturn, queryOptions);
	int len;
	float *data = (float*)obj[tempFileName].binData(len);
	LOG(LNOTICE)<<"len : "<<len;
	int width = obj.getField("width").Int();//480
	int height = obj.getField("height").Int();//640
	int channels = obj.getField("channels").Int();
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
			xyz_p[0+j]=data[i*height*channels+j];
			xyz_p[1+j]=data[i*height*channels+j+1];
			xyz_p[2+j]=data[i*height*channels+j+2];
		}
	}
	cv::FileStorage fs("xyzTest.yaml", cv::FileStorage::WRITE);
	fs << "img" << imageXYZRGB;
	fs.release();
	//out_yaml.write(imageXYZRGB);
}

void MongoBase::readFile()
{
	LOG(LTRACE)<<"MongoBase::readFile";
	int queryOptions = 0;
	const BSONObj *fieldsToReturn = 0;

	// get bson object from collection
	BSONObj obj = c->findOne(dbCollectionPath, Query(BSON("_id" << fileOID)), fieldsToReturn, queryOptions);

	LOG(LERROR)<<"obj: "<<obj<<", childOID: "<<fileOID;
	string place = obj.getField("place").str();
	int size = obj.getField("size").Int();
	string tempFileName = obj.getField("filename").str();
	LOG(LERROR)<<"place: "<<place<<" size: "<<size<<", fileName: "<<tempFileName;

	if(place=="document")
	{
		string extension = obj.getField("extension").str();
		//readFromCollection();
		if(extension=="jpg" || extension=="png")
		{
			readImageFromDocument(fileOID, size);
		}
		else if(extension=="pcd")
		{
			readPointCloudFromDocument(fileOID, size);
		}
		else if(extension=="yml" || extension=="yaml")
		{
			readXYZMatFromDocument(fileOID, size);
		}
		else if(extension=="txt")
		{
			readTextFileFromDocument(fileOID, size);
		}
	}
	else
	{
		// if saved in grid
		GridFS fs(*c,dbCollectionPath);
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
			writeToSinkFromFile(mime, filename);
		}
	}
}

void MongoBase::readImageFromDocument(mongo::OID& fileOID, int size)
{
	LOG(LERROR)<<"Read image\n";
	const BSONObj *fieldsToReturn = 0;
	int queryOptions = 0;
	BSONObj obj = c->findOne(dbCollectionPath, Query(BSON("_id" << fileOID)), fieldsToReturn, queryOptions);
	int len;
	uchar *data = (uchar*)obj[tempFileName].binData(len);
	std::vector<uchar> v(data, data+len);
	LOG(LERROR)<<*data;
	cv::Mat image = cv::imdecode(cv::Mat(v), -1);
	LOG(LERROR)<<image.total();
	//out_img.write(image);
	// only in test purposes, it's to remove
	imwrite( "Gray_Image.jpg", image );
}

void MongoBase::readPointCloudFromDocument(mongo::OID& fileOID, int size)
{
	//TODO dodac pole cloudType i nie bawic sie w ify bo przy else sie moze wywalic teraz!!!
	LOG(LERROR)<<"pcd ";
	string cloudType;
	if (tempFileName.find("xyzrgb") != std::string::npos)
	{
		cloudType="xyzrgb";
	}
	else if (tempFileName.find("xyzsift") != std::string::npos)
	{
		cloudType="xyzsift";
	}
	else if (tempFileName.find("xyzshot") != std::string::npos)
	{
		cloudType="xyzshot";
	}
	else if (tempFileName.find("xyz") != std::string::npos)
	{
		cloudType="xyz";
	}
	else
	{
		LOG(LERROR)<<"Don't know such PC cloud!!!";
	}
	try
	{
		const BSONObj *fieldsToReturn = 0;
		int queryOptions = 0;
		BSONObj obj = c->findOne(dbCollectionPath, Query(BSON("_id" << fileOID)), fieldsToReturn, queryOptions);
		if(cloudType=="xyz")
		{
			// read data to buffer
			int totalSize;
			float* buffer = (float*)obj[tempFileName].binData(totalSize);
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
			// save to file, only in test purposes
			pcl::io::savePCDFile("newCloudXYZ.pcd", *cloudXYZ, false);
		}
		else if(cloudType=="xyzrgb")
		{
			// read data to buffer
			int totalSize;
			float* buffer = (float*)obj[tempFileName].binData(totalSize);
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
			// save to file, only in test purposes
			pcl::io::savePCDFile("newCloudXYZRGB.pcd", *cloudXYZRGB, false);
		}
		else if(cloudType=="xyzsift")
		{
			// read data to buffer
			int totalSize;
			float* buffer = (float*)obj[tempFileName].binData(totalSize);
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
			// save to file, only in test purposes
			pcl::io::savePCDFile("newCloudSIFT.pcd", *cloudXYZSIFT, false);
		}
		LOG(LERROR)<<"ViewWriter::insertFileIntoCollection: END";
	}catch(Exception &ex)
	{
		LOG(LERROR)<<ex.what();
	}
}

void MongoBase::saveXYZFileOnDisc(Base::Property<bool>& suffix, Base::Property<bool> binary, std::string& fn)
{
	if(suffix)
	{
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyz.pcd");
	}
	LOG(LINFO) <<"FileName:"<<fn;
	pcl::io::savePCDFile(fn, *cloudXYZ, binary);
}

void MongoBase::saveXYZRGBFileOnDisc(Base::Property<bool>& suffix, Base::Property<bool> binary, std::string& fn)
{
	if(suffix)
	{
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyzrgb.pcd");
	}
	LOG(LINFO) <<"FileName:"<<fn;
	pcl::io::savePCDFile(fn, *cloudXYZRGB, binary);
}

void MongoBase::saveXYZSIFTFileOnDisc(Base::Property<bool>& suffix, Base::Property<bool> binary, std::string& fn)
{
	if(suffix)
	{
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyzsift.pcd");
	}
	LOG(LINFO) <<"FileName:"<<fn;
	pcl::io::savePCDFile(fn, *cloudXYZSIFT, binary);
}

void MongoBase::cloudEncoding(OID& oid, string& tempFileName, string & cloudType)
{
	try{
		pcl::io::OctreePointCloudCompression<pcl::PointXYZ>* PointCloudDecoderXYZ;
		pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudDecoderXYZRGB;
		int QueryOptions = 0;
		const BSONObj *fieldsToReturn = 0;
		LOG(LNOTICE)<<"dbCollectionPath: "<<dbCollectionPath<<"\n\n";

		// get bson object from collection
		LOG(LNOTICE)<<oid<<endl;
		BSONObj obj = c->findOne(dbCollectionPath, Query( BSON("_id" << oid)), fieldsToReturn, QueryOptions);

		// read data to buffer
		int readSize;
		LOG(LNOTICE)<<obj<<"\n";

		const char* charPtr= (const char*)(obj[tempFileName].binData(readSize));
		LOG(LNOTICE)<<"Readed: "<<readSize<<" B.";
		LOG(LNOTICE)<<"charPtr: "<<charPtr;
		stringstream * strPtr = (stringstream*)charPtr;
		LOG(LNOTICE)<<"strPtr: "<<strPtr;
		LOG(LNOTICE)<<"charPtr[0]: "<<charPtr[0];
		LOG(LNOTICE)<<*charPtr;
		LOG(LNOTICE)<<*strPtr;
		LOG(LNOTICE)<<strPtr->str();


		if(cloudType=="xyz")
		{
			PointCloudDecoderXYZ=new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>();
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ (new pcl::PointCloud<pcl::PointXYZ>);
			PointCloudDecoderXYZ->decodePointCloud (*strPtr, cloudXYZ);
			pcl::io::savePCDFile("newCloud2.pcd", *cloudXYZ, false);
		}
		else if(cloudType=="xyzrgb")
		{
			PointCloudDecoderXYZRGB=new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>();
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
			PointCloudDecoderXYZRGB->decodePointCloud (*strPtr, cloudXYZRGB);
			pcl::io::savePCDFile("newCloud2.pcd", *cloudXYZRGB, false);
		}
		LOG(LNOTICE)<<"ViewWriter::insertFileIntoCollection: END";
	}catch(Exception &ex){LOG(LNOTICE)<<ex.what();}
}
void MongoBase::initView(const string & viewName, bool addToObjectFlag, Base::Property<string>& nodeNameProp, Base::Property<string>& objectName, Base::Property<string>& description)
{
	BSONElement oi;
    OID o;
    BSONArrayBuilder stereoPCArrayBuilder, kinectPCArrayBuilder, tofPCArrayBuilder, objectArrayBuilder, viewArrayBuilder, stereoArrayBuilder, kinectArrayBuilder, tofArrayBuilder, viewBuilder;
    LOG(LNOTICE)<<"Init View\n";
    //add view to object
    if(addToObjectFlag)
    {
    	addToObject(nodeNameProp, viewName, objectName, description);
    }
    // add childs to arraysBuilder
    for(std::vector<string>::iterator it = docViewsNames.begin(); it != docViewsNames.end(); ++it){
		BSONObj document = BSONObjBuilder().genOID().append("NodeName", *it).append("ObjectName", objectName).append("ViewName", viewName).append("description", description).obj();
		c->insert(dbCollectionPath, document);
		document.getObjectID(oi);
		o=oi.__oid();
		//docViewsNames.push_back("ToFRGBD");
		//docViewsNames.push_back("ToFRX");
		//docViewsNames.push_back("ToFRXM");
		LOG(LNOTICE)<< "Add to "<< *it<<" , this: "<<o.toString()<<" in "<<dbCollectionPath<<"\n";
		if(*it=="Stereo" || *it=="Kinect" || *it=="ToF")
			viewArrayBuilder.append(BSONObjBuilder().append("childOID", o.toString()).obj());
		else if(*it=="StereoLR" || *it=="StereoRX" || *it=="StereoRXM" || *it=="StereoPC")
			stereoArrayBuilder.append(BSONObjBuilder().append("childOID", o.toString()).obj());
		else if(*it=="KinectRGBD" || *it=="KinectRX" || *it=="KinectRXM"  || *it=="KinectPC")
			kinectArrayBuilder.append(BSONObjBuilder().append("childOID", o.toString()).obj());
		else if(*it=="ToFRGBD" || *it=="ToFRX" || *it=="ToFRXM" || *it=="ToFPC")
			tofArrayBuilder.append(BSONObjBuilder().append("childOID", o.toString()).obj());
		else if(*it=="KinectPCXYZRGB" || *it=="KinectPCXYZSIFT" || *it== "KinectPCXYZSHOT"  ) //
			kinectPCArrayBuilder.append(BSONObjBuilder().append("childOID", o.toString()).obj());
		else if(*it=="StereoPCXYZRGB" || *it=="StereoPCXYZSIFT" || *it=="StereoPCXYZSHOT") //"StereoPC"
			stereoPCArrayBuilder.append(BSONObjBuilder().append("childOID", o.toString()).obj());
		else if(*it== "ToFPCXYZRGB" || *it=="ToFPCXYZSIFT" || *it=="ToFPCXYZSHOT") //ToFPC
			tofPCArrayBuilder.append(BSONObjBuilder().append("childOID", o.toString()).obj());
    }
    // create arrays
	BSONArray viewArr = viewArrayBuilder.arr();
    BSONArray stereoArr = stereoArrayBuilder.arr();
    BSONArray kinectArr = kinectArrayBuilder.arr();
    BSONArray tofArr = tofArrayBuilder.arr();
    BSONArray kinectPCArr = kinectPCArrayBuilder.arr();
    BSONArray stereoPC = stereoPCArrayBuilder.arr();
    BSONArray tofPCArr = tofPCArrayBuilder.arr();
    // update documents
    c->update(dbCollectionPath, Query(BSON("NodeName"<<"View"<<"ObjectName"<<objectName<<"ViewName"<<viewName)), BSON("$set"<<BSON("childOIDs"<<viewArr)), false, true);
    c->update(dbCollectionPath, Query(BSON("NodeName"<<"ToF"<<"ObjectName"<<objectName<<"ViewName"<<viewName)), BSON("$set"<<BSON("childOIDs"<<tofArr)), false, true);
    c->update(dbCollectionPath, Query(BSON("NodeName"<<"Kinect"<<"ObjectName"<<objectName<<"ViewName"<<viewName)), BSON("$set"<<BSON("childOIDs"<<kinectArr)), false, true);
    c->update(dbCollectionPath, Query(BSON("NodeName"<<"Stereo"<<"ObjectName"<<objectName<<"ViewName"<<viewName)), BSON("$set"<<BSON("childOIDs"<<stereoArr)), false, true);
    c->update(dbCollectionPath, Query(BSON("NodeName"<<"KinectPC"<<"ObjectName"<<objectName<<"ViewName"<<viewName)), BSON("$set"<<BSON("childOIDs"<<kinectPCArr)), false, true);
    c->update(dbCollectionPath, Query(BSON("NodeName"<<"StereoPC"<<"ObjectName"<<objectName<<"ViewName"<<viewName)), BSON("$set"<<BSON("childOIDs"<<stereoPC)), false, true);
    c->update(dbCollectionPath, Query(BSON("NodeName"<<"ToFPC"<<"ObjectName"<<objectName<<"ViewName"<<viewName)), BSON("$set"<<BSON("childOIDs"<<tofPCArr)), false, true);
    c->createIndex(dbCollectionPath, BSON("ObjectName"<<1));
    c->createIndex(dbCollectionPath, BSON("ViewName"<<1));
    c->createIndex(dbCollectionPath, BSON("NodeName"<<1));
}

void MongoBase::addToObject(const Base::Property<string>& nodeNameProp,const string & name, Base::Property<string>& objectName, Base::Property<string>& description)
{
	LOG(LNOTICE)<<"Create View";
	int options=0;
	int limit=0;
	int skip=0;
	BSONElement oi;
	OID o;
	string type;
	string nodeName;
	nodeName = nodeNameProp;
	if(nodeName=="View"||nodeName=="Model")
		type=nodeName;
	else if(isModelLastLeaf(nodeNameProp))
		type="Model";
	else if(isViewLastLeaf(nodeNameProp))
		type="View";
	//CLOG(LTRACE)<<"Type: " <<type;

	unsigned long long nr = c->count(dbCollectionPath, BSON("ObjectName"<<objectName<<"NodeName"<<"Object"), options, limit, skip);
	// add object
	if(nr==0)
	{
		LOG(LNOTICE)<<"Create Object";
		//CLOG(LTRACE) <<"Object does not exists in "<< dbCollectionPath;
		BSONObj object = BSONObjBuilder().genOID().append("NodeName", "Object").append("ObjectName", objectName).append("description", description).obj();
		c->insert(dbCollectionPath, object);
		addScenes(object, objectName);
	}
	// add model/view
	LOG(LNOTICE)<<"Create Model/View";
	BSONObj modelorView = BSONObjBuilder().genOID().append("NodeName", type).append("ObjectName", objectName).append(type+"Name", name).append("description", description).obj();
	c->insert(dbCollectionPath, modelorView);
	modelorView.getObjectID(oi);
	o=oi.__oid();
	LOG(LNOTICE) << "\n"<<o.toString() << "\n";
	c->update(dbCollectionPath, Query(BSON("ObjectName"<<objectName<<"NodeName"<<"Object")), BSON("$addToSet"<<BSON("childOIDs"<<BSON("childOID"<<o.toString()))), false, true);
}

void MongoBase::addScenes(BSONObj& object, Base::Property<string>& objectName)
{
	int items = 0;
	bool objectInTheScene = false;
	OID o;
	BSONElement bsonElement;
	BSONElement oi;
	BSONArrayBuilder bsonBuilder;

	for(std::vector<string>::iterator itSceneName = splitedSceneNames.begin(); itSceneName != splitedSceneNames.end(); ++itSceneName)
	{
		//CLOG(LINFO)<<"Scene: "<<*itSceneName;
		// if scene exist
		items = c->count(dbCollectionPath, (Query(BSON("SceneName"<<*itSceneName))));
		if(items>0)
		{
			auto_ptr<DBClientCursor> cursorCollection =c->query(dbCollectionPath, Query(BSON("SceneName"<<*itSceneName)));
			BSONObj scene = cursorCollection->next();
			LOG(LNOTICE)<<"Add scene to the object!";
			scene.getObjectID(oi);
			o=oi.__oid();

			c->update(dbCollectionPath, Query(BSON("ObjectName"<<objectName<<"NodeName"<<"Object")), BSON("$addToSet"<<BSON("sceneOIDs"<<BSON("sceneOID"<<o.toString()))), false, true);
		//	CLOG(LTRACE)<<scene;

			vector<OID> childsVector;
			if(getChildOIDS(scene, "objectsOIDs", "objectOID", childsVector)>0)
			{
				for (unsigned int i = 0; i<childsVector.size(); i++)
				{
					auto_ptr<DBClientCursor> childCursor = c->query(dbCollectionPath, (Query(BSON("_id"<<childsVector[i]))));
					if( childCursor->more())
					{
						BSONObj childObj = childCursor->next();
						string _id = childObj.getField("_id").str();
						if(_id==o.toString())
						{
							objectInTheScene = true;
							//CLOG(LERROR)<< "Object exists in the scene!";
							break;
						}
					}
				}
			}
			if(!objectInTheScene)
			{
				LOG(LNOTICE)<<"Adding object to the scene";
				object.getObjectID(oi);
				o=oi.__oid();
				c->update(dbCollectionPath, Query(BSON("SceneName"<<*itSceneName)), BSON("$addToSet"<<BSON("objectsOIDs"<<BSON("objectOID"<<o.toString()))), false, true);
			}
		}//if
		else
		{
			c->createIndex(dbCollectionPath, BSON("SceneName"<<1));
			LOG(LNOTICE)<<"Create scene and add object to array list";
			BSONObj scene = BSONObjBuilder().genOID().append("SceneName", *itSceneName).obj();
			c->insert(dbCollectionPath, scene);

			//CLOG(LINFO)<<"Adding object to the scene";
			object.getObjectID(oi);
			o=oi.__oid();
			c->update(dbCollectionPath, Query(BSON("SceneName"<<*itSceneName)), BSON("$addToSet"<<BSON("objectsOIDs"<<BSON("objectOID"<<o.toString()))), false, true);

			//CLOG(LINFO)<<"Add scene to object!";
			scene.getObjectID(oi);
			o=oi.__oid();
			c->update(dbCollectionPath, Query(BSON("ObjectName"<<objectName<<"NodeName"<<"Object")), BSON("$addToSet"<<BSON("sceneOIDs"<<BSON("sceneOID"<<o.toString()))), false, true);
		}
	}
}


void MongoBase::initModel(const string & modelName, bool addToModelFlag,  Base::Property<string>& nodeNameProp, Base::Property<string>& objectName, Base::Property<string>& description)
{
	LOG(LNOTICE)<<"initModel";
	BSONElement oi;
	OID o;
	BSONArrayBuilder objectArrayBuilder, modelArrayBuilder, somArrayBuilder, ssomArrayBuilder;

	if(addToModelFlag)
	{
		addToObject(nodeNameProp, modelName, objectName, description);
	}

	for(std::vector<string>::iterator it = docModelsNames.begin(); it != docModelsNames.end(); ++it){
		LOG(LNOTICE)<<"*it: "<<*it<<"\n";
		BSONObj document = BSONObjBuilder().genOID().append("NodeName", *it).append("ObjectName", objectName).append("ModelName", modelName).append("description", description).obj();
		c->insert(dbCollectionPath, document);

		document.getObjectID(oi);
		o=oi.__oid();
		LOG(LNOTICE)<<"OID: "<<o.toString()<<"\n";

		if(*it=="SOM" || *it=="SSOM")
			modelArrayBuilder.append(BSONObjBuilder().append("childOID", o.toString()).obj());
		else if(*it=="SomXYZRgb" || *it=="SomXYZSift")
			somArrayBuilder.append(BSONObjBuilder().append("childOID", o.toString()).obj());
		else if(*it=="SsomXYZRgb" || *it=="SsomXYZSift" || *it=="SsomXYZShot")
			ssomArrayBuilder.append(BSONObjBuilder().append("childOID", o.toString()).obj());

	}
	c->createIndex(dbCollectionPath, BSON("ObjectName"<<1));
	c->createIndex(dbCollectionPath, BSON("ModelName"<<1));
	c->createIndex(dbCollectionPath, BSON("NodeName"<<1));

	BSONArray modelArr = modelArrayBuilder.arr();
	BSONArray somArr = somArrayBuilder.arr();
	BSONArray ssomArr = ssomArrayBuilder.arr();

	c->update(dbCollectionPath, Query(BSON("NodeName"<<"Model"<<"ObjectName"<<objectName<<"ModelName"<<modelName)), BSON("$set"<<BSON("childOIDs"<<modelArr)), false, true);
	c->update(dbCollectionPath, Query(BSON("NodeName"<<"SOM"<<"ObjectName"<<objectName<<"ModelName"<<modelName)), BSON("$set"<<BSON("childOIDs"<<somArr)), false, true);
	c->update(dbCollectionPath, Query(BSON("NodeName"<<"SSOM"<<"ObjectName"<<objectName<<"ModelName"<<modelName)), BSON("$set"<<BSON("childOIDs"<<ssomArr)), false, true);
}
void MongoBase::getFileFromGrid(const GridFile& file)
{
	LOG(LTRACE)<<"MongoBase::getFileFromGrid";
	stringstream ss;
	string str = ss.str();
	char *tempFilename = (char*)tempFileOnDisc.c_str();
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

void MongoBase::setModelOrViewName(const string& childNodeName, const BSONObj& childObj, string& newName)
{
	string type = childNodeName;
	newName = childObj.getField(type+"Name").str();
}

void MongoBase::setMime( const string& extension,  string& mime)
{
	LOG(LNOTICE)<<"Extension : "<<extension<<std::endl;
	if (extension=="png")
		mime="image/png";
	else if(extension=="jpg")
		mime= "image/jpeg";
	else if(extension=="txt" || extension=="pcd")
		mime="text/plain";
	else if(extension=="yaml" || extension=="yml")
			mime="text/plain";
	else
	{
		LOG(LNOTICE) <<"I don't know such file extension! Please add extension to the `if` statement from http://www.sitepoint.com/web-foundations/mime-types-complete-list/";
		return;
	}
}

void MongoBase::connectToMongoDB(string& hostname)
{
	try{
		//if(!c->isStillConnected())
		{
	//		LOG(LNOTICE)<<"\n\n\nNot Connected?\n\n\n";
			c->connect(hostname);
			LOG(LNOTICE)<<"Connected to base\n";
		}
	//	else
	//		LOG(LNOTICE)<<"\n\n\nConnected?\n\n\n";

	}catch(ConnectException& ex)
	{
		LOG(LNOTICE)<<ex.what();
	}
}

void MongoBase::initViewNames()
{
	docViewsNames.push_back("Stereo");
	docViewsNames.push_back("Kinect");
	docViewsNames.push_back("ToF");
	docViewsNames.push_back("StereoPC");
	docViewsNames.push_back("StereoPCXYZRGB");
	docViewsNames.push_back("StereoPCXYZSIFT");
	docViewsNames.push_back("StereoPCXYZSHOT");
	docViewsNames.push_back("StereoLR");
	docViewsNames.push_back("StereoRX");
	docViewsNames.push_back("StereoRXM");
	docViewsNames.push_back("KinectPC");
	docViewsNames.push_back("KinectPCXYZRGB");
	docViewsNames.push_back("KinectPCXYZSIFT");
	docViewsNames.push_back("KinectPCXYZSHOT");
	docViewsNames.push_back("KinectRGBD");
	docViewsNames.push_back("KinectRX");
	docViewsNames.push_back("KinectRXM");
	docViewsNames.push_back("ToFPC");
	docViewsNames.push_back("ToFPCXYZRGB");
	docViewsNames.push_back("ToFPCXYZSIFT");
	docViewsNames.push_back("ToFPCXYZSHOT");
	docViewsNames.push_back("ToFRGBD");
	docViewsNames.push_back("ToFRX");
	docViewsNames.push_back("ToFRXM");
}

void MongoBase::initModelNames()
{
	docModelsNames.push_back("SomXYZRgb");
	docModelsNames.push_back("SomXYZSift");
	docModelsNames.push_back("SsomXYZRgb");
	docModelsNames.push_back("SsomXYZSift");
	docModelsNames.push_back("SsomXYZShot");
	docModelsNames.push_back("SSOM");
	docModelsNames.push_back("SOM");
}

vector<string> MongoBase::getAllFiles(const string& pattern)
{
	glob_t glob_result;
	glob(pattern.c_str(),GLOB_TILDE,NULL,&glob_result);
	vector<string> files;
	for(unsigned int i=0;i<glob_result.gl_pathc;++i){
	    files.push_back(string(glob_result.gl_pathv[i]));
	}
	globfree(&glob_result);
	return files;
}

vector<string> MongoBase::getAllFolders(const string& directoryPath)
{
	vector<string> directories;
	const char *cstr = directoryPath.c_str();
	DIR *dir = opendir(cstr);
    struct dirent *entry = readdir(dir);
    while (entry != NULL)
    {
    	if (entry->d_type == DT_DIR)
    		directories.push_back(entry->d_name);
    	entry = readdir(dir);
    }
    closedir(dir);
    return directories;
}

int MongoBase::getChildOIDS(BSONObj &obj, const string & fieldName, const string & childfieldName, vector<OID>& oidsVector)
{
	string output = obj.getField(fieldName);
	if(output!="EOO")
	{
		vector<BSONElement> v = obj.getField(fieldName).Array();
		for (unsigned int i = 0; i<v.size(); i++)
		{
			string readedOid =v[i][childfieldName].str();
			OID o = OID(readedOid);
			oidsVector.push_back(o);
		}
		return 1;
	}
	else
		return 0;
}

void  MongoBase::findDocumentInCollection(DBClientConnection& c, string& dbCollectionPath, Base::Property<string>& objectName, const string &nodeName, auto_ptr<DBClientCursor> & cursorCollection, const string & modelOrViewName, const string & type, int& items)
{
	int options=0;
	int limit=0;
	int skip=0;
	LOG(LNOTICE)<<"\nMongoBase::findDocumentInCollection\n";

      try{
    	  if(type!="")
    	  {
			  if(type=="View")
			  {
				  LOG(LNOTICE)<<"\nView\n";
				  items = c.count(dbCollectionPath, (BSON("NodeName"<<nodeName<<"ObjectName"<<objectName<<"ViewName"<<modelOrViewName)), options, limit, skip);
				  if(items>0)
					  cursorCollection =c.query(dbCollectionPath, (Query(BSON("NodeName"<<nodeName<<"ObjectName"<<objectName<<"ViewName"<<modelOrViewName))));
			  }
			  else if(type=="Model")
			  {
				  LOG(LNOTICE)<<"\nModel\n";
				  items = c.count(dbCollectionPath, BSON("NodeName"<<nodeName<<"ObjectName"<<objectName<<"ModelName"<<modelOrViewName), options, limit, skip);
				  if (items>0)
					  cursorCollection =c.query(dbCollectionPath, Query(BSON("NodeName"<<nodeName<<"ObjectName"<<objectName<<"ModelName"<<modelOrViewName)));
			  }
			  else
			  {
				  items = c.count(dbCollectionPath, BSON("NodeName"<<nodeName<<"ObjectName"<<objectName<<type<<modelOrViewName), options, limit, skip);

				  if(items>0)
					  cursorCollection =c.query(dbCollectionPath, (Query(BSON("NodeName"<<nodeName<<"ObjectName"<<objectName<<type<<modelOrViewName))));
			  }
		  }
    	  else
    	  {
    		  LOG(LNOTICE)<<"\nObject\n";
			  LOG(LNOTICE)<<"NodeName"<<nodeName<<"\tObjectName"<<objectName<<"\n";

    		  items = c.count(dbCollectionPath, BSON("NodeName"<<nodeName<<"ObjectName"<<objectName), options, limit, skip);
    		  LOG(LNOTICE)<<"items: "<<items<<"\n";
    		  if(items>0)
    			  cursorCollection =c.query(dbCollectionPath, (Query(BSON("NodeName"<<nodeName<<"ObjectName"<<objectName))));
    	  }
    	 }
      catch(DBException &e)
      {
    	  LOG(LERROR) <<"findDocumentInCollection(). Something goes wrong... :<";
    	  LOG(LERROR) <<c.getLastError();
      }

      return;
}
bool MongoBase::isViewLastLeaf(const string& nodeName)
{
	if(nodeName=="StereoPCXYZRGB" || nodeName=="StereoPCXYZSIFT" || nodeName=="StereoPCXYZSHOT" || nodeName=="ToFPCXYZSIFT" || nodeName=="ToFPCXYZRGB" || nodeName=="ToFPCXYZRGB"  || nodeName=="ToFPCXYZSHOT" || nodeName=="KinectPCXYZSHOT"  || nodeName=="KinectPCXYZSIFT" || nodeName=="KinectPCXYZRGB" || nodeName=="StereoLR" || nodeName=="KinectRGBD" || nodeName=="ToFRGBD" || nodeName=="StereoRX" || nodeName=="KinectRX" ||  nodeName=="ToFRX" || nodeName=="StereoRXM" || nodeName=="KinectRXM" || nodeName=="ToFRXM")
		return true;
	else
		return false;
}

bool MongoBase::isModelLastLeaf(const string& nodeName)
{
	if(nodeName=="SomXYZRgb" || nodeName=="SomXYZSift" ||  nodeName=="SsomXYZRgb" || nodeName=="SsomXYZSift" || nodeName=="SsomXYZShot")
		return true;
	else
		return false;
}

void MongoBase::ReadPCDCloud(const string& filename)
{
	LOG(LTRACE) << "ViewReader::ReadPCDCloud";
	// Try to read the cloud of XYZRGB points.
	if(filename.find("xyzrgb")!=string::npos)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (tempFileOnDisc, *cloud_xyzrgb) == -1){
			LOG(LWARNING) <<"Cannot read PointXYZRGB cloud from "<<tempFileOnDisc;
			return;
		}else{
		//	out_cloud_xyzrgb.write(cloud_xyzrgb);
			LOG(LINFO) <<"PointXYZRGB cloud loaded properly from "<<tempFileOnDisc;
		}
	}
	if(filename.find("xyzsift.pcd")!=string::npos)
	{
		pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift (new pcl::PointCloud<PointXYZSIFT>);
		if (pcl::io::loadPCDFile<PointXYZSIFT> (tempFileOnDisc, *cloud_xyzsift) == -1){
			LOG(LWARNING) <<"Cannot read PointXYZSIFT cloud from "<<tempFileOnDisc;
			return;
		}else{
		//	out_cloud_xyzsift.write(cloud_xyzsift);
			LOG(LINFO) <<"PointXYZSIFT cloud loaded properly from "<<tempFileOnDisc;
		}
	}

	else if(filename.find("xyz")!=string::npos)
	{
		// Try to read the cloud of XYZ points.
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (tempFileOnDisc, *cloud_xyz) == -1){
			LOG(LWARNING) <<"Cannot read PointXYZ cloud from "<<tempFileOnDisc;
			return;
		}else{
		//	out_cloud_xyz.write(cloud_xyz);
			LOG(LINFO) <<"PointXYZ cloud loaded properly from "<<tempFileOnDisc;
		}
	}
}

void MongoBase::writeToSinkFromFile(string& mime, string& fileName)
{
	LOG(LNOTICE)<<"MongoBase::writeToSink";
	if(mime=="image/png" || mime=="image/jpeg")
	{
		cv::Mat image = imread(tempFileOnDisc, CV_LOAD_IMAGE_UNCHANGED);
		//out_img.write(image);
	}
	else if(mime=="text/plain")
	{
		LOG(LINFO)<<"mime==text/plain";
		if((fileName.find("pcd"))!=string::npos)
		{
			LOG(LINFO)<<"pcd :)";
			ReadPCDCloud(fileName);
		}
		else if(fileName.find("txt")!=string::npos)
		{
			LOG(LINFO)<<"txt :)";
			string CIPFile;
			char const* charFileName = tempFileOnDisc.c_str();
			LOG(LINFO)<<tempFileOnDisc;
			std::ifstream t(charFileName);
			std::stringstream buffer;
			buffer << t.rdbuf();
			CIPFile = buffer.str();
		//	cipFileOut.write(CIPFile);
			LOG(LINFO)<<CIPFile;
		}
		else
			LOG(LERROR)<<"Nie wiem co to za plik :/";
	}
}
} /* namespace MongoBase */
#endif /* MONGOBASE_HPP_ */
