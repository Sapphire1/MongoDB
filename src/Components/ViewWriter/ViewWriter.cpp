/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

// todo dodac zapis gdzie zapisane i rozmiar
#include "ViewWriter.hpp"
#include <pcl/point_representation.h>
#include <Eigen/Core>
#include <boost/archive/tmpdir.hpp>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/serialization.hpp>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/assume_abstract.hpp>

#define siftPointSize 133
#define xyzPointSize 3
#define xyzrgbPointSize 4

namespace Processors {
namespace ViewWriter  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace boost::posix_time;
using namespace Eigen;

ViewWriter::ViewWriter(const string & name) : Base::Component(name),
	mongoDBHost("mongoDBHost", string("localhost")),
	objectName("objectName", string("GreenCup")),
	description("description", string("My green coffe cup")),
	collectionName("collectionName", string("containers")),
	viewNameProp("viewName", string("lab012")),
	fileName("fileName", string("tempFile")),
	nodeNameProp("nodeNameProp", string("StereoLR")),
	remoteFileName("remoteFileName", string("sweetFoto")),
	mean_viewpoint_features_number("mean_viewpoint_features_number", int(12)),
	sceneNamesProp("sceneNamesProp", string("scene1,scene2,scene3")),
	binary("binary", false),
	suffix("suffix", false)
{
	registerProperty(mongoDBHost);
	registerProperty(mean_viewpoint_features_number);
	registerProperty(objectName);
	registerProperty(description);
	registerProperty(collectionName);
	registerProperty(viewNameProp);
	registerProperty(sceneNamesProp);
	registerProperty(fileName);
	registerProperty(nodeNameProp);
	registerProperty(remoteFileName);
	registerProperty(binary);
	registerProperty(suffix);

	sizeOfCloud=0.0;
	CLOG(LTRACE) << "Hello ViewWriter";

}

ViewWriter::~ViewWriter()
{
	CLOG(LTRACE) << "Good bye ViewWriter";
}

void ViewWriter::prepareInterface() {
	CLOG(LTRACE) << "ViewWriter::prepareInterface";
	registerHandler("writeViewTXT2DB", boost::bind(&ViewWriter::writeTXT2DB, this));
	registerHandler("writeViewImage2DB", boost::bind(&ViewWriter::writeImage2DB, this));
	registerHandler("Write_xyz", boost::bind(&ViewWriter::Write_cloud<pcl::PointXYZ>, this));
	registerHandler("Write_xyzrgb", boost::bind(&ViewWriter::Write_cloud<pcl::PointXYZRGB>, this));
	registerHandler("Write_xyzsift", boost::bind(&ViewWriter::Write_cloud<PointXYZSIFT>, this));
	registerHandler("Write_xyzrgbsift", boost::bind(&ViewWriter::Write_cloud<PointXYZRGBSIFT>, this));

	registerStream("in_img", &in_img);
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_xyzrgbsift", &in_cloud_xyzrgbsift);
	registerStream("cipFileIn", &cipFileIn);

	addDependency("writeViewImage2DB", &in_img);
	addDependency("writeViewTXT2DB", &cipFileIn);
	addDependency("Write_xyzrgb", &in_cloud_xyzrgb);
	addDependency("Write_xyz", &in_cloud_xyz);
	addDependency("Write_xyzsift", &in_cloud_xyzsift);
	addDependency("Write_xyzrgbsift", &in_cloud_xyzsift);
}

void ViewWriter::writeTXT2DB()
{
	CLOG(LNOTICE) << "ViewWriter::writeTXT2DB";
	string sceneNames = sceneNamesProp;
	boost::split(MongoBase::splitedSceneNames, sceneNames, is_any_of(","));
	string fileType = "txt";
	if(viewNameProp!="")
		insert2MongoDB(nodeNameProp,viewNameProp, "View", fileType);
	else
		CLOG(LERROR)<<"Add view name and try again";
}

void ViewWriter::writeImage2DB()
{
	CLOG(LNOTICE) << "ViewWriter::writeImage2DB";
	string sceneNames = sceneNamesProp;
	boost::split(MongoBase::splitedSceneNames, sceneNames, is_any_of(","));
	string fileType = "png";
	if(viewNameProp!="")
		insert2MongoDB(nodeNameProp,viewNameProp, "View", fileType);
	else
		CLOG(LERROR)<<"Add view name and try again";
}

void ViewWriter::writePCD2DB()
{
	CLOG(LNOTICE) << "ViewWriter::writePCD2DB";
	string sceneNames = sceneNamesProp;
	boost::split(MongoBase::splitedSceneNames, sceneNames, is_any_of(","));
	CLOG(LERROR)<<"cloudType: "<<cloudType;
	string fileType = "pcd";
	if(viewNameProp!="")
		insert2MongoDB(nodeNameProp,viewNameProp, "View", fileType);
	else
		CLOG(LERROR)<<"Add view name and try again";
}

template <class PointT>
void ViewWriter::Write_cloud()
{
	CLOG(LTRACE) << "ViewWriter::Write_cloud()";
	std::vector< pcl::PCLPointField> fields;
	pcl::getFields<PointT>(fields);

	if(typeid(PointT) == typeid(pcl::PointXYZ))
	{
		cloudType="xyz";
		cloudXYZ = in_cloud_xyz.read();
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeOfCloud+=(float)pcl::getFieldSize(it->datatype)*cloudXYZ->size();
	}
	else if(typeid(PointT) == typeid(pcl::PointXYZRGB))
	{
		cloudType="xyzrgb";
		cloudXYZRGB = in_cloud_xyzrgb.read();
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeOfCloud+=(float)pcl::getFieldSize(it->datatype)*cloudXYZRGB->size();
	}
	else if(typeid(PointT) == typeid(PointXYZSIFT))
	{
		cloudType="xyzsift";
		cloudXYZSIFT = in_cloud_xyzsift.read();

		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeOfCloud+=(float)pcl::getFieldSize(it->datatype)*cloudXYZSIFT->size();
	}
	else if(typeid(PointT) == typeid(PointXYZSIFT))
	{
		cloudType="xyzrgbsift";
		cloudXYZRGBSIFT = in_cloud_xyzrgbsift.read();
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeOfCloud+=(float)pcl::getFieldSize(it->datatype)*cloudXYZRGBSIFT->size();
	}
	CLOG(LINFO)<<"CloudType: "<<cloudType;
	CLOG(LINFO)<<"PointCloudSize = "<< sizeOfCloud;
	writePCD2DB();

}

void ViewWriter::saveXYZFileOnDisc()
{
	std::string fn = fileName;
	if(suffix)
	{
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyz.pcd");
	}
	CLOG(LINFO) <<"FileName:"<<fn;
	pcl::io::savePCDFile(fn, *cloudXYZ, binary);
}

void ViewWriter::saveXYZRGBFileOnDisc()
{
	std::string fn = fileName;
	if(suffix)
	{
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyzrgb.pcd");
	}
	CLOG(LINFO) <<"FileName:"<<fn;
	pcl::io::savePCDFile(fn, *cloudXYZRGB, binary);
}

void ViewWriter::saveXYZSIFTFileOnDisc()
{
	std::string fn = fileName;
	if(suffix)
	{
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyzsift.pcd");
	}
	CLOG(LINFO) <<"FileName:"<<fn;
	pcl::io::savePCDFile(fn, *cloudXYZSIFT, binary);
}

bool ViewWriter::onInit()
{
      CLOG(LTRACE) << "ViewWriter::initialize";
      try
      {
    	  cloudType="";
    	  string hostname = mongoDBHost;
    	  connectToMongoDB(hostname);
		  if(collectionName=="containers")
			MongoBase::dbCollectionPath=dbCollectionPath="images.containers";
		 initViewNames();
      }
	 catch(DBException &e)
	 {
		 CLOG(LERROR) <<"Something goes wrong... :<";
		 CLOG(LERROR) <<c->getLastError();
	 }

	 return true;
}

bool ViewWriter::onFinish()
{
        CLOG(LTRACE) << "ViewWriter::finish";

        return true;
}

bool ViewWriter::onStep()
{
        CLOG(LTRACE) << "ViewWriter::step";
        return true;
}

bool ViewWriter::onStop()
{
        return true;
}

bool ViewWriter::onStart()
{
        return true;
}


void ViewWriter::createModelOrView(const std::vector<string>::iterator it, const string& nodeName, BSONArrayBuilder& bsonBuilder)
{
	BSONElement bsonElement;
	if(*it=="." || *it=="..")
		return;
	BSONObj model = BSONObjBuilder().genOID().append("NodeName", nodeName).append("ObjectName", objectName).append(nodeName+"Name", *it).append("description", description).obj();
	c->insert(dbCollectionPath, model);
	model.getObjectID(bsonElement);
	OID oid=bsonElement.__oid();
	bsonBuilder.append(BSONObjBuilder().append("childOID", oid.toString()).obj());
	if(nodeName=="Model")
		initModel(*it, true, nodeNameProp, objectName, description);
	else if(nodeName=="View")
		initView(*it, true, nodeNameProp, objectName, description);
}

void ViewWriter::initObject()
{
	CLOG(LTRACE) <<"Create template of object";
	BSONArrayBuilder bsonBuilder;
	bool objectInTheScene = false;
	try
	{
		CLOG(LTRACE) <<"create object";
		BSONObj object = BSONObjBuilder().genOID().append("NodeName", "Object").append("ObjectName", objectName).append("description", description).obj();
		CLOG(LTRACE) <<"insertToDB";
		c->insert(dbCollectionPath, object);
		c->createIndex(dbCollectionPath, BSON("ObjectName"<<1));
		addScenes(object, objectName);
		CLOG(LTRACE) <<"initObject::end";
	}
	  catch(DBException &e)
	  {
		CLOG(LERROR) <<"Something goes wrong... :<";
		CLOG(LERROR) <<c->getLastError();
	  }
}

void ViewWriter::insertFileIntoGrid(OID& oid, const string& fileType, string& tempFileName, int totalSize)
{
	try{
		BSONObj object;
		BSONElement bsonElement;
		string mime="";
		setMime(fileType, mime);
		std::stringstream time;
		CLOG(LNOTICE)<<"File should be written to GRIDFS!";

		// save file on disc in order to write it to gridFS
		if (fileType=="png" || fileType=="jpg")
		{
			tempFileName = string(fileName)+"."+string(fileType);
			cv::imwrite(tempFileName, tempImg);
		}
		else if(fileType=="txt")	// save to file pcd
		{
			CLOG(LINFO) << "CIP file";
			string CIP = cipFileIn.read();
			tempFileName = string(fileName)+"."+string(fileType);
			char const* ca = tempFileName.c_str();
			std::ofstream out(ca);
			out << CIP;
			out.close();
			CLOG(LINFO)<<"Size of CIP is lower then 16 MB: ";
		}
		else if(fileType=="pcd")
		{
			CLOG(LINFO) << "PCD file";
			if(cloudType=="xyz")
				saveXYZFileOnDisc();
			else if(cloudType=="xyzrgb")
				saveXYZRGBFileOnDisc();
			else if(cloudType=="xyzsift")
				saveXYZSIFTFileOnDisc();
		}
		else
		{
			CLOG(LERROR)<<"I don't know such extension file :(";
			exit(1);
		}
		CLOG(LERROR)<<"cloudType: "<<cloudType;


		// TODO add XYZ mat <float> 3C - CV_32F3C, or sth similiar

		CLOG(LINFO) << "tempFileName: "<< tempFileName << endl;
		boost::posix_time::time_facet *facet = new boost::posix_time::time_facet("%d_%m_%Y_%H_%M_%S");
		time.imbue(locale(cout.getloc(), facet));
		time<<second_clock::local_time();
		CLOG(LINFO) << "Time: "<< time.str() << endl;

		// create GridFS client
		GridFS fs(*c, collectionName);
		string fileNameInMongo;
		if(cloudType!="")
			fileNameInMongo = (string)remoteFileName+"_"+ cloudType + time.str()+"."+string(fileType);
		else
			fileNameInMongo = (string)remoteFileName + time.str()+"."+string(fileType);
		CLOG(LERROR)<<"tempFileName: "<<tempFileName;

		// save in grid
		object = fs.storeFile(tempFileName, fileNameInMongo, mime);

		BSONObj b;
		if(cloudType=="xyzsift")
			b = BSONObjBuilder().appendElements(object).append("ObjectName", objectName).append("size", totalSize).append("place", "grid").append("mean_viewpoint_features_number", mean_viewpoint_features_number).obj();
		else
			b = BSONObjBuilder().appendElements(object).append("ObjectName", objectName).append("size", totalSize).append("place", "grid").obj();
		c->insert(dbCollectionPath, b);
		b.getObjectID(bsonElement);
		oid=bsonElement.__oid();
		cloudType="";
		c->createIndex(dbCollectionPath, BSON("filename"<<1));
	}catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :<";
		CLOG(LERROR) <<c->getLastError();
		CLOG(LERROR) << e.what();
	}
}
float ViewWriter::getFileSize(const string& fileType, string& tempFileName)
{
	float size = 0.0;
	if (fileType=="png" || fileType=="jpg")
	{
		CLOG(LINFO)<<"Image!";
		tempImg = in_img.read();
		size = (float)tempImg.elemSize1()*(float)tempImg.total();
		CLOG(LINFO)<<"Size of image file: " << size<<" B";
	}
	else if(fileType=="pcd")	// save to file pcd
	{
		CLOG(LINFO)<<"Cloud!";

		if(cloudType=="xyzrgb")
			tempFileName=std::string(fileName) + std::string("_xyzrgb.pcd");
		else if(cloudType=="xyz")
			tempFileName=std::string(fileName) + std::string("_xyz.pcd");
		else if(cloudType=="xyzsift")
			tempFileName=std::string(fileName) + std::string("_xyzsift.pcd");

		size=sizeOfCloud;
		CLOG(LINFO) << "cloudType: "<< cloudType << endl;
		CLOG(LINFO)<<"Size of PCD cloud: " << size<<" MB";

	}
	else if(fileType=="txt")	// save to file pcd
	{
		CLOG(LINFO) << "CIP file";
		size=1.0;
		tempFileName=std::string(fileName) + std::string(".txt");
	}
	else
	{
		CLOG(LERROR)<<"I don't know such extension file :(";
		exit(1);
	}
	return size;
}

void ViewWriter::writeNode2MongoDB(const string &destination, const string &nodeName,string modelOrViewName, const string& fileType)
{
	CLOG(LTRACE) <<"writeNode2MongoDB";
	OID oid;
	CLOG(LERROR)<<"ViewWriter::writeNode2MongoDB, cloudType: "<<cloudType;
	CLOG(LTRACE) <<"Filename: " << fileName << " destination: "<< destination<<" dbCollectionPath: "<<dbCollectionPath;
    try{
    	string tempFileName = string(fileName)+"."+string(fileType);
    	CLOG(LERROR)<<tempFileName;
    	float sizeOfFileBytes = getFileSize(fileType, tempFileName);
    	float sizeOfFileMBytes = sizeOfFileBytes/(1024*1024);
    	if(sizeOfFileMBytes>15)
    	{
    		CLOG(LERROR)<<"ViewWriter::writeNode2MongoDB, cloudType: "<<cloudType;
			insertFileIntoGrid(oid, fileType, tempFileName, sizeOfFileBytes);
			c->update(dbCollectionPath, Query(BSON("ObjectName"<<objectName<<nodeName+"Name"<<modelOrViewName<<"NodeName"<<destination)), BSON("$addToSet"<<BSON("childOIDs"<<BSON("childOID"<<oid.toString()))), false, true);
    	}
		else
		{
			CLOG(LERROR)<<"ViewWriter::writeNode2MongoDB, cloudType: "<<cloudType;
			if(sizeOfFileBytes>0.0)
			{
				CLOG(LERROR)<<"sizeOfFileBytes: "<<sizeOfFileBytes;
				insertFileIntoCollection(oid, fileType, tempFileName, sizeOfFileBytes);
				CLOG(LERROR)<<"UPDATE: "<<oid.toString();
				c->update(dbCollectionPath, Query(BSON("ObjectName"<<objectName<<nodeName+"Name"<<modelOrViewName<<"NodeName"<<destination)), BSON("$addToSet"<<BSON("childOIDs"<<BSON("childOID"<<oid.toString()))), false, true);
			}
		}
		CLOG(LTRACE) <<"File saved successfully";
    }
	catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :<";
		CLOG(LERROR) <<c->getLastError();
	}
}

void ViewWriter::copyXYZPointToFloatArray (const pcl::PointXYZ &p, float * out) const
{
	out[0] = p.x;	// 4 bytes
	out[1] = p.y;	// 4 bytes
	out[2] = p.z;	// 4 bytes
}

void ViewWriter::copyXYZRGBPointToFloatArray (const pcl::PointXYZRGB &p, float * out) const
{
	out[0] = p.x;	// 4 bytes
	out[1] = p.y;	// 4 bytes
	out[2] = p.z;	// 4 bytes
	out[3] = p.rgb;	// 4 bytes
}

void ViewWriter::copyXYZSiftPointToFloatArray (const PointXYZSIFT &p, float * out) const
{
	Eigen::Vector3f outCoordinates = p.getArray3fMap();
	out[0] = outCoordinates[0];	// 4 bytes
	out[1] = outCoordinates[1];	// 4 bytes
	out[2] = outCoordinates[2];	// 4 bytes
	//CLOG(LERROR)<<"out[0]: "<<out[0]<<"\t out[1]: "<<out[1]<<"\t out[2]: "<<out[2];
	memcpy(&out[3], &p.multiplicity, sizeof(int)); // 4 bytes
	memcpy(&out[4], &p.pointId, sizeof(int));	// 4 bytes
	memcpy(&out[5], &p.descriptor, 128*sizeof(float)); // 128 * 4 bytes = 512 bytes
}

void ViewWriter::insertFileIntoCollection(OID& oid, const string& fileType, string& tempFileName, int size)
{
	CLOG(LTRACE)<<"ViewWriter::insertFileIntoCollection";
	//TODO add fileName and size to document
	BSONObjBuilder builder;
	BSONObj b;
	BSONElement bsonElement;
	// todo dodac obsluge cv mat xyz std::vector<float> buf;

	if (fileType=="png" || fileType=="jpg")	// save image
	{
		CLOG(LTRACE)<<"ViewWriter::insertFileIntoCollection, Image file";
		std::vector<uchar> buf;
		std::vector<int> params(2);
		params[0] = CV_IMWRITE_JPEG_QUALITY;
		params[1] = 95;
		cv::imencode(".jpg", tempImg, buf, params);
		b=BSONObjBuilder().genOID().appendBinData(tempFileName, buf.size(), mongo::BinDataGeneral, &buf[0]).append("fileName", tempFileName).append("size", size).append("place", "document").append("extension", fileType).obj();
		b.getObjectID(bsonElement);
		oid=bsonElement.__oid();
		c->insert(dbCollectionPath, b);
	}
	else if(fileType=="pcd")	// save cloud
	{
		if(cloudType=="xyzrgb")
		{
			/*
			bool showStatistics = true;
			// for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
			pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
			std::stringstream compressedData;
			pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder;
		    PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> (compressionProfile, showStatistics);
			 // compress point cloud
			PointCloudEncoder->encodePointCloud (cloudXYZRGB, compressedData);
			compressedData.seekp(0, ios::end);
			stringstream::pos_type offset = compressedData.tellp();
			CLOG(LERROR)<< "Size of compressed data: "<< offset;
			int intOffSet = (int)offset;
			char* data = serializeTable(intOffSet, compressedData);

			CLOG(LERROR)<< "data[0]: "<< data[0];
			CLOG(LERROR)<< "data[1]: "<< data[1];
			CLOG(LERROR)<< "data[2]: "<< data[2];
			CLOG(LERROR)<< "data[3]: "<< data[3];
			CLOG(LERROR)<< "data[4]: "<< data[4];
			CLOG(LERROR)<< "data[5]: "<< data[5];
			CLOG(LERROR)<< "data[6]: "<< data[6];
			CLOG(LERROR)<< "data[7]: "<< data[7];
			CLOG(LERROR)<< "data[8]: "<< data[8];
			CLOG(LERROR)<< "data[9]: "<< data[9];
			CLOG(LERROR)<< "data[10]: "<< data[10];
			CLOG(LERROR)<< "data[11]: "<< data[11];

			b=BSONObjBuilder().genOID().appendBinData(tempFileName, offset, mongo::BinDataGeneral, data).append("fileName", tempFileName).append("size", size).append("place", "document").append("extension", fileType).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			c->insert(dbCollectionPath, b);
			*/

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
			b=BSONObjBuilder().genOID().appendBinData(tempFileName, totalSize, mongo::BinDataGeneral, &buff[0]).append("fileName", tempFileName).append("size", totalSize).append("place", "document").append("extension", fileType).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			c->insert(dbCollectionPath, b);

	//		delete (PointCloudEncoder);
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
			b=BSONObjBuilder().genOID().appendBinData(tempFileName, totalSize, mongo::BinDataGeneral, &buff[0]).append("fileName", tempFileName).append("size", totalSize).append("place", "document").append("extension", fileType).obj();
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
			b=BSONObjBuilder().genOID().appendBinData(tempFileName, totalSize, mongo::BinDataGeneral, &buff[0]).append("fileName", tempFileName).append("size", totalSize).append("place", "document").append("extension", fileType).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			c->insert(dbCollectionPath, b);
		}
		CLOG(LTRACE)<<"ViewWriter::insertFileIntoCollection, PCD file end";
	}
	else if(fileType=="txt")
	{
		CLOG(LTRACE)<<"ViewWriter::insertFileIntoCollection, txt file";
		string input;

		// read string from input
		input =cipFileIn.read()+" ";
		CLOG(LERROR)<<"Input:"<< input;

		// convert to char*
		char const *cipCharTable = input.c_str();
		int size = strlen(cipCharTable);
		CLOG(LERROR)<<string(cipCharTable);
		CLOG(LERROR)<<"Size: "<<size;
		// create bson object
		b = BSONObjBuilder().genOID().appendBinData(tempFileName, size, BinDataGeneral,  cipCharTable).append("fileName", tempFileName).append("size", size).append("place", "document").append("extension", fileType).obj();

		// insert object into collection
		c->insert(dbCollectionPath, b);

		b.getObjectID(bsonElement);
		oid=bsonElement.__oid();
	}
	cloudType="";
}

char* ViewWriter::serializeTable( int &length, 	std::stringstream& compressedData)
{
    std::istreambuf_iterator<char> itt(compressedData.rdbuf()), eos;
    std::vector<char> serialVector;
    serialVector.reserve(619999); // just a guess
    while(itt != eos) {
        char c = *itt++;
        serialVector.push_back(c);
    }

    length = serialVector.size();

    char *serial = new char[length];
    for (int i=0;i<length;i++)
    {
        serial[i] = serialVector[i];
    }
    return serial;
}

void ViewWriter::insert2MongoDB(const string &destination, const string&  modelOrViewName, const string&  nodeName,  const string&  fileType)
{
	CLOG(LINFO)<<"ViewWriter::insert2MongoDB";
	auto_ptr<DBClientCursor> cursorCollection;
	string source;
	int items=0;
	CLOG(LERROR)<<"cloudType: "<<cloudType;
	try{
		if(destination=="Object")
		{
			unsigned long long nr = c->count(dbCollectionPath, BSON("ObjectName"<<objectName<<"NodeName"<<"Object"),0,0,0);
			if(nr==0)
			{
				CLOG(LTRACE) <<"Object does not exists in "<< dbCollectionPath;
				initObject();
			}
			else
			{
					CLOG(LERROR) <<"Object "<<objectName<<" exist in "<< dbCollectionPath;
					return;
			}
		}
		if(isViewLastLeaf(destination) || isModelLastLeaf(destination))
		{
			CLOG(LTRACE)<<"if(base->isViewLastLeaf(destination)  Check if object exists";
			unsigned long long nr = c->count(dbCollectionPath, BSON("ObjectName"<<objectName<<"NodeName"<<"Object"),0,0,0);
			if(nr==0)
			{
				CLOG(LTRACE) <<"Object does not exists in "<< dbCollectionPath;
				initObject();
				insert2MongoDB(destination, modelOrViewName, nodeName, fileType);
				return;
			}
			else
			{
				CLOG(LTRACE)<<"Object now exist";
				int items = c->count(dbCollectionPath, Query(BSON("ObjectName"<<objectName<<"NodeName"<<nodeName<<nodeName+"Name"<<modelOrViewName)));
				CLOG(LTRACE)<<"Items: "<<items;
				if(items==0)
				{
					CLOG(LTRACE)<<"No such model/view";
					CLOG(LTRACE)<<"NodeName: "<<nodeName;
					initView(modelOrViewName, true, nodeNameProp, objectName, description);
				}
				cursorCollection = c->query(dbCollectionPath, Query(BSON("ObjectName"<<objectName<<"NodeName"<<nodeName<<nodeName+"Name"<<modelOrViewName)));
				BSONObj obj;
				if(cursorCollection->more())
					obj = cursorCollection->next();
				else
				{
					CLOG(LERROR)<<"No objects views/models";
				}
				vector<OID> childsVector;
				// check if node has some files
				if(getChildOIDS(obj, "childOIDs", "childOID", childsVector)>0 && childsVector.size()>0)
				{
					CLOG(LTRACE)<<nodeName <<"There are some files in Mongo in this node!";
				}
			}
			CLOG(LINFO)<<"Write to view";
			writeNode2MongoDB(destination, nodeName, modelOrViewName, fileType);
		}
    }//try
	catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :<";
		exit(1);
	}
}

} //: namespace ViewWriter
} //: namespace Processors
