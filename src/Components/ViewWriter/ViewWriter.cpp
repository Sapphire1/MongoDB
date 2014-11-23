/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

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
using namespace MongoBase;

ViewWriter::ViewWriter(const string & name) : Base::Component(name),
	mongoDBHost("mongoDBHost", string("localhost")),
	objectName("objectName", string("GreenCup")),
	description("description", string("My green coffe cup")),
	collectionName("collectionName", string("containers")),
	viewName("viewName", string("lab012")),
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
	registerProperty(viewName);
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
	registerHandler("writeXML2DB", boost::bind(&ViewWriter::writeData<xml>, this));
	registerHandler("writeXYZ2DB", boost::bind(&ViewWriter::writeData<xyz>, this));
	registerHandler("writeRGB2DB", boost::bind(&ViewWriter::writeData<rgb>, this));
	registerHandler("writeD2DB", boost::bind(&ViewWriter::writeData<density>, this));
	registerHandler("writeI2DB", boost::bind(&ViewWriter::writeData<intensity>, this));
	registerHandler("writeMask2DB", boost::bind(&ViewWriter::writeData<mask>, this));
	registerHandler("writeStereoL2DB", boost::bind(&ViewWriter::writeData<stereoL>, this));
	registerHandler("writeStereoR2DB", boost::bind(&ViewWriter::writeData<stereoR>, this));
	registerHandler("writeStereoLTextured2DB", boost::bind(&ViewWriter::writeData<stereoLTextured>, this));
	registerHandler("writeStereoRTextured2DB", boost::bind(&ViewWriter::writeData<stereoRTextured>, this));

	// PCL
	registerHandler("write_xyz", boost::bind(&ViewWriter::writeData<pc_xyz>, this));
	registerHandler("write_xyzrgb", boost::bind(&ViewWriter::writeData<pc_xyzrgb>, this));
	registerHandler("write_xyzsift", boost::bind(&ViewWriter::writeData<pc_xyzsift>, this));
	registerHandler("write_xyzrgbsift", boost::bind(&ViewWriter::writeData<pc_xyzrgbsift>, this));
	registerHandler("write_xyzshot", boost::bind(&ViewWriter::writeData<pc_xyzshot>, this));
	registerHandler("write_xyzrgbnormal", boost::bind(&ViewWriter::writeData<pc_xyzrgbnormal>, this));	//4 floats

	// streams registration
	registerStream("in_xml", &in_xml);
	registerStream("in_xyz", &in_xyz);
	registerStream("in_rgb", &in_rgb);
	registerStream("in_density", &in_density);
	registerStream("in_intensity", &in_intensity);
	registerStream("in_mask", &in_mask);
	registerStream("in_stereoL", &in_stereoL);
	registerStream("in_stereoR", &in_stereoR);
	registerStream("in_stereoLTextured", &in_stereoLTextured);
	registerStream("in_stereoRTextured", &in_stereoRTextured);

	// PCL
	registerStream("in_pc_xyz", &in_pc_xyz);
	registerStream("in_pc_xyzrgb", &in_pc_xyzrgb);
	registerStream("in_pc_xyzsift", &in_pc_xyzsift);
	registerStream("in_pc_xyzrgbsift", &in_pc_xyzrgbsift);
	registerStream("in_pc_xyzshot", &in_pc_xyzshot);
	registerStream("in_pc_xyzrgnormal", &in_pc_xyzrgbnormal);

	// adding dependency
	addDependency("writeXML2DB", &in_xml);
	addDependency("writeRGB2DB", &in_rgb);
	addDependency("writeXYZ2DB", &in_xyz);
	addDependency("writeXYZ2DB", &in_density);
	addDependency("writeD2DB", &in_density);
	addDependency("writeI2DB", &in_intensity);
	addDependency("writeMask2DB", &in_mask);
	addDependency("writeStereoL2DB", &in_stereoL);
	addDependency("writeStereoR2DB", &in_stereoR);
	addDependency("writeStereoLTextured2DB", &in_stereoLTextured);
	addDependency("writeStereoRTextured2DB", &in_stereoRTextured);
	addDependency("write_xyz", &in_pc_xyz);
	addDependency("write_xyzrgb", &in_pc_xyzrgb);
	addDependency("write_xyzsift", &in_pc_xyzsift);
	addDependency("write_xyzrgbsift", &in_pc_xyzrgbsift);
	addDependency("write_xyzshot", &in_pc_xyzshot);
	addDependency("write_xyzrgbnormal", &in_pc_xyzrgbnormal);
}

template <keyTypes keyType>
void ViewWriter::writeData()
{
	CLOG(LNOTICE) << "ViewWriter::writeData";
	string vn = string (viewName);
	shared_ptr<View> viewPtr(new View(vn));
	bool exist = viewPtr->checkIfExist();
	if(!exist)
		viewPtr->create();
	else
	{
		CLOG(LERROR)<<"View exist in data base!!!";
		exit(-1);
	}
	std::vector<keyTypes> requiredTypes;
	//viewPtr->setRequiredKeyTypes(requiredTypes);

	// read dataTypes from mongo when inserting add to readed from mongo
	// and then check if such type exist in view
	bool fileExist = viewPtr->checkIfFileExist(keyType);
	if(fileExist)
	{
		LOG(LERROR)<<"File exist in data base. You can't write file. File type is: "<< keyType;
		exit(-1);
	}


	CLOG(LNOTICE)<<"keyType : "<< keyType;

	// read data from input
	switch(keyType)
	{
		case xml:
		{
			string xmlData = in_xml.read();
			viewPtr->putStringToFile(xmlData, keyType);
			break;
		}
		case rgb:
		{
			cv::Mat rgbData = in_rgb.read();
			viewPtr->putMatToFile(rgbData, keyType);
			break;
		}
		case density:
		{
			cv::Mat densityData = in_density.read();
			viewPtr->putMatToFile(densityData, keyType);
			break;
		}
		case intensity:
		{
			cv::Mat intensityData = in_intensity.read();
			viewPtr->putMatToFile(intensityData, keyType);
			break;
		}
		case mask:
		{
			cv::Mat maskData = in_mask.read();
			viewPtr->putMatToFile(maskData, keyType);
			break;
		}
		case stereoL:
		{
			cv::Mat stereoLData = in_stereoL.read();
			viewPtr->putMatToFile(stereoLData, keyType);
			break;
		}
		case stereoR:
		{
			cv::Mat stereoRData = in_stereoR.read();
			viewPtr->putMatToFile(stereoRData, keyType);
			break;
		}
		case stereoLTextured:
		{
			cv::Mat stereoLTexturedData = in_stereoLTextured.read();
			viewPtr->putMatToFile(stereoLTexturedData, keyType);
			break;
		}
		case stereoRTextured:
		{
			cv::Mat stereoRTexturedData = in_stereoRTextured.read();
			viewPtr->putMatToFile(stereoRTexturedData, keyType);
			break;
		}
		case pc_xyz:
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr pc_xyz_Data = in_pc_xyz.read();
			viewPtr->putPCxyzToFile(pc_xyz_Data, keyType);
			break;
		}
		case pc_xyzrgb:
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_xyzrgb_Data = in_pc_xyzrgb.read();
			viewPtr->putPCyxzrgbToFile(pc_xyzrgb_Data, keyType);
			break;
		}
		case pc_xyzsift:
		{
			pcl::PointCloud<PointXYZSIFT>::Ptr pc_xyzsift_Data = in_pc_xyzsift.read();
			viewPtr->putPCxyzsiftToFile(pc_xyzsift_Data, keyType);
			break;
		}
		case pc_xyzrgbsift:
		{
			pcl::PointCloud<PointXYZRGBSIFT>::Ptr pc_xyzrgbsift_Data = in_pc_xyzrgbsift.read();
			viewPtr->putPCxyzrgbsiftToFile(pc_xyzrgbsift_Data, keyType);
			break;
		}
		case pc_xyzshot:
		{
			pcl::PointCloud<PointXYZSHOT>::Ptr pc_xyzshot_Data = in_pc_xyzshot.read();
			viewPtr->putPCxyzshotToFile(pc_xyzshot_Data, keyType);
			break;
		}
		case pc_xyzrgbnormal:
		{
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pc_xyzrgbnormal_Data = in_pc_xyzrgbnormal.read();
			viewPtr->putPCxyzrgbNormalToFile(pc_xyzrgbnormal_Data, keyType);
			break;
		}
	}
}

void ViewWriter::writeTXT2DB()
{
	CLOG(LNOTICE) << "ViewWriter::writeTXT2DB";
	string sceneNames = sceneNamesProp;
	boost::split(MongoBase::splitedSceneNames, sceneNames, is_any_of(","));
	string fileExtension = "txt";
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_xyzrgb_normals;
	tempFileName = string(fileName)+"."+string(fileExtension);
	if(viewName!="")
		insert2MongoDB(nodeNameProp,viewName, "View", fileExtension);
	else
		CLOG(LERROR)<<"Add view name and try again";
}

void ViewWriter::writeYAML2DB()
{
	CLOG(LNOTICE) << "ViewWriter::writeYAML2DB";
	string sceneNames = sceneNamesProp;
	boost::split(MongoBase::splitedSceneNames, sceneNames, is_any_of(","));
	string fileExtension = "yaml";
	tempFileName = string(fileName)+"."+string(fileExtension);
	if(viewName!="")
		insert2MongoDB(nodeNameProp,viewName, "View", fileExtension);
	else
		CLOG(LERROR)<<"Add view name and try again";
}


void ViewWriter::writeImage2DB()
{
	CLOG(LNOTICE) << "ViewWriter::writeImage2DB";
	string sceneNames = sceneNamesProp;
	boost::split(MongoBase::splitedSceneNames, sceneNames, is_any_of(","));
	string fileExtension = "png";
	tempFileName = string(fileName)+"."+string(fileExtension);
	if(viewName!="")
		insert2MongoDB(nodeNameProp,viewName, "View", fileExtension);
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
	if(viewName!="")
		insert2MongoDB(nodeNameProp,viewName, "View", fileType);
	else
		CLOG(LERROR)<<"Add view name and try again";
}
/*
template <class PointT>
void ViewWriter::Read_cloud()
{
	CLOG(LTRACE) << "ViewWriter::Write_cloud()";
	if(typeid(PointT) == typeid(pcl::PointXYZ))
	{
		cloudType="xyz";
		tempFileName=std::string(fileName) + std::string("_xyz.pcd");
		cloudXYZ = in_cloud_xyz.read();
	}
	else if(typeid(PointT) == typeid(pcl::PointXYZRGB))
	{
		cloudType="xyzrgb";
		tempFileName=std::string(fileName) + std::string("_xyzrgb.pcd");
		cloudXYZRGB = in_cloud_xyzrgb.read();
	}
	else if(typeid(PointT) == typeid(PointXYZSIFT))
	{
		cloudType="xyzsift";
		tempFileName=std::string(fileName) + std::string("_xyzsift.pcd");
		cloudXYZSIFT = in_cloud_xyzsift.read();
	}
	else if(typeid(PointT) == typeid(PointXYZSIFT))
	{
		cloudType="xyzrgbsift";
		tempFileName=std::string(fileName) + std::string("_xyzrgbsift.pcd");
		cloudXYZRGBSIFT = in_cloud_xyzrgbsift.read();
	}
	CLOG(LINFO)<<"CloudType: "<<cloudType;
	CLOG(LINFO)<<"PointCloudSize = "<< sizeOfCloud;
	writePCD2DB();
}
*/
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

void ViewWriter::insertFileIntoGrid(OID& oid, const string& fileType, int totalSize)
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
			//tempFileName = string(fileName)+"."+string(fileType);
			cv::imwrite(tempFileName, tempImg);
		}
		else if(fileType=="txt")	// save to file pcd
		{
			CLOG(LINFO) << "CIP file";
			//string CIP = cipFileIn.read();
			//tempFileName = string(fileName)+"."+string(fileType);
			char const* ca = tempFileName.c_str();
			std::ofstream out(ca);
			//out << CIP;
			out.close();
			CLOG(LINFO)<<"Size of CIP is lower then 16 MB: ";
		}
		else if(fileType=="yaml" || fileType=="yml")	// save to yaml file
		{
			CLOG(LINFO) << "YAML file";
			//tempFileName = string(fileName)+"."+string(fileType);
			cv::FileStorage fs(tempFileName, cv::FileStorage::WRITE);
			fs << "img" << xyzimage;
			fs.release();
		}
		else if(fileType=="pcd")
		{
			CLOG(LINFO) << "PCD file";
			std::string fn = fileName;
			if(cloudType=="xyz")
				saveXYZFileOnDisc(suffix, binary, fn);
			else if(cloudType=="xyzrgb")
				saveXYZRGBFileOnDisc(suffix, binary, fn);
			else if(cloudType=="xyzsift")
				saveXYZSIFTFileOnDisc(suffix, binary, fn);
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


void ViewWriter::writeNode2MongoDB(const string &destination, const string &nodeName,string modelOrViewName, const string& fileType)
{
	CLOG(LTRACE) <<"writeNode2MongoDB";
	OID oid;
	CLOG(LERROR)<<"ViewWriter::writeNode2MongoDB, cloudType: "<<cloudType;
	CLOG(LTRACE) <<"Filename: " << fileName << " destination: "<< destination<<" dbCollectionPath: "<<dbCollectionPath;
    try{
    	//string tempFileName = string(fileName)+"."+string(fileType);
    	CLOG(LERROR)<<tempFileName;
    	float sizeOfFileBytes = getFileSize(fileType);
    	float sizeOfFileMBytes = sizeOfFileBytes/(1024*1024);

    	//TODO change treshold to parameter
    	if(sizeOfFileMBytes>15.0)
    	{
    		CLOG(LERROR)<<"ViewWriter::writeNode2MongoDB, cloudType: "<<cloudType;
			insertFileIntoGrid(oid, fileType, sizeOfFileBytes);
			c->update(dbCollectionPath, Query(BSON("ObjectName"<<objectName<<nodeName+"Name"<<modelOrViewName<<"NodeName"<<destination)), BSON("$addToSet"<<BSON("childOIDs"<<BSON("childOID"<<oid.toString()))), false, true);
    	}
		else
		{
			CLOG(LERROR)<<"ViewWriter::writeNode2MongoDB, cloudType: "<<cloudType;

				CLOG(LERROR)<<"sizeOfFileBytes: "<<sizeOfFileBytes;
				insertFileIntoCollection(oid, fileType, tempFileName, sizeOfFileBytes);
				CLOG(LERROR)<<"UPDATE: "<<oid.toString();
				c->update(dbCollectionPath, Query(BSON("ObjectName"<<objectName<<nodeName+"Name"<<modelOrViewName<<"NodeName"<<destination)), BSON("$addToSet"<<BSON("childOIDs"<<BSON("childOID"<<oid.toString()))), false, true);
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
		b=BSONObjBuilder().genOID().appendBinData(tempFileName, buf.size(), mongo::BinDataGeneral, &buf[0]).append("filename", tempFileName).append("size", size).append("place", "document").append("extension", fileType).obj();
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

			b=BSONObjBuilder().genOID().appendBinData(tempFileName, offset, mongo::BinDataGeneral, data).append("filename", tempFileName).append("size", size).append("place", "document").append("extension", fileType).obj();
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
			b=BSONObjBuilder().genOID().appendBinData(tempFileName, totalSize, mongo::BinDataGeneral, &buff[0]).append("filename", tempFileName).append("size", totalSize).append("place", "document").append("extension", fileType).obj();
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
		CLOG(LTRACE)<<"ViewWriter::insertFileIntoCollection, PCD file end";
	}
	else if (fileType=="yaml" || fileType=="yml")	// save image
	{
		CLOG(LERROR)<<xyzimage.size();
		CLOG(LTRACE)<<"ViewWriter::insertFileIntoCollection, cv::Mat - XYZRGB";
		int bufSize = xyzimage.total()*xyzimage.channels()*4;
		int width = xyzimage.size().width;
		int height = xyzimage.size().height;
		int channels = xyzimage.channels();
		float* buf;
		buf = (float*)xyzimage.data;
		CLOG(LERROR)<<"Set buffer YAML";
		b=BSONObjBuilder().genOID().appendBinData(tempFileName, bufSize, mongo::BinDataGeneral, &buf[0]).append("filename", tempFileName).append("size", size).append("place", "document").append("extension", fileType).append("width", width).append("height", height).append("channels", channels).obj();
		CLOG(LERROR)<<"YAML inserted";
		b.getObjectID(bsonElement);
		oid=bsonElement.__oid();
		c->insert(dbCollectionPath, b);
	}
	else if(fileType=="txt")
	{
		CLOG(LTRACE)<<"ViewWriter::insertFileIntoCollection, txt file";
		string input;

		// read string from input
		//input =cipFileIn.read()+" ";
		CLOG(LERROR)<<"Input:"<< input;
		input="";
		// convert to char*
		char const *cipCharTable = input.c_str();
		int size = strlen(cipCharTable);
		CLOG(LERROR)<<string(cipCharTable);
		CLOG(LERROR)<<"Size: "<<size;
		// create bson object
		b = BSONObjBuilder().genOID().appendBinData(tempFileName, size, BinDataGeneral,  cipCharTable).append("filename", tempFileName).append("size", size).append("place", "document").append("extension", fileType).obj();

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

void ViewWriter::insert2MongoDB(const string &destination, const string&  modelOrViewName, const string&  nodeName,  const string&  fileExtension)
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
				insert2MongoDB(destination, modelOrViewName, nodeName, fileExtension);
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
			writeNode2MongoDB(destination, nodeName, modelOrViewName, fileExtension);
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
