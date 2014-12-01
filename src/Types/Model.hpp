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
#include <Types/MongoProxy.hpp>
#include <Types/PrimitiveFile.hpp>

#include <vector>
#include <list>
#include <iostream>
#include <boost/variant.hpp>
#include <boost/lexical_cast.hpp>

namespace MongoDB {
using namespace PrimitiveFile;
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;

class Model
{
private:
	std::string ModelName;					// model1
	std::vector<shared_ptr<PrimitiveFile::PrimitiveFile> > files;
	//std::vector<AbstractObject*>& models;
	string dateOfInsert;					// 02042013
	std::vector<string>	allFileTypes;			// [MASK, IMAGE, …, IMAGE3D]
	std::string description;
	string hostname;
	string objectName;

	// all required types to store
	boost::shared_ptr<std::vector<fileTypes> > requiredKeyTypes;


	// inserted file types of file
	std::vector<fileTypes> insertedKeyTypes;

public:
	Model(string& modelName, string& host) : ModelName(modelName), hostname(host)
	{
	};
	void setRequiredKeyTypes(boost::shared_ptr<std::vector<fileTypes> > &requiredKeyTypes)
	{
		this->requiredKeyTypes = requiredKeyTypes;
	};
	void setObjectNames(std::string & objectName)
	{
		this->objectName = objectName;
	}

	bool checkIfExist();
	void create();

	// check if exist this same kind of file
	bool checkIfFileExist(fileTypes key);
	void pushFile(shared_ptr<PrimitiveFile::PrimitiveFile>&, fileTypes);

	bool checkIfAllFiles();
	void putStringToFile(const std::string& str, fileTypes key, string& fileName);
	void putMatToFile(const cv::Mat& image, fileTypes key, string& fileName);
	void putPCxyzToFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr&, fileTypes key, string& fileName);
	void putPCyxzrgbToFile(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, fileTypes key, string& fileName);
	void putPCxyzsiftToFile(const pcl::PointCloud<PointXYZSIFT>::Ptr&, fileTypes key, string& fileName);
	void putPCxyzrgbsiftToFile(const pcl::PointCloud<PointXYZRGBSIFT>::Ptr&, fileTypes key, string& fileName);
	void putPCxyzshotToFile(const pcl::PointCloud<PointXYZSHOT>::Ptr&, fileTypes key, string& fileName);
	void putPCxyzrgbNormalToFile(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr&, fileTypes key, string& fileName);

	void addFile();
	void getAllFiles();
	void saveAllFiles();
	void removeFile();
	void removeAllFiles();
	void setModelName();
	void getModelName();
	void setSensorType();
	void getSensorType();
	void setDateOfInsert();
	void getDateOfInsert();
	void loadModel();
	std::vector<AbstractObject*> getModels();
};// class Model

void Model::saveAllFiles()
{

	for(std::vector<boost::shared_ptr<PrimitiveFile::PrimitiveFile> >::iterator it = files.begin(); it != files.end(); ++it)
	{
		it->get()->saveIntoMongoBase();
	}
	return ;
}
void Model::pushFile(shared_ptr<PrimitiveFile::PrimitiveFile>& file, fileTypes key)
{
	file->setModelName(ModelName);
	// add file to vector
	LOG(LNOTICE)<<"Push file to the vector files";
	files.push_back(file);
	LOG(LNOTICE)<<"files.size = "<<files.size();
	insertedKeyTypes.push_back(key);

	// check if all required files are present in model
	bool allFiles = checkIfAllFiles();
	if(allFiles)
	{
		LOG(LNOTICE)<<"Create Model";
		create();
		LOG(LNOTICE)<<"Write files to model";
		saveAllFiles();
	}
	else
	{
		LOG(LNOTICE) << "Waiting for all files to write them to mongoDB";
	}
}

// check if all required types of file are present in vector
// if types are stereo, check  all stereo files and stereo textured files if needed

bool Model::checkIfAllFiles()
{
	LOG(LNOTICE)<<"checkIfAllFiles";
	bool present = false;

	for(std::vector<fileTypes>::iterator reqTypes = requiredKeyTypes->begin(); reqTypes != requiredKeyTypes->end(); ++reqTypes)
	{
		LOG(LNOTICE)<<"requiredKeyTypes loop: ";
		for(std::vector<fileTypes>::iterator insTypes = insertedKeyTypes.begin(); insTypes != insertedKeyTypes.end(); ++insTypes)
		{
			LOG(LNOTICE)<<"insertedKeyTypes loop: ";
			if(*reqTypes==*insTypes)
			{
				LOG(LNOTICE)<<"Present in files, type: "<< *reqTypes;
				present = true;
				break;
			}
		}// for
		if(present)
		{
			LOG(LNOTICE)<<"Present";
			present = false;
		}
		else
			return false;
	}
	return true;
}

bool Model::checkIfExist()
{
	int options=0;
	int limit=0;
	int skip=0;
	BSONObj b = BSON("ModelName"<<ModelName<<"DocumentType"<<"Model");
	int items = MongoProxy::MongoProxy::getSingleton(hostname).count(b);
	LOG(LNOTICE)<<"items: "<<items<<"\n";
	if(items==0)
		return false;
	LOG(LNOTICE)<<"Model document founded! Change model name and try again";
	return true;
}
// check if in model exist this same kind of file
bool Model::checkIfFileExist(fileTypes key)
{
	LOG(LNOTICE)<<"checkIfFileExist, key: "<<key<<", insertedSize: "<<files.size() ;
	for(std::vector<boost::shared_ptr<PrimitiveFile::PrimitiveFile> >::iterator it = files.begin(); it != files.end(); ++it)
	{
		LOG(LNOTICE)<<"file: "<< it->get()->getFileName();
		if(key==it->get()->getType())
			return true;
	}
	return false;
}

void Model::create()
{
	int options=0;
	int limit=0;
	int skip=0;
	BSONArrayBuilder objectArrayBuilder;
	BSONObj model = BSONObjBuilder().genOID().append("ModelName", ModelName).append("DocumentType","Model").append("description", description).obj();
	// get model oid
	BSONElement bsonElement;
	model.getObjectID(bsonElement);
	OID modelOID;
	modelOID=bsonElement.__oid();
	MongoProxy::MongoProxy::getSingleton(hostname).insert(model);
	// thesis: model doesn't exist so doesn't contain any object

	// check if object exist
	BSONObj b = BSON("ObjectName"<<objectName<<"DocumentType"<<"Object");
	int items = MongoProxy::MongoProxy::getSingleton(hostname).count(b);
	// document of object doesn't exist
	if(items==0)
	{
		BSONObj object = BSONObjBuilder().genOID().append("ObjectName", objectName).append("DocumentType","Object").obj();
		MongoProxy::MongoProxy::getSingleton(hostname).insert(object);
	}
	BSONObj query = BSON("ObjectName"<<objectName<<"DocumentType"<<"Object");
	BSONObj update = BSON("$addToSet"<<BSON("ModelsList"<<BSON("modelOID"<<modelOID.toString())));
	// insert model oid to object document
	MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);

	// insert object oid to model document
	BSONObj object;
	query = BSON("ObjectName"<<objectName<<"DocumentType"<<"Object");
	auto_ptr<DBClientCursor> cursorCollection =MongoProxy::MongoProxy::getSingleton(hostname).query(query);
	while(cursorCollection->more())
	{
		object = cursorCollection->next();
	}
	object.getObjectID(bsonElement);
	OID objectOID;
	objectOID=bsonElement.__oid();
	query = BSON("ModelName"<<ModelName<<"DocumentType"<<"Model");
	update = BSON("$addToSet"<<BSON("ObjectsList"<<BSON("objectOID"<<objectOID.toString())));
	MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);
	return;
}

}//namespace MongoBase

#endif /* MODEL_HPP_ */
