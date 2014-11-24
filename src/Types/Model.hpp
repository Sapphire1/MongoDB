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
	std::vector<shared_ptr<PrimitiveFile::PrimitiveFile> > files;
	//std::vector<AbstractObject*>& models;
	string dateOfInsert;					// 02042013
	std::vector<string>	fileTypes;			// [MASK, IMAGE, …, IMAGE3D]
	std::string description;
	string hostname;
	string objectName;

	// all required types to store
	boost::shared_ptr<std::vector<keyTypes> > requiredKeyTypes;

	// inserted file types of file
	std::vector<keyTypes> insertedKeyTypes;


public:
	Model(string& modelName, string& host) : ModelName(modelName), hostname(host)
	{
		dbCollectionPath="images.containers";
		connectToMongoDB(hostname);
	};
	void setRequiredKeyTypes(boost::shared_ptr<std::vector<keyTypes> > &requiredKeyTypes)
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
	bool checkIfFileExist(keyTypes key);
	void pushFile(shared_ptr<PrimitiveFile::PrimitiveFile>&, keyTypes);

	bool checkIfAllFiles();
	void putStringToFile(const std::string& str, keyTypes key, string& fileName);
	void putMatToFile(const cv::Mat& image, keyTypes key, string& fileName);
	void putPCxyzToFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr&, keyTypes key, string& fileName);
	void putPCyxzrgbToFile(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, keyTypes key, string& fileName);
	void putPCxyzsiftToFile(const pcl::PointCloud<PointXYZSIFT>::Ptr&, keyTypes key, string& fileName);
	void putPCxyzrgbsiftToFile(const pcl::PointCloud<PointXYZRGBSIFT>::Ptr&, keyTypes key, string& fileName);
	void putPCxyzshotToFile(const pcl::PointCloud<PointXYZSHOT>::Ptr&, keyTypes key, string& fileName);
	void putPCxyzrgbNormalToFile(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr&, keyTypes key, string& fileName);

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
void Model::pushFile(shared_ptr<PrimitiveFile::PrimitiveFile>& file, keyTypes key)
{
	string documentType = "Model";
	file->setDocumentType(documentType);
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
	bool stereoLPresent = false;
	bool stereoRPresent = false;
	bool stereoLTexturedPresent = false;
	bool stereoRTexturedPresent = false;

	for(std::vector<keyTypes>::iterator reqTypes = requiredKeyTypes->begin(); reqTypes != requiredKeyTypes->end(); ++reqTypes)
	{
		LOG(LNOTICE)<<"requiredKeyTypes loop: ";
		for(std::vector<keyTypes>::iterator insTypes = insertedKeyTypes.begin(); insTypes != insertedKeyTypes.end(); ++insTypes)
		{
			LOG(LNOTICE)<<"insertedKeyTypes loop: ";
			if(*reqTypes==*insTypes)
			{
				LOG(LNOTICE)<<"Present in files, type: "<< *reqTypes;
				present = true;
				break;
			}
			else if(*insTypes==stereoL)
				stereoLPresent = true;
			else if(*insTypes==stereoR)
				stereoRPresent = true;
			else if(*insTypes==stereoLTextured)
				stereoLTexturedPresent = true;
			else if(*insTypes==stereoRTextured)
				stereoRTexturedPresent = true;
		}// for
		if(present)
		{
			LOG(LNOTICE)<<"Present";
			present = false;
		}
		else if(*reqTypes==stereo)
		{
			if(!stereoLPresent || !stereoRPresent)
				return false;
		}
		else if(*reqTypes==stereoTextured)
		{
			if(!stereoLPresent ||  !stereoRPresent || !stereoLTexturedPresent || !stereoRTexturedPresent)
				return false;
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
	int items = c->count(dbCollectionPath, BSON("ModelName"<<ModelName<<"DocumentType"<<"Model"), options, limit, skip);
	LOG(LNOTICE)<<"items: "<<items<<"\n";
	if(items==0)
		return false;
	LOG(LNOTICE)<<"Model document founded! Change model name and try again";
	return true;
}
// check if in model exist this same kind of file
bool Model::checkIfFileExist(keyTypes key)
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
	c->insert(dbCollectionPath, model);
	// thesis: model doesn't exist so doesn't contain any object

	// check if object exist
	int items = c->count(dbCollectionPath, BSON("ObjectName"<<objectName<<"DocumentType"<<"Object"), options, limit, skip);
	// document of object doesn't exist
	if(items==0)
	{
		BSONObj object = BSONObjBuilder().genOID().append("ObjectName", objectName).append("DocumentType","Object").obj();
		c->insert(dbCollectionPath, object);
	}
	// insert model oid to object document
	c->update(dbCollectionPath, Query(BSON("ObjectName"<<objectName<<"DocumentType"<<"Object")), BSON("$addToSet"<<BSON("ModelsList"<<BSON("modelOID"<<modelOID.toString()))), false, true);

	// insert object oid to model document
	BSONObj object;
	auto_ptr<DBClientCursor> cursorCollection =c->query(dbCollectionPath, (Query(BSON("ObjectName"<<objectName<<"DocumentType"<<"Object"))));
	while(cursorCollection->more())
	{
		object = cursorCollection->next();
	}
	object.getObjectID(bsonElement);
	OID objectOID;
	objectOID=bsonElement.__oid();
	c->update(dbCollectionPath, Query(BSON("ModelName"<<ModelName<<"DocumentType"<<"Model")), BSON("$addToSet"<<BSON("ObjectsList"<<BSON("objectOID"<<objectOID.toString()))), false, true);
	return;
}

void Model::putMatToFile(const cv::Mat& img, keyTypes key, string& fileName)
{
	LOG(LNOTICE)<< "Model::putMatToFile";
	LOG(LNOTICE)<< "key: "<<key;
	shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(img, key, c, fileName));
	pushFile(file, key);
}

void Model::putStringToFile(const std::string& str, keyTypes key, string& fileName)
{
	LOG(LNOTICE)<< "Model::putStringToFile";
	LOG(LNOTICE)<< "key: "<<key;
	shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(str, key, c, fileName));
	pushFile(file, key);
}

void Model::putPCxyzToFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudXYZ, keyTypes key, string& fileName)
{
	LOG(LNOTICE)<< "Model::putPCxyzToFile";
	LOG(LNOTICE)<< "key: "<<key;
	shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(cloudXYZ, key, c, fileName));
	pushFile(file, key);
}
void Model::putPCyxzrgbToFile(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudXYZRGB, keyTypes key, string& fileName)
{
	LOG(LNOTICE)<< "Model::putPCyxzrgbToFile";
	LOG(LNOTICE)<< "key: "<<key;
	shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(cloudXYZRGB, key, c, fileName));
	pushFile(file, key);
}
void Model::putPCxyzsiftToFile(const pcl::PointCloud<PointXYZSIFT>::Ptr& cloudXYZSIFT, keyTypes key, string& fileName)
{
	LOG(LNOTICE)<< "Model::putPCxyzsiftToFile";
	LOG(LNOTICE)<< "key: "<<key;
	shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(cloudXYZSIFT, key, c, fileName));
	pushFile(file, key);
}

void Model::putPCxyzrgbsiftToFile(const pcl::PointCloud<PointXYZRGBSIFT>::Ptr& cloudXYZRGBSIFT, keyTypes key, string& fileName)
{
	LOG(LNOTICE)<< "Model::putPCxyzrgbsiftToFile";
	LOG(LNOTICE)<< "key: "<<key;
	shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(cloudXYZRGBSIFT, key, c, fileName));
	pushFile(file, key);
}

void Model::putPCxyzshotToFile(const pcl::PointCloud<PointXYZSHOT>::Ptr& cloudXYZSHOT, keyTypes key, string& fileName)
{
	LOG(LNOTICE)<< "Model::putPCxyzshotToFile";
	LOG(LNOTICE)<< "key: "<<key;
	shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(cloudXYZSHOT, key, c, fileName));
	pushFile(file, key);
}

void Model::putPCxyzrgbNormalToFile(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloudXYZRGBNormal, keyTypes key, string& fileName)
{
	LOG(LNOTICE)<< "Model::putPCxyzrgbNormalToFile";
	LOG(LNOTICE)<< "key: "<<key;
	shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(cloudXYZRGBNormal, key, c, fileName));
	pushFile(file, key);
}

}//namespace MongoBase

#endif /* MODEL_HPP_ */
