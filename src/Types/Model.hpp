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
#include <exception>

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
	string dateOfInsert;					// 02042013
	std::vector<string>	allFileTypes;			// [MASK, IMAGE, …, IMAGE3D]
	std::string description;
	string hostname;
	string viewSetName;
	BSONObj modelDocument;

	// all required types to store
	boost::shared_ptr<std::vector<fileTypes> > requiredKeyTypes;


	// inserted file types of file
	std::vector<fileTypes> insertedKeyTypes;

public:
	Model(string& modelName, string& host) : ModelName(modelName), hostname(host)
	{
		//readModelDocument();
	};
	void setRequiredKeyTypes(boost::shared_ptr<std::vector<fileTypes> > &requiredKeyTypes)
	{
		this->requiredKeyTypes = requiredKeyTypes;
	};
	void setViewsSetNames(std::string & viewSetName)
	{
		this->viewSetName = viewSetName;
	}

	bool checkIfExist();
	void create();

	// check if exist this same kind of file
	bool checkIfFileExist(fileTypes key);
	void pushFile(shared_ptr<PrimitiveFile::PrimitiveFile>&, fileTypes);
	void readModelDocument();
	bool checkIfAllFiles();
	int getAllFilesOIDS(vector<OID>& oidsVector);
	void putStringToFile(const std::string& str, fileTypes key, string& fileName);
	void putMatToFile(const cv::Mat& image, fileTypes key, string& fileName);
	void putPCxyzToFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr&, fileTypes key, string& fileName);
	void putPCyxzrgbToFile(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, fileTypes key, string& fileName);
	void putPCxyzsiftToFile(const pcl::PointCloud<PointXYZSIFT>::Ptr&, fileTypes key, string& fileName);
	void putPCxyzrgbsiftToFile(const pcl::PointCloud<PointXYZRGBSIFT>::Ptr&, fileTypes key, string& fileName);
	void putPCxyzshotToFile(const pcl::PointCloud<PointXYZSHOT>::Ptr&, fileTypes key, string& fileName);
	void putPCxyzrgbNormalToFile(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr&, fileTypes key, string& fileName);
	void readFiles(vector<OID>& fileOIDSVector, vector<fileTypes>& requiredFileTypes);
	int getFilesSize()
	{
		return files.size();
	};
	fileTypes getFileType(int i)
	{
		return files[i]->getType();
	};
	shared_ptr<PrimitiveFile::PrimitiveFile> getFile(int pos)
	{
		return files[pos];
	}
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
	bool checkIfContain(std::vector<fileTypes> & requiredFileTypes);
	bool getModelTypes(BSONObj &obj, const string & fieldName, const string & childfieldName, vector<fileTypes>& typesVector);
	void addFile(shared_ptr<PrimitiveFile::PrimitiveFile>& file, string& type, bool dataInBuffer, string& path);


};// class Model
void Model::addFile(shared_ptr<PrimitiveFile::PrimitiveFile>& file, string& type, bool dataInBuffer, string& path)
{
	OID oid;
	string documentType= "Model";
	file->saveIntoMongoBase(documentType, ModelName, true, path, oid);
	BSONObj query;
	// update document
	query = BSON("ModelName"<<ModelName<<"DocumentType"<<documentType);

	LOG(LDEBUG)<<"OID: "<<oid.toString();
	BSONObj update = BSON("$addToSet"<<BSON("fileOIDs"<<BSON("fileOID"<<oid.toString())));
	MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);

	update = BSON("$addToSet"<<BSON("FileTypes"<<BSON("Type"<<FTypes[file->getType()])));
	MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);
}

void Model::readFiles(vector<OID>& fileOIDSVector, vector<fileTypes>& requiredFileTypes)
{
	LOG(LNOTICE)<<"Model::readFiles";
	for(std::vector<OID>::iterator fileOIDIter = fileOIDSVector.begin(); fileOIDIter != fileOIDSVector.end(); ++fileOIDIter)
	{
		BSONObj query = BSON("_id" << *fileOIDIter);
		BSONObj file = MongoProxy::MongoProxy::getSingleton(hostname).findOne(query);
		string fileType = file.getField("fileType").str();
		LOG(LNOTICE)<<"fileType : " <<fileType;
		// map from string to enum
		fileTypes ft;
		//for(int i=0; i<18; i++)
		for(int i=0; i<StereoTextured; i++)
		{
			if(fileType == FTypes[i])
			{
				LOG(LNOTICE)<<fileType <<"="<< FTypes[i];
				ft = (fileTypes)i;
				break;
			}
		}

		for(std::vector<fileTypes>::iterator reqFileType = requiredFileTypes.begin(); reqFileType != requiredFileTypes.end(); ++reqFileType)
		{
			LOG(LNOTICE)<<"for(std::vector<fileTypes>::iterator reqFileType";
			LOG(LNOTICE)<<"ft : " <<ft << "*reqFileType: " <<*reqFileType;
			if(ft==*reqFileType)
			{
				LOG(LNOTICE)<<"READ FILE!!!";
				string empty = "";
				shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(ft, hostname, *fileOIDIter));
				file->readFile(true, empty, false);
				files.push_back(file);
			}
		}
	}
}

int Model::getAllFilesOIDS(vector<OID>& oidsVector)
{
	string fieldName = "fileOIDs";
	string childfieldName = "fileOID";
	string output = modelDocument.getField(fieldName);
	if(output!="EOO")
	{
		vector<BSONElement> v = modelDocument.getField(fieldName).Array();
		for (unsigned int i = 0; i<v.size(); i++)
		{
			string readedOid =v[i][childfieldName].str();
			OID o = OID(readedOid);
			oidsVector.push_back(o);
			LOG(LNOTICE)<<"FileOID : "<<o;
		}
		return 1;
	}
	else
		return -1;
}

void Model::readModelDocument()
{
	BSONObj query = BSON("ModelName"<<ModelName<<"DocumentType"<<"Model");
	modelDocument = MongoProxy::MongoProxy::getSingleton(hostname).findOne(query);
}

bool Model::getModelTypes(BSONObj &obj, const string & fieldName, const string & childfieldName, vector<fileTypes>& fileTypesVector)
{
	LOG(LNOTICE)<<"Model::getModelTypes";
	LOG(LNOTICE)<<"fieldName : "<<fieldName;

	 string output = obj.getField(fieldName);
	 LOG(LNOTICE)<<output;


	if(output!="EOO")
	{
		LOG(LNOTICE)<<output;
		vector<BSONElement> v = obj.getField(fieldName).Array();
		for (unsigned int i = 0; i<v.size(); i++)
		{
			// read to string
			LOG(LNOTICE)<<v[i][childfieldName].String();
			fileTypes ft=fileTypes(-1);
			LOG(LNOTICE)<<"modelFileType: "<<v[i][childfieldName].String();
			// map from string to enum
			//for(int j=0; j<18;j++)
			for(int j=0; j<StereoTextured;j++)
			{

				if(v[i][childfieldName].String() == FTypes[j])
				{
					LOG(LNOTICE)<<v[i][childfieldName].String() <<" == " << FTypes[j];
					ft = (fileTypes)j;
					LOG(LNOTICE)<<ft;
					break;
				}
			//	else
			//		ft=fileTypes(-1);
			}
			LOG(LNOTICE)<<" ft: "<<ft;
			// insert enum to vector
			if(ft>-1)
				fileTypesVector.push_back(ft);
		}
		return true;
	}
	else

		return false;


}

bool Model::checkIfContain(std::vector<fileTypes> & requiredFileTypes)
{
	//read fileTypes from view document
	vector<fileTypes> fileTypesVector;
	string fieldName = "FileTypes";
	string childfieldName = "Type";

	LOG(LNOTICE)<<"modelDocument: "<<modelDocument;

	bool arrayNotEmpty = getModelTypes(modelDocument,fieldName, childfieldName, fileTypesVector);

	if(!arrayNotEmpty)
	{
		LOG(LNOTICE)<<"Array is empty!!!";
		return false;
	}

	bool present = false;

	if(requiredFileTypes.size()>fileTypesVector.size())
	{
		LOG(LERROR)<<"You want to get more files then view has! "<< requiredFileTypes.size()<<" : "<<fileTypesVector.size();
		return false;
	}
	for(std::vector<fileTypes>::iterator reqTypes = requiredFileTypes.begin(); reqTypes != requiredFileTypes.end(); ++reqTypes)
	{
		for(std::vector<fileTypes>::iterator modelTypes = fileTypesVector.begin(); modelTypes != fileTypesVector.end(); ++modelTypes)
		{
			LOG(LNOTICE)<<"reqTypes : "<<*reqTypes;
			LOG(LNOTICE)<<"modelTypes : "<<*modelTypes;

			if(*modelTypes==*reqTypes)
			{
				present = true;
				break;
			}
		}
		if(present)
			present = false;
		else
			return false;

	}

	return true;
}

void Model::saveAllFiles()
{

	for(std::vector<boost::shared_ptr<PrimitiveFile::PrimitiveFile> >::iterator it = files.begin(); it != files.end(); ++it)
	{
		string type = "Model";
		//it->get()->saveIntoMongoBase(type, ModelName, false, "");
	}
	return ;
}

void Model::pushFile(shared_ptr<PrimitiveFile::PrimitiveFile>& file, fileTypes key)
{
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

	// check if ViewsSet exist
	BSONObj b = BSON("ViewsSetName"<<viewSetName<<"DocumentType"<<"ViewsSet");
	int items = MongoProxy::MongoProxy::getSingleton(hostname).count(b);
	// document of object doesn't exist
	if(items==0)
	{
		BSONObj viewSet = BSONObjBuilder().genOID().append("ViewsSetName", viewSetName).append("DocumentType","ViewsSet").obj();
		MongoProxy::MongoProxy::getSingleton(hostname).insert(viewSet);
	}
	BSONObj query = BSON("ViewsSetName"<<viewSetName<<"DocumentType"<<"ViewsSet");
	BSONObj update = BSON("$addToSet"<<BSON("ModelsList"<<BSON("modelOID"<<modelOID.toString())));
	// insert model oid to object document
	MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);

	// insert viewSet oid to model document
	query = BSON("ViewsSetName"<<viewSetName<<"DocumentType"<<"ViewsSet");
	BSONObj viewSet= MongoProxy::MongoProxy::getSingleton(hostname).findOne(query);
	viewSet.getObjectID(bsonElement);
	OID viewSetOID=bsonElement.__oid();
	query = BSON("ModelName"<<ModelName<<"DocumentType"<<"Model");
	update = BSON("$addToSet"<<BSON("ViewsSetsList"<<BSON("objectOID"<<viewSetOID.toString())));
	MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);

	update = BSON("$addToSet"<<BSON("viewsSetNamesList"<<BSON("ViewsSetName"<<viewSetName)));
	MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);
	return;
}

}//namespace MongoBase

#endif /* MODEL_HPP_ */
