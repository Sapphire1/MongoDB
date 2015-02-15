/*
 * View.hpp
 *
 *  Created on: Nov 20, 2014
 *      Author: lzmuda
 */

#ifndef VIEW_HPP_
#define VIEW_HPP_

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
#include <iostream>

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

namespace MongoDB{
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace PrimitiveFile;

class View
{
private:
	string ViewName;						// lab012
	string SensorType;						// Stereo, ToF...
	string dateOfInsert;					// 02042013
	std::vector<string>	allFileTypes;			// [MASK, IMAGE, …, IMAGE3D]
	std::string description;
	std::vector<std::string> splitedViewsSet;
	string hostname;
	BSONObj viewDocument;
	// all required types to store
	boost::shared_ptr<std::vector<fileTypes> > requiredKeyTypes;

	// inserted file types of file
	std::vector<fileTypes> insertedKeyTypes;

	// pointer to MongoBase object
//	boost::shared_ptr<MongoBase> basePtr;
	std::vector<shared_ptr<PrimitiveFile::PrimitiveFile> > files;


public:

	View(string& viewName, string& host) : ViewName(viewName), hostname(host)
	{
	};
	void setRequiredKeyTypes(boost::shared_ptr<std::vector<fileTypes> > &requiredKeyTypes)
	{
		this->requiredKeyTypes = requiredKeyTypes;
	};
	void getAllFiles();
	void saveAllFiles();
	void setViewName();
	void getViewName();
	shared_ptr<PrimitiveFile::PrimitiveFile> getFile(int pos);
	int getFilesSize()
	{
		return files.size();
	};
	int getAllFilesOIDS(vector<OID>& oidsVector);
	void setViewsSetNames(std::vector<std::string> & splitedViewsSet)
	{
		this->splitedViewsSet = splitedViewsSet;
	}
	void setSensorType(string& type)
	{
		SensorType = type;
	};
	BSONObj getDocument()
	{
		return viewDocument;
	}
	void getSensorType();
	void setDateOfInsert();
	bool checkIfExist();
	void getDateOfInsert();
	//void create();
	void create(OID& sceneOID, OID& viewOID, string& sceneName);

	// check if exist this same kind of file
	bool checkIfFileExist(fileTypes key);
	//void pushFile(shared_ptr<PrimitiveFile::PrimitiveFile>&, fileTypes);
	bool checkIfContain(std::vector<fileTypes> & requiredFileTypes);
	void readViewDocument();
	bool checkIfAllFiles();
	void putStringToFile(const std::string& str, fileTypes typ, string& fileName);
	void putMatToFile(const cv::Mat& image, fileTypes typ, string& fileName);
	void putPCxyzToFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr&, fileTypes typ, string& fileName);
	void putPCyxzrgbToFile(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, fileTypes typ, string& fileName);
	void putPCxyzsiftToFile(const pcl::PointCloud<PointXYZSIFT>::Ptr&, fileTypes typ, string& fileName);
	void putPCxyzrgbsiftToFile(const pcl::PointCloud<PointXYZRGBSIFT>::Ptr&, fileTypes typ, string& fileName);
	void putPCxyzshotToFile(const pcl::PointCloud<PointXYZSHOT>::Ptr&, fileTypes typ, string& fileName);
	void putPCxyzrgbNormalToFile(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr&, fileTypes typ, string& fileName);
	bool getViewTypes(BSONObj &obj, const string & fieldName, const string & childfieldName, vector<fileTypes>& typesVector);
	void getRequiredFiles(vector<fileTypes>& requiredFileTypes);
	void addScene(string& sceneName, OID& sceneOID);
	void getSceneName(string& name);
	fileTypes getFileType(int i);
	void getID(OID& id);
	void getViewsSetName(string& name);
	void getViewsSetOID(vector<OID>& viewsSetOIDS, string& tableName, string& fieldName);
};// class View

void View::getSceneName(string& name)
{
	name = viewDocument["SceneName"].toString(false,false);
	name.erase(name.begin());
	name.erase(name.end()-1);
}

/*
void View::getViewsSetName(string& name)
{
	name = viewDocument["ViewsSet"].toString(false,false);
	LOG(LNOTICE) << "name: " <<name;
	name.erase(name.begin());
	LOG(LNOTICE) << "name: " <<name;
	name.erase(name.end()-1);
	LOG(LNOTICE) << "name: " <<name;
}
*/

// return viewsSetsOids in vector of string
void View::getViewsSetOID(vector<OID>& viewsSetOIDS, string& tableName, string& fieldName)
{
	vector<BSONElement> v = viewDocument.getField(tableName).Array();
	for (unsigned int i = 0; i<v.size(); i++)
	{
		string readedOid =v[i][fieldName].str();
		OID o = OID(readedOid);
		viewsSetOIDS.push_back(o);
		LOG(LNOTICE)<<"DocumentOID : "<<o;
	}
}

void View::getID(OID& id)
{
	BSONElement oi;
	viewDocument.getObjectID(oi);
	id= oi.__oid();
}

void View::getAllFiles()
{
	LOG(LNOTICE)<<"View::getAllFiles";
	vector<OID> fileOIDSVector;
	int filesNumber = getAllFilesOIDS(fileOIDSVector);
	for(std::vector<OID>::iterator fileOIDIter = fileOIDSVector.begin(); fileOIDIter != fileOIDSVector.end(); ++fileOIDIter)
	{
		BSONObj query = BSON("_id" << *fileOIDIter);
		BSONObj bsonfile = MongoProxy::MongoProxy::getSingleton(hostname).findOne(query);
		string fileType = bsonfile.getField("fileType").str();
		LOG(LNOTICE)<<"fileType : " <<fileType;

		// map from string to enum
		// now stereoTextured is equal 18, it's the last one type
		fileTypes ft;
		for(int i=0; i<StereoTextured;i++)
		{
			if(fileType == FTypes[i])
			{
				ft = (fileTypes)i;
				break;
			}
		}
		string empty = "";
		LOG(LNOTICE)<<"READ FILE!!!";
		shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(ft, hostname, *fileOIDIter));
		file->readFile(true, empty, false);
		files.push_back(file);
	}
}
void View::saveAllFiles()
{

	for(std::vector<boost::shared_ptr<PrimitiveFile::PrimitiveFile> >::iterator it = files.begin(); it != files.end(); ++it)
	{
		string type = "View";
		//it->get()->saveIntoMongoBase(type, ViewName);
	}
	return ;
}

void View::readViewDocument()
{
	BSONObj query = BSON("ViewName"<<ViewName<<"DocumentType"<<"View");
	viewDocument = MongoProxy::MongoProxy::getSingleton(hostname).findOne(query);
}

/*
void View::pushFile(shared_ptr<PrimitiveFile::PrimitiveFile>& file, fileTypes typ)
{
	// add file to vector
	files.push_back(file);
	insertedKeyTypes.push_back(typ);

	// check if all required files are present in view
	bool allFiles = checkIfAllFiles();
	if(allFiles)
	{
		LOG(LNOTICE)<<"Create View";
		create();
		LOG(LNOTICE)<<"Write files to view";
		saveAllFiles();
	}
	else
	{
		LOG(LNOTICE) << "Waiting for all files to write them to mongoDB";
	}
}
*/
// check if all required types of file are present in vector
// if types are stereo, check  all stereo files and stereo textured files if needed

/*void View::getDataOfFile(int iterator)
{



}
*/

bool View::checkIfAllFiles()
{
	LOG(LNOTICE)<<"checkIfAllFiles";
	bool present = false;
	bool stereoLPresent = false;
	bool stereoRPresent = false;
	bool stereoLTexturedPresent = false;
	bool stereoRTexturedPresent = false;

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
			else if(*insTypes==StereoLeft)
				stereoLPresent = true;
			else if(*insTypes==StereoRight)
				stereoRPresent = true;
			else if(*insTypes==StereoLeftTextured)
				stereoLTexturedPresent = true;
			else if(*insTypes==StereoRightTextured)
				stereoRTexturedPresent = true;
		}// for
		if(present)
		{
			LOG(LNOTICE)<<"Present";
			present = false;
		}
		else if(*reqTypes==Stereo)
		{
			if(!stereoLPresent || !stereoRPresent)
				return false;
		}
		else if(*reqTypes==StereoTextured)
		{
			if(!stereoLPresent ||  !stereoRPresent || !stereoLTexturedPresent || !stereoRTexturedPresent)
				return false;
		}
		else
			return false;
	}
	return true;
}

bool View::checkIfExist()
{
	BSONObj b = BSON("ViewName"<<ViewName<<"DocumentType"<<"View");\
	int items = MongoProxy::MongoProxy::getSingleton(hostname).count(b);
	LOG(LNOTICE)<<"items: "<<items<<"\n";
	if(items==0)
		return false;
	else
	{
		LOG(LNOTICE)<<"View document founded!";
		return true;
	}
}
// check if in view exist this same kind of file
bool View::checkIfFileExist(fileTypes typ)
{
	for(std::vector<boost::shared_ptr<PrimitiveFile::PrimitiveFile> >::iterator it = files.begin(); it != files.end(); ++it)
	{
		if(typ==it->get()->getType())
			return true;
	}
	return false;
}

fileTypes View::getFileType(int i)
{
	return files[i]->getType();
}

bool View::getViewTypes(BSONObj &obj, const string & fieldName, const string & childfieldName, vector<fileTypes>& fileTypesVector)
{

	LOG(LNOTICE)<<"View::getModelTypes";
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
			LOG(LNOTICE)<<"viewFileType: "<<v[i][childfieldName].String();
			// map from string to enum

			for(int j=0; j<18;j++)
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

void View::getRequiredFiles(vector<fileTypes>& requiredFileTypes)
{
	// read vector of files OIDs
	vector<OID> fileOIDSVector;
	getAllFilesOIDS(fileOIDSVector);
	LOG(LNOTICE)<<"View::readFiles";
	for(std::vector<OID>::iterator fileOIDIter = fileOIDSVector.begin(); fileOIDIter != fileOIDSVector.end(); ++fileOIDIter)
	{
		BSONObj query = BSON("_id" << *fileOIDIter);
		BSONObj file = MongoProxy::MongoProxy::getSingleton(hostname).findOne(query);
		string fileType = file.getField("fileType").str();
		LOG(LNOTICE)<<"fileType : " <<fileType;
		// map from string to enum
		fileTypes ft;
		// now stereoTextured is equal 18, it's the last one type
		for(int i=0; i<StereoTextured;i++)
		{
			if(fileType == FTypes[i])
			{
				ft = (fileTypes)i;
				break;
			}
		}

		for(std::vector<fileTypes>::iterator reqFileType = requiredFileTypes.begin(); reqFileType != requiredFileTypes.end(); ++reqFileType)
		{
			LOG(LNOTICE)<<"for(std::vector<fileTypes>::iterator reqFileType";
			LOG(LNOTICE)<<"ft : " <<ft << "*reqFileType: " <<*reqFileType;
			string empty = "";
			if(ft==*reqFileType)
			{
				LOG(LNOTICE)<<"READ FILE!!!";
				shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(ft, hostname, *fileOIDIter));
				file->readFile(true, empty, false);
				files.push_back(file);
			}
		}
	}
}

shared_ptr<PrimitiveFile::PrimitiveFile> View::getFile(int pos)
{
	return files[pos];
}

bool View::checkIfContain(std::vector<fileTypes> & requiredFileTypes)
{
	//read fileTypes from view document
	vector<fileTypes> fileTypesVector;
	string fieldName = "FileTypes";
	string childfieldName = "Type";
	bool arrayNotEmpty = getViewTypes(viewDocument,fieldName, childfieldName, fileTypesVector);

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
		for(std::vector<fileTypes>::iterator viewTypes = fileTypesVector.begin(); viewTypes != fileTypesVector.end(); ++viewTypes)
		{
			LOG(LNOTICE)<<"reqTypes : "<<*reqTypes;
			LOG(LNOTICE)<<"viewTypes : "<<*viewTypes;

			if(*viewTypes==*reqTypes)
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

int View::getAllFilesOIDS(vector<OID>& oidsVector)
{
	string fieldName = "fileOIDs";
	string childfieldName = "fileOID";
	string output = viewDocument.getField(fieldName);
	if(output!="EOO")
	{
		vector<BSONElement> v = viewDocument.getField(fieldName).Array();
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

void View::create(OID& sceneOID, OID& viewOID, string& sceneName)
{
	BSONArrayBuilder objectArrayBuilder;
	BSONObj view = BSONObjBuilder().genOID().append("ViewName", ViewName).append("SceneOID", sceneOID).append("SceneName", sceneName).append("DocumentType","View").append("sensorType", SensorType).append("description", description).obj();
	// get view oid
	BSONElement bsonElement;
	view.getObjectID(bsonElement);
	viewOID=bsonElement.__oid();
	MongoProxy::MongoProxy::getSingleton(hostname).insert(view);
	for(std::vector<string>::iterator viewSetIt = splitedViewsSet.begin(); viewSetIt != splitedViewsSet.end(); ++viewSetIt)
	{
		BSONObj viewsSet;
		bool created = false;
		BSONObj b = BSON("ViewsSetName"<<*viewSetIt<<"DocumentType"<<"ViewsSet");
		// check if object exist
		int items = MongoProxy::MongoProxy::getSingleton(hostname).count(b);
		// document of viewsSet doesn't exist, create it
		if(items==0)
		{
			viewsSet = BSONObjBuilder().genOID().append("ViewsSetName", *viewSetIt).append("DocumentType","ViewsSet").obj();
			MongoProxy::MongoProxy::getSingleton(hostname).insert(viewsSet);
			created=true;
		}
		// insert view oid to viewsSet document
		BSONObj query = BSON("ViewsSetName"<<*viewSetIt<<"DocumentType"<<"ViewsSet");
		BSONObj update = BSON("$addToSet"<<BSON("ViewsList"<<BSON("viewOID"<<viewOID.toString())));
		MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);

		// insert viewsSet oid to view document
		if(!created)
		{
			query = BSON("ViewsSetName"<<*viewSetIt<<"DocumentType"<<"ViewsSet");
			viewsSet =  MongoProxy::MongoProxy::getSingleton(hostname).findOne(query);
		}
		viewsSet.getObjectID(bsonElement);
		OID viewSetOID;
		// insert viewsetOID to view
		viewSetOID=bsonElement.__oid();
		query = BSON("ViewName"<<ViewName<<"DocumentType"<<"View");
		update = BSON("$addToSet"<<BSON("viewSetList"<<BSON("ViewsSetOID"<<viewSetOID.toString())));
		MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);
	}
	return;
}

void View::addScene(string& sceneName, OID& sceneOID)
{
	BSONObj query = BSON("ViewName"<<ViewName<<"DocumentType"<<"View");
	BSONObj update = BSON("$set"<<BSON("SceneName"<<sceneName<<"sceneOID"<<sceneOID));
	MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);
}

void View::putMatToFile(const cv::Mat& img, fileTypes typ, string& fileName)
{
	LOG(LNOTICE)<< "View::putMatToFile";
	LOG(LNOTICE)<< "key: "<<typ;
}

void View::putStringToFile(const std::string& str, fileTypes typ, string& fileName)
{
	LOG(LNOTICE)<< "View::putStringToFile";
	LOG(LNOTICE)<< "key: "<<typ;

}

void View::putPCxyzToFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudXYZ, fileTypes typ, string& fileName)
{
	LOG(LNOTICE)<< "View::putPCxyzToFile";
	LOG(LNOTICE)<< "key: "<<typ;
}
void View::putPCyxzrgbToFile(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudXYZRGB, fileTypes typ, string& fileName)
{
	LOG(LNOTICE)<< "View::putPCyxzrgbToFile";
	LOG(LNOTICE)<< "key: "<<typ;
}
void View::putPCxyzsiftToFile(const pcl::PointCloud<PointXYZSIFT>::Ptr& cloudXYZSIFT, fileTypes typ, string& fileName)
{
	LOG(LNOTICE)<< "View::putPCxyzsiftToFile";
	LOG(LNOTICE)<< "key: "<<typ;
}

void View::putPCxyzrgbsiftToFile(const pcl::PointCloud<PointXYZRGBSIFT>::Ptr& cloudXYZRGBSIFT, fileTypes typ, string& fileName)
{
	LOG(LNOTICE)<< "View::putPCxyzrgbsiftToFile";
	LOG(LNOTICE)<< "key: "<<typ;
}

void View::putPCxyzshotToFile(const pcl::PointCloud<PointXYZSHOT>::Ptr& cloudXYZSHOT, fileTypes typ, string& fileName)
{
	LOG(LNOTICE)<< "View::putPCxyzshotToFile";
	LOG(LNOTICE)<< "key: "<<typ;
}

void View::putPCxyzrgbNormalToFile(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloudXYZRGBNormal, fileTypes typ, string& fileName)
{
	LOG(LNOTICE)<< "View::putPCxyzrgbNormalToFile";
	LOG(LNOTICE)<< "key: "<<typ;
}

}//namespace MongoBase




#endif /* VIEW_HPP_ */
