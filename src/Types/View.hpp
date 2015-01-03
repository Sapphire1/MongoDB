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
	std::vector<std::string> splitedObjectNames;
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
	void addFile();
	void getAllFiles();
	void saveAllFiles();
	void removeFile();
	void removeAllFiles();
	void setViewName();
	void getViewName();
	shared_ptr<PrimitiveFile::PrimitiveFile> getFile(int pos);
	int getFilesSize()
	{
		return files.size();
	};
	int getAllFilesOIDS(vector<OID>& oidsVector);
	void setObjectNames(std::vector<std::string> & splitedObjectNames)
	{
		this->splitedObjectNames = splitedObjectNames;
	}
	void setSensorType(string& type)
	{
		SensorType = type;
	};
	void getSensorType();
	void setDateOfInsert();
	bool checkIfExist();
	void getDateOfInsert();
	void create();

	// check if exist this same kind of file
	bool checkIfFileExist(fileTypes key);
	void pushFile(shared_ptr<PrimitiveFile::PrimitiveFile>&, fileTypes);
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
	void readFiles(vector<OID>& fileOIDSVector, vector<fileTypes>& requiredFileTypes);
	fileTypes getFileType(int i);
};// class View

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
	auto_ptr<DBClientCursor> cursorCollection;
	BSONObj query = BSON("ViewName"<<ViewName<<"DocumentType"<<"View");
	cursorCollection =MongoProxy::MongoProxy::getSingleton(hostname).query(query);
	while(cursorCollection->more())
	{
		viewDocument = cursorCollection->next();
	}
}

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
		LOG(LNOTICE)<<"View document founded! Change view name and try again";
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

void View::readFiles(vector<OID>& fileOIDSVector, vector<fileTypes>& requiredFileTypes)
{
	LOG(LNOTICE)<<"View::readFiles";
	for(std::vector<OID>::iterator fileOIDIter = fileOIDSVector.begin(); fileOIDIter != fileOIDSVector.end(); ++fileOIDIter)
	{
		BSONObj query = BSON("_id" << *fileOIDIter);
		BSONObj file = MongoProxy::MongoProxy::getSingleton(hostname).findOne(query);
		string fileType = file.getField("fileType").str();
		LOG(LNOTICE)<<"fileType : " <<fileType;
		// map from string to enum
		fileTypes ft;
		for(int i=0; i<18;i++)
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
			if(ft==*reqFileType)
			{
				LOG(LNOTICE)<<"READ FILE!!!";
				shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(ft, hostname, *fileOIDIter));
				file->readFile();
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

void View::create()
{
	BSONArrayBuilder objectArrayBuilder;
	BSONObj view = BSONObjBuilder().genOID().append("ViewName", ViewName).append("DocumentType","View").append("sensorType", SensorType).append("description", description).obj();
	// get view oid
	BSONElement bsonElement;
	view.getObjectID(bsonElement);
	OID viewOID;
	viewOID=bsonElement.__oid();
	MongoProxy::MongoProxy::getSingleton(hostname).insert(view);
	for(std::vector<string>::iterator itObject = splitedObjectNames.begin(); itObject != splitedObjectNames.end(); ++itObject)
	{
		BSONObj b = BSON("ObjectName"<<*itObject<<"DocumentType"<<"Object");
		// check if object exist
		int items = MongoProxy::MongoProxy::getSingleton(hostname).count(b);
		// document of object doesn't exist
		if(items==0)
		{
			BSONObj object = BSONObjBuilder().genOID().append("ObjectName", *itObject).append("DocumentType","Object").obj();
			MongoProxy::MongoProxy::getSingleton(hostname).insert(object);
		}
		// insert view oid to object document
		BSONObj query = BSON("ObjectName"<<*itObject<<"DocumentType"<<"Object");
		BSONObj update = BSON("$addToSet"<<BSON("ViewsList"<<BSON("viewOID"<<viewOID.toString())));
		MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);

		// insert object oid to view document
		BSONObj object;
		query = BSON("ObjectName"<<*itObject<<"DocumentType"<<"Object");
		auto_ptr<DBClientCursor> cursorCollection =MongoProxy::MongoProxy::getSingleton(hostname).query(query);
		while(cursorCollection->more())
		{
			object = cursorCollection->next();
		}
		object.getObjectID(bsonElement);
		OID objectOID;
		objectOID=bsonElement.__oid();
		query = BSON("ViewName"<<ViewName<<"DocumentType"<<"View");
		update = BSON("$addToSet"<<BSON("ObjectsList"<<BSON("objectOID"<<objectOID.toString())));
		MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);
	}
	return;
}

void View::putMatToFile(const cv::Mat& img, fileTypes typ, string& fileName)
{
	LOG(LNOTICE)<< "View::putMatToFile";
	LOG(LNOTICE)<< "key: "<<typ;
	//shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(img, typ, fileName));
	//pushFile(file, typ);
}

void View::putStringToFile(const std::string& str, fileTypes typ, string& fileName)
{
	LOG(LNOTICE)<< "View::putStringToFile";
	LOG(LNOTICE)<< "key: "<<typ;
	//shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(str, typ, fileName, hostname));
	//pushFile(file, typ);
}

void View::putPCxyzToFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudXYZ, fileTypes typ, string& fileName)
{
	LOG(LNOTICE)<< "View::putPCxyzToFile";
	LOG(LNOTICE)<< "key: "<<typ;
	//shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(cloudXYZ, typ, fileName, hostname));
	//pushFile(file, typ);
}
void View::putPCyxzrgbToFile(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudXYZRGB, fileTypes typ, string& fileName)
{
	LOG(LNOTICE)<< "View::putPCyxzrgbToFile";
	LOG(LNOTICE)<< "key: "<<typ;
	//shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(cloudXYZRGB, typ, fileName, hostname));
	//pushFile(file, typ);
}
void View::putPCxyzsiftToFile(const pcl::PointCloud<PointXYZSIFT>::Ptr& cloudXYZSIFT, fileTypes typ, string& fileName)
{
	LOG(LNOTICE)<< "View::putPCxyzsiftToFile";
	LOG(LNOTICE)<< "key: "<<typ;
	//shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(cloudXYZSIFT, typ, fileName, hostname));
//	pushFile(file, typ);
}

void View::putPCxyzrgbsiftToFile(const pcl::PointCloud<PointXYZRGBSIFT>::Ptr& cloudXYZRGBSIFT, fileTypes typ, string& fileName)
{
	LOG(LNOTICE)<< "View::putPCxyzrgbsiftToFile";
	LOG(LNOTICE)<< "key: "<<typ;
	//shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(cloudXYZRGBSIFT, typ, fileName, hostname));
	//pushFile(file, typ);
}

void View::putPCxyzshotToFile(const pcl::PointCloud<PointXYZSHOT>::Ptr& cloudXYZSHOT, fileTypes typ, string& fileName)
{
	LOG(LNOTICE)<< "View::putPCxyzshotToFile";
	LOG(LNOTICE)<< "key: "<<typ;
	//shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(cloudXYZSHOT, typ, fileName, hostname));
	//pushFile(file, typ);
}

void View::putPCxyzrgbNormalToFile(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloudXYZRGBNormal, fileTypes typ, string& fileName)
{
	LOG(LNOTICE)<< "View::putPCxyzrgbNormalToFile";
	LOG(LNOTICE)<< "key: "<<typ;
	//shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(cloudXYZRGBNormal, typ, fileName, hostname));
	//pushFile(file, typ);
}

}//namespace MongoBase




#endif /* VIEW_HPP_ */
