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


namespace MongoBase {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;

class MongoBase {
public:

    boost::shared_ptr<DBClientConnection> c;
	vector<string>  docViewsNames;
	vector<string>  docModelsNames;

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
	void connectToMongoDB(string&);
};

MongoBase::MongoBase() {
	DBClientConnection *c_ptr = new DBClientConnection();
	c = boost::shared_ptr<DBClientConnection>(c_ptr);
}

MongoBase::~MongoBase() {
}

void MongoBase::connectToMongoDB(string& hostname)
{
	try{
		if(!c->isStillConnected())
			c->connect(hostname);
	}catch(ConnectException& ex)
	{
		std::cout<<ex.what();
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
	docViewsNames.push_back("ToFSiRX");
	docViewsNames.push_back("ToFSiRXM");

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

void  MongoBase::findDocumentInCollection(DBClientConnection& c, string& dbCollectionPath, Base::Property<string>& objectName, const string &nodeType, auto_ptr<DBClientCursor> & cursorCollection, const string & modelOrViewName, const string & type, int& items)
{
      try{
    	  if(type!="")
    	  {
			  if(type=="View")
			  {
				  items = c.count(dbCollectionPath, (QUERY("Type"<<nodeType<<"ObjectName"<<objectName<<"ViewName"<<modelOrViewName)));
				  if(items>0)
					  cursorCollection =c.query(dbCollectionPath, (QUERY("Type"<<nodeType<<"ObjectName"<<objectName<<"ViewName"<<modelOrViewName)));
			  }
			  else if(type=="Model")
			  {
				  items = c.count(dbCollectionPath, (QUERY("Type"<<nodeType<<"ObjectName"<<objectName<<"ModelName"<<modelOrViewName)));
				  if (items>0)
					  cursorCollection =c.query(dbCollectionPath, (QUERY("Type"<<nodeType<<"ObjectName"<<objectName<<"ModelName"<<modelOrViewName)));
			  }
			  else
			  {
				  items = c.count(dbCollectionPath, (QUERY("Type"<<nodeType<<"ObjectName"<<objectName<<type<<modelOrViewName)));
				  if(items>0)
					  cursorCollection =c.query(dbCollectionPath, (QUERY("Type"<<nodeType<<"ObjectName"<<objectName<<type<<modelOrViewName)));
			  }
		  }
    	  else
    	  {
    		  items = c.count(dbCollectionPath, (QUERY("Type"<<nodeType<<"ObjectName"<<objectName)));
    		  if(items>0)
    			  cursorCollection =c.query(dbCollectionPath, (QUERY("Type"<<nodeType<<"ObjectName"<<objectName)));
    	  }
    	 }
      catch(DBException &e)
      {
    	  //CLOG(LERROR) <<"findDocumentInCollection(). Something goes wrong... :<";
    	  //CLOG(LERROR) <<c.getLastError();
      }

      return;
}
bool MongoBase::isViewLastLeaf(const string& nodeType)
{
	if(nodeType=="StereoPC" || nodeType=="StereoPCXYZRGB" || nodeType=="StereoPCXYZSIFT" || nodeType=="StereoPCXYZSHOT" || nodeType=="ToFPCXYZSIFT" || nodeType=="ToFPCXYZRGB" || nodeType=="ToFPCXYZRGB" || nodeType=="ToFPC" || nodeType=="ToFPCXYZSHOT" || nodeType=="KinectPCXYZSHOT"  || nodeType=="KinectPCXYZSIFT" || nodeType=="KinectPCXYZRGB" || nodeType=="KinectPC" || nodeType=="StereoLR" || nodeType=="KinectRGBD" || nodeType=="ToFRGBD" || nodeType=="StereoRX" || nodeType=="KinectRX" ||  nodeType=="ToFRX" || nodeType=="StereoRXM" || nodeType=="KinectRXM" || nodeType=="ToFRXM")
		return true;
	else
		return false;
}

bool MongoBase::isModelLastLeaf(const string& nodeType)
{
	if(nodeType=="SomXYZRgb" || nodeType=="SomXYZSift" ||  nodeType=="SsomXYZRgb" || nodeType=="SsomXYZSift" || nodeType=="SsomXYZShot")
		return true;
	else
		return false;
}

} /* namespace MongoBase */
#endif /* MONGOBASE_HPP_ */
