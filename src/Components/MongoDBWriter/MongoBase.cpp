/*
 * MongoBase.cpp
 *
 *  Created on: Sep 23, 2014
 *      Author: lzmuda
 */

#include "MongoBase.hpp"
using namespace cv;
using namespace mongo;
using namespace std;


namespace MongoBase {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;

MongoBase::MongoBase() {
	//this->c=c;
	//this->dbCollectionPath = dbCollectionPath;
	//this->objectName = objectName;
}

MongoBase::~MongoBase() {
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

vector<OID>  MongoBase::getChildOIDS(BSONObj &obj, const string & fieldName, const string & childfieldName)
{
	  vector<BSONElement> v = obj.getField(fieldName).Array();
	  vector<OID> oidsVector;
	  for (unsigned int i = 0; i<v.size(); i++)
	  {
		string readedOid =v[i][childfieldName].str();
		OID o = OID(readedOid);
		oidsVector.push_back(o);
	  }
	  return oidsVector;
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
	if(nodeType=="StereoSiLR" || nodeType=="KinectSiLR" || nodeType=="ToFSiLR" || nodeType=="StereoSiRX" || nodeType=="KinectSiRX" ||  nodeType=="ToFSiRX" || nodeType=="StereoSiRXM" || nodeType=="KinectSiRXM" || nodeType=="ToFSiRXM")
		return true;
	else
		return false;
}

bool MongoBase::isModelLastLeaf(const string& nodeType)
{
	if(nodeType=="SomRgb" || nodeType=="SomSift" ||  nodeType=="SsomRgb" || nodeType=="SsomSift" || nodeType=="SsomShot")
		return true;
	else
		return false;
}
} /* namespace MongoBase */
