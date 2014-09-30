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

void MongoBase::writeNode2MongoDB(const string &destination, const string &type,string modelOrViewName)
{
	//CLOG(LTRACE) <<"writeNode2MongoDB";
	OID oid;
	//CLOG(LTRACE) <<"Filename: " << fileName << " destination: "<< destination<<" dbCollectionPath: "<<dbCollectionPath;
    try{
		insertFileToGrid(oid);
		//c.update(dbCollectionPath, QUERY("ObjectName"<<objectName<<type+"Name"<<modelOrViewName<<"Type"<<destination), BSON("$addToSet"<<BSON("childOIDs"<<BSON("childOID"<<oid.str()))), false, true);
		//CLOG(LTRACE) <<"Files saved successfully";
    }
	catch(DBException &e)
	{
		//CLOG(LERROR) <<"Something goes wrong... :<";
		//CLOG(LERROR) <<c.getLastError();
	}
}

void MongoBase::insertFileToGrid(OID& oid)
{
	/*
	BSONObj object;
	BSONElement bsonElement;
	string mime="";
	setMime(extension, mime);
	cv::Mat tempImg = in_img.read();
	std::stringstream ss;
	boost::posix_time::time_facet *facet = new boost::posix_time::time_facet("%d_%m_%Y_%H_%M_%S");
	ss.imbue(locale(cout.getloc(), facet));
	ss<<second_clock::local_time();
	CLOG(LINFO) << ss.str() << endl;
	string tempFileName = string(fileName)+"."+string(extension);
	cv::imwrite(tempFileName, tempImg);
	GridFS fs(c, collectionName);
	string fileNameInmongo = (string)remoteFileName + ss.str();
	object = fs.storeFile(tempFileName, fileNameInmongo, mime);
	BSONObj b = BSONObjBuilder().appendElements(object).append("ObjectName", objectName).obj();
	c.insert(dbCollectionPath, b);
	b.getObjectID(bsonElement);
	oid=bsonElement.__oid();
	*/
}

} /* namespace MongoBase */
