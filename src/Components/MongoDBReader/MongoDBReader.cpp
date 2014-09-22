/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

#include "MongoDBReader.h"

namespace Processors {
namespace MongoDBReader  {
using namespace cv;
using namespace mongo;
using namespace boost::property_tree;

MongoDBReader::MongoDBReader(const std::string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", string("localhost")),
		objectName("objectName", string("GreenCup")),
		collectionName("collectionName", string("containers")),
		nodeType("nodeType", string("Object")),
		viewOrModelName("viewOrModelName", string("")),
		type("type", string("")),
		folderName("folderName", string("/home/lzmuda/mongo_driver_tutorial/test/"))
{
		registerProperty(mongoDBHost);
		registerProperty(objectName);
		registerProperty(collectionName);
		registerProperty(nodeType);
		registerProperty(viewOrModelName);
		registerProperty(folderName);
		registerProperty(type);
        CLOG(LTRACE) << "Hello MongoDBReader";
}
void MongoDBReader::connect2MongoDB()
{

}

MongoDBReader::~MongoDBReader()
{
        CLOG(LTRACE) << "Good bye MongoDBReader";
}

void MongoDBReader::readfromDB()
{
	CLOG(LNOTICE) << "MongoDBReader::readfromDB";
	readFromMongoDB(nodeType, viewOrModelName, type);
}
void MongoDBReader::prepareInterface() {
        CLOG(LTRACE) << "MongoDBReader::prepareInterface";

        h_readfromDB.setup(this, &MongoDBReader::readfromDB);
        registerHandler("Read", &h_readfromDB);

//        registerStream("in_img", &in_img);
//        registerStream("out_img", &out_img);
//        addDependency("onNewImage", &in_img);
}

bool MongoDBReader::onInit()
{
        CLOG(LTRACE) << "MongoDBReader::initialize";
        if(collectionName=="containers")
        	dbCollectionPath="images.containers";
        else if(collectionName=="food")
            dbCollectionPath="images.food";
        else if(collectionName=="dish")
            dbCollectionPath="images.dish";
        else if(collectionName=="other")
            dbCollectionPath="images.other";
        try
        {
      	  c.connect(mongoDBHost);
         }
         catch(DBException &e)
         {
        	 CLOG(LERROR) <<"Something goes wrong... :>";
        	 CLOG(LERROR) <<c.getLastError();
         }
        return true;
}

bool MongoDBReader::onFinish()
{
        CLOG(LTRACE) << "MongoDBReader::finish";
        return true;
}

bool MongoDBReader::onStep()
{
        CLOG(LTRACE) << "MongoDBReader::step";
        return true;
}

bool MongoDBReader::onStop()
{
        return true;
}

bool MongoDBReader::onStart()
{
        return true;
}

void MongoDBReader::onNewImage()
{
        CLOG(LNOTICE) << "MongoDBReader::onNewImage";
}

vector<OID>  MongoDBReader::getchildOIDS(BSONObj &obj)
{
	  CLOG(LTRACE) << "Processing JSON document: " << obj.toString() << std::endl;
	  vector<BSONElement> v = obj.getField("childOIDs").Array();
	  vector<OID> oidsVector;
	  for (unsigned int i = 0; i<v.size(); i++)
	  {
		string readedOid =v[i]["childOID"].str();
		OID o = OID(readedOid);
		CLOG(LTRACE) <<"OID: "<< o.str() << endl;
		oidsVector.push_back(o);
	  }
	  CLOG(LTRACE) <<"getchildOIDS - END"<< endl;
	  return oidsVector;
}

void  MongoDBReader::findDocumentInCollection(const string &nodeType, auto_ptr<DBClientCursor> & cursorCollection, const string & modelOrViewName, const string & type)
{
      try{
    	  CLOG(LINFO)<<"NodeName "<< nodeType;
    	  if(type!="")
    	  {
			  if(type=="View")
			  {
				  CLOG(LINFO)<<"findDocumentInCollection- View";
				  cursorCollection =c.query(dbCollectionPath, (QUERY("Type"<<nodeType<<"ObjectName"<<objectName<<"ViewName"<<modelOrViewName)));
				  CLOG(LINFO)<<"Founded Documents: "<<c.count(dbCollectionPath, (QUERY("Type"<<nodeType<<"ObjectName"<<objectName<<"ViewName"<<modelOrViewName)));
			  }
			  else if(type=="Model")
			  {
				  CLOG(LINFO)<<"findDocumentInCollection- Model";
				  cursorCollection =c.query(dbCollectionPath, (QUERY("Type"<<nodeType<<"ObjectName"<<objectName<<"ModelName"<<modelOrViewName)));
				  CLOG(LINFO)<<"Founded Documents: "<<c.count(dbCollectionPath, (QUERY("Type"<<nodeType<<"ObjectName"<<objectName<<"ModelName"<<modelOrViewName)));
			  }
			  else
			  {
				  CLOG(LINFO)<<"findDocumentInCollection- no view or model name, know type";
				  cursorCollection =c.query(dbCollectionPath, (QUERY("Type"<<nodeType<<"ObjectName"<<objectName<<type<<modelOrViewName)));
				  CLOG(LINFO)<<"Founded Documents: "<<c.count(dbCollectionPath, (QUERY("Type"<<nodeType<<"ObjectName"<<objectName<<type<<modelOrViewName)));
			  }
    	  }
    	  else
    	  {
    		  CLOG(LINFO)<<"findDocumentInCollection- don't know type";
    	      cursorCollection =c.query(dbCollectionPath, (QUERY("Type"<<nodeType<<"ObjectName"<<objectName)));
    	      CLOG(LINFO)<<"Founded Documents: "<<c.count(dbCollectionPath, (QUERY("Type"<<nodeType<<"ObjectName"<<objectName)));
    	  }
    	 }
      catch(DBException &e)
      {
    	  CLOG(LERROR) <<"Something goes wrong... :>";
    	  CLOG(LERROR) <<c.getLastError();
      }
      return;
}

void MongoDBReader::readFromMongoDB(const string& nodeType, const string& modelOrViewName, const string& type)
{
	CLOG(LINFO) <<"readFromMongoDB - START" << nodeType;
	CLOG(LINFO) <<"NodeType :" << nodeType;
	CLOG(LINFO) <<"modelOrViewName :" << modelOrViewName;
	CLOG(LINFO) <<"type :" << type;
	try{
		CLOG(LTRACE) <<"Get document";
		findDocumentInCollection(nodeType, cursorCollection, modelOrViewName, type);
		int k=0;
		while (cursorCollection->more())
		{
			BSONObj obj = cursorCollection->next();
			CLOG(LINFO)<<obj;

			vector<OID> childsVector =  getchildOIDS(obj);
			string name;
			for (unsigned int i = 0; i<childsVector.size(); i++)
			{
				CLOG(LINFO)<<"iteration: "<<i<<" k: "<<++k;
				childCursor =c.query(dbCollectionPath, (QUERY("_id"<<childsVector[i])));
				if(childCursor->more())
				{
					BSONObj childObj = childCursor->next();
					string childNodeName= childObj.getField("Type").str();
					CLOG(LINFO)<<"childNodeName "<< childNodeName;

					if(childNodeName!="EOO")
					{
						if(childNodeName=="View")
						{
							string type = "View";
							string modelOrViewName = childObj.getField("ViewName").str();
							CLOG(LINFO)<<"set modelOrViewName: " <<modelOrViewName;
							CLOG(LINFO)<<"modelOrViewName: "<<modelOrViewName<< " childNodeName: "<<childNodeName;
							// if node consists of nodes read from child
							CLOG(LTRACE)<<"Get childs";
							readFromMongoDB(childNodeName, modelOrViewName, type);
						}
						else if(childNodeName=="Model")
						{
							string modelOrViewName = childObj.getField("ModelName").str();
							CLOG(LINFO)<<"set modelOrViewName: " <<modelOrViewName;
							string type = "Model";
							CLOG(LINFO)<<"modelOrViewName: "<<modelOrViewName<< " childNodeName: "<<childNodeName;
							// if node consists of nodes read from child
							CLOG(LTRACE)<<"Get childs";
							readFromMongoDB(childNodeName, modelOrViewName, type);
						}
						else
						{
							readFromMongoDB(childNodeName, modelOrViewName, type);
							CLOG(LTRACE)<<"Not view or model";
						}
						if(nodeType=="StereoSiLR" || nodeType=="KinectSiLR" || nodeType=="ToFSiLR" || nodeType=="StereoSiRX" || nodeType=="KinectSiRX" ||  nodeType=="ToFSiRX" || nodeType=="StereoSiRXM" || nodeType=="KinectSiRXM" || nodeType=="ToFSiRXM")
						{
							GridFS fs(c,collectionName);
							GridFile file = fs.findFile(QUERY("_id" << childsVector[i]));
							if (!file.exists())
							{
								CLOG(LTRACE) << "File not found in grid";
							}//if
							else
							{
								CLOG(LTRACE) <<"Found!";
								string filename;
								filename = file.getFileField("filename").str();
								name = (string)folderName+"View/"+modelOrViewName+"/"+nodeType+"/"+filename;
								CLOG(LINFO)<< name;
								CLOG(LTRACE) << "Filename "<<filename<<" Chunks: " << file.getNumChunks();
								stringstream ss;
								string str = ss.str();
								char *fileName = (char*)name.c_str();
								ofstream ofs(fileName);
								gridfs_offset off = file.write(ofs);
								if (off != file.getContentLength())
								{
									CLOG(LTRACE) << "Failed to read a file from mongoDB";
								}
								else
								{
									CLOG(LTRACE) << "Success read a file from mongoDB";
								}
							}
						}
						else if(nodeType=="SomRgb" || nodeType=="SomSift" ||  nodeType=="SsomRgb" || nodeType=="SsomSift" || nodeType=="SsomShot")
						{
							GridFS fs(c,collectionName);
							GridFile file = fs.findFile(QUERY("_id" << childsVector[i]));
							if (!file.exists())
							{
								CLOG(LTRACE) << "File not found in grid";
							}//if
							else
							{
								CLOG(LTRACE) <<"Found!";
								string filename;
								filename = file.getFileField("filename").str();
								name = (string)folderName+"Model/"+modelOrViewName+"/"+nodeType+"/"+filename;
								CLOG(LINFO)<< name;
								CLOG(LTRACE) << "Filename "<<filename<<" Chunks: " << file.getNumChunks();
								stringstream ss;
								string str = ss.str();
								char *fileName = (char*)name.c_str();
								ofstream ofs(fileName);
								gridfs_offset off = file.write(ofs);
								if (off != file.getContentLength())
								{
									CLOG(LTRACE) << "Failed to read a file from mongoDB";
								}
								else
								{
									CLOG(LTRACE) << "Success read a file from mongoDB";
								}
							}
						}
					}
				}
			}//for
		}//while
	}//try
	catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :>";
		CLOG(LERROR) <<c.getLastError();
	}
}
} //: namespace MongoDBReader
} //: namespace Processors
