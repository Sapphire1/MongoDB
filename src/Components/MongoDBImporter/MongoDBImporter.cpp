/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

#include "MongoDBImporter.hpp"


namespace Processors {
namespace MongoDBImporter  {
using namespace cv;
using namespace mongo;
using namespace boost::property_tree;

MongoDBImporter::MongoDBImporter(const std::string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", string("localhost")),
		objectName("objectName", string("GreenCup")),
		collectionName("collectionName", string("containers")),
		nodeNameProp("nodeName", string("Object")),
		viewOrModelName("viewOrModelName", string("")),
		type("type", string("")),
		folderName("folderName", string("/home/lzmuda/mongo_driver_tutorial/test/"))
{
		registerProperty(mongoDBHost);
		registerProperty(objectName);
		registerProperty(collectionName);
		registerProperty(nodeNameProp);
		registerProperty(viewOrModelName);
		registerProperty(folderName);
		registerProperty(type);
        CLOG(LTRACE) << "Hello MongoDBImporter";
}

MongoDBImporter::~MongoDBImporter()
{
        CLOG(LTRACE) << "Good bye MongoDBImporter";
}

void MongoDBImporter::readfromDB()
{
	CLOG(LNOTICE) << "MongoDBImporter::readfromDB";
	readFromMongoDB(nodeNameProp, viewOrModelName, type);
}
void MongoDBImporter::prepareInterface() {
        CLOG(LTRACE) << "MongoDBImporter::prepareInterface";

        h_readfromDB.setup(this, &MongoDBImporter::readfromDB);
        registerHandler("Read", &h_readfromDB);

//        registerStream("in_img", &in_img);
//        registerStream("out_img", &out_img);
//        addDependency("onNewImage", &in_img);
}

bool MongoDBImporter::onInit()
{
        CLOG(LTRACE) << "MongoDBImporter::initialize";
        if(collectionName=="containers")
        	dbCollectionPath="images.containers";
        string hostname = mongoDBHost;
        connectToMongoDB(hostname);
        return true;
}

bool MongoDBImporter::onFinish()
{
        CLOG(LTRACE) << "MongoDBImporter::finish";
        return true;
}

bool MongoDBImporter::onStep()
{
        CLOG(LTRACE) << "MongoDBImporter::step";
        return true;
}

bool MongoDBImporter::onStop()
{
        return true;
}

bool MongoDBImporter::onStart()
{
        return true;
}

void MongoDBImporter::getFileFromGrid(const GridFile& file, const string& modelOrViewName, const string& nodeName, const string& type)
{
	CLOG(LTRACE)<<"MongoDBImporter::getFileFromGrid";
	string filename;
	filename = file.getFileField("filename").str();
	// type in "View","Model"
	CLOG(LINFO)<<(string)folderName+type+"/"+modelOrViewName+"/"+nodeName+"/"+filename;
	string name = (string)folderName+type+"/"+modelOrViewName+"/"+nodeName+"/"+filename;
	stringstream ss;
	string str = ss.str();
	char *fileName = (char*)name.c_str();
	ofstream ofs(fileName);
	gridfs_offset off = file.write(ofs);
	if (off != file.getContentLength())
	{
		CLOG(LERROR) << "Failed to read a file from mongoDB";
	}
	else
	{
		CLOG(LTRACE) << "Success read a file from mongoDB";
	}
}

void MongoDBImporter::readFile(const string& modelOrViewName, const string& nodeName, const string& type, const OID& childOID)
{
	CLOG(LTRACE)<<"MongoDBImporter::readFile";
	GridFS fs(*c,collectionName);
	CLOG(LTRACE)<<"_id"<<childOID;
	GridFile file = fs.findFile(Query(BSON("_id" << childOID)));
	if (!file.exists())
	{
		CLOG(LERROR) << "File not found in grid";
	}
	else
	{
		getFileFromGrid(file, modelOrViewName, nodeName, type);
	}
}

void MongoDBImporter::readFromMongoDB(const string& nodeName, const string& modelOrViewName, const string& type)
{
	CLOG(LTRACE)<<"MongoDBImporter::readFromMongoDB";
	string name;
	try{
		int items=0;
		findDocumentInCollection(*c, dbCollectionPath, objectName, nodeName, cursorCollection, modelOrViewName, type, items);

		if(items>0)
		{
			CLOG(LINFO)<<"Founded some data";
			while (cursorCollection->more())
			{
				BSONObj obj = cursorCollection->next();
				CLOG(LTRACE)<<obj;
				vector<OID> childsVector;
				int items =  getChildOIDS(obj, "childOIDs", "childOID", childsVector);
				if(items>0)
				{
					CLOG(LTRACE)<<"There are childs "<<childsVector.size();
					for (unsigned int i = 0; i<childsVector.size(); i++)
					{
						childCursor =c->query(dbCollectionPath, BSON("_id"<<childsVector[i]),0,0,0);
						if(childCursor->more())
						{
							BSONObj childObj = childCursor->next();
							string childNodeName= childObj.getField("Type").str();
							if(childNodeName!="EOO")
							{
								if(isViewLastLeaf(nodeName) || isModelLastLeaf(nodeName))
								{
									CLOG(LTRACE)<<"LastLeaf"<<" childNodeName "<<childNodeName;
									readFile(modelOrViewName, nodeName, type, childsVector[i]);
								}
								else if(childNodeName=="View" || childNodeName=="Model")
								{
									string newName;
									setModelOrViewName(childNodeName, childObj, newName);
									readFromMongoDB(childNodeName, newName, type);
								}
								else
									readFromMongoDB(childNodeName, modelOrViewName, type);
							}
						}//if(childNodeName!="EOO")
					}//for
				}//if
			}//while
		}//if
		else
		{
			CLOG(LTRACE)<<"10";
			if(nodeNameProp==nodeName)
				CLOG(LERROR)<<"Wrong name";
			CLOG(LTRACE)<<"No results";
		}
	}//try
	catch(DBException &e)
	{
		CLOG(LERROR) <<"ReadFromMongoDB(). Something goes wrong... :<";
		exit(1);
	}
}
} //: namespace MongoDBImporter
} //: namespace Processors
