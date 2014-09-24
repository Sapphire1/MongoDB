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
		nodeTypeProp("nodeType", string("Object")),
		viewOrModelName("viewOrModelName", string("")),
		type("type", string("")),
		folderName("folderName", string("/home/lzmuda/mongo_driver_tutorial/test/"))
{
		registerProperty(mongoDBHost);
		registerProperty(objectName);
		registerProperty(collectionName);
		registerProperty(nodeTypeProp);
		registerProperty(viewOrModelName);
		registerProperty(folderName);
		registerProperty(type);
        CLOG(LTRACE) << "Hello MongoDBReader";

        base = new MongoBase::MongoBase();
}

MongoDBReader::~MongoDBReader()
{
        CLOG(LTRACE) << "Good bye MongoDBReader";
}

void MongoDBReader::readfromDB()
{
	CLOG(LNOTICE) << "MongoDBReader::readfromDB";
	readFromMongoDB(nodeTypeProp, viewOrModelName, type);
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
      	  //base = MongoBase::MongoBase(c,dbCollectionPath,objectName);
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

void MongoDBReader::getFileFromGrid(const GridFile& file, const string& modelOrViewName, const string& nodeType, const string& type)
{
	string filename;
	filename = file.getFileField("filename").str();
	// type in "View","Model"
	string name = (string)folderName+type+"/"+modelOrViewName+"/"+nodeType+"/"+filename;
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

void MongoDBReader::setModelOrViewName(const string& childNodeName, const BSONObj& childObj)
{
		string type = childNodeName;
		string modelOrViewName = childObj.getField(type+"Name").str();
		readFromMongoDB(childNodeName, modelOrViewName, type);
}
void MongoDBReader::readFile(const string& modelOrViewName, const string& nodeType, const string& type, const OID& childOID)
{
	GridFS fs(c,collectionName);
	GridFile file = fs.findFile(QUERY("_id" << childOID));
	if (!file.exists())
	{
		CLOG(LERROR) << "File not found in grid";
	}
	else
	{
		getFileFromGrid(file, modelOrViewName, nodeType, type);
	}
}

void MongoDBReader::readFromMongoDB(const string& nodeType, const string& modelOrViewName, const string& type)
{
	try{
		int items=0;
		base->findDocumentInCollection(c, dbCollectionPath, objectName, nodeType, cursorCollection, modelOrViewName, type, items);
		if(items>0)
		{
			CLOG(LINFO)<<"Founded some data";
			while (cursorCollection->more())
			{
				BSONObj obj = cursorCollection->next();
				vector<OID> childsVector =  base->getChildOIDS(obj, "childOIDs", "childOID");
				string name;
				for (unsigned int i = 0; i<childsVector.size(); i++)
				{
					childCursor =c.query(dbCollectionPath, (QUERY("_id"<<childsVector[i])));
					if(childCursor->more())
					{
						BSONObj childObj = childCursor->next();
						string childNodeName= childObj.getField("Type").str();
						if(childNodeName!="EOO")
						{
							if(childNodeName=="View" || childNodeName=="Model")
							{
								setModelOrViewName(childNodeName, childObj);
							}
							else
								readFromMongoDB(childNodeName, modelOrViewName, type);

							if(base->isViewLastLeaf(nodeType) || base->isModelLastLeaf(nodeType))
							{
								 readFile(modelOrViewName, nodeType, type, childsVector[i]);
							}
						}
					}//if(childNodeName!="EOO")
				}//for
			}//while
		}//if
		else
		{
			if(nodeTypeProp==nodeType)
				CLOG(LERROR)<<"Wrong name";
			CLOG(LTRACE)<<"No results";
		}
	}//try
	catch(DBException &e)
	{
		CLOG(LERROR) <<"ReadFromMongoDB(). Something goes wrong... :<";
		CLOG(LERROR) <<c.getLastError();
	}
}
} //: namespace MongoDBReader
} //: namespace Processors
