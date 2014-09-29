/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

#include "ModelReader.hpp"


namespace Processors {
namespace MongoDBImporter  {
using namespace cv;
using namespace mongo;
using namespace boost::property_tree;

MongoDBImporter::MongoDBImporter(const std::string & name) : Base::Component(name),
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
        CLOG(LTRACE) << "Hello MongoDBImporter";

        base = new MongoBase::MongoBase();
}

MongoDBImporter::~MongoDBImporter()
{
        CLOG(LTRACE) << "Good bye MongoDBImporter";
}

void MongoDBImporter::readfromDB()
{
	CLOG(LNOTICE) << "MongoDBImporter::readfromDB";
	readFromMongoDB(nodeTypeProp, viewOrModelName, type);
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

void MongoDBImporter::getFileFromGrid(const GridFile& file, const string& modelOrViewName, const string& nodeType, const string& type)
{
	CLOG(LTRACE)<<"MongoDBImporter::getFileFromGrid";
	string filename;
	filename = file.getFileField("filename").str();
	// type in "View","Model"
	CLOG(LINFO)<<(string)folderName+type+"/"+modelOrViewName+"/"+nodeType+"/"+filename;
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

void MongoDBImporter::setModelOrViewName(const string& childNodeName, const BSONObj& childObj)
{
	CLOG(LTRACE)<<"MongoDBImporter::setModelOrViewName";
	string type = childNodeName;
	string modelOrViewName = childObj.getField(type+"Name").str();
	readFromMongoDB(childNodeName, modelOrViewName, type);
}
void MongoDBImporter::readFile(const string& modelOrViewName, const string& nodeType, const string& type, const OID& childOID)
{
	CLOG(LTRACE)<<"MongoDBImporter::readFile";
	GridFS fs(c,collectionName);
	CLOG(LTRACE)<<"_id"<<childOID;
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

void MongoDBImporter::readFromMongoDB(const string& nodeType, const string& modelOrViewName, const string& type)
{
	CLOG(LTRACE)<<"MongoDBImporter::readFromMongoDB";
	string name;
	try{
		int items=0;
		base->findDocumentInCollection(c, dbCollectionPath, objectName, nodeType, cursorCollection, modelOrViewName, type, items);

		if(items>0)
		{
			CLOG(LINFO)<<"Founded some data";
			while (cursorCollection->more())
			{
				BSONObj obj = cursorCollection->next();
				CLOG(LTRACE)<<obj;
				vector<OID> childsVector;
				int items =  base->getChildOIDS(obj, "childOIDs", "childOID", childsVector);
				if(items>0)
				{
					CLOG(LTRACE)<<"There are childs "<<childsVector.size();
					for (unsigned int i = 0; i<childsVector.size(); i++)
					{
						childCursor =c.query(dbCollectionPath, (QUERY("_id"<<childsVector[i])));
						if(childCursor->more())
						{
							BSONObj childObj = childCursor->next();
							string childNodeName= childObj.getField("Type").str();
							if(childNodeName!="EOO")
							{
								if(base->isViewLastLeaf(nodeType) || base->isModelLastLeaf(nodeType))
								{
									CLOG(LTRACE)<<"LastLeaf"<<" childNodeName "<<childNodeName;
									readFile(modelOrViewName, nodeType, type, childsVector[i]);
								}
								else if(childNodeName=="View" || childNodeName=="Model")
								{
									setModelOrViewName(childNodeName, childObj);
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
			if(nodeTypeProp==nodeType)
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
