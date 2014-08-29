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
		folderName("folderName", string("/home/lzmuda/mongo_driver_tutorial/test/"))
{
		registerProperty(mongoDBHost);
		registerProperty(objectName);
		registerProperty(collectionName);
		registerProperty(nodeType);
		registerProperty(folderName);
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
        CLOG(LNOTICE) << "MongoDBReader::write2DB";
        readFromMongoDB(nodeType);
}
void MongoDBReader::prepareInterface() {
        CLOG(LTRACE) << "MongoDBReader::prepareInterface";

        h_readfromDB.setup(this, &MongoDBReader::readfromDB);
        registerHandler("onNewImage", &h_readfromDB);

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
	  return oidsVector;
}

auto_ptr<DBClientCursor>  MongoDBReader::findDocumentInCollection(string nodeName)
{
      auto_ptr<DBClientCursor> cursorCollection;
      try{
    	  cursorCollection =c.query(dbCollectionPath, (QUERY("Type"<<nodeName<<"Name"<<objectName)));
      }
      catch(DBException &e)
      {
    	  CLOG(LERROR) <<"Something goes wrong... :>";
    	  CLOG(LERROR) <<c.getLastError();
      }
      return cursorCollection;
}

void MongoDBReader::readFromMongoDB(string nodeType)
{
	CLOG(LTRACE) <<"NodeType :" << nodeType;
	try{
		auto_ptr<DBClientCursor> cursorCollection =findDocumentInCollection(nodeType);

		while (cursorCollection->more())
		{
			BSONObj obj = cursorCollection->next();
			vector<OID> childsVector =  getchildOIDS(obj);

			for (unsigned int i = 0; i<childsVector.size(); i++)
			{
				auto_ptr<DBClientCursor> childCursor =c.query(dbCollectionPath, (QUERY("_id"<<childsVector[i])));
				if( childCursor->more())
				{
					BSONObj childObj = childCursor->next();
					string childNodeName = childObj.getField("Type").str();
					// if node consists of nodes read from child
					if(childNodeName!="EOO")
						readFromMongoDB(childNodeName);
				}//if
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
					CLOG(LTRACE) << "Filename "<<filename<<" Chunks: " << file.getNumChunks();
					string name = (string)folderName+nodeType+"/"+filename;
					stringstream ss;
					string str = ss.str();
					char *fileName = (char*)name.c_str();
					ofstream ofs(fileName);
					gridfs_offset off = file.write(ofs);
					if (off != file.getContentLength())
					{
						CLOG(LTRACE) << "Failed to write a file from mongoDB to disc";
					}
					else
					{
						CLOG(LTRACE) << "Success write a file from mongoDB to disc";
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

 
