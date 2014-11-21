/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

#include "ViewReader.hpp"


namespace Processors {
namespace ViewReader  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace boost::posix_time;

ViewReader::ViewReader(const std::string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", string("localhost")),
		objectName("objectName", string("GreenCup")),
		collectionName("collectionName", string("containers")),
		nodeNameProp("nodeName", string("StereoLR")),
		viewOrModelName("viewName", string("lab012"))
{
		registerProperty(mongoDBHost);
		registerProperty(objectName);
		registerProperty(collectionName);
		registerProperty(nodeNameProp);
		registerProperty(viewOrModelName);
		this->position=0;
		if(nodeNameProp=="Object")
			nodeType="";
		else
			nodeType="View";
        CLOG(LTRACE) << "Hello ViewReader";
}

ViewReader::~ViewReader()
{
        CLOG(LTRACE) << "Good bye ViewReader";
}

void ViewReader::readfromDB()
{
	CLOG(LNOTICE) << "ViewReader::readfromDB";
	readFromMongoDB(nodeNameProp, viewOrModelName, nodeType);
}
void ViewReader::prepareInterface() {
        CLOG(LTRACE) << "ViewReader::prepareInterface";
        h_readfromDB.setup(this, &ViewReader::readfromDB);
        registerHandler("Read", &h_readfromDB);
        registerStream("out_cloud_xyz", &out_cloud_xyz);
        registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
        registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
        registerStream("out_img", &out_img);
        registerStream("in_trigger", &in_trigger);
        registerStream("cipFileOut", &cipFileOut);
		registerHandler("onTriggeredReadAllFiles", boost::bind(&ViewReader::readAllFilesTriggered, this));
		addDependency("onTriggeredReadAllFiles", &in_trigger);
}

bool ViewReader::onInit()
{
        CLOG(LTRACE) << "ViewReader::initialize";
        if(collectionName=="containers")
      		MongoBase::dbCollectionPath=dbCollectionPath="images.containers";
        string hostname = mongoDBHost;
        connectToMongoDB(hostname);
        return true;
}

bool ViewReader::onFinish()
{
        CLOG(LTRACE) << "ViewReader::finish";
        return true;
}

bool ViewReader::onStep()
{
        CLOG(LTRACE) << "ViewReader::step";
        return true;
}

bool ViewReader::onStop()
{
        return true;
}

bool ViewReader::onStart()
{
        return true;
}

void ViewReader::addToAllChilds(std::vector<OID> & childsVector)
{
	CLOG(LTRACE)<<"ViewReader::addToAllChilds";
	allChildsVector+=childsVector;
}

void ViewReader::readAllFilesTriggered()
{
	CLOG(LTRACE)<<"ViewReader::readAllFiles";
	fileOID = allChildsVector[position];
    readFile();
    if(position<allChildsVector.size())
    	++position;
    else
    	position=0;
}

void ViewReader::readFromMongoDB(const string& nodeName, const string& modelOrViewName, const string& nodeType)
{
	CLOG(LTRACE)<<"ViewReader::readFromMongoDB";
	string name;
	try{
		int items=0;
		findDocumentInCollection(*c, dbCollectionPath, objectName, nodeName, cursorCollection, modelOrViewName, nodeType, items);
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
					if(isViewLastLeaf(nodeName) || isModelLastLeaf(nodeName))
						addToAllChilds(childsVector);
					else
					{
						CLOG(LTRACE)<<"There are childs "<<childsVector.size();
						for (unsigned int i = 0; i<childsVector.size(); i++)
						{
							childCursor =c->query(dbCollectionPath, Query(BSON("_id"<<childsVector[i])));
							if(childCursor->more())
							{
								BSONObj childObj = childCursor->next();
								string childNodeName= childObj.getField("NodeName").str();
								if(childNodeName!="EOO")
								{
									if(childNodeName=="View")
									{
										string newName;
										setModelOrViewName(childNodeName, childObj, newName);
										readFromMongoDB(childNodeName, newName, "View");
									}
									else if(childNodeName=="Model")
									{
										CLOG(LTRACE)<<"It's a model. Do nothing.";
									}
									else
										readFromMongoDB(childNodeName, modelOrViewName, nodeType);
								}
							}//if(childNodeName!="EOO")
						}//for
					}
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
} //: namespace ViewReader
} //: namespace Processors
