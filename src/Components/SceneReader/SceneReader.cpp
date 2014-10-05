/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

#include "SceneReader.hpp"


namespace Processors {
namespace SceneReader  {
using namespace cv;
using namespace mongo;
using namespace boost::property_tree;

SceneReader::SceneReader(const std::string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", string("localhost")),
		objectName("objectName", string("GreenCup")),
		sceneName("sceneName", string("scene1")),
		collectionName("collectionName", string("containers")),
		getObjectsActive("Get objects", bool("1")),
		getScenesActive("Get scenes", bool("0")),
		getObjectFromSceneActive("Get object from scene", bool("0"))

{
		registerProperty(mongoDBHost);
		registerProperty(sceneName);
		registerProperty(objectName);
		registerProperty(collectionName);
		registerProperty(getObjectsActive);
		registerProperty(getScenesActive);
		registerProperty(getObjectFromSceneActive);
        CLOG(LTRACE) << "Hello SceneReader";
}

SceneReader::~SceneReader()
{
        CLOG(LTRACE) << "Good bye SceneReader";
}

void SceneReader::readfromDB()
{
	CLOG(LNOTICE) << "SceneReader::readfromDB";
	if(getObjectsActive)
	{
		getObjects();
	}
	else if(getScenesActive)
	{
		getScenes();
	}
	else if(getObjectFromSceneActive)
	{
		getObjectFromScene();
	}
}

void SceneReader::getObjects()
{
	CLOG(LINFO)<<"SceneReader::getObjects()";
	OID objectOID;
	OID sceneOID;
	BSONElement bsonElement;
	BSONElement sceneOI;
	BSONElement objectOI;
	auto_ptr<DBClientCursor> cursorCollection;
	int items = c.count(dbCollectionPath, (QUERY("SceneName"<<sceneName)));
	if(items>0)
	{
		cursorCollection  =c.query(dbCollectionPath, (QUERY("SceneName"<<sceneName)));
		vector<OID> childsVector;
		BSONObj scene = cursorCollection->next();
		if(getChildOIDS(scene, "objectsOIDs", "objectOID", childsVector)>0)
		{
			for (unsigned int i = 0; i<childsVector.size(); i++)
			{
				auto_ptr<DBClientCursor> childCursor =c.query(dbCollectionPath, (QUERY("_id"<<childsVector[i])));
				while(childCursor->more())
				{
					BSONObj objectDocument = childCursor->next();
					//CLOG(LINFO)<<"objectDocument: "<< objectDocument;
					string objectNameFromScene = objectDocument.getField("ObjectName").str();
					CLOG(LINFO)<<"Object: "<< objectNameFromScene;
				}
			}
		}
	}
	else
	{
		CLOG(LERROR)<<"No scene founded!";
	}
}

void SceneReader::getScenes()
{
	CLOG(LINFO)<<"SceneReader::getScenes()";
	OID objectOID;
	OID sceneOID;
	BSONElement bsonElement;
	BSONElement sceneOI;
	BSONElement objectOI;
	auto_ptr<DBClientCursor> cursorCollection;
	int items = c.count(dbCollectionPath, (QUERY("ObjectName"<<objectName)));
	if(items>0)
	{
		cursorCollection  =c.query(dbCollectionPath, (QUERY("ObjectName"<<objectName)));
		vector<OID> childsVector;
		BSONObj object = cursorCollection->next();
		if(getChildOIDS(object, "sceneOIDs", "sceneOID", childsVector)>0)
		{
			for (unsigned int i = 0; i<childsVector.size(); i++)
			{
				auto_ptr<DBClientCursor> childCursor =c.query(dbCollectionPath, (QUERY("_id"<<childsVector[i])));
				while(childCursor->more())
				{
					BSONObj objectDocument = childCursor->next();
					//CLOG(LINFO)<<"objectDocument: "<< objectDocument;
					string sceneNameFromObject = objectDocument.getField("SceneName").str();
					CLOG(LINFO)<<"Scene: "<< sceneNameFromObject;
				}
			}
		}
	}
	else
	{
		CLOG(LERROR)<<"No scene founded!";
	}

}

void SceneReader::getObjectFromScene()
{
	CLOG(LINFO)<<"SceneReader::getObjectFromScene()";
	OID objectOID;
	OID sceneOID;
	BSONElement bsonElement;
	BSONElement sceneOI;
	BSONElement objectOI;
	bool objectFound = false;
	auto_ptr<DBClientCursor> cursorCollection;
	int items = c.count(dbCollectionPath, (QUERY("SceneName"<<sceneName)));
	if(items>0)
	{
		cursorCollection  =c.query(dbCollectionPath, (QUERY("SceneName"<<sceneName)));
		vector<OID> childsVector;
		BSONObj scene = cursorCollection->next();
		if(getChildOIDS(scene, "objectsOIDs", "objectOID", childsVector)>0)
		{
			for (unsigned int i = 0; i<childsVector.size(); i++)
			{
				auto_ptr<DBClientCursor> childCursor =c.query(dbCollectionPath, (QUERY("_id"<<childsVector[i])));
				while(childCursor->more())
				{
					BSONObj objectDocument = childCursor->next();
					//CLOG(LINFO)<<"objectDocument: "<< objectDocument;
					string objectNameFromScene = objectDocument.getField("ObjectName").str();
					if(objectNameFromScene==(string)objectName)
					{
						objectFound=true;
						CLOG(LINFO)<<"Object: "<< objectNameFromScene;
					}
				}
			}
			if(!objectFound)
				CLOG(LERROR)<<"No object founded!";
		}
	}
	else
	{
		CLOG(LERROR)<<"No scene founded!";
	}

}

void SceneReader::prepareInterface() {
        CLOG(LTRACE) << "SceneReader::prepareInterface";

        h_readfromDB.setup(this, &SceneReader::readfromDB);
        registerHandler("Read", &h_readfromDB);
}

bool SceneReader::onInit()
{
        CLOG(LTRACE) << "SceneReader::initialize";
        if(collectionName=="containers")
        	dbCollectionPath="images.containers";
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

bool SceneReader::onFinish()
{
        CLOG(LTRACE) << "SceneReader::finish";
        return true;
}

bool SceneReader::onStep()
{
        CLOG(LTRACE) << "SceneReader::step";
        return true;
}

bool SceneReader::onStop()
{
        return true;
}

bool SceneReader::onStart()
{
        return true;
}


} //: namespace SceneReader
} //: namespace Processors
