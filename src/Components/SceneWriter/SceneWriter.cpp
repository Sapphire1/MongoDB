/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */


#include "SceneWriter.hpp"
namespace Processors {
namespace SceneWriter  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
#include <stddef.h>

SceneWriter::SceneWriter(const string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", string("localhost")),
		objectName("objectName", string("GreenCup")),
		description("description", string("My green coffe cup")),
		collectionName("collectionName", string("containers")),
		nodeTypeProp("nodeType", string("Object")),
		sceneNamesProp("sceneNamesProp", string("scene1,scene2,scene3"))
{
        registerProperty(mongoDBHost);
        registerProperty(objectName);
        registerProperty(description);
        registerProperty(collectionName);
        registerProperty(nodeTypeProp);
        registerProperty(sceneNamesProp);
        base = new MongoBase::MongoBase();

        CLOG(LTRACE) << "Hello SceneWriter";
}

SceneWriter::~SceneWriter()
{
        CLOG(LTRACE) << "Good bye SceneWriter";
}
void SceneWriter::write2DB()
{
        CLOG(LNOTICE) << "SceneWriter::write2DB";
        string sceneNames = sceneNamesProp;
        boost::split(splitedSceneNames, sceneNames, is_any_of(","));
        BSONObj object;
        initObject();
}

void SceneWriter::prepareInterface() {
        CLOG(LTRACE) << "SceneWriter::prepareInterface";
        h_write2DB.setup(this, &SceneWriter::write2DB);
        registerHandler("write2DB", &h_write2DB);
}

bool SceneWriter::onInit()
{
      CLOG(LTRACE) << "SceneWriter::initialize";
      try
      {
		  c.connect(mongoDBHost);
		  if(collectionName=="containers")
			dbCollectionPath="images.containers";
		  else if(collectionName=="food")
			dbCollectionPath="images.food";
		  else if(collectionName=="dish")
			dbCollectionPath="images.dish";
		  else if(collectionName=="other")
			dbCollectionPath="images.other";
      }
	 catch(DBException &e)
	 {
		 CLOG(LERROR) <<"Something goes wrong... :<";
		 CLOG(LERROR) <<c.getLastError();
	 }
	 return true;
}

bool SceneWriter::onFinish()
{
        CLOG(LTRACE) << "SceneWriter::finish";

        return true;
}

bool SceneWriter::onStep()
{
        CLOG(LTRACE) << "SceneWriter::step";
        return true;
}

bool SceneWriter::onStop()
{
        return true;
}

bool SceneWriter::onStart()
{
        return true;
}

void SceneWriter::addScenes(BSONObj& object)
{
	int items = 0;
	bool objectInTheScene = false;
	OID objectOID;
	OID sceneOID;
	BSONElement bsonElement;
	BSONElement sceneOI;
	BSONElement objectOI;
	BSONArrayBuilder bsonBuilder;

	object.getObjectID(objectOI);
	objectOID=objectOI.__oid();
	CLOG(LINFO)<<"objectOID.str(): "<<objectOID.str();

try{
	for(std::vector<string>::iterator itSceneName = splitedSceneNames.begin(); itSceneName != splitedSceneNames.end(); ++itSceneName)
	{
		CLOG(LTRACE)<<"Scene: "<<*itSceneName;
		// if scene exist
		items = c.count(dbCollectionPath, (QUERY("SceneName"<<*itSceneName)));

		// jesli scena istnieje
		if(items>0)
		{
			auto_ptr<DBClientCursor> cursorCollection =c.query(dbCollectionPath, (QUERY("SceneName"<<*itSceneName)));
			BSONObj scene = cursorCollection->next();
			scene.getObjectID(sceneOI);
			sceneOID=sceneOI.__oid();
			CLOG(LINFO)<<"Add scene to the object items>0!";

			vector<OID> childsVector;
			if(base->getChildOIDS(scene, "objectsOIDs", "objectOID", childsVector)>0)
			{

				CLOG(LINFO)<<"Sprawdzam czy scena zawiera juz ten obiekt";
				for (unsigned int i = 0; i<childsVector.size(); i++)
				{
					CLOG(LINFO)<<"Sprawdzam obiekt";
					auto_ptr<DBClientCursor> childCursor =c.query(dbCollectionPath, (QUERY("_id"<<childsVector[i])));
					CLOG(LINFO)<<"childsVector[i].str(): "<<childsVector[i].str();
					CLOG(LINFO)<<"objectOID.str(): "<<objectOID.str();
					if(childsVector[i].str()==objectOID.str())
					{
						objectInTheScene = true;
						CLOG(LERROR)<< "Object exists in the scene!";
						break;
					}
				}
			}
			if(!objectInTheScene)
			{
				CLOG(LINFO)<<"objectOID.str(): "<<objectOID.str();
				CLOG(LINFO)<<"Adding object to the scene and scene to the object";
				c.update(dbCollectionPath, QUERY("SceneName"<<*itSceneName), BSON("$addToSet"<<BSON("objectsOIDs"<<BSON("objectOID"<<objectOID.str()))), false, true);
				// jak nie ma obiektu w scenie to w obiekcie nie ma sceny
				CLOG(LINFO)<<"sceneOID.str(): "<<sceneOID.str();
				c.update(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"), BSON("$addToSet"<<BSON("sceneOIDs"<<BSON("sceneOID"<<sceneOID.str()))), false, true);
			}
		}//if
		else
		{
			CLOG(LINFO)<<"Create scene and add object to array list";
			BSONObj scene = BSONObjBuilder().genOID().append("SceneName", *itSceneName).obj();
			c.insert(dbCollectionPath, scene);

			CLOG(LINFO)<<"Adding object to the scene";
			if(object.isEmpty())
				CLOG(LINFO)<<"Object is empty";
			CLOG(LINFO)<<"OID: "<< objectOID.str();
			c.update(dbCollectionPath, QUERY("SceneName"<<*itSceneName), BSON("$addToSet"<<BSON("objectsOIDs"<<BSON("objectOID"<<objectOID.str()))), false, true);

			CLOG(LINFO)<<"Add scene to object!";
			scene.getObjectID(sceneOI);
			sceneOID=sceneOI.__oid();
			c.update(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"), BSON("$addToSet"<<BSON("sceneOIDs"<<BSON("sceneOID"<<sceneOID.str()))), false, true);
		}
	}
}catch(DBException & ex)
{
	CLOG(LERROR)<<ex.what();
}
}

void SceneWriter::initObject()
{
	CLOG(LTRACE) <<"Create template of object";
	BSONArrayBuilder bsonBuilder;
	bool objectInTheScene = false;
	auto_ptr<DBClientCursor> cursorCollection;
	BSONObj object;
	BSONElement oi;
	try
	{
		int items = c.count(dbCollectionPath, (QUERY("Type"<<"Object"<<"ObjectName"<<objectName)));
		if(items>0)
		{
			CLOG(LTRACE)<<"Object exist";
			auto_ptr<DBClientCursor> cursorCollection =c.query(dbCollectionPath, (QUERY("Type"<<"Object"<<"ObjectName"<<objectName)));
			object=cursorCollection->next();
			OID objectOID;
			object.getObjectID(oi);
			objectOID=oi.__oid();
			CLOG(LINFO)<<"objectOID.str(): "<<objectOID.str();
		}
		else
		{
			CLOG(LTRACE)<<"Object does not exist, create object";
			object = BSONObjBuilder().genOID().append("Type", "Object").append("ObjectName", objectName).append("description", description).obj();
			c.insert(dbCollectionPath, object);
		}
		addScenes(object);
  }
	  catch(DBException &e)
	  {
		CLOG(LERROR) <<"Something goes wrong... :<";
		CLOG(LERROR) <<c.getLastError();
	  }
}


} //: namespace SceneWriter
} //: namespace Processors
