/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */
#include "SceneRemover.hpp"

#define siftPointSize 133
#define xyzPointSize 3
#define xyzrgbPointSize 4

namespace Processors {
namespace SceneRemover  {
using namespace cv;
using namespace mongo;
using namespace boost::property_tree;

SceneRemover::SceneRemover(const std::string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", string("localhost")),
		sceneName("sceneName", string("scene3")),
		collectionName("collectionName", string("containers"))
{
		registerProperty(mongoDBHost);
		registerProperty(sceneName);
		registerProperty(collectionName);

        CLOG(LTRACE) << "Hello SceneRemover";

        //base = new MongoBase::MongoBase();
}

SceneRemover::~SceneRemover()
{
        CLOG(LTRACE) << "Good bye SceneRemover";
}

void SceneRemover::readfromDB()
{
	CLOG(LNOTICE) << "SceneRemover::readfromDB";
	removeSceneFromMongoDB();
}
void SceneRemover::prepareInterface() {
	CLOG(LTRACE) << "SceneRemover::prepareInterface";
	h_readfromDB.setup(this, &SceneRemover::readfromDB);
	registerHandler("Remove", &h_readfromDB);
}

bool SceneRemover::onInit()
{
	CLOG(LTRACE) << "SceneRemover::initialize";
	if(collectionName=="containers")
		dbCollectionPath="images.containers";

	name_cloud_xyz="";
	name_cloud_xyzrgb="";
	name_cloud_xyzsift="";

	string hostname = mongoDBHost;
	connectToMongoDB(hostname);
	if(collectionName=="containers")
		MongoBase::dbCollectionPath=dbCollectionPath="images.containers";

	return true;
}

bool SceneRemover::onFinish()
{
        CLOG(LTRACE) << "SceneRemover::finish";
        return true;
}

bool SceneRemover::onStep()
{
        CLOG(LTRACE) << "SceneRemover::step";
        return true;
}

bool SceneRemover::onStop()
{
        return true;
}

bool SceneRemover::onStart()
{
        return true;
}

void SceneRemover::removeSceneFromMongoDB()
{
	CLOG(LTRACE)<<"SceneRemover::readFromMongoDB";
	string name;
	std::vector<AbstractObject*> models;
	try{
		int items=0;
		cursorCollection  =c->query(dbCollectionPath, Query(BSON("SceneName"<<sceneName)));
		CLOG(LINFO)<<"Founded some data";
		while (cursorCollection->more())
		{
			BSONObj obj = cursorCollection->next();
			BSONObj objTemp = obj;
			//remove document
			BSONElement oi;
			objTemp.getObjectID(oi);
			OID docOID = oi.__oid();
			CLOG(LFATAL)<<"Usuwamy: "<<docOID.toString();
			c->remove(dbCollectionPath , Query(BSON("_id" << docOID)));
			CLOG(LFATAL)<<"usunieto";

			vector<OID> childsVector;
			int items =  getChildOIDS(obj, "objectsOIDs", "objectOID", childsVector);
			// if node has a child
			int queryOptions = 0;
			const BSONObj *fieldsToReturn = 0;

			vector<OID> scenesVector;
			// get all scenes of object
			//int items =  getChildOIDS(obj, "sceneOIDs", "sceneOID", scenesVector);
			for(std::vector<OID>::iterator it = childsVector.begin(); it != childsVector.end(); ++it)
			{
				BSONObj objFile = c->findOne(dbCollectionPath, Query(BSON("_id" << *it)), fieldsToReturn, queryOptions);
				string objectName = objFile.getField("ObjectName").str();
				//objFile.getObjectID(oi);
				//OID docOIDObject = oi.__oid();

				// remove object from scene
				CLOG(LERROR)<<"remove scene from: "<<objectName;
				c->update(dbCollectionPath, Query(BSON("ObjectName"<<objectName)), BSON("$pull"<<BSON("sceneOIDs"<<BSON("sceneOID"<<docOID.toString()))), false, true);
			}
		}
	}//try
	catch(DBException &e)
	{
		CLOG(LERROR) <<"RemoveFromMongoDB(). Something goes wrong... :<";
		e.what();
		exit(1);
	}
}
} //: namespace SceneRemover
} //: namespace Processors
