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
using namespace MongoProxy;

SceneRemover::SceneRemover(const std::string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", string("localhost")),
		sceneName("sceneName", string("scene3"))
{
		registerProperty(mongoDBHost);
		registerProperty(sceneName);

        CLOG(LTRACE) << "Hello SceneRemover";

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
	hostname = mongoDBHost;
	MongoProxy::MongoProxy::getSingleton(hostname);
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
	try{
		int items=0;
		BSONObj query = BSON("SceneName"<<sceneName<<"DocumentType"<<"Scene");
		cursorCollection = MongoProxy::MongoProxy::getSingleton(hostname).query(query);
		while (cursorCollection->more())
		{
			BSONObj obj = cursorCollection->next();
			//remove document
			BSONElement oi;
			obj.getObjectID(oi);
			OID sceneOID = oi.__oid();
			CLOG(LINFO)<<"Usuwamy: "<<sceneOID.toString();
			MongoProxy::MongoProxy::getSingleton(hostname).remove(sceneOID);
			CLOG(LINFO)<<"Usunieto";

			vector<OID> viewsList;
			int items =  MongoProxy::MongoProxy::getSingleton(hostname).getChildOIDS(obj, "ViewsList", "viewOID", viewsList);
			if (items==0)
			{
				LOG(LINFO)<<"No views founded to update";
				return;
			}

			for(std::vector<OID>::iterator it = viewsList.begin(); it != viewsList.end(); ++it)
			{
				BSONObj query = BSON("_id" << *it);
				BSONObj objFile = MongoProxy::MongoProxy::getSingleton(hostname).findOne(query);
				string viewName = objFile.getField("ViewName").str();

				// remove scene from view
				CLOG(LINFO)<<"remove scene from: "<<*it;
				query = BSON("ViewName"<<viewName<<"DocumentType"<<"View");

				// remove sceneName and sceneOID, if you want to add, use $set operator
				BSONObj update = BSON("$unset"<<BSON("SceneOID"<<1 <<"SceneName"<<1));
				MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);
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
