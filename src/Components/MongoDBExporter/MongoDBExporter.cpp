/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */


#include "MongoDBExporter.hpp"
namespace Processors {
namespace MongoDBExporter  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;

MongoDBExporter::MongoDBExporter(const string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", string("localhost")),
		objectName("objectName", string("GreenCup")),
		description("description", string("My green coffe cup")),
		collectionName("collectionName", string("containers")),
		extensions("extensions", string("*.png,*.jpg,*.txt")),
		nodeTypeProp("nodeType", string("Object")),
		folderName("folderName", string("/home/lzmuda/mongo_driver_tutorial/")),
		viewNameProp("viewName", string("")),
		sceneNamesProp("sceneNamesProp", string("scene1,scene2,scene3")),
		modelNameProp("modelName", string(""))
		//folderName("folderName", string("./"))
{
        registerProperty(mongoDBHost);
        registerProperty(objectName);
        registerProperty(description);
        registerProperty(collectionName);
        registerProperty(extensions);
        registerProperty(nodeTypeProp);
        registerProperty(folderName);
        registerProperty(viewNameProp);
        registerProperty(modelNameProp);
        registerProperty(sceneNamesProp);

        //base = new MongoBase::MongoBase();

        CLOG(LTRACE) << "Hello MongoDBExporter";
}

MongoDBExporter::~MongoDBExporter()
{
        CLOG(LTRACE) << "Good bye MongoDBExporter";
}
void MongoDBExporter::write2DB()
{
        CLOG(LNOTICE) << "MongoDBExporter::write2DB";

        string ext =extensions;
        boost::split(fileExtensions, ext, is_any_of(","));
        string sceneNames = sceneNamesProp;
        boost::split(MongoBase::splitedSceneNames, sceneNames, is_any_of(","));
        if(modelNameProp!="")
        	insert2MongoDB(nodeTypeProp,modelNameProp, "Model");
        else if(viewNameProp!="")
            insert2MongoDB(nodeTypeProp,viewNameProp, "View");
        else
        	insert2MongoDB(nodeTypeProp,"", "");
}

void MongoDBExporter::prepareInterface() {
        CLOG(LTRACE) << "MongoDBExporter::prepareInterface";
        h_write2DB.setup(this, &MongoDBExporter::write2DB);
        registerHandler("write2DB", &h_write2DB);
	
       	//insert2MongoDB(c, fs); 
//      registerStream("in_img", &in_img);
//      registerStream("out_img", &out_img);

//	addDependency("insert2MongoDB", &in_img);
}

bool MongoDBExporter::onInit()
{
      CLOG(LTRACE) << "MongoDBExporter::initialize";
      try
      {
    	  string hostname = mongoDBHost;
    	  connectToMongoDB(hostname);
		  if(collectionName=="containers")
			dbCollectionPath="images.containers";
		  initViewNames();
		  initModelNames();
      }
	 catch(DBException &e)
	 {
		 CLOG(LERROR) <<"Something goes wrong... :<";
		 CLOG(LERROR) <<c->getLastError();
	 }
	 return true;
}

bool MongoDBExporter::onFinish()
{
        CLOG(LTRACE) << "MongoDBExporter::finish";

        return true;
}

bool MongoDBExporter::onStep()
{
        CLOG(LTRACE) << "MongoDBExporter::step";
        return true;
}

bool MongoDBExporter::onStop()
{
        return true;
}

bool MongoDBExporter::onStart()
{
        return true;
}


void MongoDBExporter::createModelOrView(const std::vector<string>::iterator it, const string& type, BSONArrayBuilder& bsonBuilder)
{
	BSONElement bsonElement;
	if(*it=="." || *it=="..")
		return;
	BSONObj model = BSONObjBuilder().genOID().append("Type", type).append("ObjectName", objectName).append(type+"Name", *it).append("description", description).obj();
	c->insert(dbCollectionPath, model);
	model.getObjectID(bsonElement);
	OID oid=bsonElement.__oid();
	bsonBuilder.append(BSONObjBuilder().append("childOID", oid.str()).obj());
	if(type=="Model")
		initModel(*it, true, nodeTypeProp, objectName, description);
	else if(type=="View")
		initView(*it, true, nodeTypeProp, objectName, description);
}

void MongoDBExporter::initObject()
{
	CLOG(LTRACE) <<"Create template of object";
	BSONArrayBuilder bsonBuilder;
	bool objectInTheScene = false;
	try
	{
		BSONObj object = BSONObjBuilder().genOID().append("Type", "Object").append("ObjectName", objectName).append("description", description).obj();
		c->insert(dbCollectionPath, object);
		c->createIndex(dbCollectionPath, BSON("ObjectName"<<1));
		addScenes(object, objectName);

		vector<string> models = getAllFolders((string)folderName+"/Model/");
		for(std::vector<string>::iterator it = models.begin(); it != models.end(); ++it)
		{
			string type = "Model";
			CLOG(LTRACE)<<"Create Model";
			createModelOrView(it, type, bsonBuilder);
		}

		vector<string> views = getAllFolders((string)folderName+"/View/");
		for(std::vector<string>::iterator it = views.begin(); it != views.end(); ++it)
		{
			string type = "View";
			CLOG(LTRACE)<<"Create View";
			createModelOrView(it, type, bsonBuilder);
		}

		BSONArray destArr = bsonBuilder.arr();
		c->update(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"), BSON("$set"<<BSON("childOIDs"<<destArr)), false, true);
	  }
	  catch(DBException &e)
	  {
		CLOG(LERROR) <<"Something goes wrong... :<";
		CLOG(LERROR) <<c->getLastError();
	  }
}



void MongoDBExporter::insertFileToGrid( const std::vector<string>::iterator itExtension, const std::vector<string>::iterator it, const string& newFileName, BSONArrayBuilder& bsonBuilder)
{
	BSONObj o;
	BSONElement bsonElement;
	OID oid;
	string mime="";
	setMime(*itExtension, mime);
	GridFS fs(*c, collectionName);
	o = fs.storeFile(*it, newFileName, mime);
	BSONObj b = BSONObjBuilder().appendElements(o).append("ObjectName", objectName).obj();
	c->createIndex(dbCollectionPath, BSON("filename"<<1));
	c->insert(dbCollectionPath, b);
	b.getObjectID(bsonElement);
	oid=bsonElement.__oid();
	bsonBuilder.append(BSONObjBuilder().append("childOID", oid.str()).obj());
}

void MongoDBExporter::writeNode2MongoDB(const string &source, const string &destination, const string &type,string modelOrViewName)
{
	BSONArrayBuilder bsonBuilder;
	CLOG(LTRACE) <<"Source: " << source << " destination: "<< destination<<" dbCollectionPath: "<<dbCollectionPath;
    try{
    	for(std::vector<string>::iterator itExtension = fileExtensions.begin(); itExtension != fileExtensions.end(); ++itExtension) {
    		CLOG(LTRACE) <<"source+*itExtension "<<source+*itExtension;
			vector<string> files = getAllFiles(source+*itExtension);
			for(std::vector<string>::iterator it = files.begin(); it != files.end(); ++it)
			{
				string fileName = *it;
				string newFileName;
				const size_t last_slash_idx = fileName.find_last_of("/");
				if (std::string::npos != last_slash_idx)
				{
					newFileName = fileName.erase(0, last_slash_idx + 1);
				}
				insertFileToGrid(itExtension, it, newFileName, bsonBuilder);
			}
    	}
		BSONArray destArr = bsonBuilder.arr();
		//if(type=="View" || type=="Model")
		c->update(dbCollectionPath, QUERY("Type"<<destination<<"ObjectName"<<objectName<<type+"Name"<<modelOrViewName), BSON("$set"<<BSON("childOIDs"<<destArr)), false, true);
		CLOG(LTRACE) <<"Files saved successfully";
    }
	catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :<";
		CLOG(LERROR) <<c->getLastError();
	}
}

void MongoDBExporter::insert2MongoDB(const string &destination, const string&  modelOrViewName, const string&  type)
{
	auto_ptr<DBClientCursor> cursorCollection;
	string source;
	int items=0;
	try{
		if(destination=="Object")
		{
			unsigned long long nr = c->count(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"));
			if(nr==0)
			{
				CLOG(LTRACE) <<"Object does not exists in "<< dbCollectionPath;
				initObject();
			}
			else
			{
					CLOG(LERROR) <<"Object "<<objectName<<" exist in "<< dbCollectionPath;
					return;
			}
		}
		if(isViewLastLeaf(destination) || isModelLastLeaf(destination))
		{
			unsigned long long nr = c->count(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"));
			if(nr==0)
			{
				CLOG(LTRACE) <<"Object does not exists in "<< dbCollectionPath;
				initObject();
			}
			else
			{
				int items = c->count(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<type<<type+"Name"<<modelOrViewName));
				if(items==0)
				{
					CLOG(LTRACE)<<"No such model/view";
					CLOG(LTRACE)<<"Type: "<<type;
					if(type=="View")
						initView(modelOrViewName, true, nodeTypeProp, objectName, description);
					else if(type=="Model")
						initModel(modelOrViewName, true, nodeTypeProp, objectName, description);
				}
				cursorCollection = c->query(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<type<<type+"Name"<<modelOrViewName));
				BSONObj obj = cursorCollection->next();
				vector<OID> childsVector;
				// check if node has some files
				if(getChildOIDS(obj, "childOIDs", "childOID", childsVector)>0 && childsVector.size()>0)
				{
					CLOG(LTRACE)<<type <<"There are some files in Mongo in this node!";
				}
			}
			CLOG(LINFO)<<"Write to model or view";
			source = (string)folderName+type+"/"+modelOrViewName+"/"+destination+"/";
			writeNode2MongoDB(source, destination, type, modelOrViewName);
		}
		else
		{
			if(destination=="Model" || destination=="View")
			{
				if(nodeTypeProp=="Model" || nodeTypeProp=="View")
				{
					unsigned long long nr = c->count(dbCollectionPath, (QUERY("Type"<<type<<"ObjectName"<<objectName<<type+"Name"<<modelOrViewName)));
					if(nr>0)
					{
						CLOG(LERROR)<<type+" "<< modelOrViewName<<" exists in db for object "<<objectName;
						return;
					}
					else
					{
						if(destination=="Model")
							initModel(modelOrViewName, true, nodeTypeProp, objectName, description);
						else if(destination=="View")
							initView(modelOrViewName, true, nodeTypeProp, objectName, description);
					}
				}
			}
			findDocumentInCollection(*c, dbCollectionPath, objectName, destination, cursorCollection, modelOrViewName, type, items);
			if(items>0)
			{
				while (cursorCollection->more())
				{
						BSONObj obj = cursorCollection->next();
						CLOG(LTRACE)<< obj;
						vector<OID> childsVector;
						if(getChildOIDS(obj, "childOIDs", "childOID", childsVector)>0)
						{
							for (unsigned int i = 0; i<childsVector.size(); i++)
							{
								auto_ptr<DBClientCursor> childCursor =c->query(dbCollectionPath, (QUERY("_id"<<childsVector[i])));
								if( childCursor->more())
								{
									BSONObj childObj = childCursor->next();
									string childNodeName = childObj.getField("Type").str();
									CLOG(LINFO)<< "childNodeName: "<<childNodeName;
									if(childNodeName!="EOO")
									{
										if(childNodeName=="View"||childNodeName=="Model")
										{
											string newName;
											setModelOrViewName(childNodeName, childObj, newName);
											insert2MongoDB(childNodeName, newName, type);
										}
										else
											insert2MongoDB(childNodeName, modelOrViewName, type);
									}
								}
							}
						}
				}//while
			}
			else
				CLOG(LTRACE)<"Wrong name";
		}//else
    }//try
	catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :<";
		CLOG(LERROR) <<c->getLastError();
	}
}

} //: namespace MongoDBExporter
} //: namespace Processors
