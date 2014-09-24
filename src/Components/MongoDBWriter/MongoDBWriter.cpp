/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */


#include "MongoDBWriter.h"
namespace Processors {
namespace MongoDBWriter  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
//using namespace MongoBase;

MongoDBWriter::MongoDBWriter(const string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", string("localhost")),
		objectName("objectName", string("GreenCup")),
		description("description", string("My green coffe cup")),
		collectionName("collectionName", string("containers")),
		extensions("extensions", string("*.png,*.jpg,*.txt")),
		nodeTypeProp("nodeType", string("Object")),
		folderName("folderName", string("/home/lzmuda/mongo_driver_tutorial/")),
		viewNameProp("viewName", string("")),
		sceneName("sceneName", string("")),
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
        registerProperty(sceneName);

        base = new MongoBase::MongoBase();

        CLOG(LTRACE) << "Hello MongoDBWriter";
}

MongoDBWriter::~MongoDBWriter()
{
        CLOG(LTRACE) << "Good bye MongoDBWriter";
}
void MongoDBWriter::write2DB()
{
        CLOG(LNOTICE) << "MongoDBReader::write2DB";
        if(modelNameProp!="")
        	insert2MongoDB(nodeTypeProp,modelNameProp, "Model");
        else if(viewNameProp!="")
            insert2MongoDB(nodeTypeProp,viewNameProp, "View");
        else
        	insert2MongoDB(nodeTypeProp,"", "");

}

void MongoDBWriter::prepareInterface() {
        CLOG(LTRACE) << "MongoDBWriter::prepareInterface";
        h_write2DB.setup(this, &MongoDBWriter::write2DB);
        registerHandler("write2DB", &h_write2DB);
	
       	//insert2MongoDB(c, fs); 
//      registerStream("in_img", &in_img);
//      registerStream("out_img", &out_img);

//	addDependency("insert2MongoDB", &in_img);
}

bool MongoDBWriter::onInit()
{
      CLOG(LTRACE) << "MongoDBWriter::initialize";
      try
      {
		  string ext =extensions;
		  boost::split(fileExtensions, ext, is_any_of(","));
		  c.connect(mongoDBHost);
		  if(collectionName=="containers")
			dbCollectionPath="images.containers";
		  else if(collectionName=="food")
			dbCollectionPath="images.food";
		  else if(collectionName=="dish")
			dbCollectionPath="images.dish";
		  else if(collectionName=="other")
			dbCollectionPath="images.other";

		  docViewsNames.push_back("Stereo");
		  docViewsNames.push_back("Kinect");
		  docViewsNames.push_back("ToF");
		  docViewsNames.push_back("StereoSiLR");
		  docViewsNames.push_back("StereoSiRX");
		  docViewsNames.push_back("StereoSiRXM");
		  docViewsNames.push_back("KinectSiLR");
		  docViewsNames.push_back("KinectSiRX");
		  docViewsNames.push_back("KinectSiRXM");
		  docViewsNames.push_back("ToFSiLR");
		  docViewsNames.push_back("ToFSiRX");
		  docViewsNames.push_back("ToFSiRXM");

		  docModelsNames.push_back("SomRgb");
		  docModelsNames.push_back("SomSift");
		  docModelsNames.push_back("SsomRgb");
		  docModelsNames.push_back("SsomSift");
		  docModelsNames.push_back("SsomShot");
		  docModelsNames.push_back("SSOM");
		  docModelsNames.push_back("SOM");


      }
	 catch(DBException &e)
	 {
		 CLOG(LERROR) <<"Something goes wrong... :<";
		 CLOG(LERROR) <<c.getLastError();
	 }
	 return true;
}

bool MongoDBWriter::onFinish()
{
        CLOG(LTRACE) << "MongoDBWriter::finish";

        return true;
}

bool MongoDBWriter::onStep()
{
        CLOG(LTRACE) << "MongoDBWriter::step";
        return true;
}

bool MongoDBWriter::onStop()
{
        return true;
}

bool MongoDBWriter::onStart()
{
        return true;
}

// dodac splitowanie nazwy scen i dla kazdej ze scen sprawdzac czy jest w bazie i dodawac jak nie ma
void MongoDBWriter::initObject()
{
	CLOG(LTRACE) <<"Create template of object";
	OID o;
	BSONElement bsonElement;
	BSONElement oi;
	BSONArrayBuilder bsonBuilder;
	bool objectInTheScene = false;
	OID oid;
	int items = 0;
	try{
		BSONObj object = BSONObjBuilder().genOID().append("Type", "Object").append("ObjectName", objectName).append("description", description).obj();
		c.insert(dbCollectionPath, object);
		object.getObjectID(oi);
		o=oi.__oid();
		// check if scene exist
		// while()
		//{
			// if exist
			items = c.count(dbCollectionPath, (QUERY("SceneName"<<sceneName)));
			if(items>0)
			{
				auto_ptr<DBClientCursor> cursorCollection =c.query(dbCollectionPath, (QUERY("SceneName"<<sceneName)));
				BSONObj scene = cursorCollection->next();
				CLOG(LINFO)<<"Add scene to the object!";
				scene.getObjectID(oi);
				CLOG(LINFO)<<"1";
				o=oi.__oid();
				CLOG(LINFO)<<"2";

				c.update(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"), BSON("$addToSet"<<BSON("sceneOIDs"<<BSON("sceneOID"<<o.str()))), false, true);
				CLOG(LINFO)<<"3";
				CLOG(LTRACE)<<scene;

				vector<OID> childsVector =  base->getChildOIDS(scene, "objectsOIDs", "objectOID");
				CLOG(LINFO)<<"4";

				for (unsigned int i = 0; i<childsVector.size(); i++)
				{
					CLOG(LINFO)<<"5";

					auto_ptr<DBClientCursor> childCursor =c.query(dbCollectionPath, (QUERY("_id"<<childsVector[i])));
					CLOG(LINFO)<<"6";

					if( childCursor->more())
					{
						CLOG(LINFO)<<"7";

						BSONObj childObj = childCursor->next();
						string _id = childObj.getField("_id").str();
						if(_id==o.str())
						{
							CLOG(LINFO)<<"8";

							objectInTheScene = true;
							CLOG(LERROR)<< "Object exists in the scene!";
							break;
						}
					}
				}
				if(!objectInTheScene)
				{
					CLOG(LINFO)<<"Adding object to the scene";
					object.getObjectID(oi);
					o=oi.__oid();
					c.update(dbCollectionPath, QUERY("SceneName"<<sceneName), BSON("$addToSet"<<BSON("objectsOIDs"<<BSON("objectOID"<<o.str()))), false, true);
				}
				else
					CLOG(LINFO)<<"DUUUUUUUUUUPA!";
			}//if
			else
			{
				CLOG(LINFO)<<"Create scene and add object to array list";
				BSONObj scene = BSONObjBuilder().genOID().append("SceneName", sceneName).obj();
				c.insert(dbCollectionPath, scene);
				CLOG(LINFO)<<"Adding object to the scene";
				c.update(dbCollectionPath, QUERY("SceneName"<<sceneName), BSON("$addToSet"<<BSON("objectsOIDs"<<BSON("objectOID"<<o.str()))), false, true);
				CLOG(LINFO)<<"Add scene to object!";
				scene.getObjectID(oi);
				o=oi.__oid();
				c.update(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"), BSON("$addToSet"<<BSON("sceneOIDs"<<BSON("sceneOID"<<o.str()))), false, true);
			}
		//}

		  vector<string> models = base->getAllFolders((string)folderName+"/Model/");
		  for(std::vector<string>::iterator it = models.begin(); it != models.end(); ++it)
		  {
			  if(*it=="." || *it=="..")
				  continue;
			  BSONObj model = BSONObjBuilder().genOID().append("Type", "Model").append("ObjectName", objectName).append("ModelName", *it).append("description", description).obj();
			  c.insert(dbCollectionPath, model);
			  model.getObjectID(bsonElement);
			  oid=bsonElement.__oid();
			  bsonBuilder.append(BSONObjBuilder().append("childOID", oid.str()).obj());
			  initModel(*it);
		  }
		  vector<string> views = base->getAllFolders((string)folderName+"/View/");
		  for(std::vector<string>::iterator it = views.begin(); it != views.end(); ++it)
		  {
			  if(*it=="." || *it=="..")
				  continue;
			  BSONObj view = BSONObjBuilder().genOID().append("Type", "View").append("ObjectName", objectName).append("ViewName", *it).append("description", description).obj();
			  c.insert(dbCollectionPath, view);
			  view.getObjectID(bsonElement);
			  oid=bsonElement.__oid();
			  bsonBuilder.append(BSONObjBuilder().append("childOID", oid.str()).obj());
			  initView(*it);
		  }
		  BSONArray destArr = bsonBuilder.arr();
		  c.update(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"), BSON("$set"<<BSON("childOIDs"<<destArr)), false, true);
	  }
	  catch(DBException &e)
	  {
		CLOG(LERROR) <<"Something goes wrong... :<";
		CLOG(LERROR) <<c.getLastError();
	  }
}

void MongoDBWriter::initView(const string & viewName)
{
	BSONElement oi;
    OID o;
    BSONArrayBuilder objectArrayBuilder, viewArrayBuilder, stereoArrayBuilder, kinectArrayBuilder, tofArrayBuilder, viewBuilder;

    if(nodeTypeProp=="View")
    {
    	unsigned long long nr = c.count(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"));
		// add object
		if(nr==0)
		{
			CLOG(LTRACE) <<"Object does not exists in "<< dbCollectionPath;
			BSONObj object = BSONObjBuilder().genOID().append("Type", "Object").append("ObjectName", objectName).append("description", description).append("sceneName", sceneName).obj();
			c.insert(dbCollectionPath, object);
		}
		// add view
		BSONObj view = BSONObjBuilder().genOID().append("Type", "View").append("ObjectName", objectName).append("ViewName", viewName).append("description", description).obj();
		c.insert(dbCollectionPath, view);
		view.getObjectID(oi);
		o=oi.__oid();
		c.update(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"), BSON("$addToSet"<<BSON("childOIDs"<<BSON("childOID"<<o.str()))), false, true);
	}

    for(std::vector<string>::iterator it = docViewsNames.begin(); it != docViewsNames.end(); ++it){
		BSONObj document = BSONObjBuilder().genOID().append("Type", *it).append("ObjectName", objectName).append("ViewName", viewName).append("description", description).obj();
		c.insert(dbCollectionPath, document);

		document.getObjectID(oi);
		o=oi.__oid();

		if(*it=="Stereo" || *it=="Kinect" || *it=="ToF")
			viewArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
		else if(*it=="StereoSiLR" || *it=="StereoSiRX" || *it=="StereoSiRXM")
			stereoArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
		else if(*it=="KinectSiLR" || *it=="KinectSiRX" || *it=="KinectSiRXM")
			kinectArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
		else if(*it=="ToFSiLR" || *it=="ToFSiRX" || *it=="ToFSiRXM")
			tofArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
	}
	BSONArray viewArr = viewArrayBuilder.arr();
    BSONArray stereoArr = stereoArrayBuilder.arr();
    BSONArray kinectArr = kinectArrayBuilder.arr();
    BSONArray tofArr = tofArrayBuilder.arr();

    c.update(dbCollectionPath, QUERY("Type"<<"View"<<"ObjectName"<<objectName<<"ViewName"<<viewName), BSON("$set"<<BSON("childOIDs"<<viewArr)), false, true);
    c.update(dbCollectionPath, QUERY("Type"<<"ToF"<<"ObjectName"<<objectName<<"ViewName"<<viewName), BSON("$set"<<BSON("childOIDs"<<tofArr)), false, true);
    c.update(dbCollectionPath, QUERY("Type"<<"Kinect"<<"ObjectName"<<objectName<<"ViewName"<<viewName), BSON("$set"<<BSON("childOIDs"<<kinectArr)), false, true);
    c.update(dbCollectionPath, QUERY("Type"<<"Stereo"<<"ObjectName"<<objectName<<"ViewName"<<viewName), BSON("$set"<<BSON("childOIDs"<<stereoArr)), false, true);
}

void MongoDBWriter::initModel(const string & modelName)
{
	CLOG(LTRACE)<<"initModel";
	BSONElement oi;
	OID o;
	BSONArrayBuilder objectArrayBuilder, modelArrayBuilder, somArrayBuilder, ssomArrayBuilder;

	if(nodeTypeProp=="Model")
	{
		unsigned long long nr = c.count(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"));
		// add object
		if(nr==0)
		{
			CLOG(LTRACE) <<"Object does not exists in "<< dbCollectionPath;
			BSONObj object = BSONObjBuilder().genOID().append("Type", "Object").append("ObjectName", objectName).append("description", description).append("sceneName", sceneName).obj();
			c.insert(dbCollectionPath, object);
		}
		// add model
		BSONObj model = BSONObjBuilder().genOID().append("Type", "Model").append("ObjectName", objectName).append("ModelName", modelName).append("description", description).obj();
		c.insert(dbCollectionPath, model);
		model.getObjectID(oi);
		o=oi.__oid();
		c.update(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"), BSON("$addToSet"<<BSON("childOIDs"<<BSON("childOID"<<o.str()))), false, true);
	}

	for(std::vector<string>::iterator it = docModelsNames.begin(); it != docModelsNames.end(); ++it){
		BSONObj document = BSONObjBuilder().genOID().append("Type", *it).append("ObjectName", objectName).append("ModelName", modelName).append("description", description).obj();
		c.insert(dbCollectionPath, document);

		document.getObjectID(oi);
		o=oi.__oid();

		if(*it=="SOM" || *it=="SSOM")
			modelArrayBuilder.append(BSONObjBuilder().append("chobjectsOIDsildOID", o.str()).obj());
		else if(*it=="SomRgb" || *it=="SomSift")
			somArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
		else if(*it=="SsomRgb" || *it=="SsomSift" || *it=="SsomShot")
			ssomArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
	}

	BSONArray modelArr = modelArrayBuilder.arr();
	BSONArray somArr = somArrayBuilder.arr();
	BSONArray ssomArr = ssomArrayBuilder.arr();

	c.update(dbCollectionPath, QUERY("Type"<<"Model"<<"ObjectName"<<objectName<<"ModelName"<<modelName), BSON("$set"<<BSON("childOIDs"<<modelArr)), false, true);
	c.update(dbCollectionPath, QUERY("Type"<<"SOM"<<"ObjectName"<<objectName<<"ModelName"<<modelName), BSON("$set"<<BSON("childOIDs"<<somArr)), false, true);
	c.update(dbCollectionPath, QUERY("Type"<<"SSOM"<<"ObjectName"<<objectName<<"ModelName"<<modelName), BSON("$set"<<BSON("childOIDs"<<ssomArr)), false, true);
}

void  MongoDBWriter::setMime( const std::vector<string>::iterator itExtension,  string& mime)
{
	if (*itExtension=="*.png")
		mime="image/png";
	else if(*itExtension=="*.jpg")
		mime= "image/jpeg";
	else if(*itExtension=="*.txt" || *itExtension=="*.pcd")
		mime="text/plain";
	else
	{
		CLOG(LERROR) <<"I don't know such file extension! Please add extension to the `if` statement from http://www.sitepoint.com/web-foundations/mime-types-complete-list/";
		return;
	}
}

void MongoDBWriter::insertFileToGrid( const std::vector<string>::iterator itExtension, const std::vector<string>::iterator it, const string& newFileName, BSONArrayBuilder& bsonBuilder)
{
	BSONObj o;
	BSONElement bsonElement;
	OID oid;
	string mime="";
	setMime(itExtension, mime);
	GridFS fs(c, collectionName);
	o = fs.storeFile(*it, newFileName, mime);
	BSONObj b = BSONObjBuilder().appendElements(o).append("ObjectName", objectName).obj();
	c.insert(dbCollectionPath, b);
	b.getObjectID(bsonElement);
	oid=bsonElement.__oid();
	bsonBuilder.append(BSONObjBuilder().append("childOID", oid.str()).obj());
}

void MongoDBWriter::writeNode2MongoDB(const string &source, const string &destination, const string &type,string modelOrViewName)
{
	BSONArrayBuilder bsonBuilder;
	CLOG(LTRACE) <<"Source: " << source << " destination: "<< destination<<" dbCollectionPath: "<<dbCollectionPath;
    try{
    	for(std::vector<string>::iterator itExtension = fileExtensions.begin(); itExtension != fileExtensions.end(); ++itExtension) {
    		CLOG(LTRACE) <<"source+*itExtension "<<source+*itExtension;
			vector<string> files = base->getAllFiles(source+*itExtension);
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
		c.update(dbCollectionPath, QUERY("Type"<<destination<<"ObjectName"<<objectName<<type+"Name"<<modelOrViewName), BSON("$set"<<BSON("childOIDs"<<destArr)), false, true);
		CLOG(LTRACE) <<"Files saved successfully";
    }
	catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :<";
		CLOG(LERROR) <<c.getLastError();
	}
}

void MongoDBWriter::setModelOrViewName(const string& childNodeName, const BSONObj& childObj)
{
	string type = childNodeName;
	string modelOrViewName = childObj.getField(type+"Name").str();
	insert2MongoDB(childNodeName, modelOrViewName, type);
}

void MongoDBWriter::insert2MongoDB(const string &destination, const string&  modelOrViewName, const string&  type)
{
	auto_ptr<DBClientCursor> cursorCollection;
	string source;
	int items=0;
	try{
		if(destination=="Object")
		{
			unsigned long long nr = c.count(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"));
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
		if(base->isViewLastLeaf(destination) || base->isModelLastLeaf(destination))
		{
				source = (string)folderName+type+"/"+modelOrViewName+"/"+destination+"/";
				writeNode2MongoDB(source, destination, type, modelOrViewName);
		}
		else
		{
			if(destination=="Model" || destination=="View")
			{
				if(nodeTypeProp=="Model" || nodeTypeProp=="View")
				{
					unsigned long long nr = c.count(dbCollectionPath, (QUERY("Type"<<type<<"ObjectName"<<objectName<<type+"Name"<<modelOrViewName)));
					if(nr>0)
					{
						CLOG(LERROR)<<type+" "<< modelOrViewName<<" exists in db for object "<<objectName;
						return;
					}
					else
					{
						if(destination=="Model")
							initModel(modelOrViewName);
						else if(destination=="View")
							initView(modelOrViewName);
					}
				}
			}
			base->findDocumentInCollection(c, dbCollectionPath, objectName, destination, cursorCollection, modelOrViewName, type, items);
			if(items>0)
			{
				while (cursorCollection->more())
				{
						BSONObj obj = cursorCollection->next();
						CLOG(LTRACE)<< obj;
						vector<OID> childsVector =  base->getChildOIDS(obj, "childOIDs", "childOID");
						for (unsigned int i = 0; i<childsVector.size(); i++)
						{
							auto_ptr<DBClientCursor> childCursor =c.query(dbCollectionPath, (QUERY("_id"<<childsVector[i])));
							if( childCursor->more())
							{
								BSONObj childObj = childCursor->next();
								string childNodeName = childObj.getField("Type").str();
								CLOG(LINFO)<< "childNodeName: "<<childNodeName;
								if(childNodeName!="EOO")
								{
									if(childNodeName=="View"||childNodeName=="Model")
									{
										setModelOrViewName(childNodeName, childObj);
									}
									else
										insert2MongoDB(childNodeName, modelOrViewName, type);
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
		CLOG(LERROR) <<c.getLastError();
	}
}

} //: namespace MongoDBWriter
} //: namespace Processors
