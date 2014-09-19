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

MongoDBWriter::MongoDBWriter(const string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", string("localhost")),
		objectName("objectName", string("GreenCup")),
		description("description", string("My green coffe cup")),
		collectionName("collectionName", string("containers")),
		extensions("extensions", string("*.png,*.jpg,*.txt")),
		nodeTypeProp("nodeType", string("Object")),
		folderName("folderName", string("/home/lzmuda/mongo_driver_tutorial/")),
		viewNameProp("viewName", string("")),
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
        CLOG(LTRACE) << "Hello MongoDBWriter";
}
void MongoDBWriter::connect2MongoDB()
{
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
        	 CLOG(LERROR) <<"Something goes wrong... :>";
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

void MongoDBWriter::onNewImage()
{
        CLOG(LNOTICE) << "MongoDBWriter::onNewImage";
}

vector<string> MongoDBWriter::getAllFiles(const string& pattern)
{
	glob_t glob_result;
	glob(pattern.c_str(),GLOB_TILDE,NULL,&glob_result);
	vector<string> files;
	for(unsigned int i=0;i<glob_result.gl_pathc;++i){
	    files.push_back(string(glob_result.gl_pathv[i]));
	}
	globfree(&glob_result);
	return files;

}

vector<string> MongoDBWriter::getAllFolders(const string& directoryPath)
{
	vector<string> directories;
	const char *cstr = directoryPath.c_str();
	DIR *dir = opendir(cstr);
    struct dirent *entry = readdir(dir);
    while (entry != NULL)
    {
    	if (entry->d_type == DT_DIR)
    		directories.push_back(entry->d_name);
    	entry = readdir(dir);
    }
    closedir(dir);
    return directories;
}

vector<OID>  MongoDBWriter::getChildOIDS(BSONObj &obj)
{
	  //CLOG(LTRACE) << "Processing JSON document: " << obj.toString() << std::endl;
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

auto_ptr<DBClientCursor>  MongoDBWriter::findDocumentInCollection(string nodeName, string type, string name)
{
      auto_ptr<DBClientCursor> cursorCollection;
      try{
    	 if(type=="" || name=="")
    		 cursorCollection =c.query(dbCollectionPath, (QUERY("Type"<<nodeName<<"ObjectName"<<objectName)));
    	 else if(type=="View")
    		 cursorCollection =c.query(dbCollectionPath, (QUERY("Type"<<nodeName<<"ObjectName"<<objectName<<"ViewName"<<name)));
    	 else if(type=="Model")
    		 cursorCollection =c.query(dbCollectionPath, (QUERY("Type"<<nodeName<<"ObjectName"<<objectName<<"ModelName"<<name)));

      }catch(DBException &e)
      {
    	  CLOG(LERROR) <<"Something goes wrong... :>";
    	  CLOG(LERROR) <<c.getLastError();
      }
    return cursorCollection;

}

auto_ptr<DBClientCursor>  MongoDBWriter::findModelDocumentInCollection(string modelName)
{
      auto_ptr<DBClientCursor> cursorCollection;
      try{
	cursorCollection =c.query(dbCollectionPath, (QUERY("Type"<<"Model"<<"ObjectName"<<objectName<<"ModelName"<<modelName)));
      }
      catch(DBException &e)
      {
    	  CLOG(LERROR) <<"Something goes wrong... :>";
    	  CLOG(LERROR) <<c.getLastError();
      }
    return cursorCollection;
}
auto_ptr<DBClientCursor>  MongoDBWriter::findViewDocumentInCollection(string viewName)
{
      auto_ptr<DBClientCursor> cursorCollection;
      try{
	cursorCollection =c.query(dbCollectionPath, (QUERY("Type"<<"View"<<"ObjectName"<<objectName<<"ViewName"<<viewName)));
      }
      catch(DBException &e)
      {
    	  CLOG(LERROR) <<"Something goes wrong... :>";
    	  CLOG(LERROR) <<c.getLastError();
      }
    return cursorCollection;
}

void MongoDBWriter::initObject()
	{
		  CLOG(LTRACE) <<"Create template of object";
		  BSONObj o;
		  BSONElement bsonElement;
		  BSONArrayBuilder bsonBuilder;
		  OID oid;

	      try{
		      BSONObj object = BSONObjBuilder().genOID().append("Type", "Object").append("ObjectName", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, object);
		      vector<string> models = getAllFolders((string)folderName+"/Model/");
			  for(std::vector<string>::iterator it = models.begin(); it != models.end(); ++it){
				  if(*it=="." || *it=="..")
					  continue;
				  BSONObj model = BSONObjBuilder().genOID().append("Type", "Model").append("ObjectName", objectName).append("ModelName", *it).append("description", description).obj();
				  c.insert(dbCollectionPath, model);
				  model.getObjectID(bsonElement);
				  oid=bsonElement.__oid();
				  bsonBuilder.append(BSONObjBuilder().append("childOID", oid.str()).obj());
			      initModel(*it);
			  }
			  vector<string> views = getAllFolders((string)folderName+"/View/");
			  for(std::vector<string>::iterator it = views.begin(); it != views.end(); ++it){
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
	  		CLOG(LERROR) <<"Something goes wrong... :>";
	    	CLOG(LERROR) <<c.getLastError();
	      }
	}

void MongoDBWriter::initView(const string & viewName)
{
	BSONElement oi;
    OID o;
    BSONArrayBuilder viewArrayBuilder, stereoArrayBuilder, kinectArrayBuilder, tofArrayBuilder, viewBuilder;

    if(nodeTypeProp=="View")
    {
    	unsigned long long nr = c.count(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"));
		// add object
		if(nr==0)
		{
			CLOG(LTRACE) <<"Object does not exists in "<< dbCollectionPath;
			BSONObj object = BSONObjBuilder().genOID().append("Type", "Object").append("ObjectName", objectName).append("description", description).obj();
			c.insert(dbCollectionPath, object);
		}
		// add view
		BSONObj view = BSONObjBuilder().genOID().append("Type", "View").append("ObjectName", objectName).append("ViewName", viewName).append("description", description).obj();
		c.insert(dbCollectionPath, view);
		view.getObjectID(oi);
		o=oi.__oid();
		// add view to object
		c.update(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"), BSON("$push"<<BSON("childOIDs"<<o)), false, true);
	}

    for(std::vector<string>::iterator it = docViewsNames.begin(); it != docViewsNames.end(); ++it){
		BSONObj document = BSONObjBuilder().genOID().append("Type", *it).append("ObjectName", objectName).append("ViewName", viewName).append("description", description).obj();
		c.insert(dbCollectionPath, document);

		if(*it=="Stereo" || *it=="Kinect" || *it=="ToF")
		{
			document.getObjectID(oi);
			o=oi.__oid();
			viewArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
		}

		else if(*it=="StereoSiLR" || *it=="StereoSiRX" || *it=="StereoSiRXM")
		{
			document.getObjectID(oi);
			o=oi.__oid();
			stereoArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
		}
		else if(*it=="KinectSiLR" || *it=="KinectSiRX" || *it=="KinectSiRXM")
		{
			document.getObjectID(oi);
			o=oi.__oid();
			kinectArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
		}
		else if(*it=="ToFSiLR" || *it=="ToFSiRX" || *it=="ToFSiRXM")
		{
			document.getObjectID(oi);
			o=oi.__oid();
			tofArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
		}
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
	BSONElement oi;
	OID o;
	BSONArrayBuilder modelArrayBuilder, somArrayBuilder, ssomArrayBuilder;

	if(nodeTypeProp=="Model")
	{
		unsigned long long nr = c.count(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"));
		// add object
		if(nr==0)
		{
			CLOG(LTRACE) <<"Object does not exists in "<< dbCollectionPath;
			BSONObj object = BSONObjBuilder().genOID().append("Type", "Object").append("ObjectName", objectName).append("description", description).obj();
			c.insert(dbCollectionPath, object);
		}
		// add model
		BSONObj model = BSONObjBuilder().genOID().append("Type", "Model").append("ObjectName", objectName).append("ModelName", modelName).append("description", description).obj();
		c.insert(dbCollectionPath, model);
		model.getObjectID(oi);
		o=oi.__oid();
		// add model to object
		c.update(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"), BSON("$push"<<BSON("childOIDs"<<o)), false, true);
	}

	for(std::vector<string>::iterator it = docModelsNames.begin(); it != docModelsNames.end(); ++it){
		BSONObj document = BSONObjBuilder().genOID().append("Type", *it).append("ObjectName", objectName).append("ModelName", modelName).append("description", description).obj();
		c.insert(dbCollectionPath, document);

		if(*it=="SOM" || *it=="SSOM")
		{
			document.getObjectID(oi);
			o=oi.__oid();
			modelArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
		}
		else if(*it=="SomRgb" || *it=="SomSift")
		{
			document.getObjectID(oi);
			o=oi.__oid();
			somArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
		}
		else if(*it=="SsomRgb" || *it=="SsomSift" || *it=="SsomShot")
		{
			document.getObjectID(oi);
			o=oi.__oid();
			ssomArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
		}
	}
	BSONArray modelArr = modelArrayBuilder.arr();
	BSONArray somArr = somArrayBuilder.arr();
	BSONArray ssomArr = ssomArrayBuilder.arr();

	c.update(dbCollectionPath, QUERY("Type"<<"Model"<<"ObjectName"<<objectName<<"ModelName"<<modelName), BSON("$set"<<BSON("childOIDs"<<modelArr)), false, true);
	c.update(dbCollectionPath, QUERY("Type"<<"SOM"<<"ObjectName"<<objectName<<"ModelName"<<modelName), BSON("$set"<<BSON("childOIDs"<<somArr)), false, true);
	c.update(dbCollectionPath, QUERY("Type"<<"SSOM"<<"ObjectName"<<objectName<<"ModelName"<<modelName), BSON("$set"<<BSON("childOIDs"<<ssomArr)), false, true);
}

void MongoDBWriter::writeNode2MongoDB(const string &source, const string &destination, const string &option,string modelOrViewName)
{
	BSONObj o;
	BSONElement bsonElement;
	BSONArrayBuilder bsonBuilder;
	OID oid;
	string mime;
	CLOG(LTRACE) <<"Source: " << source << " destination: "<< destination<<" dbCollectionPath: "<<dbCollectionPath;
    try{
    	for(std::vector<string>::iterator itExtension = fileExtensions.begin(); itExtension != fileExtensions.end(); ++itExtension) {
    		CLOG(LTRACE) <<"source+*itExtension "<<source+*itExtension;
			vector<string> files = getAllFiles(source+*itExtension);
			for(std::vector<string>::iterator it = files.begin(); it != files.end(); ++it) {
				CLOG(LTRACE) <<"fileName"<<*it;
				string fileName = *it;
				string newFileName;
				const size_t last_slash_idx = fileName.find_last_of("/");
				if (std::string::npos != last_slash_idx)
				{
					newFileName = fileName.erase(0, last_slash_idx + 1);
				}
				if (*itExtension=="*.png")
					mime="image/png";
				else if(*itExtension=="*.jpg")
					mime= "image/jpeg";
				else if(*itExtension=="*.txt" || *itExtension=="*.pcd")
					mime="text/plain";
				else
				{
					CLOG(LERROR) <<"I don't know such file extension! Please add extension to the code from http://www.sitepoint.com/web-foundations/mime-types-complete-list/";
					return;
				}
				GridFS fs(c, collectionName);
				o = fs.storeFile(*it, newFileName, mime);
				BSONObj b = BSONObjBuilder().appendElements(o).append("ObjectName", objectName).obj();
				c.insert(dbCollectionPath, b);
				b.getObjectID(bsonElement);
				oid=bsonElement.__oid();
				bsonBuilder.append(BSONObjBuilder().append("childOID", oid.str()).obj());
			}
    	}
		BSONArray destArr = bsonBuilder.arr();
		if(option=="View")
			c.update(dbCollectionPath, QUERY("Type"<<destination<<"ObjectName"<<objectName<<"ViewName"<<modelOrViewName), BSON("$set"<<BSON("childOIDs"<<destArr)), false, true);
		if(option=="Model")
			c.update(dbCollectionPath, QUERY("Type"<<destination<<"ObjectName"<<objectName<<"ModelName"<<modelOrViewName), BSON("$set"<<BSON("childOIDs"<<destArr)), false, true);

		CLOG(LTRACE) <<"Files saved successfully";
    }
	catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :>";
		CLOG(LERROR) <<c.getLastError();
	}
}

void MongoDBWriter::insert2MongoDB(const string &destination, const string&  modelOrViewName, const string&  type)
{
	CLOG(LTRACE)<< "destination: "<<destination;
	auto_ptr<DBClientCursor> cursorCollection;
	string source;
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
		if(destination=="StereoSiLR" || destination=="KinectSiLR" || destination=="ToFSiLR" || destination=="StereoSiRX" || destination=="KinectSiRX" ||  destination=="ToFSiRX" || destination=="StereoSiRXM" || destination=="KinectSiRXM" || destination=="ToFSiRXM")
		{
				source = (string)folderName+"View/"+modelOrViewName+"/"+destination+"/";
				CLOG(LINFO)<< "Source!!! "<<source;
				writeNode2MongoDB(source, destination, "View", modelOrViewName);
		}
		else if(destination=="SomRgb" || destination=="SomSift" ||  destination=="SsomRgb" || destination=="SsomSift" || destination=="SsomShot")
		{
				source = (string)folderName+"Model/"+modelOrViewName+"/"+destination+"/";
				CLOG(LINFO)<< "Source!!! "<<source;
				writeNode2MongoDB(source, destination, "Model", modelOrViewName);
		}
		else
		{
			if(destination=="Model")
			{
				CLOG(LINFO)<< "Model!";
				if(nodeTypeProp=="Model")
				{
					unsigned long long nr = c.count(dbCollectionPath, (QUERY("Type"<<"Model"<<"ObjectName"<<objectName<<"ModelName"<<modelOrViewName)));
					if(nr>0)
					{
						CLOG(LERROR)<<"Model "<< modelOrViewName<<" exists in db for object "<<objectName;
						return;
					}
					else
						initModel(modelOrViewName);
				}
				cursorCollection =findModelDocumentInCollection(modelOrViewName);
			}
			else if(destination=="View")
			{
				CLOG(LINFO)<< "View!";
				if(nodeTypeProp=="View")
				{
					CLOG(LINFO)<<"Count Views\tViewName=" <<modelOrViewName;
					unsigned long long nr = c.count(dbCollectionPath, (QUERY("Type"<<"View"<<"ObjectName"<<objectName<<"ViewName"<<modelOrViewName)));
					if(nr>0)
					{
						CLOG(LERROR)<<"View "<< modelOrViewName<<" exists in db for object "<<objectName;
						return;
					}
					else
						initView(modelOrViewName);
				}
				cursorCollection =findViewDocumentInCollection(modelOrViewName);
			}
			else
				cursorCollection =findDocumentInCollection(destination, type, modelOrViewName);

			while (cursorCollection->more())
			{
					BSONObj obj = cursorCollection->next();
					CLOG(LTRACE)<< obj;
					vector<OID> childsVector =  getChildOIDS(obj);
					for (unsigned int i = 0; i<childsVector.size(); i++)
					{
						auto_ptr<DBClientCursor> childCursor =c.query(dbCollectionPath, (QUERY("_id"<<childsVector[i])));
						if( childCursor->more())
						{
							BSONObj childObj = childCursor->next();
							string childNodeName = childObj.getField("Type").str();
							CLOG(LINFO)<< "childNodeName: "<<childNodeName;
							// if node consists child read it
							if(childNodeName!="EOO")
							{
								if(childNodeName=="View")
								{
									string modelOrViewName = childObj.getField("ViewName").str();
									CLOG(LTRACE)<< "viewName: "<<modelOrViewName;
									string type = "View";
									insert2MongoDB(childNodeName, modelOrViewName, type);
								}
								else if(childNodeName=="Model")
								{
									string modelOrViewName = childObj.getField("ModelName").str();
									string type = "Model";
									CLOG(LTRACE)<< "modelName: "<<modelOrViewName;
									insert2MongoDB(childNodeName, modelOrViewName, type);
								}
								else
									insert2MongoDB(childNodeName, modelOrViewName, type);
							}
						}
					}
				}//while
			}//else
    }//try
	catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :>";
		CLOG(LERROR) <<c.getLastError();
	}
}

} //: namespace MongoDBWriter
} //: namespace Processors

 
