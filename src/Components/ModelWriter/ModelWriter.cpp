/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */


#include "ModelWriter.hpp"
namespace Processors {
namespace ModelWriter  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace boost::posix_time;

ModelWriter::ModelWriter(const string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", string("localhost")),
		objectName("objectName", string("GreenCup")),
		description("description", string("My green coffe cup")),
		collectionName("collectionName", string("containers")),
		extension("extension", string("pcd")),
		//extension("extension", string("png")),
		modelNameProp("modelName", string("lab012")),
		fileName("fileName", string("tempFile")),
		nodeTypeProp("nodeTypeProp", string("SomXYZRgb")),
		remoteFileName("remoteFileName", string("sweetCloud")),
		sceneNamesProp("sceneNamesProp", string("scene1,scene2,scene3")),
		mean_viewpoint_features_number("mean_viewpoint_features_number", int(12)),
		binary("binary", false),
		suffix("suffix", false)
{
	registerProperty(mongoDBHost);
	registerProperty(objectName);
	registerProperty(mean_viewpoint_features_number);
	registerProperty(description);
	registerProperty(collectionName);
	registerProperty(extension);
	registerProperty(modelNameProp);
	registerProperty(sceneNamesProp);
	registerProperty(fileName);
	registerProperty(nodeTypeProp);
	registerProperty(remoteFileName);
	registerProperty(binary);
	registerProperty(suffix);

	//base = new MongoBase::MongoBase();
    CLOG(LTRACE) << "Hello ModelWriter";
}

ModelWriter::~ModelWriter()
{
       CLOG(LTRACE) << "Good bye ModelWriter";
}
void ModelWriter::write2DB()
{
	CLOG(LERROR)<<"New image!";

   CLOG(LNOTICE) << "ModelWriter::write2DB";
   CLOG(LNOTICE)<<"File on input";
   string sceneNames = sceneNamesProp;
   boost::split(MongoBase::splitedSceneNames, sceneNames, is_any_of(","));

   if(modelNameProp!="")
	   insert2MongoDB(nodeTypeProp,modelNameProp, "Model");
   else
	   CLOG(LERROR)<<"Add model name and try again";

}

void ModelWriter::prepareInterface() {
	CLOG(LTRACE) << "ModelWriter::prepareInterface";
	h_write2DB.setup(this, &ModelWriter::write2DB);
	registerHandler("write2DB", &h_write2DB);
	registerHandler("Write_xyz", boost::bind(&ModelWriter::Write_xyz, this));
	registerHandler("Write_xyzrgb", boost::bind(&ModelWriter::Write_xyzrgb, this));
	registerHandler("Write_xyzsift", boost::bind(&ModelWriter::Write_xyzsift, this));

	registerStream("in_img", &in_img);
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);

	//addDependency("write2DB", &in_img);
	addDependency("Write_xyzrgb", &in_cloud_xyzrgb);
	addDependency("Write_xyz", &in_cloud_xyz);
	addDependency("Write_xyzsift", &in_cloud_xyzsift);

}
void ModelWriter::Write_xyzsift()
{
	CLOG(LTRACE) << "ModelWriter::Write_xyzsift";
	try{
		cloudType="xyzsift";
		CLOG(LTRACE)<<"Set cloudType: "<<cloudType;
		pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();

		std::string fn = fileName;
		if(suffix){
			CLOG(LTRACE)<<"suffix: "<<suffix;
			size_t f = fn.find(".pcd");
			if(f != std::string::npos)
			{
				fn.erase(f);
			}
			fn = std::string(fn) + std::string("_xyzsift.pcd");
		}
		CLOG(LTRACE)<<"Test";
		CLOG(LINFO) <<"FileName:"<<fn;
		CLOG(LINFO) << "Saving " << cloud->points.size() << " XYZ points to "<< fileName << "\n";
		pcl::io::savePCDFile (fn, *cloud, binary);

		write2DB();
	}catch(Exception & ex){ex.what();}
}

void ModelWriter::Write_xyz()
{
	cloudType="xyz";
	CLOG(LTRACE)<<"Set cloudType: "<<cloudType;
	CLOG(LTRACE) << "ModelWriter::Write_xyz";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	std::string fn = fileName;
	if(suffix){
		CLOG(LTRACE)<<"suffix: "<<suffix;
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyz.pcd");
	}
	pcl::io::savePCDFile (fn, *cloud, binary);
	CLOG(LINFO) <<"FileName:"<<fn;
	//CLOG(LINFO) << "Saved " << cloud->points.size() << " XYZ points to "<< fileName << "\n";
	write2DB();
}

void ModelWriter::Write_xyzrgb()
{
	cloudType="xyzrgb";
	CLOG(LINFO) << "ModelWriter::Write_xyzrgb";
	CLOG(LINFO)<<"Set cloudType: "<<cloudType;
	CLOG(LINFO)<<"suffix: "<<suffix;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	    std::string fn = fileName;
	    if(suffix){
	        size_t f = fn.find(".pcd");
	        if(f != std::string::npos)
	        {
	            fn.erase(f);
	        }
	        fn = std::string(fn) + std::string("_xyzrgb.pcd");
	    }
		CLOG(LINFO) <<"FileName:"<<fn;
	    pcl::io::savePCDFile (fn, *cloud, binary);
		//CLOG(LINFO) << "Saved " << cloud->points.size() << " XYZRGB points to "<< fileName << "\n";
	    write2DB();
}

bool ModelWriter::onInit()
{
      CLOG(LTRACE) << "ModelWriter::initialize";
      try
      {
    	  cloudType="";
    	  string hostname = mongoDBHost;
    	  connectToMongoDB(hostname);
		  initModelNames();
      }
	 catch(DBException &e)
	 {
		 CLOG(LERROR) <<"Something goes wrong... :<";
		 CLOG(LERROR) <<c->getLastError();
	 }
	 return true;
}

bool ModelWriter::onFinish()
{
        CLOG(LTRACE) << "ModelWriter::finish";

        return true;
}

bool ModelWriter::onStep()
{
        CLOG(LTRACE) << "ModelWriter::step";
        return true;
}

bool ModelWriter::onStop()
{
        return true;
}

bool ModelWriter::onStart()
{
        return true;
}

void ModelWriter::createModelOrView(const std::vector<string>::iterator it, const string& type, BSONArrayBuilder& bsonBuilder)
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
		initModel(*it, true, nodeTypeProp, objectName, description);
}

void ModelWriter::initObject()
{
	CLOG(LTRACE) <<"Create template of object";
	BSONArrayBuilder bsonBuilder;
	bool objectInTheScene = false;
	try
	{
		BSONObj object = BSONObjBuilder().genOID().append("Type", "Object").append("ObjectName", objectName).append("description", description).obj();
		c->insert(dbCollectionPath, object);

		addScenes(object, objectName);
	}
	catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :<";
	}
}

void ModelWriter::insertFileToGrid(OID& oid)
{
	try{
		BSONObj object;
		BSONElement bsonElement;
		string mime="";
		setMime(extension, mime);
		std::stringstream time;
		string tempFileName;
		if (extension=="png" || extension=="jpg")
		{
			CLOG(LINFO)<<"Image!";
			cv::Mat tempImg = in_img.read();
			tempFileName = string(fileName)+"."+string(extension);
			cv::imwrite(tempFileName, tempImg);
		}
		else if(extension=="pcd")	// save to file pcd
		{
			CLOG(LINFO)<<"Cloud!";
			if(cloudType=="xyzrgb")
				tempFileName=std::string(fileName) + std::string("_xyzrgb.pcd");
			else if(cloudType=="xyz")
				tempFileName=std::string(fileName) + std::string("_xyz.pcd");
			else if(cloudType=="xyzsift")
				tempFileName=std::string(fileName) + std::string("_xyzsift.pcd");

			CLOG(LINFO) << "cloudType: "<< cloudType << endl;
		}
		else
		{
			CLOG(LERROR)<<"I dont know such extension file :(";
			exit(1);
		}
		CLOG(LINFO) << "tempFileName: "<< tempFileName << endl;

		boost::posix_time::time_facet *facet = new boost::posix_time::time_facet("%d_%m_%Y_%H_%M_%S");
		time.imbue(locale(cout.getloc(), facet));
		time<<second_clock::local_time();
		CLOG(LINFO) << "Time: "<< time.str() << endl;

		GridFS fs(*c, collectionName);
		string fileNameInMongo;
		if(cloudType!="")
			fileNameInMongo = (string)remoteFileName+"_"+ cloudType + time.str()+"."+string(extension);
		else
			fileNameInMongo = (string)remoteFileName + time.str()+"."+string(extension);

		object = fs.storeFile(tempFileName, fileNameInMongo, mime);
		BSONObj b;
		if(cloudType=="xyzsift")
			b = BSONObjBuilder().appendElements(object).append("ObjectName", objectName).append("mean_viewpoint_features_number", mean_viewpoint_features_number).obj();
		else
			b = BSONObjBuilder().appendElements(object).append("ObjectName", objectName).obj();
		cloudType="";

		//dodac indexy przy inicjalizacji obiektu, widoku/modelu, wczytywaniu pliku
		// indeks na objectName, OIDScenes, OIDChilds, viewName, modelName
		//c->createIndex(dbCollectionPath, BSON("objectName"<<1));

		c->insert(dbCollectionPath, b);
		b.getObjectID(bsonElement);
		oid=bsonElement.__oid();
	}catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :<";
		CLOG(LERROR) <<c->getLastError();
	}
}

void ModelWriter::writeNode2MongoDB(const string &destination, const string &type,string modelOrViewName)
{
	CLOG(LTRACE) <<"writeNode2MongoDB";
	OID oid;
	CLOG(LTRACE) <<"Filename: " << fileName << " destination: "<< destination<<" dbCollectionPath: "<<dbCollectionPath;
    try{
		insertFileToGrid(oid);
		c->update(dbCollectionPath, QUERY("ObjectName"<<objectName<<type+"Name"<<modelOrViewName<<"Type"<<destination), BSON("$addToSet"<<BSON("childOIDs"<<BSON("childOID"<<oid.str()))), false, true);
		CLOG(LTRACE) <<"Files saved successfully";
    }
	catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :<";
		CLOG(LERROR) <<c->getLastError();
	}
}

void ModelWriter::insert2MongoDB(const string &destination, const string&  modelOrViewName, const string&  type)
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
				insert2MongoDB(destination, modelOrViewName, type);
				return;
			}
			else
			{
				CLOG(LTRACE)<<"Object now exist";
				int items = c->count(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<type<<type+"Name"<<modelOrViewName));
				if(items==0)
				{
					CLOG(LTRACE)<<"No such model/view";
					CLOG(LTRACE)<<"Type: "<<type;
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
			CLOG(LINFO)<<"Write to model";
			writeNode2MongoDB(destination, type, modelOrViewName);
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
							initView(modelOrViewName, true,  nodeTypeProp, objectName, description);
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
	}
}

} //: namespace ModelWriter
} //: namespace Processors
