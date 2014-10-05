/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */


#include "ViewWriter.hpp"
namespace Processors {
namespace ViewWriter  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace boost::posix_time;

ViewWriter::ViewWriter(const string & name) : Base::Component(name),
	mongoDBHost("mongoDBHost", string("localhost")),
	objectName("objectName", string("GreenCup")),
	description("description", string("My green coffe cup")),
	collectionName("collectionName", string("containers")),
	//extension("extension", string("pcd")),
	extension("extension", string("png")),
	viewNameProp("viewName", string("lab012")),
	fileName("fileName", string("tempFile")),
	nodeTypeProp("nodeTypeProp", string("StereoLR")),
	remoteFileName("remoteFileName", string("sweetFoto")),
	mean_viewpoint_features_number("mean_viewpoint_features_number", int(12)),
	sceneNamesProp("sceneNamesProp", string("scene1,scene2,scene3")),
	binary("binary", false),
	suffix("suffix", false)
{
	registerProperty(mongoDBHost);
	registerProperty(mean_viewpoint_features_number);
	registerProperty(objectName);
	registerProperty(description);
	registerProperty(collectionName);
	registerProperty(extension);
	registerProperty(viewNameProp);
	registerProperty(sceneNamesProp);
	registerProperty(fileName);
	registerProperty(nodeTypeProp);
	registerProperty(remoteFileName);
	registerProperty(binary);
	registerProperty(suffix);
	CLOG(LTRACE) << "Hello ViewWriter";

}

ViewWriter::~ViewWriter()
{
	CLOG(LTRACE) << "Good bye ViewWriter";
}
void ViewWriter::write2DB()
{
	CLOG(LNOTICE) << "ViewWriter::write2DB";
	string sceneNames = sceneNamesProp;
	boost::split(splitedSceneNames, sceneNames, is_any_of(","));

	if(viewNameProp!="")
		insert2MongoDB(nodeTypeProp,viewNameProp, "View");
	else
		CLOG(LERROR)<<"Add view name and try again";

}

void ViewWriter::prepareInterface() {
	CLOG(LTRACE) << "ViewWriter::prepareInterface";
	h_write2DB.setup(this, &ViewWriter::write2DB);
	registerHandler("writeView2DB", &h_write2DB);
	registerHandler("write2DB", &h_write2DB);
	registerHandler("Write_xyz", boost::bind(&ViewWriter::Write_xyz, this));
	registerHandler("Write_xyzrgb", boost::bind(&ViewWriter::Write_xyzrgb, this));
	registerHandler("Write_xyzsift", boost::bind(&ViewWriter::Write_xyzsift, this));

	registerStream("in_img", &in_img);
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);

	//addDependency("writeView2DB", &in_img);
	addDependency("write2DB", &in_img);
	addDependency("Write_xyzrgb", &in_cloud_xyzrgb);
	addDependency("Write_xyz", &in_cloud_xyz);
	addDependency("Write_xyzsift", &in_cloud_xyzsift);
}

void ViewWriter::Write_xyzsift()
{
	CLOG(LTRACE) << "ModelWriter::Write_xyzsift";

	cloudType="xyzsift";
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();
	std::string fn = fileName;
	if(suffix){
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyzsift.pcd");
	}
	pcl::io::savePCDFile (fn, *cloud, binary);
	write2DB();

	//CLOG(LINFO) << "Saved " << cloud->points.size() << " XYZ points to "<< fileName << "\n";
}

void ViewWriter::Write_xyz()
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

void ViewWriter::Write_xyzrgb()
{
	cloudType="xyzrgb";
	CLOG(LNOTICE) << "ModelWriter::Write_xyzrgb";
	CLOG(LNOTICE)<<"Set cloudType: "<<cloudType;
	CLOG(LNOTICE)<<"suffix: "<<suffix;
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

bool ViewWriter::onInit()
{
      CLOG(LTRACE) << "ViewWriter::initialize";
      try
      {
    	  cloudType="";
    	  string hostname = mongoDBHost;
    	  connectToMongoDB(hostname);
		  if(collectionName=="containers")
			dbCollectionPath="images.containers";
		 initViewNames();
      }
	 catch(DBException &e)
	 {
		 CLOG(LERROR) <<"Something goes wrong... :<";
		 CLOG(LERROR) <<c->getLastError();
	 }

	 return true;
}

bool ViewWriter::onFinish()
{
        CLOG(LTRACE) << "ViewWriter::finish";

        return true;
}

bool ViewWriter::onStep()
{
        CLOG(LTRACE) << "ViewWriter::step";
        return true;
}

bool ViewWriter::onStop()
{
        return true;
}

bool ViewWriter::onStart()
{
        return true;
}

void ViewWriter::addScenes(BSONObj& object)
{
	int items = 0;
	bool objectInTheScene = false;
	OID o;
	BSONElement bsonElement;
	BSONElement oi;
	BSONArrayBuilder bsonBuilder;

	for(std::vector<string>::iterator itSceneName = splitedSceneNames.begin(); itSceneName != splitedSceneNames.end(); ++itSceneName)
	{
		CLOG(LINFO)<<"Scene: "<<*itSceneName;
		// if scene exist
		items = c->count(dbCollectionPath, (QUERY("SceneName"<<*itSceneName)));
		if(items>0)
		{
			auto_ptr<DBClientCursor> cursorCollection =c->query(dbCollectionPath, (QUERY("SceneName"<<*itSceneName)));
			BSONObj scene = cursorCollection->next();
			CLOG(LINFO)<<"Add scene to the object!";
			scene.getObjectID(oi);
			o=oi.__oid();

			c->update(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"), BSON("$addToSet"<<BSON("sceneOIDs"<<BSON("sceneOID"<<o.str()))), false, true);
			CLOG(LTRACE)<<scene;

			vector<OID> childsVector;
			if(getChildOIDS(scene, "objectsOIDs", "objectOID", childsVector)>0)
			{
				for (unsigned int i = 0; i<childsVector.size(); i++)
				{

					auto_ptr<DBClientCursor> childCursor =c->query(dbCollectionPath, (QUERY("_id"<<childsVector[i])));

					if( childCursor->more())
					{

						BSONObj childObj = childCursor->next();
						string _id = childObj.getField("_id").str();
						if(_id==o.str())
						{
							objectInTheScene = true;
							CLOG(LERROR)<< "Object exists in the scene!";
							break;
						}
					}
				}
			}
			if(!objectInTheScene)
			{
				CLOG(LINFO)<<"Adding object to the scene";
				object.getObjectID(oi);
				o=oi.__oid();
				c->update(dbCollectionPath, QUERY("SceneName"<<*itSceneName), BSON("$addToSet"<<BSON("objectsOIDs"<<BSON("objectOID"<<o.str()))), false, true);
			}
		}//if
		else
		{
			CLOG(LINFO)<<"Create scene and add object to array list";
			BSONObj scene = BSONObjBuilder().genOID().append("SceneName", *itSceneName).obj();
			c->insert(dbCollectionPath, scene);

			CLOG(LINFO)<<"Adding object to the scene";
			object.getObjectID(oi);
			o=oi.__oid();
			c->update(dbCollectionPath, QUERY("SceneName"<<*itSceneName), BSON("$addToSet"<<BSON("objectsOIDs"<<BSON("objectOID"<<o.str()))), false, true);

			CLOG(LINFO)<<"Add scene to object!";
			scene.getObjectID(oi);
			o=oi.__oid();
			c->update(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"), BSON("$addToSet"<<BSON("sceneOIDs"<<BSON("sceneOID"<<o.str()))), false, true);
		}
	}
}

void ViewWriter::createModelOrView(const std::vector<string>::iterator it, const string& type, BSONArrayBuilder& bsonBuilder)
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
		initModel(*it, true);
	else if(type=="View")
		initView(*it, true);
}

////////////////////////////INIT////////////////////////////////////

void ViewWriter::initObject()
{
	CLOG(LTRACE) <<"Create template of object";
	BSONArrayBuilder bsonBuilder;
	bool objectInTheScene = false;
	try
	{
		BSONObj object = BSONObjBuilder().genOID().append("Type", "Object").append("ObjectName", objectName).append("description", description).obj();
		c->insert(dbCollectionPath, object);
		addScenes(object);

	}
	  catch(DBException &e)
	  {
		CLOG(LERROR) <<"Something goes wrong... :<";
		CLOG(LERROR) <<c->getLastError();
	  }
}

void ViewWriter::addToObject(const Base::Property<string>& nodeTypeProp,const string & name)
{
	BSONElement oi;
	OID o;
	string type;
	string nodeType;
	nodeType = nodeTypeProp;
	if(nodeType=="View"||nodeType=="Model")
		type=nodeType;
	else if(isModelLastLeaf(nodeTypeProp))
		type="Model";
	else if(isViewLastLeaf(nodeTypeProp))
		type="View";
	CLOG(LTRACE)<<"Type: " <<type;

	unsigned long long nr = c->count(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"));
	// add object
	if(nr==0)
	{
		CLOG(LTRACE) <<"Object does not exists in "<< dbCollectionPath;
		BSONObj object = BSONObjBuilder().genOID().append("Type", "Object").append("ObjectName", objectName).append("description", description).obj();
		c->insert(dbCollectionPath, object);
		addScenes(object);
	}
	// add model/view
	BSONObj modelorView = BSONObjBuilder().genOID().append("Type", type).append("ObjectName", objectName).append(type+"Name", name).append("description", description).obj();
	c->insert(dbCollectionPath, modelorView);
	modelorView.getObjectID(oi);
	o=oi.__oid();
	c->update(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<"Object"), BSON("$addToSet"<<BSON("childOIDs"<<BSON("childOID"<<o.str()))), false, true);
}

void ViewWriter::initView(const string & viewName, bool addToObjectFlag)
{
	BSONElement oi;
    OID o;
    BSONArrayBuilder stereoPCArrayBuilder, kinectPCArrayBuilder, tofPCArrayBuilder, objectArrayBuilder, viewArrayBuilder, stereoArrayBuilder, kinectArrayBuilder, tofArrayBuilder, viewBuilder;
    CLOG(LTRACE)<<"Init View";
    //add view to object
    if(addToObjectFlag)
    {
    	addToObject(nodeTypeProp, viewName);
    }
    // add childs to arraysBuilder
    for(std::vector<string>::iterator it = docViewsNames.begin(); it != docViewsNames.end(); ++it){
		BSONObj document = BSONObjBuilder().genOID().append("Type", *it).append("ObjectName", objectName).append("ViewName", viewName).append("description", description).obj();
		c->insert(dbCollectionPath, document);
		document.getObjectID(oi);
		o=oi.__oid();
		if(*it=="Stereo" || *it=="Kinect" || *it=="ToF")
			viewArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
		else if(*it=="StereoLR" || *it=="StereoRX" || *it=="StereoRXM" || *it=="StereoPC")
			stereoArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
		else if(*it=="KinectRGBD" || *it=="KinectRX" || *it=="KinectRXM"  || *it=="KinectPC")
			kinectArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
		else if(*it=="ToFRGBD" || *it=="ToFRX" || *it=="ToFRXM" || *it=="ToFPC")
			tofArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
		else if(*it=="KinectPCXYZRGB" || *it=="KinectPCXYZSIFT" || *it== "KinectPCXYZSHOT"  ) //
			kinectPCArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
		else if(*it=="StereoPCXYZRGB" || *it=="StereoPCXYZSIFT" || *it=="StereoPCXYZSHOT") //"StereoPC"
			stereoPCArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
		else if(*it== "ToFPCXYZRGB" || *it=="ToFPCXYZSIFT" || *it=="ToFPCXYZSHOT") //ToFPC
			tofPCArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
    }

    // create arrays
	BSONArray viewArr = viewArrayBuilder.arr();
    BSONArray stereoArr = stereoArrayBuilder.arr();
    BSONArray kinectArr = kinectArrayBuilder.arr();
    BSONArray tofArr = tofArrayBuilder.arr();
    BSONArray kinectPCArr = kinectPCArrayBuilder.arr();
    BSONArray stereoPC = stereoPCArrayBuilder.arr();
    BSONArray tofPCArr = tofPCArrayBuilder.arr();

    // update documents
    c->update(dbCollectionPath, QUERY("Type"<<"View"<<"ObjectName"<<objectName<<"ViewName"<<viewName), BSON("$set"<<BSON("childOIDs"<<viewArr)), false, true);
    c->update(dbCollectionPath, QUERY("Type"<<"ToF"<<"ObjectName"<<objectName<<"ViewName"<<viewName), BSON("$set"<<BSON("childOIDs"<<tofArr)), false, true);
    c->update(dbCollectionPath, QUERY("Type"<<"Kinect"<<"ObjectName"<<objectName<<"ViewName"<<viewName), BSON("$set"<<BSON("childOIDs"<<kinectArr)), false, true);
    c->update(dbCollectionPath, QUERY("Type"<<"Stereo"<<"ObjectName"<<objectName<<"ViewName"<<viewName), BSON("$set"<<BSON("childOIDs"<<stereoArr)), false, true);
    c->update(dbCollectionPath, QUERY("Type"<<"KinectPC"<<"ObjectName"<<objectName<<"ViewName"<<viewName), BSON("$set"<<BSON("childOIDs"<<kinectPCArr)), false, true);
    c->update(dbCollectionPath, QUERY("Type"<<"StereoPC"<<"ObjectName"<<objectName<<"ViewName"<<viewName), BSON("$set"<<BSON("childOIDs"<<stereoPC)), false, true);
    c->update(dbCollectionPath, QUERY("Type"<<"ToFPCX"<<"ObjectName"<<objectName<<"ViewName"<<viewName), BSON("$set"<<BSON("childOIDs"<<tofPCArr)), false, true);

}

void ViewWriter::initModel(const string & modelName, bool addToModelFlag)
{
	CLOG(LTRACE)<<"initModel";
	BSONElement oi;
	OID o;
	BSONArrayBuilder objectArrayBuilder, modelArrayBuilder, somArrayBuilder, ssomArrayBuilder;

	if(addToModelFlag)
	{
		addToObject(nodeTypeProp, modelName);
	}

	for(std::vector<string>::iterator it = docModelsNames.begin(); it != docModelsNames.end(); ++it){
		BSONObj document = BSONObjBuilder().genOID().append("Type", *it).append("ObjectName", objectName).append("ModelName", modelName).append("description", description).obj();
		c->insert(dbCollectionPath, document);

		document.getObjectID(oi);
		o=oi.__oid();

		if(*it=="SOM" || *it=="SSOM")
			modelArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
		else if(*it=="SomXYZRgb" || *it=="SomXYZSift")
			somArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
		else if(*it=="SsomXYZRgb" || *it=="SsomXYZSift" || *it=="SsomXYZShot")
			ssomArrayBuilder.append(BSONObjBuilder().append("childOID", o.str()).obj());
	}

	BSONArray modelArr = modelArrayBuilder.arr();
	BSONArray somArr = somArrayBuilder.arr();
	BSONArray ssomArr = ssomArrayBuilder.arr();

	c->update(dbCollectionPath, QUERY("Type"<<"Model"<<"ObjectName"<<objectName<<"ModelName"<<modelName), BSON("$set"<<BSON("childOIDs"<<modelArr)), false, true);
	c->update(dbCollectionPath, QUERY("Type"<<"SOM"<<"ObjectName"<<objectName<<"ModelName"<<modelName), BSON("$set"<<BSON("childOIDs"<<somArr)), false, true);
	c->update(dbCollectionPath, QUERY("Type"<<"SSOM"<<"ObjectName"<<objectName<<"ModelName"<<modelName), BSON("$set"<<BSON("childOIDs"<<ssomArr)), false, true);
}
////////////////////////////INIT_END////////////////////////////////////

void ViewWriter::insertFileToGrid(OID& oid)
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
		///TODO
		/// add txt extension
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
		c->insert(dbCollectionPath, b);
		b.getObjectID(bsonElement);
		oid=bsonElement.__oid();
		cloudType="";
	}catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :<";
		CLOG(LERROR) <<c->getLastError();
	}
}

void ViewWriter::writeNode2MongoDB(const string &destination, const string &type,string modelOrViewName)
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

void ViewWriter::insert2MongoDB(const string &destination, const string&  modelOrViewName, const string&  type)
{
	CLOG(LINFO)<<"ViewWriter::insert2MongoDB";
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
			CLOG(LTRACE)<<"if(base->isViewLastLeaf(destination)  Check if object exists";
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
				CLOG(LTRACE)<<"Items: "<<items;
				if(items==0)
				{
					CLOG(LTRACE)<<"No such model/view";
					CLOG(LTRACE)<<"Type: "<<type;
					initView(modelOrViewName, true);
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
			CLOG(LINFO)<<"Write to view";
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
							initModel(modelOrViewName, true);
						else if(destination=="View")
							initView(modelOrViewName, true);
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

} //: namespace ViewWriter
} //: namespace Processors
