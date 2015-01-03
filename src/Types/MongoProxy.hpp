/*
 * MongoProxy.hpp
 *
 *  Created on: Sep 23, 2014
 *      Author: lzmuda
 */

#ifndef MONGOPROXY_HPP_
#define MONGOPROXY_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/variant.hpp>

#include "Logger.hpp"

#include <cstdlib>
#include <iostream>
#include <glob.h>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "Logger.hpp"
#include "mongo/client/dbclient.h"
#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include <dirent.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZRGBSIFT.hpp>
#include <Types/SIFTObjectModelFactory.hpp>

#include <vector>
#include <list>
#include <iostream>
#include <boost/variant.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <iostream>

#define siftPointSize 133
#define xyzPointSize 3
#define xyzrgbPointSize 4

namespace MongoProxy {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;

boost::mutex io_mutex;

class MongoProxy {
private:
	string dbCollectionPath;
	string hostname;
    boost::shared_ptr<DBClientConnection> c;
    MongoProxy(string& hostname) : hostname(hostname)
    {
    	LOG(LNOTICE)<<"Connect :)";
    	LOG(LNOTICE)<<"hostname: "<<hostname;
    	c = boost::shared_ptr<DBClientConnection>(new DBClientConnection());
    	mongo::client::initialize();
    	connectToMongoDB();
    	sizeOfCloud=0;
    	tempFileOnDisc="tempFile";
    	fileOID = OID("000000000000000000000000");
    	dbCollectionPath="images.containers";
    	collectionName = "containers";
    }
    MongoProxy(const MongoProxy & );
public:
   static MongoProxy & getSingleton(string& hostname)
   {
	   boost::mutex::scoped_lock lock(io_mutex);
	   static MongoProxy singleton(hostname);
	   return singleton;
   }
    string collectionName;
	vector<string>  docViewsNames;
	vector<string>  docModelsNames;
	vector<string>	splitedSceneNames;
    float sizeOfCloud;
	string cloudType;
	cv::Mat tempImg;
	cv::Mat xyzimage;
	std::string tempFileOnDisc;
	OID fileOID;
	string tempFileName;


	vector<string> getAllFiles(const string& pattern);
	vector<string> getAllFolders(const string& pattern);
	int getChildOIDS(BSONObj &obj, const string&,  const string&, vector<OID>&);
	bool isModelLastLeaf(const string&);
	bool isViewLastLeaf(const string&);
	void findDocumentInCollection(DBClientConnection&, string&, Base::Property<string> &, const string &, auto_ptr<DBClientCursor> &, const string &, const string & , int&);
	void initViewNames();
	void initModelNames();

	void connectToMongoDB();
	void setModelOrViewName(const string& childNodeName, const BSONObj& childObj, string& newName);
    void getFileFromGrid(const GridFile &);
    void addScenes(BSONObj& object, Base::Property<string>& objectName);
    void initModel(const string & modelName, bool addToModelFlag, Base::Property<string>& nodeNameProp, Base::Property<string>& objectName, Base::Property<string>& description);
    void addToObject(const Base::Property<string>& nodeNameProp,const string & name, Base::Property<string>& objectName, Base::Property<string>& description);
    void initView(const string & viewName, bool addToObjectFlag, Base::Property<string>& nodeNameProp, Base::Property<string>& objectName, Base::Property<string>& description);
    void cloudEncoding(OID& oid, string& tempFileName, string & cloudType);
    void ReadPCDCloud(const string& filename);
    void writeToSinkFromFile(string& mime, string& fileName);
    void readTextFileFromDocument(mongo::OID& childOID, int size);
    void readXYZMatFromDocument(mongo::OID& childOID, int size);
    void readPointCloudFromDocument(mongo::OID& childOID, int size);
    void readImageFromDocument(mongo::OID& childOID, int size);
    void readFile();
    void insert(BSONObj&);
    int count(BSONObj & object);
    void update(BSONObj& query, BSONObj& update);
    auto_ptr<DBClientCursor> query(BSONObj& query);
    boost::shared_ptr<DBClientConnection>  getClient();
    void index(string& field);
    BSONObj findOne(BSONObj& query);
};


boost::shared_ptr<DBClientConnection>  MongoProxy::getClient()
{
	boost::shared_ptr<DBClientConnection> c2 = c;
	return c2;
}


void MongoProxy::insert(BSONObj & object)
{
	c->insert(dbCollectionPath, object);
}

int MongoProxy::count(BSONObj & object)
{
	int options=0;
	int limit=0;
	int skip=0;
	return c->count(dbCollectionPath, object, options, limit, skip);
}

void MongoProxy::update(BSONObj& query, BSONObj& update)
{
	c->update(dbCollectionPath, Query(query), update, false, true);
}

BSONObj MongoProxy::findOne(BSONObj& query)
{
	return c->findOne(dbCollectionPath, Query(query));
}

auto_ptr<DBClientCursor> MongoProxy::query(BSONObj& query)
{
	return c->query(dbCollectionPath, (Query(query)));
}

void MongoProxy::index(string& field)
{
	c->createIndex(dbCollectionPath, BSON(field<<1));
}

void MongoProxy::cloudEncoding(OID& oid, string& tempFileName, string & cloudType)
{
	try{
		pcl::io::OctreePointCloudCompression<pcl::PointXYZ>* PointCloudDecoderXYZ;
		pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudDecoderXYZRGB;
		int QueryOptions = 0;
		const BSONObj *fieldsToReturn = 0;
		LOG(LNOTICE)<<"dbCollectionPath: "<<dbCollectionPath<<"\n\n";

		// get bson object from collection
		LOG(LNOTICE)<<oid<<endl;
		BSONObj obj = c->findOne(dbCollectionPath, Query( BSON("_id" << oid)), fieldsToReturn, QueryOptions);

		// read data to buffer
		int readSize;
		LOG(LNOTICE)<<obj<<"\n";

		const char* charPtr= (const char*)(obj[tempFileName].binData(readSize));
		LOG(LNOTICE)<<"Readed: "<<readSize<<" B.";
		LOG(LNOTICE)<<"charPtr: "<<charPtr;
		stringstream * strPtr = (stringstream*)charPtr;
		LOG(LNOTICE)<<"strPtr: "<<strPtr;
		LOG(LNOTICE)<<"charPtr[0]: "<<charPtr[0];
		LOG(LNOTICE)<<*charPtr;
		LOG(LNOTICE)<<*strPtr;
		LOG(LNOTICE)<<strPtr->str();


		if(cloudType=="xyz")
		{
			PointCloudDecoderXYZ=new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>();
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ (new pcl::PointCloud<pcl::PointXYZ>);
			PointCloudDecoderXYZ->decodePointCloud (*strPtr, cloudXYZ);
			pcl::io::savePCDFile("newCloud2.pcd", *cloudXYZ, false);
		}
		else if(cloudType=="xyzrgb")
		{
			PointCloudDecoderXYZRGB=new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>();
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
			PointCloudDecoderXYZRGB->decodePointCloud (*strPtr, cloudXYZRGB);
			pcl::io::savePCDFile("newCloud2.pcd", *cloudXYZRGB, false);
		}
		LOG(LNOTICE)<<"ViewWriter::insertFileIntoCollection: END";
	}catch(Exception &ex){LOG(LNOTICE)<<ex.what();}
}
void MongoProxy::initView(const string & viewName, bool addToObjectFlag, Base::Property<string>& nodeNameProp, Base::Property<string>& objectName, Base::Property<string>& description)
{
	BSONElement oi;
    OID o;
    BSONArrayBuilder stereoPCArrayBuilder, kinectPCArrayBuilder, tofPCArrayBuilder, objectArrayBuilder, viewArrayBuilder, stereoArrayBuilder, kinectArrayBuilder, tofArrayBuilder, viewBuilder;
    LOG(LNOTICE)<<"Init View\n";
    //add view to object
    if(addToObjectFlag)
    {
    	addToObject(nodeNameProp, viewName, objectName, description);
    }
    // add childs to arraysBuilder
    for(std::vector<string>::iterator it = docViewsNames.begin(); it != docViewsNames.end(); ++it){
		BSONObj document = BSONObjBuilder().genOID().append("NodeName", *it).append("ObjectName", objectName).append("ViewName", viewName).append("description", description).obj();
		c->insert(dbCollectionPath, document);
		document.getObjectID(oi);
		o=oi.__oid();
		//docViewsNames.push_back("ToFRGBD");
		//docViewsNames.push_back("ToFRX");
		//docViewsNames.push_back("ToFRXM");
		LOG(LNOTICE)<< "Add to "<< *it<<" , this: "<<o.toString()<<" in "<<dbCollectionPath<<"\n";
		if(*it=="Stereo" || *it=="Kinect" || *it=="ToF")
			viewArrayBuilder.append(BSONObjBuilder().append("childOID", o.toString()).obj());
		else if(*it=="StereoLR" || *it=="StereoRX" || *it=="StereoRXM" || *it=="StereoPC")
			stereoArrayBuilder.append(BSONObjBuilder().append("childOID", o.toString()).obj());
		else if(*it=="KinectRGBD" || *it=="KinectRX" || *it=="KinectRXM"  || *it=="KinectPC")
			kinectArrayBuilder.append(BSONObjBuilder().append("childOID", o.toString()).obj());
		else if(*it=="ToFRGBD" || *it=="ToFRX" || *it=="ToFRXM" || *it=="ToFPC")
			tofArrayBuilder.append(BSONObjBuilder().append("childOID", o.toString()).obj());
		else if(*it=="KinectPCXYZRGB" || *it=="KinectPCXYZSIFT" || *it== "KinectPCXYZSHOT"  ) //
			kinectPCArrayBuilder.append(BSONObjBuilder().append("childOID", o.toString()).obj());
		else if(*it=="StereoPCXYZRGB" || *it=="StereoPCXYZSIFT" || *it=="StereoPCXYZSHOT") //"StereoPC"
			stereoPCArrayBuilder.append(BSONObjBuilder().append("childOID", o.toString()).obj());
		else if(*it== "ToFPCXYZRGB" || *it=="ToFPCXYZSIFT" || *it=="ToFPCXYZSHOT") //ToFPC
			tofPCArrayBuilder.append(BSONObjBuilder().append("childOID", o.toString()).obj());
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
    c->update(dbCollectionPath, Query(BSON("NodeName"<<"View"<<"ObjectName"<<objectName<<"ViewName"<<viewName)), BSON("$set"<<BSON("childOIDs"<<viewArr)), false, true);
    c->update(dbCollectionPath, Query(BSON("NodeName"<<"ToF"<<"ObjectName"<<objectName<<"ViewName"<<viewName)), BSON("$set"<<BSON("childOIDs"<<tofArr)), false, true);
    c->update(dbCollectionPath, Query(BSON("NodeName"<<"Kinect"<<"ObjectName"<<objectName<<"ViewName"<<viewName)), BSON("$set"<<BSON("childOIDs"<<kinectArr)), false, true);
    c->update(dbCollectionPath, Query(BSON("NodeName"<<"Stereo"<<"ObjectName"<<objectName<<"ViewName"<<viewName)), BSON("$set"<<BSON("childOIDs"<<stereoArr)), false, true);
    c->update(dbCollectionPath, Query(BSON("NodeName"<<"KinectPC"<<"ObjectName"<<objectName<<"ViewName"<<viewName)), BSON("$set"<<BSON("childOIDs"<<kinectPCArr)), false, true);
    c->update(dbCollectionPath, Query(BSON("NodeName"<<"StereoPC"<<"ObjectName"<<objectName<<"ViewName"<<viewName)), BSON("$set"<<BSON("childOIDs"<<stereoPC)), false, true);
    c->update(dbCollectionPath, Query(BSON("NodeName"<<"ToFPC"<<"ObjectName"<<objectName<<"ViewName"<<viewName)), BSON("$set"<<BSON("childOIDs"<<tofPCArr)), false, true);
    c->createIndex(dbCollectionPath, BSON("ObjectName"<<1));
    c->createIndex(dbCollectionPath, BSON("ViewName"<<1));
    c->createIndex(dbCollectionPath, BSON("NodeName"<<1));
}

void MongoProxy::addToObject(const Base::Property<string>& nodeNameProp,const string & name, Base::Property<string>& objectName, Base::Property<string>& description)
{
	LOG(LNOTICE)<<"Create View";
	int options=0;
	int limit=0;
	int skip=0;
	BSONElement oi;
	OID o;
	string type;
	string nodeName;
	nodeName = nodeNameProp;
	if(nodeName=="View"||nodeName=="Model")
		type=nodeName;
	else if(isModelLastLeaf(nodeNameProp))
		type="Model";
	else if(isViewLastLeaf(nodeNameProp))
		type="View";
	//CLOG(LTRACE)<<"Type: " <<type;

	unsigned long long nr = c->count(dbCollectionPath, BSON("ObjectName"<<objectName<<"NodeName"<<"Object"), options, limit, skip);
	// add object
	if(nr==0)
	{
		LOG(LNOTICE)<<"Create Object";
		//CLOG(LTRACE) <<"Object does not exists in "<< dbCollectionPath;
		BSONObj object = BSONObjBuilder().genOID().append("NodeName", "Object").append("ObjectName", objectName).append("description", description).obj();
		c->insert(dbCollectionPath, object);
		addScenes(object, objectName);
	}
	// add model/view
	LOG(LNOTICE)<<"Create Model/View";
	BSONObj modelorView = BSONObjBuilder().genOID().append("NodeName", type).append("ObjectName", objectName).append(type+"Name", name).append("description", description).obj();
	c->insert(dbCollectionPath, modelorView);
	modelorView.getObjectID(oi);
	o=oi.__oid();
	LOG(LNOTICE) << "\n"<<o.toString() << "\n";
	c->update(dbCollectionPath, Query(BSON("ObjectName"<<objectName<<"NodeName"<<"Object")), BSON("$addToSet"<<BSON("childOIDs"<<BSON("childOID"<<o.toString()))), false, true);
}

void MongoProxy::addScenes(BSONObj& object, Base::Property<string>& objectName)
{
	int items = 0;
	bool objectInTheScene = false;
	OID o;
	BSONElement bsonElement;
	BSONElement oi;
	BSONArrayBuilder bsonBuilder;

	for(std::vector<string>::iterator itSceneName = splitedSceneNames.begin(); itSceneName != splitedSceneNames.end(); ++itSceneName)
	{
		//CLOG(LINFO)<<"Scene: "<<*itSceneName;
		// if scene exist
		items = c->count(dbCollectionPath, (Query(BSON("SceneName"<<*itSceneName))));
		if(items>0)
		{
			auto_ptr<DBClientCursor> cursorCollection =c->query(dbCollectionPath, Query(BSON("SceneName"<<*itSceneName)));
			BSONObj scene = cursorCollection->next();
			LOG(LNOTICE)<<"Add scene to the object!";
			scene.getObjectID(oi);
			o=oi.__oid();

			c->update(dbCollectionPath, Query(BSON("ObjectName"<<objectName<<"NodeName"<<"Object")), BSON("$addToSet"<<BSON("sceneOIDs"<<BSON("sceneOID"<<o.toString()))), false, true);
		//	CLOG(LTRACE)<<scene;

			vector<OID> childsVector;
			if(getChildOIDS(scene, "objectsOIDs", "objectOID", childsVector)>0)
			{
				for (unsigned int i = 0; i<childsVector.size(); i++)
				{
					auto_ptr<DBClientCursor> childCursor = c->query(dbCollectionPath, (Query(BSON("_id"<<childsVector[i]))));
					if( childCursor->more())
					{
						BSONObj childObj = childCursor->next();
						string _id = childObj.getField("_id").str();
						if(_id==o.toString())
						{
							objectInTheScene = true;
							//CLOG(LERROR)<< "Object exists in the scene!";
							break;
						}
					}
				}
			}
			if(!objectInTheScene)
			{
				LOG(LNOTICE)<<"Adding object to the scene";
				object.getObjectID(oi);
				o=oi.__oid();
				c->update(dbCollectionPath, Query(BSON("SceneName"<<*itSceneName)), BSON("$addToSet"<<BSON("objectsOIDs"<<BSON("objectOID"<<o.toString()))), false, true);
			}
		}//if
		else
		{
			c->createIndex(dbCollectionPath, BSON("SceneName"<<1));
			LOG(LNOTICE)<<"Create scene and add object to array list";
			BSONObj scene = BSONObjBuilder().genOID().append("SceneName", *itSceneName).obj();
			c->insert(dbCollectionPath, scene);

			//CLOG(LINFO)<<"Adding object to the scene";
			object.getObjectID(oi);
			o=oi.__oid();
			c->update(dbCollectionPath, Query(BSON("SceneName"<<*itSceneName)), BSON("$addToSet"<<BSON("objectsOIDs"<<BSON("objectOID"<<o.toString()))), false, true);

			//CLOG(LINFO)<<"Add scene to object!";
			scene.getObjectID(oi);
			o=oi.__oid();
			c->update(dbCollectionPath, Query(BSON("ObjectName"<<objectName<<"NodeName"<<"Object")), BSON("$addToSet"<<BSON("sceneOIDs"<<BSON("sceneOID"<<o.toString()))), false, true);
		}
	}
}


void MongoProxy::initModel(const string & modelName, bool addToModelFlag,  Base::Property<string>& nodeNameProp, Base::Property<string>& objectName, Base::Property<string>& description)
{
	LOG(LNOTICE)<<"initModel";
	BSONElement oi;
	OID o;
	BSONArrayBuilder objectArrayBuilder, modelArrayBuilder, somArrayBuilder, ssomArrayBuilder;

	if(addToModelFlag)
	{
		addToObject(nodeNameProp, modelName, objectName, description);
	}

	for(std::vector<string>::iterator it = docModelsNames.begin(); it != docModelsNames.end(); ++it){
		LOG(LNOTICE)<<"*it: "<<*it<<"\n";
		BSONObj document = BSONObjBuilder().genOID().append("NodeName", *it).append("ObjectName", objectName).append("ModelName", modelName).append("description", description).obj();
		c->insert(dbCollectionPath, document);

		document.getObjectID(oi);
		o=oi.__oid();
		LOG(LNOTICE)<<"OID: "<<o.toString()<<"\n";

		if(*it=="SOM" || *it=="SSOM")
			modelArrayBuilder.append(BSONObjBuilder().append("childOID", o.toString()).obj());
		else if(*it=="SomXYZRgb" || *it=="SomXYZSift")
			somArrayBuilder.append(BSONObjBuilder().append("childOID", o.toString()).obj());
		else if(*it=="SsomXYZRgb" || *it=="SsomXYZSift" || *it=="SsomXYZShot")
			ssomArrayBuilder.append(BSONObjBuilder().append("childOID", o.toString()).obj());

	}
	c->createIndex(dbCollectionPath, BSON("ObjectName"<<1));
	c->createIndex(dbCollectionPath, BSON("ModelName"<<1));
	c->createIndex(dbCollectionPath, BSON("NodeName"<<1));

	BSONArray modelArr = modelArrayBuilder.arr();
	BSONArray somArr = somArrayBuilder.arr();
	BSONArray ssomArr = ssomArrayBuilder.arr();

	c->update(dbCollectionPath, Query(BSON("NodeName"<<"Model"<<"ObjectName"<<objectName<<"ModelName"<<modelName)), BSON("$set"<<BSON("childOIDs"<<modelArr)), false, true);
	c->update(dbCollectionPath, Query(BSON("NodeName"<<"SOM"<<"ObjectName"<<objectName<<"ModelName"<<modelName)), BSON("$set"<<BSON("childOIDs"<<somArr)), false, true);
	c->update(dbCollectionPath, Query(BSON("NodeName"<<"SSOM"<<"ObjectName"<<objectName<<"ModelName"<<modelName)), BSON("$set"<<BSON("childOIDs"<<ssomArr)), false, true);
}


void MongoProxy::setModelOrViewName(const string& childNodeName, const BSONObj& childObj, string& newName)
{
	string type = childNodeName;
	newName = childObj.getField(type+"Name").str();
}

void MongoProxy::connectToMongoDB()
{
	try{
		//if(!c->isStillConnected())
		{
			LOG(LNOTICE)<<"hostname "<<hostname;
			c->connect(hostname);
			LOG(LNOTICE)<<"Connected to base\n";
		}
	//	else
	//		LOG(LNOTICE)<<"\n\n\nConnected?\n\n\n";

	}catch(ConnectException& ex)
	{
		LOG(LNOTICE)<<ex.what();
	}
}

void MongoProxy::initViewNames()
{
	docViewsNames.push_back("Stereo");
	docViewsNames.push_back("Kinect");
	docViewsNames.push_back("ToF");
	docViewsNames.push_back("StereoPC");
	docViewsNames.push_back("StereoPCXYZRGB");
	docViewsNames.push_back("StereoPCXYZSIFT");
	docViewsNames.push_back("StereoPCXYZSHOT");
	docViewsNames.push_back("StereoLR");
	docViewsNames.push_back("StereoRX");
	docViewsNames.push_back("StereoRXM");
	docViewsNames.push_back("KinectPC");
	docViewsNames.push_back("KinectPCXYZRGB");
	docViewsNames.push_back("KinectPCXYZSIFT");
	docViewsNames.push_back("KinectPCXYZSHOT");
	docViewsNames.push_back("KinectRGBD");
	docViewsNames.push_back("KinectRX");
	docViewsNames.push_back("KinectRXM");
	docViewsNames.push_back("ToFPC");
	docViewsNames.push_back("ToFPCXYZRGB");
	docViewsNames.push_back("ToFPCXYZSIFT");
	docViewsNames.push_back("ToFPCXYZSHOT");
	docViewsNames.push_back("ToFRGBD");
	docViewsNames.push_back("ToFRX");
	docViewsNames.push_back("ToFRXM");
}

void MongoProxy::initModelNames()
{
	docModelsNames.push_back("SomXYZRgb");
	docModelsNames.push_back("SomXYZSift");
	docModelsNames.push_back("SsomXYZRgb");
	docModelsNames.push_back("SsomXYZSift");
	docModelsNames.push_back("SsomXYZShot");
	docModelsNames.push_back("SSOM");
	docModelsNames.push_back("SOM");
}


int MongoProxy::getChildOIDS(BSONObj &obj, const string & fieldName, const string & childfieldName, vector<OID>& oidsVector)
{
	string output = obj.getField(fieldName);
	if(output!="EOO")
	{
		vector<BSONElement> v = obj.getField(fieldName).Array();
		for (unsigned int i = 0; i<v.size(); i++)
		{
			string readedOid =v[i][childfieldName].str();
			OID o = OID(readedOid);
			oidsVector.push_back(o);
		}
		return 1;
	}
	else
		return 0;
}

void  MongoProxy::findDocumentInCollection(DBClientConnection& c, string& dbCollectionPath, Base::Property<string>& objectName, const string &nodeName, auto_ptr<DBClientCursor> & cursorCollection, const string & modelOrViewName, const string & type, int& items)
{
	int options=0;
	int limit=0;
	int skip=0;
	LOG(LNOTICE)<<"\nMongoProxy::findDocumentInCollection\n";

      try{
    	  if(type!="")
    	  {
			  if(type=="View")
			  {
				  LOG(LNOTICE)<<"\nView\n";
				  items = c.count(dbCollectionPath, (BSON("NodeName"<<nodeName<<"ObjectName"<<objectName<<"ViewName"<<modelOrViewName)), options, limit, skip);
				  if(items>0)
					  cursorCollection =c.query(dbCollectionPath, (Query(BSON("NodeName"<<nodeName<<"ObjectName"<<objectName<<"ViewName"<<modelOrViewName))));
			  }
			  else if(type=="Model")
			  {
				  LOG(LNOTICE)<<"\nModel\n";
				  items = c.count(dbCollectionPath, BSON("NodeName"<<nodeName<<"ObjectName"<<objectName<<"ModelName"<<modelOrViewName), options, limit, skip);
				  if (items>0)
					  cursorCollection =c.query(dbCollectionPath, Query(BSON("NodeName"<<nodeName<<"ObjectName"<<objectName<<"ModelName"<<modelOrViewName)));
			  }
			  else
			  {
				  items = c.count(dbCollectionPath, BSON("NodeName"<<nodeName<<"ObjectName"<<objectName<<type<<modelOrViewName), options, limit, skip);

				  if(items>0)
					  cursorCollection =c.query(dbCollectionPath, (Query(BSON("NodeName"<<nodeName<<"ObjectName"<<objectName<<type<<modelOrViewName))));
			  }
		  }
    	  else
    	  {
    		  LOG(LNOTICE)<<"\nObject\n";
			  LOG(LNOTICE)<<"NodeName"<<nodeName<<"\tObjectName"<<objectName<<"\n";

    		  items = c.count(dbCollectionPath, BSON("NodeName"<<nodeName<<"ObjectName"<<objectName), options, limit, skip);
    		  LOG(LNOTICE)<<"items: "<<items<<"\n";
    		  if(items>0)
    			  cursorCollection =c.query(dbCollectionPath, (Query(BSON("NodeName"<<nodeName<<"ObjectName"<<objectName))));
    	  }
    	 }
      catch(DBException &e)
      {
    	  LOG(LERROR) <<"findDocumentInCollection(). Something goes wrong... :<";
    	  LOG(LERROR) <<c.getLastError();
      }

      return;
}
bool MongoProxy::isViewLastLeaf(const string& nodeName)
{
	if(nodeName=="StereoPCXYZRGB" || nodeName=="StereoPCXYZSIFT" || nodeName=="StereoPCXYZSHOT" || nodeName=="ToFPCXYZSIFT" || nodeName=="ToFPCXYZRGB" || nodeName=="ToFPCXYZRGB"  || nodeName=="ToFPCXYZSHOT" || nodeName=="KinectPCXYZSHOT"  || nodeName=="KinectPCXYZSIFT" || nodeName=="KinectPCXYZRGB" || nodeName=="StereoLR" || nodeName=="KinectRGBD" || nodeName=="ToFRGBD" || nodeName=="StereoRX" || nodeName=="KinectRX" ||  nodeName=="ToFRX" || nodeName=="StereoRXM" || nodeName=="KinectRXM" || nodeName=="ToFRXM")
		return true;
	else
		return false;
}

bool MongoProxy::isModelLastLeaf(const string& nodeName)
{
	if(nodeName=="SomXYZRgb" || nodeName=="SomXYZSift" ||  nodeName=="SsomXYZRgb" || nodeName=="SsomXYZSift" || nodeName=="SsomXYZShot")
		return true;
	else
		return false;
}



} /* namespace MongoProxy */
#endif /* MONGOPROXY_HPP_ */
