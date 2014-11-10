/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */


#include "MongoDBExporter.hpp"

#define siftPointSize 133
#define xyzPointSize 3
#define xyzrgbPointSize 4

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
		nodeNameProp("nodeName", string("Object")),
		folderName("folderName", string("/home/lzmuda/mongo_driver_tutorial/")),
		viewNameProp("viewName", string("")),
		sceneNamesProp("sceneNamesProp", string("scene1,scene2,scene3")),
		modelNameProp("modelName", string(""))
{
        registerProperty(mongoDBHost);
        registerProperty(objectName);
        registerProperty(description);
        registerProperty(collectionName);
        registerProperty(nodeNameProp);
        registerProperty(folderName);
        registerProperty(viewNameProp);
        registerProperty(modelNameProp);
        registerProperty(sceneNamesProp);
        fileExtensions.push_back("*.png");
        fileExtensions.push_back("*.jpg");
        fileExtensions.push_back("*.txt");
        fileExtensions.push_back("*.pcd");


        CLOG(LTRACE) << "Hello MongoDBExporter";
}

MongoDBExporter::~MongoDBExporter()
{
        CLOG(LTRACE) << "Good bye MongoDBExporter";
}
void MongoDBExporter::write2DB()
{
        CLOG(LNOTICE) << "MongoDBExporter::write2DB";

        string sceneNames = sceneNamesProp;
        boost::split(MongoBase::splitedSceneNames, sceneNames, is_any_of(","));
        if(modelNameProp!="")
        	insert2MongoDB(nodeNameProp,modelNameProp, "Model");
        else if(viewNameProp!="")
            insert2MongoDB(nodeNameProp,viewNameProp, "View");
        else
        	insert2MongoDB(nodeNameProp,"", "");

}

void MongoDBExporter::prepareInterface() {
        CLOG(LTRACE) << "MongoDBExporter::prepareInterface";
        h_write2DB.setup(this, &MongoDBExporter::write2DB);
        registerHandler("write2DB", &h_write2DB);
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
	CLOG(LERROR)<<"*it : " << *it;
	if(*it=="." || *it=="..")
		return;
	BSONObj model = BSONObjBuilder().genOID().append("NodeName", type).append("ObjectName", objectName).append(type+"Name", *it).append("description", description).obj();
	c->insert(dbCollectionPath, model);
	model.getObjectID(bsonElement);
	OID oid=bsonElement.__oid();
	CLOG(LERROR)<< "oid: " <<oid.toString();
	bsonBuilder.append(BSONObjBuilder().append("childOID", oid.toString()).obj());
	CLOG(LERROR) << "NodeName : "  << type;
	if(type=="Model")
		initModel(*it, false, nodeNameProp, objectName, description);
	else if(type=="View")
		initView(*it, false, nodeNameProp, objectName, description);
}

void MongoDBExporter::initObject()
{
	CLOG(LTRACE) <<"Create template of object";
	BSONArrayBuilder bsonBuilder;
	bool objectInTheScene = false;
	try
	{
		BSONObj object = BSONObjBuilder().genOID().append("NodeName", "Object").append("ObjectName", objectName).append("description", description).obj();
		c->insert(dbCollectionPath, object);
	//	c->createIndex(dbCollectionPath, BSON("ObjectName"<<1));
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
		c->update(dbCollectionPath, Query(BSON("ObjectName"<<objectName<<"NodeName"<<"Object")), BSON("$set"<<BSON("childOIDs"<<destArr)), false, true);
	  }
	  catch(DBException &e)
	  {
		CLOG(LERROR) <<"Something goes wrong... :<";
		CLOG(LERROR) <<c->getLastError();
	  }
}

void MongoDBExporter::insertFileToGrid(string&  extension, const std::vector<string>::iterator it, string & newFileName, OID& oid, int totalSize)
{
	BSONObj o;
	BSONElement bsonElement;
	string mime="";
	setMime(extension, mime);
	GridFS fs(*c, collectionName);
	o = fs.storeFile(*it, newFileName, mime);
	BSONObj b;
	//if(cloudType=="xyzsift")
	//	b = BSONObjBuilder().appendElements(o).append("ObjectName", objectName).append("size", totalSize).append("place", "grid").append("mean_viewpoint_features_number", mean_viewpoint_features_number).obj();
	//else
		b = BSONObjBuilder().appendElements(o).append("ObjectName", objectName).append("size", totalSize).append("place", "grid").obj();
	//c->createIndex(dbCollectionPath, BSON("filename"<<1));
	c->insert(dbCollectionPath, b);
	b.getObjectID(bsonElement);
	oid=bsonElement.__oid();
	//bsonBuilder.append(BSONObjBuilder().append("childOID", oid.toString()).obj());
}

void MongoDBExporter::writeNode2MongoDB(const string &source, const string &destination, const string &type,string modelOrViewName)
{
	CLOG(LTRACE) <<"Source: " << source << " destination: "<< destination<<" dbCollectionPath: "<<dbCollectionPath;
    try{
    	for(std::vector<string>::iterator itExtension = fileExtensions.begin(); itExtension != fileExtensions.end(); ++itExtension) {
    		CLOG(LTRACE) <<"source+*itExtension "<<source+*itExtension;
			vector<string> files = getAllFiles(source+*itExtension);
			for(std::vector<string>::iterator it = files.begin(); it != files.end(); ++it)
			{
				string fileName = *it;
				string newFileName;
				float sizeOfFileBytes = get_file_size(fileName);
				float sizeOfFileMBytes = sizeOfFileBytes/(1024.0*1024.0);
				CLOG(LERROR)<<"sizeOfFile: "<<sizeOfFileMBytes;

				const size_t last_slash_idx = fileName.find_last_of("/");
				if (std::string::npos != last_slash_idx)
				{
					newFileName = fileName.erase(0, last_slash_idx + 1);
				}
				OID oid;
				string extension = itExtension->erase(0,2);
				if(sizeOfFileMBytes>1.0)
					insertFileToGrid(extension, it, newFileName, oid, sizeOfFileBytes);
				else if(sizeOfFileMBytes<=1.0)
				{
					string mime;
					setMime(extension, mime);
					writeToMemory(mime, *it);
					insertFileIntoCollection(oid, extension, fileName, sizeOfFileBytes);
				}
				CLOG(LERROR)<<"UPDATE: "<<oid.toString();
				c->update(dbCollectionPath, Query(BSON("ObjectName"<<objectName<<type+"Name"<<modelOrViewName<<"NodeName"<<destination)), BSON("$addToSet"<<BSON("childOIDs"<<BSON("childOID"<<oid.toString()))), false, true);
			}
    	}
		CLOG(LTRACE) <<"Files saved successfully";
    }
	catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :<";
		CLOG(LERROR) <<c->getLastError();
	}
}


float MongoDBExporter::get_file_size(std::string& filename) // path to file
{
    FILE *p_file = NULL;
    float size;
    try{
    	p_file = fopen(filename.c_str(),"rb");
    	fseek(p_file,0,SEEK_END);
    	size = (float)ftell(p_file);
    	fclose(p_file);
    }
    catch(Exception & ex){ex.what();}
    return size;
}

void MongoDBExporter::ReadPCDCloudFromFile(const string& filename)
{
	CLOG(LTRACE) << "MongoDBExporter::ReadPCDCloudFromFile";
	// Try to read the cloud of XYZRGB points.
	if(filename.find("xyzrgb")!=string::npos)
	{
		//cloudXYZRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *cloudXYZRGB) == -1){
			CLOG(LWARNING) <<"Cannot read PointXYZRGB cloud from "<<filename;
		}else{
			//out_cloud_xyzrgb.write(cloud_xyzrgb);
			CLOG(LINFO) <<"PointXYZRGB cloud loaded properly from "<<filename;
			//return;
		}// else
	}

	// Try to read the cloud of XYZSIFT points.


	 if(filename.find("xyzsift.pcd")!=string::npos)
	 {
		// cloudXYZRGBSIFT (new pcl::PointCloud<PointXYZSIFT>);
		if (pcl::io::loadPCDFile<PointXYZSIFT> (filename, *cloudXYZSIFT) == -1){
			CLOG(LWARNING) <<"Cannot read PointXYZSIFT cloud from "<<filename;
		}else{
			//out_cloud_xyzsift.write(cloud_xyzsift);
			CLOG(LINFO) <<"PointXYZSIFT cloud loaded properly from "<<filename;

			//return;
		}// else
	}

	else if(filename.find("xyz")!=string::npos)
		{
			// Try to read the cloud of XYZ points.
			//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
			if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloudXYZ) == -1){
				CLOG(LWARNING) <<"Cannot read PointXYZ cloud from "<<filename;
			}else{
				//out_cloud_xyz.write(cloud_xyz);
				CLOG(LINFO) <<"PointXYZ cloud loaded properly from "<<filename;
				//return;
			}// else
		}
}

void MongoDBExporter::writeToMemory(string& mime,  string& fileName)
{
	if(mime=="image/png" || mime=="image/jpeg")
	{
		// read from disc
		CLOG(LERROR)<<"Read file: "<<fileName;
		tempImg = imread(fileName, CV_LOAD_IMAGE_UNCHANGED);
		CLOG(LERROR)<<tempImg.size();
	}
	else if(mime=="text/plain")
	{
		CLOG(LINFO)<<"mime==text/plain";
		CLOG(LINFO)<<"fileName.find(pcd): "<<fileName.find("pcd");
		if((fileName.find("pcd"))!=string::npos)
		{
			CLOG(LINFO)<<"pcd :)";
			ReadPCDCloudFromFile(fileName);
		}
		else if(fileName.find("txt")!=string::npos)
		{
			char *tempFileName = (char*)fileName.c_str();
		    std::ifstream ifs(tempFileName); // open a file
		    string tmpstr((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
		    str = tmpstr;
		}
		else
			CLOG(LERROR)<<"Nie wiem co to za plik :/";
	}
}


void MongoDBExporter::insertFileIntoCollection(OID& oid, const string& fileType, string& tempFileName, int size)
{
	CLOG(LTRACE)<<"MongoDBExporter::insertFileIntoCollection";
	BSONObjBuilder builder;
	BSONObj b;
	BSONElement bsonElement;
	CLOG(LERROR)<<"fileType : "<<fileType;
	if (fileType=="png" || fileType=="jpg")	// save image
	{
		CLOG(LERROR)<<tempImg.size();
		CLOG(LTRACE)<<"MongoDBExporter::insertFileIntoCollection, Image file";
		std::vector<uchar> buf;
		std::vector<int> params(2);

		if(fileType=="png")
		{
			params[1] = 3;
			params[0] = CV_IMWRITE_PNG_COMPRESSION;
			cv::imencode(".png", tempImg, buf, params);
		}
		else
		{
			params[1] = 95;
			params[0] = CV_IMWRITE_JPEG_QUALITY;
			cv::imencode(".jpg", tempImg, buf, params);
		}

		b=BSONObjBuilder().genOID().appendBinData(tempFileName, buf.size(), mongo::BinDataGeneral, &buf[0]).append("fileName", tempFileName).append("size", size).append("place", "document").append("extension", fileType).obj();
		b.getObjectID(bsonElement);
		oid=bsonElement.__oid();
		c->insert(dbCollectionPath, b);
	}
	else if(fileType=="pcd")	// save cloud
	{
		if(cloudType=="xyzrgb")
		{
			int cloudSize = cloudXYZRGB->size();
			int fieldsNr = xyzrgbPointSize*cloudSize;
			int totalSize = fieldsNr*sizeof(float);
			float buff[fieldsNr];

			// copy all points to memory
			for(int iter=0; iter<cloudSize; iter++)
			{
				const pcl::PointXYZRGB p = cloudXYZRGB->points[iter];
				copyXYZRGBPointToFloatArray (p, &buff[xyzrgbPointSize*iter]);
			}
			b=BSONObjBuilder().genOID().appendBinData(tempFileName, totalSize, mongo::BinDataGeneral, &buff[0]).append("fileName", tempFileName).append("size", totalSize).append("place", "document").append("extension", fileType).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			c->insert(dbCollectionPath, b);
		}
		else if(cloudType=="xyz")
		{
			int cloudSize = cloudXYZ->size();

			int fieldsNr = xyzPointSize*cloudSize;
			int totalSize = fieldsNr*sizeof(float);
			float buff[fieldsNr];

			// copy all points to memory
			for(int iter=0; iter<cloudSize; iter++)
			{
				const pcl::PointXYZ p = cloudXYZ->points[iter];
				copyXYZPointToFloatArray (p, &buff[xyzPointSize*iter]);
			}
			b=BSONObjBuilder().genOID().appendBinData(tempFileName, totalSize, mongo::BinDataGeneral, &buff[0]).append("fileName", tempFileName).append("size", totalSize).append("place", "document").append("extension", fileType).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			c->insert(dbCollectionPath, b);
		}
		//TODO dodac SHOT'Y
		else if(cloudType=="xyzsift")
		{
			int cloudSize = cloudXYZSIFT->size();
			int fieldsNr = siftPointSize*cloudSize;
			int totalSize = fieldsNr*sizeof(float);
			float buff[fieldsNr];

			// copy all points to memory
			for(int iter=0; iter<cloudSize; iter++)
			{
				const PointXYZSIFT p = cloudXYZSIFT->points[iter];
				copyXYZSiftPointToFloatArray (p, &buff[siftPointSize*iter]);
		    }
			b=BSONObjBuilder().genOID().appendBinData(tempFileName, totalSize, mongo::BinDataGeneral, &buff[0]).append("fileName", tempFileName).append("size", totalSize).append("place", "document").append("extension", fileType).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			c->insert(dbCollectionPath, b);
		}
		CLOG(LTRACE)<<"ViewWriter::insertFileIntoCollection, PCD file end";
	}
	else if(fileType=="txt")
	{
		CLOG(LTRACE)<<"ViewWriter::insertFileIntoCollection, txt file";
		string input;

		// read string from input
		//input =cipFileIn.read()+" ";

		CLOG(LERROR)<<"Input:"<< str;

		// convert to char*
		char const *cipCharTable = str.c_str();
		int size = strlen(cipCharTable);
		CLOG(LERROR)<<string(cipCharTable);
		CLOG(LERROR)<<"Size: "<<size;
		// create bson object
		b = BSONObjBuilder().genOID().appendBinData(tempFileName, size, BinDataGeneral,  cipCharTable).append("fileName", tempFileName).append("size", size).append("place", "document").append("extension", fileType).obj();

		// insert object into collection
		c->insert(dbCollectionPath, b);

		b.getObjectID(bsonElement);
		oid=bsonElement.__oid();
	}
	cloudType="";
}

void MongoDBExporter::copyXYZPointToFloatArray (const pcl::PointXYZ &p, float * out) const
{
	out[0] = p.x;	// 4 bytes
	out[1] = p.y;	// 4 bytes
	out[2] = p.z;	// 4 bytes
}

void MongoDBExporter::copyXYZRGBPointToFloatArray (const pcl::PointXYZRGB &p, float * out) const
{
	out[0] = p.x;	// 4 bytes
	out[1] = p.y;	// 4 bytes
	out[2] = p.z;	// 4 bytes
	out[3] = p.rgb;	// 4 bytes
}

void MongoDBExporter::copyXYZSiftPointToFloatArray (const PointXYZSIFT &p, float * out) const
{
	Eigen::Vector3f outCoordinates = p.getArray3fMap();
	out[0] = outCoordinates[0];	// 4 bytes
	out[1] = outCoordinates[1];	// 4 bytes
	out[2] = outCoordinates[2];	// 4 bytes
	//CLOG(LERROR)<<"out[0]: "<<out[0]<<"\t out[1]: "<<out[1]<<"\t out[2]: "<<out[2];
	memcpy(&out[3], &p.multiplicity, sizeof(int)); // 4 bytes
	memcpy(&out[4], &p.pointId, sizeof(int));	// 4 bytes
	memcpy(&out[5], &p.descriptor, 128*sizeof(float)); // 128 * 4 bytes = 512 bytes
}


void MongoDBExporter::insert2MongoDB(const string &destination, const string&  modelOrViewName, const string&  type)
{
	auto_ptr<DBClientCursor> cursorCollection;
	string source;
	int options=0;
	int limit=0;
	int skip=0;
	int items=0;
	try{
		if(destination=="Object")
		{
			unsigned long long nr = c->count(dbCollectionPath, BSON("ObjectName"<<objectName<<"NodeName"<<"Object"), options, limit, skip);
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
			unsigned long long nr = c->count(dbCollectionPath, BSON("ObjectName"<<objectName<<"NodeName"<<"Object"));
			if(nr==0)
			{
				CLOG(LTRACE) <<"Object does not exists in "<< dbCollectionPath;
				initObject();
			}
			else
			{
				int items = c->count(dbCollectionPath, BSON("ObjectName"<<objectName<<"NodeName"<<type<<type+"Name"<<modelOrViewName), options, limit, skip);
				if(items==0)
				{
					CLOG(LTRACE)<<"No such model/view";
					CLOG(LTRACE)<<"NodeName : "<<type;
					if(type=="View")
						initView(modelOrViewName, true, nodeNameProp, objectName, description);
					else if(type=="Model")
						initModel(modelOrViewName, true, nodeNameProp, objectName, description);
				}
				cursorCollection = c->query(dbCollectionPath, Query(BSON("ObjectName"<<objectName<<"NodeName"<<type<<type+"Name"<<modelOrViewName)));
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
				if(nodeNameProp=="Model" || nodeNameProp=="View")
				{
					unsigned long long nr = c->count(dbCollectionPath, BSON("NodeName"<<type<<"ObjectName"<<objectName<<type+"Name"<<modelOrViewName), options, limit, skip);
					if(nr>0)
					{
						CLOG(LERROR)<<type+" "<< modelOrViewName<<" exists in db for object "<<objectName;
						return;
					}
					else
					{
						if(destination=="Model")
							initModel(modelOrViewName, true, nodeNameProp, objectName, description);
						else if(destination=="View")
							initView(modelOrViewName, true, nodeNameProp, objectName, description);
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
								auto_ptr<DBClientCursor> childCursor =c->query(dbCollectionPath, Query(BSON("_id"<<childsVector[i])));
								if( childCursor->more())
								{
									BSONObj childObj = childCursor->next();
									//TODO zmienic Typa na NodeName
									string childNodeName = childObj.getField("NodeName").str();
									CLOG(LINFO)<< "childNodeName: "<<childNodeName;
									if(childNodeName!="EOO")
									{
										if(childNodeName=="View"||childNodeName=="Model")
										{
											string newName;
											setModelOrViewName(childNodeName, childObj, newName);
											if(childNodeName=="View")
												insert2MongoDB(childNodeName, newName, "View");
											else
												insert2MongoDB(childNodeName, newName, "Model");
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
