/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */
// todo dodac zapis gdzie zapisane i rozmiar

#include "ModelWriter.hpp"

#define siftPointSize 133
#define xyzPointSize 3
#define xyzrgbPointSize 4

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
		modelNameProp("modelName", string("lab012")),
		fileName("fileName", string("tempFile")),
		nodeNameProp("nodeNameProp", string("SomXYZRgb")),
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
	registerProperty(modelNameProp);
	registerProperty(sceneNamesProp);
	registerProperty(fileName);
	registerProperty(nodeNameProp);
	registerProperty(remoteFileName);
	registerProperty(binary);
	registerProperty(suffix);
	sizeOfCloud=0.0;

	//base = new MongoBase::MongoBase();
    CLOG(LTRACE) << "Hello ModelWriter";
}

ModelWriter::~ModelWriter()
{
       CLOG(LTRACE) << "Good bye ModelWriter";
}

void ModelWriter::prepareInterface() {
	CLOG(LTRACE) << "ModelWriter::prepareInterface";
	registerHandler("Write_xyz", boost::bind(&ModelWriter::Write_cloud<pcl::PointXYZ>, this));
	registerHandler("Write_xyzrgb", boost::bind(&ModelWriter::Write_cloud<pcl::PointXYZRGB>, this));
	registerHandler("Write_xyzsift", boost::bind(&ModelWriter::Write_cloud<PointXYZSIFT>, this));
	registerHandler("Write_xyzrgbsift", boost::bind(&ModelWriter::Write_cloud<PointXYZRGBSIFT>, this));


	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_xyzrgbsift", &in_cloud_xyzrgbsift);

	addDependency("Write_xyzrgb", &in_cloud_xyzrgb);
	addDependency("Write_xyz", &in_cloud_xyz);
	addDependency("Write_xyzsift", &in_cloud_xyzsift);
	addDependency("Write_xyzrgbsift", &in_cloud_xyzsift);

}

void ModelWriter::writePCD2DB()
{
	CLOG(LNOTICE) << "ViewWriter::writePCD2DB";
	string sceneNames = sceneNamesProp;
	boost::split(MongoBase::splitedSceneNames, sceneNames, is_any_of(","));
	CLOG(LERROR)<<"cloudType: "<<cloudType;
	string fileType = "pcd";
	if(modelNameProp!="")
		insert2MongoDB(nodeNameProp, modelNameProp, "Model", fileType);
	else
		CLOG(LERROR)<<"Add model name and try again";
}

template <class PointT>
void ModelWriter::Write_cloud()
{
	CLOG(LTRACE) << "ViewWriter::Write_cloud()";
	std::vector< pcl::PCLPointField> fields;
	pcl::getFields<PointT>(fields);

	if(typeid(PointT) == typeid(pcl::PointXYZ))
	{
		cloudType="xyz";
		cloudXYZ = in_cloud_xyz.read();
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeOfCloud+=(float)pcl::getFieldSize(it->datatype)*cloudXYZ->size();//B
	}
	else if(typeid(PointT) == typeid(pcl::PointXYZRGB))
	{
		cloudType="xyzrgb";
		cloudXYZRGB = in_cloud_xyzrgb.read();
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeOfCloud+=(float)pcl::getFieldSize(it->datatype)*cloudXYZRGB->size();//B

	}
	else if(typeid(PointT) == typeid(PointXYZSIFT))
	{
		cloudType="xyzsift";
		cloudXYZSIFT = in_cloud_xyzsift.read();

		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeOfCloud+=(float)pcl::getFieldSize(it->datatype)*cloudXYZSIFT->size();//B
	}
	else if(typeid(PointT) == typeid(PointXYZSIFT))
	{
		cloudType="xyzrgbsift";
		cloudXYZRGBSIFT = in_cloud_xyzrgbsift.read();
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeOfCloud+=(float)pcl::getFieldSize(it->datatype)*cloudXYZRGBSIFT->size();//B
	}
	CLOG(LINFO)<<"CloudType: "<<cloudType;
	CLOG(LINFO)<<"PointCloudSize = "<< sizeOfCloud;
	writePCD2DB();
}

bool ModelWriter::onInit()
{
      CLOG(LTRACE) << "ModelWriter::initialize";
      try
      {
    	  cloudType="";
    	  string hostname = mongoDBHost;
    	  connectToMongoDB(hostname);
    	  if(collectionName=="containers")
    	  			MongoBase::dbCollectionPath=dbCollectionPath="images.containers";
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
	BSONObj model = BSONObjBuilder().genOID().append("NodeName", type).append("ObjectName", objectName).append(type+"Name", *it).append("description", description).obj();
	c->insert(dbCollectionPath, model);
	model.getObjectID(bsonElement);
	OID oid=bsonElement.__oid();
	bsonBuilder.append(BSONObjBuilder().append("childOID", oid.toString()).obj());
	if(type=="Model")
		initModel(*it, true, nodeNameProp, objectName, description);
	else if(type=="View")
		initModel(*it, true, nodeNameProp, objectName, description);
}

void ModelWriter::initObject()
{
	CLOG(LTRACE) <<"Create template of object";
	BSONArrayBuilder bsonBuilder;
	bool objectInTheScene = false;
	try
	{
		BSONObj object = BSONObjBuilder().genOID().append("NodeName", "Object").append("ObjectName", objectName).append("description", description).obj();
		c->insert(dbCollectionPath, object);
		c->createIndex(dbCollectionPath, BSON("ObjectName"<<1));
		addScenes(object, objectName);
	}
	catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :<";
	}
}

void ModelWriter::insertFileIntoGrid(OID& oid, const string& fileType, string& tempFileName, int totalSize)
{
	try{
		BSONObj object;
		BSONElement bsonElement;
		string mime="";
		setMime(fileType, mime);
		std::stringstream time;
		string tempFileName;
		if (fileType=="png" || fileType=="jpg")
		{
			CLOG(LINFO)<<"Image!";
			cv::Mat tempImg = in_img.read();
			tempFileName = string(fileName)+"."+string(fileType);
			cv::imwrite(tempFileName, tempImg);
		}
		else if(fileType=="pcd")	// save to file pcd
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
			fileNameInMongo = (string)remoteFileName+"_"+ cloudType + time.str()+"."+string(fileType);
		else
			fileNameInMongo = (string)remoteFileName + time.str()+"."+string(fileType);

		object = fs.storeFile(tempFileName, fileNameInMongo, mime);
		BSONObj b;
		if(cloudType=="xyzsift")
			b = BSONObjBuilder().appendElements(object).append("ObjectName", objectName).append("size", totalSize).append("place", "grid").append("mean_viewpoint_features_number", mean_viewpoint_features_number).obj();
		else
			b = BSONObjBuilder().appendElements(object).append("ObjectName", objectName).append("size", totalSize).append("place", "grid").obj();
		cloudType="";

		c->createIndex(dbCollectionPath, BSON("filename"<<1));

		c->insert(dbCollectionPath, b);
		b.getObjectID(bsonElement);
		oid=bsonElement.__oid();
	}catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :<";
		CLOG(LERROR) <<c->getLastError();
	}
}

void ModelWriter::writeNode2MongoDB(const string &destination, const string &type,string modelOrViewName, const string& fileType)
{
	CLOG(LTRACE) <<"writeNode2MongoDB";
	OID oid;
	CLOG(LTRACE) <<"Filename: " << fileName << " destination: "<< destination<<" dbCollectionPath: "<<dbCollectionPath;
    try{
    	string tempFileName="";
    	float sizeOfFileBytes = getFileSize(fileType, tempFileName);
    	float sizeOfFileMBytes = sizeOfFileBytes/(1024*1024);
    	if(sizeOfFileMBytes>15.0)
    	{
    		insertFileIntoGrid(oid, fileType, tempFileName, sizeOfFileBytes);
			c->update(dbCollectionPath, Query(BSON("ObjectName"<<objectName<<type+"Name"<<modelOrViewName<<"NodeName"<<destination)), BSON("$addToSet"<<BSON("childOIDs"<<BSON("childOID"<<oid.toString()))), false, true);
			CLOG(LTRACE) <<"Files saved successfully";
    	}
    	else
		{
			CLOG(LERROR)<<"sizeOfFileBytes: "<<sizeOfFileBytes;
			insertFileIntoCollection(oid, fileType, tempFileName, sizeOfFileBytes);
			CLOG(LERROR)<<"UPDATE: "<<oid.toString();
			c->update(dbCollectionPath, Query(BSON("ObjectName"<<objectName<<type+"Name"<<modelOrViewName<<"NodeName"<<destination)), BSON("$addToSet"<<BSON("childOIDs"<<BSON("childOID"<<oid.toString()))), false, true);
		}
    }
	catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :<";
		CLOG(LERROR) <<c->getLastError();
	}
}

void ModelWriter::copyXYZPointToFloatArray (const pcl::PointXYZ &p, float * out) const
{
	out[0] = p.x;	// 4 bytes
	out[1] = p.y;	// 4 bytes
	out[2] = p.z;	// 4 bytes
}

void ModelWriter::copyXYZRGBPointToFloatArray (const pcl::PointXYZRGB &p, float * out) const
{
	out[0] = p.x;	// 4 bytes
	out[1] = p.y;	// 4 bytes
	out[2] = p.z;	// 4 bytes
	out[3] = p.rgb;	// 4 bytes
}

void ModelWriter::copyXYZSiftPointToFloatArray (const PointXYZSIFT &p, float * out) const
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

void ModelWriter::insertFileIntoCollection(OID& oid, const string& fileType, string& tempFileName, int size)
{
	CLOG(LTRACE)<<"ViewWriter::insertFileIntoCollection";
	//TODO add fileName and size to document
	BSONObjBuilder builder;
	BSONObj b;
	BSONElement bsonElement;
	// todo dodac obsluge cv mat xyz std::vector<float> buf;

	//TODO sprawdzic nazwe tempFileName dla obrazow
	if (fileType=="png" || fileType=="jpg")	// save image
	{
		CLOG(LTRACE)<<"ViewWriter::insertFileIntoCollection, Image file";
		std::vector<uchar> buf;
		std::vector<int> params(2);
		params[0] = CV_IMWRITE_JPEG_QUALITY;
		params[1] = 95;
		cv::imencode(".jpg", tempImg, buf, params);
		b=BSONObjBuilder().genOID().appendBinData(tempFileName, buf.size(), mongo::BinDataGeneral, &buf[0]).append("filename", tempFileName).append("size", size).append("place", "document").append("extension", fileType).obj();
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
			b=BSONObjBuilder().genOID().appendBinData(tempFileName, totalSize, mongo::BinDataGeneral, &buff[0]).append("filename", tempFileName).append("size", totalSize).append("place", "document").append("extension", fileType).obj();
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
			b=BSONObjBuilder().genOID().appendBinData(tempFileName, totalSize, mongo::BinDataGeneral, &buff[0]).append("filename", tempFileName).append("size", totalSize).append("place", "document").append("extension", fileType).obj();
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
			b=BSONObjBuilder().genOID().appendBinData(tempFileName, totalSize, mongo::BinDataGeneral, &buff[0]).append("filename", tempFileName).append("size", totalSize).append("place", "document").append("extension", fileType).obj();
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
		input =cipFileIn.read()+" ";
		CLOG(LERROR)<<"Input:"<< input;

		// convert to char*
		char const *cipCharTable = input.c_str();
		int size = strlen(cipCharTable);
		CLOG(LERROR)<<string(cipCharTable);
		CLOG(LERROR)<<"Size: "<<size;
		// create bson object
		b = BSONObjBuilder().genOID().appendBinData(tempFileName, size, BinDataGeneral,  cipCharTable).append("filename", tempFileName).append("size", size).append("place", "document").append("extension", fileType).obj();

		// insert object into collection
		c->insert(dbCollectionPath, b);

		b.getObjectID(bsonElement);
		oid=bsonElement.__oid();
	}
	cloudType="";
}

float ModelWriter::getFileSize(const string& fileType, string& tempFileName)
{
	float size = 0.0;
	if (fileType=="png" || fileType=="jpg")
	{
		CLOG(LINFO)<<"Image!";
		tempImg = in_img.read();
		size = (float)tempImg.elemSize1()*(float)tempImg.total();//MB
		CLOG(LINFO)<<"Size of image file: " << size<<" MB";
	}
	else if(fileType=="pcd")	// save to file pcd
	{
		CLOG(LINFO)<<"Cloud!";

		if(cloudType=="xyzrgb")
			tempFileName=std::string(fileName) + std::string("_xyzrgb.pcd");
		else if(cloudType=="xyz")
			tempFileName=std::string(fileName) + std::string("_xyz.pcd");
		else if(cloudType=="xyzsift")
			tempFileName=std::string(fileName) + std::string("_xyzsift.pcd");

		size=sizeOfCloud;
		CLOG(LINFO) << "cloudType: "<< cloudType << endl;
		CLOG(LINFO)<<"Size of PCD cloud: " << size<<" MB";

	}
	else if(fileType=="txt")	// save to file pcd
	{
		CLOG(LINFO) << "CIP file";
		size=1.0;
		tempFileName=std::string(fileName) + std::string(".txt");
	}
	else
	{
		CLOG(LERROR)<<"I don't know such extension file :(";
		exit(1);
	}
	return size;
}

void ModelWriter::insert2MongoDB(const string &destination, const string&  modelOrViewName, const string&  nodeName, const string& fileType)
{
	CLOG(LERROR)<<"ModelWriter::insert2MongoDB";
	auto_ptr<DBClientCursor> cursorCollection;
	string source;
	int items=0;
	try{
		if(destination=="Object")
		{
			CLOG(LERROR)<<"Object";
			unsigned long long nr = c->count(dbCollectionPath, BSON("ObjectName"<<objectName<<"NodeName"<<"Object"),0,0,0);
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
			CLOG(LERROR)<<"Last Leaf";
			unsigned long long nr = c->count(dbCollectionPath, BSON("ObjectName"<<objectName<<"NodeName"<<"Object"),0,0,0);
			if(nr==0)
			{
				CLOG(LERROR) <<"Object does not exists in "<< dbCollectionPath;
				initObject();
				insert2MongoDB(destination, modelOrViewName, nodeName, fileType);
				return;
			}
			else
			{
				CLOG(LERROR)<<"Object now exist";
				int items = c->count(dbCollectionPath, BSON("ObjectName"<<objectName<<"NodeName"<<nodeName<<nodeName+"Name"<<modelOrViewName),1,1,1);
				if(items==0)
				{
					CLOG(LERROR)<<"No such model/view";
					CLOG(LERROR)<<"NodeName: "<<nodeName;
					initModel(modelOrViewName, true, nodeNameProp, objectName, description);
				}
				CLOG(LERROR)<<"Get document";
				cursorCollection = c->query(dbCollectionPath, Query(BSON("ObjectName"<<objectName<<"NodeName"<<nodeName<<nodeName+"Name"<<modelOrViewName)));
				BSONObj obj = cursorCollection->next();
				vector<OID> childsVector;
				// check if node has some files
				if(getChildOIDS(obj, "childOIDs", "childOID", childsVector)>0 && childsVector.size()>0)
				{
					CLOG(LTRACE)<<nodeName <<"There are some files in Mongo in this node!";
				}
			}
			CLOG(LINFO)<<"Write to model";
			writeNode2MongoDB(destination, nodeName, modelOrViewName, fileType);
		}
    }//try
	catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :<";
	}
}

} //: namespace ModelWriter
} //: namespace Processors
