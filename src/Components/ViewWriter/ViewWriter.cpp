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
#include <cstring>

ViewWriter::ViewWriter(const string & name) : Base::Component(name),
	mongoDBHost("mongoDBHost", string("localhost")),
	objectName("objectName", string("GreenCup")),
	description("description", string("My green coffe cup")),
	collectionName("collectionName", string("containers")),
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
	registerProperty(viewNameProp);
	registerProperty(sceneNamesProp);
	registerProperty(fileName);
	registerProperty(nodeTypeProp);
	registerProperty(remoteFileName);
	registerProperty(binary);
	registerProperty(suffix);

	sizeOfCloud=0.0;
	CLOG(LTRACE) << "Hello ViewWriter";

}

ViewWriter::~ViewWriter()
{
	CLOG(LTRACE) << "Good bye ViewWriter";
}

void ViewWriter::prepareInterface() {
	CLOG(LTRACE) << "ViewWriter::prepareInterface";
	registerHandler("writeViewTXT2DB", boost::bind(&ViewWriter::writeTXT2DB, this));
	registerHandler("writeViewImage2DB", boost::bind(&ViewWriter::writeImage2DB, this));
	registerHandler("Write_xyz", boost::bind(&ViewWriter::Write_cloud<pcl::PointXYZ>, this));
	registerHandler("Write_xyzrgb", boost::bind(&ViewWriter::Write_cloud<pcl::PointXYZRGB>, this));
	registerHandler("Write_xyzsift", boost::bind(&ViewWriter::Write_cloud<PointXYZSIFT>, this));
	registerHandler("Write_xyzrgbsift", boost::bind(&ViewWriter::Write_cloud<PointXYZRGBSIFT>, this));

	registerStream("in_img", &in_img);
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_xyzrgbsift", &in_cloud_xyzrgbsift);
	registerStream("cipFileIn", &cipFileIn);

	addDependency("writeViewImage2DB", &in_img);
	addDependency("writeViewTXT2DB", &cipFileIn);
	addDependency("Write_xyzrgb", &in_cloud_xyzrgb);
	addDependency("Write_xyz", &in_cloud_xyz);
	addDependency("Write_xyzsift", &in_cloud_xyzsift);
	addDependency("Write_xyzrgbsift", &in_cloud_xyzsift);
}

void ViewWriter::writeTXT2DB()
{
	CLOG(LNOTICE) << "ViewWriter::writeTXT2DB";
	string sceneNames = sceneNamesProp;
	boost::split(MongoBase::splitedSceneNames, sceneNames, is_any_of(","));
	string fileType = "txt";
	if(viewNameProp!="")
		insert2MongoDB(nodeTypeProp,viewNameProp, "View", fileType);
	else
		CLOG(LERROR)<<"Add view name and try again";
}

void ViewWriter::writeImage2DB()
{
	CLOG(LNOTICE) << "ViewWriter::writeImage2DB";
	string sceneNames = sceneNamesProp;
	boost::split(MongoBase::splitedSceneNames, sceneNames, is_any_of(","));
	string fileType = "png";
	if(viewNameProp!="")
		insert2MongoDB(nodeTypeProp,viewNameProp, "View", fileType);
	else
		CLOG(LERROR)<<"Add view name and try again";
}

void ViewWriter::writePCD2DB()
{
	CLOG(LNOTICE) << "ViewWriter::writePCD2DB";
	string sceneNames = sceneNamesProp;
	boost::split(MongoBase::splitedSceneNames, sceneNames, is_any_of(","));
	CLOG(LERROR)<<"cloudType: "<<cloudType;
	string fileType = "pcd";
	if(viewNameProp!="")
		insert2MongoDB(nodeTypeProp,viewNameProp, "View", fileType);
	else
		CLOG(LERROR)<<"Add view name and try again";
}

template <class PointT>
void ViewWriter::Write_cloud()
{
	CLOG(LTRACE) << "ViewWriter::Write_cloud()";
	std::vector< pcl::PCLPointField> fields;
	pcl::getFields<PointT>(fields);

	if(typeid(PointT) == typeid(pcl::PointXYZ))
	{
		cloudType="xyz";
		cloudXYZ = in_cloud_xyz.read();
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeOfCloud+=(float)pcl::getFieldSize(it->datatype)*cloudXYZ->size();
		sizeOfCloud/=(1000*1000);
	}
	else if(typeid(PointT) == typeid(pcl::PointXYZRGB))
	{
		cloudType="xyzrgb";
		cloudXYZRGB = in_cloud_xyzrgb.read();
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeOfCloud+=(float)pcl::getFieldSize(it->datatype)*cloudXYZRGB->size();
		sizeOfCloud/=(1000*1000);
	}
	else if(typeid(PointT) == typeid(PointXYZSIFT))
	{
		cloudType="xyzsift";
		cloudXYZSIFT = in_cloud_xyzsift.read();

		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeOfCloud+=(float)pcl::getFieldSize(it->datatype)*cloudXYZSIFT->size();
		sizeOfCloud/=(1000*1000);
	}
	else if(typeid(PointT) == typeid(PointXYZSIFT))
	{
		cloudType="xyzrgbsift";
		cloudXYZRGBSIFT = in_cloud_xyzrgbsift.read();
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeOfCloud+=(float)pcl::getFieldSize(it->datatype)*cloudXYZRGBSIFT->size();
		sizeOfCloud/=(1000*1000);
	}
	CLOG(LINFO)<<"CloudType: "<<cloudType;
	CLOG(LINFO)<<"PointCloudSize = "<< sizeOfCloud;
	if(sizeOfCloud>15)
		CLOG(LERROR)<<"File should be written to GRIDFS!";
	writePCD2DB();

}

void ViewWriter::saveXYZFileOnDisc()
{
	std::string fn = fileName;
	if(suffix)
	{
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyz.pcd");
	}
	CLOG(LINFO) <<"FileName:"<<fn;
	pcl::io::savePCDFile(fn, *cloudXYZ, binary);
}

void ViewWriter::saveXYZRGBFileOnDisc()
{
	std::string fn = fileName;
	if(suffix)
	{
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyzrgb.pcd");
	}
	CLOG(LINFO) <<"FileName:"<<fn;
	pcl::io::savePCDFile(fn, *cloudXYZRGB, binary);
}

void ViewWriter::saveXYZSIFTFileOnDisc()
{
	std::string fn = fileName;
	if(suffix)
	{
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyzsift.pcd");
	}
	CLOG(LINFO) <<"FileName:"<<fn;
	pcl::io::savePCDFile(fn, *cloudXYZSIFT, binary);
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
			MongoBase::dbCollectionPath=dbCollectionPath="images.containers";
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
		initModel(*it, true, nodeTypeProp, objectName, description);
	else if(type=="View")
		initView(*it, true, nodeTypeProp, objectName, description);
}

void ViewWriter::initObject()
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

	}
	  catch(DBException &e)
	  {
		CLOG(LERROR) <<"Something goes wrong... :<";
		CLOG(LERROR) <<c->getLastError();
	  }
}

void ViewWriter::insertFileIntoGrid(OID& oid, const string& fileType, string& tempFileName)
{
	try{
		BSONObj object;
		BSONElement bsonElement;
		string mime="";
		setMime(fileType, mime);
		std::stringstream time;
		CLOG(LNOTICE)<<"File should be written to GRIDFS!";

		// save file on disc in order to write it to gridFS
		if (fileType=="png" || fileType=="jpg")
		{
			tempFileName = string(fileName)+"."+string(fileType);
			cv::imwrite(tempFileName, tempImg);
		}
		else if(fileType=="txt")	// save to file pcd
		{
			CLOG(LINFO) << "CIP file";
			string CIP = cipFileIn.read();
			tempFileName = string(fileName)+"."+string(fileType);
			char const* ca = tempFileName.c_str();
			std::ofstream out(ca);
			out << CIP;
			out.close();
			CLOG(LINFO)<<"Size of CIP is lower then 16 MB: ";
		}
		else if(fileType=="pcd")
		{
			CLOG(LINFO) << "PCD file";
			if(cloudType=="xyz")
				saveXYZFileOnDisc();
			else if(cloudType=="xyzrgb")
				saveXYZRGBFileOnDisc();
			else if(cloudType=="xyzsift")
				saveXYZSIFTFileOnDisc();
		}
		else
		{
			CLOG(LERROR)<<"I don't know such extension file :(";
			exit(1);
		}
		CLOG(LERROR)<<"cloudType: "<<cloudType;


		// TODO add XYZ mat <float> 3C - CV_32F3C, or sth similiar

		CLOG(LINFO) << "tempFileName: "<< tempFileName << endl;
		boost::posix_time::time_facet *facet = new boost::posix_time::time_facet("%d_%m_%Y_%H_%M_%S");
		time.imbue(locale(cout.getloc(), facet));
		time<<second_clock::local_time();
		CLOG(LINFO) << "Time: "<< time.str() << endl;

		// create GridFS client
		GridFS fs(*c, collectionName);
		string fileNameInMongo;
		if(cloudType!="")
			fileNameInMongo = (string)remoteFileName+"_"+ cloudType + time.str()+"."+string(fileType);
		else
			fileNameInMongo = (string)remoteFileName + time.str()+"."+string(fileType);
		CLOG(LERROR)<<"tempFileName: "<<tempFileName;

		// save in grid
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

		c->createIndex(dbCollectionPath, BSON("filename"<<1));
	}catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :<";
		CLOG(LERROR) <<c->getLastError();
		CLOG(LERROR) << e.what();
	}
}
int ViewWriter::getFileSize(const string& fileType, string& tempFileName)
{
	float size = 0.0;
	if (fileType=="png" || fileType=="jpg")
	{
		CLOG(LINFO)<<"Image!";
		tempImg = in_img.read();
		size = (float)tempImg.elemSize1()*(float)tempImg.total();
		size = size/(1024*1024);
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

void ViewWriter::writeNode2MongoDB(const string &destination, const string &type,string modelOrViewName, const string& fileType)
{
	CLOG(LTRACE) <<"writeNode2MongoDB";
	OID oid;
	CLOG(LERROR)<<"ViewWriter::writeNode2MongoDB, cloudType: "<<cloudType;
	CLOG(LTRACE) <<"Filename: " << fileName << " destination: "<< destination<<" dbCollectionPath: "<<dbCollectionPath;
    try{
    	string tempFileName="";
    	int sizeOfFile = getFileSize(fileType, tempFileName);
    	//TODO odwrocic ta relacje
    	if(sizeOfFile<15)
    	{
    	//	CLOG(LERROR)<<"ViewWriter::writeNode2MongoDB, cloudType: "<<cloudType;
		//	insertFileIntoGrid(oid, fileType, tempFileName);
		//	c->update(dbCollectionPath, QUERY("ObjectName"<<objectName<<type+"Name"<<modelOrViewName<<"Type"<<destination), BSON("$addToSet"<<BSON("childOIDs"<<BSON("childOID"<<oid.str()))), false, true);
    	}
    	//TODO dodac tego else'a!!!
		//else
			CLOG(LERROR)<<"ViewWriter::writeNode2MongoDB, cloudType: "<<cloudType;
			insertFileIntoCollection(oid, fileType, tempFileName, sizeOfFile);
			CLOG(LERROR)<<"UPDATE: "<<oid.str();
			c->update(dbCollectionPath, QUERY("ObjectName"<<objectName<<type+"Name"<<modelOrViewName<<"Type"<<destination), BSON("$addToSet"<<BSON("childOIDs"<<BSON("childOID"<<oid.str()))), false, true);

		CLOG(LTRACE) <<"File saved successfully";
    }
	catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :<";
		CLOG(LERROR) <<c->getLastError();
	}
}

void ViewWriter::insertFileIntoCollection(OID& oid, const string& fileType, const string& tempFileName, int size)
{
	CLOG(LTRACE)<<"ViewWriter::insertFileIntoCollection";
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
		b=BSONObjBuilder().genOID().appendBinData(tempFileName, buf.size(), mongo::BinDataGeneral, &buf[0]).append("place", "collection").append("extension", fileType).obj();
		b.getObjectID(bsonElement);
		oid=bsonElement.__oid();
		c->insert(dbCollectionPath, b);
	}
	else if(fileType=="pcd")	// save cloud
	{
		std::stringstream compressedData;
		compressedData.str("");
		bool showStatistics = true;
		pcl::io::compression_Profiles_e compressionProfile = pcl::io::HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
		if(cloudType!="xyzsift")
		{
			// cloud encoding
			if(cloudType=="xyzrgb")
			{
				CLOG(LERROR)<<"WriteXYZRGB!";
				pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder =
						new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> (compressionProfile, showStatistics);
				PointCloudEncoder->encodePointCloud (cloudXYZRGB, compressedData);
				delete(PointCloudEncoder);
			}
			else if(cloudType=="xyz")
			{
				pcl::io::OctreePointCloudCompression<pcl::PointXYZ>* PointCloudEncoder =
						new pcl::io::OctreePointCloudCompression<pcl::PointXYZ> (compressionProfile, showStatistics);
				PointCloudEncoder->encodePointCloud (cloudXYZ, compressedData);
				delete(PointCloudEncoder);
			}

			//get size of compressed cloud
			compressedData.seekp(0, ios::end);
			stringstream::pos_type size = compressedData.tellp();

			// convert size to int
			int sizeInt = size;

			//build bson object
			b = BSONObjBuilder().genOID().appendBinData(tempFileName, size, BinDataGeneral,  &compressedData).append("place", "collection").append("extension", fileType).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();

			// insert object to collection
			c->insert(dbCollectionPath, b);

			// create index on place field
			c->createIndex(dbCollectionPath, BSON("place"<<1));

			// cloud encoding
			try{
				pcl::io::OctreePointCloudCompression<pcl::PointXYZ>* PointCloudDecoderXYZ;
				pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudDecoderXYZRGB;
				int queryOptions = 0;
				const BSONObj *fieldsToReturn = 0;

				// get bson object from collection
				BSONObj obj = c->findOne(dbCollectionPath, QUERY("_id" << oid), fieldsToReturn, queryOptions);

				// read data to buffor
				stringstream* buff= (stringstream *)obj[tempFileName].binData(sizeInt);

				if(cloudType=="xyz")
				{
					PointCloudDecoderXYZ=new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>();
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ (new pcl::PointCloud<pcl::PointXYZ>);
					PointCloudDecoderXYZ->decodePointCloud (*buff, cloudXYZ);
					pcl::io::savePCDFile("newCloud2.pcd", *cloudXYZ, binary);
				}
				else if(cloudType=="xyzrgb")
				{
					PointCloudDecoderXYZRGB=new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>();
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
					PointCloudDecoderXYZRGB->decodePointCloud (*buff, cloudXYZRGB);
					pcl::io::savePCDFile("newCloud2.pcd", *cloudXYZRGB, binary);
				}
				CLOG(LERROR)<<"ViewWriter::insertFileIntoCollection: END";
			}catch(Exception &ex){CLOG(LERROR)<<ex.what();}
		}
		//TODO dodac SHOT'Y
		else if(cloudType=="xyzsift")
		{
			pcl::PCLPointCloud2 msg;
			size_t cloudSize=0;

			//convert pcl cloud to PCLPointCloud2 blob
			pcl::toPCLPointCloud2(*cloudXYZSIFT, msg);

			// get fields
			std::vector< pcl::PCLPointField> fields;
			pcl::getFields<PointXYZSIFT>(fields);

			// count cloud size
			for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
				cloudSize+=(float)pcl::getFieldSize(it->datatype)*cloudXYZSIFT->size();

			static const size_t fieldSize = sizeof(pcl::PCLPointField);

			// count full size of PCLPointCloud2 object
			int fullSizeBytes = sizeof(msg.height)+sizeof(msg.width)
					+sizeof(msg.is_bigendian)+sizeof(msg.point_step)+sizeof(msg.row_step)
					+sizeof(msg.is_dense)+sizeof(msg.header)+cloudSize+sizeof(msg.fields)+fieldSize*6+2
					+sizeof(msg.data)+msg.header.frame_id.size();

			CLOG(LNOTICE)<<"fullSizeBytes: "<<fullSizeBytes<< " B";

			// build bson object
			b = BSONObjBuilder().genOID().appendBinData(tempFileName, fullSizeBytes, BinDataGeneral, &msg).append("place", "collection").append("extension", fileType).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();

			// insert object into collection
			c->insert(dbCollectionPath, b);

			// read object
			const BSONObj *fieldsToReturn = 0;
			int queryOptions = 0;

			// get bson object
			BSONObj obj = c->findOne(dbCollectionPath, QUERY("_id" << oid), fieldsToReturn, queryOptions);
			pcl::PCLPointCloud2* msg2;

			// read data to buffor
			msg2 = (pcl::PCLPointCloud2*) obj[tempFileName].binData(fullSizeBytes);

			pcl::PointCloud<PointXYZSIFT>::Ptr cloudXYZSIFT2 (new pcl::PointCloud<PointXYZSIFT>);

			// convert PointCloud2 to cloud
			fromPCLPointCloud2 (*msg2, *cloudXYZSIFT2);

			//TODO zmienic to!
			string newCloud = "newCloud.pcd";
			CLOG(LNOTICE)<< "Cloud size2: " <<  cloudXYZSIFT2->size();

			// save cloud into file, its temporary, only test purposes
			pcl::io::savePCDFile(newCloud, *cloudXYZSIFT2, binary);
		}
		CLOG(LTRACE)<<"ViewWriter::insertFileIntoCollection, PCD file end";
	}
	else if(fileType=="txt")
	{
		CLOG(LTRACE)<<"ViewWriter::insertFileIntoCollection, txt file";
		string input;

		// read string from input
		input =cipFileIn.read();
		int size = input.size();

		// convert to char*
		// TODO check if it's neccessary
		char const *cipCharTable = input.c_str();

		// create bson object
		b = BSONObjBuilder().genOID().appendBinData(tempFileName, size, BinDataGeneral,  cipCharTable).append("place", "collection").append("extension", fileType).obj();

		// insert object into collection
		c->insert(dbCollectionPath, b);

		b.getObjectID(bsonElement);
		oid=bsonElement.__oid();

		// read from collection
		const BSONObj *fieldsToReturn = 0;
		int queryOptions = 0;

		// get bson object
		BSONObj obj = c->findOne(dbCollectionPath, QUERY("_id" << oid), fieldsToReturn, queryOptions);

		const char *buffer;

		// get data to buffer
		buffer = obj[tempFileName].binData(size);

		CLOG(LERROR)<<*buffer;
		CLOG(LERROR)<<"size: "<<size;
	}
	cloudType="";
}

void ViewWriter::insert2MongoDB(const string &destination, const string&  modelOrViewName, const string&  type,  const string&  fileType)
{
	CLOG(LINFO)<<"ViewWriter::insert2MongoDB";
	auto_ptr<DBClientCursor> cursorCollection;
	string source;
	int items=0;
	CLOG(LERROR)<<"cloudType: "<<cloudType;
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
				insert2MongoDB(destination, modelOrViewName, type, fileType);
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
					initView(modelOrViewName, true, nodeTypeProp, objectName, description);
				}
				cursorCollection = c->query(dbCollectionPath, QUERY("ObjectName"<<objectName<<"Type"<<type<<type+"Name"<<modelOrViewName));
				BSONObj obj;
				if(cursorCollection->more())
					obj = cursorCollection->next();
				else
				{
					CLOG(LERROR)<<"No objects views/models";
				}
				vector<OID> childsVector;
				// check if node has some files
				if(getChildOIDS(obj, "childOIDs", "childOID", childsVector)>0 && childsVector.size()>0)
				{
					CLOG(LTRACE)<<type <<"There are some files in Mongo in this node!";
				}
			}
			CLOG(LINFO)<<"Write to view";
			writeNode2MongoDB(destination, type, modelOrViewName, fileType);
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
											insert2MongoDB(childNodeName, newName, type, fileType);
										}
										else
											insert2MongoDB(childNodeName, modelOrViewName, type, fileType);
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
		exit(1);
	}
}

} //: namespace ViewWriter
} //: namespace Processors
