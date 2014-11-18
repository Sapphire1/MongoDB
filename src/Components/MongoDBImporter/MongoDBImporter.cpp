/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

#include "MongoDBImporter.hpp"
#define siftPointSize 133
#define xyzPointSize 3
#define xyzrgbPointSize 4

namespace Processors {
namespace MongoDBImporter  {
using namespace cv;
using namespace mongo;
using namespace boost::property_tree;

MongoDBImporter::MongoDBImporter(const std::string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", string("localhost")),
		objectName("objectName", string("GreenCup")),
		collectionName("collectionName", string("containers")),
		nodeNameProp("nodeName", string("Object")),
		viewOrModelName("viewOrModelName", string("")),
		type("NodeName", string("")),
		folderName("folderName", string("/home/lzmuda/mongo_driver_tutorial/test/"))
{
		registerProperty(mongoDBHost);
		registerProperty(objectName);
		registerProperty(collectionName);
		registerProperty(nodeNameProp);
		registerProperty(viewOrModelName);
		registerProperty(folderName);
		registerProperty(type);
        CLOG(LTRACE) << "Hello MongoDBImporter";
}

MongoDBImporter::~MongoDBImporter()
{
        CLOG(LTRACE) << "Good bye MongoDBImporter";
}

void MongoDBImporter::readfromDB()
{
	CLOG(LNOTICE) << "MongoDBImporter::readfromDB";
	string nodeName=(string)nodeNameProp;
	string viewOrModName=(string)viewOrModelName;
	string typ=(string)type;
	readFromMongoDB(nodeName, viewOrModName, typ);
}
void MongoDBImporter::prepareInterface() {
        CLOG(LTRACE) << "MongoDBImporter::prepareInterface";

        h_readfromDB.setup(this, &MongoDBImporter::readfromDB);
        registerHandler("Read", &h_readfromDB);

//        registerStream("in_img", &in_img);
//        registerStream("out_img", &out_img);
//        addDependency("onNewImage", &in_img);
}

bool MongoDBImporter::onInit()
{
        CLOG(LTRACE) << "MongoDBImporter::initialize";
        if(collectionName=="containers")
        	dbCollectionPath="images.containers";
        string hostname = mongoDBHost;
        connectToMongoDB(hostname);
        return true;
}

bool MongoDBImporter::onFinish()
{
        CLOG(LTRACE) << "MongoDBImporter::finish";
        return true;
}

bool MongoDBImporter::onStep()
{
        CLOG(LTRACE) << "MongoDBImporter::step";
        return true;
}

bool MongoDBImporter::onStop()
{
        return true;
}

bool MongoDBImporter::onStart()
{
        return true;
}

void MongoDBImporter::getFileFromGrid(const GridFile& file, const string& modelOrViewName, const string& nodeName, const string& type)
{
	CLOG(LTRACE)<<"MongoDBImporter::getFileFromGrid";
	string filename;
	filename = file.getFileField("filename").str();
	// type in "View","Model"
	CLOG(LINFO)<<(string)folderName+type+"/"+modelOrViewName+"/"+nodeName+"/"+filename;
	string name = (string)folderName+type+"/"+modelOrViewName+"/"+nodeName+"/"+filename;
	//stringstream ss;
	//string str = ss.str();
	char *fileName = (char*)name.c_str();
	ofstream ofs(fileName);
	gridfs_offset off = file.write(ofs);
	if (off != file.getContentLength())
	{
		CLOG(LERROR) << "Failed to read a file from mongoDB";
	}
	else
	{
		CLOG(LTRACE) << "Success read a file from mongoDB";
	}
}

void MongoDBImporter::readFile(const string& modelOrViewName, const string& nodeName, const string& type, const OID& childOID)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<PointXYZSIFT>::Ptr new_cloud_xyzsift (new pcl::PointCloud<PointXYZSIFT>);
	int queryOptions = 0;
	const BSONObj *fieldsToReturn = 0;

	// get bson object from collection
	BSONObj obj = c->findOne(dbCollectionPath, Query(BSON("_id" << childOID)), fieldsToReturn, queryOptions);
	string featuresNumber = obj.getField("mean_viewpoint_features_number").str();;
	CLOG(LERROR)<<"obj: "<<obj<<", childOID: "<<childOID;
	CLOG(LERROR)<<"featuresNumber: "<<featuresNumber;
	string place = obj.getField("place").str();
	CLOG(LERROR) << obj.getField("size").str();
	CLOG(LERROR) << obj.getField("size").Int();
	//float sizeFloat = obj.getField("size").Double();
	string tempFileName;
	int size = obj.getField("size").Int();
	//if(place=="grid")
	tempFileName = obj.getField("filename").str();
	//else
	//	tempFileName = obj.getField("fileName").str();
	CLOG(LERROR)<<"place: "<<place<<" size: "<<size<<", filename: "<<tempFileName<<", type: "<<type;

	CLOG(LERROR)<<(string)folderName+type+"/"+modelOrViewName+"/"+nodeName+"/"+tempFileName;
	string newFileName = (string)folderName+type+"/"+modelOrViewName+"/"+nodeName+"/"+tempFileName;

	if(place=="document")
	{
		//readFromCollection();
		string extension = obj.getField("extension").str();
		if(extension=="jpg" || extension=="png")
		{
			CLOG(LERROR)<<"Read image\n";
			//BSONObj obj = c->findOne(dbCollectionPath, Query(BSON("_id" << childOID)), fieldsToReturn, queryOptions);
			int len;
			uchar *data = (uchar*)obj[tempFileName].binData(len);
			std::vector<uchar> v(data, data+len);
			CLOG(LERROR)<<*data;
			cv::Mat image = cv::imdecode(cv::Mat(v), -1);
			CLOG(LERROR)<<image.total();
			imwrite(newFileName, image );
		}
		else if(extension=="yml" || extension=="yaml")
		{
			CLOG(LERROR)<<"Read image\n";
			//BSONObj obj = c->findOne(dbCollectionPath, Query(BSON("_id" << childOID)), fieldsToReturn, queryOptions);
			int len;
			float *data = (float*)obj[tempFileName].binData(len);
			CLOG(LNOTICE)<<"len : "<<len;
			int width = obj.getField("width").Int();//480
			int height = obj.getField("height").Int();//640
			int channels = obj.getField("channels").Int();
			// if(channels==3)
			cv::Mat imageXYZRGB(width, height, CV_32FC3);
			//imageXYZRGB.convertTo(imageXYZRGB, CV_32FC4);
			vector<float> img;
			//rows = rows*3;
			float img_ptr[channels];
			for (int i = 0; i < width; ++i)
			{
				CLOG(LERROR)<<"i : "<<i;
				float* xyz_p = imageXYZRGB.ptr <float> (i);
				for (int j = 0; j < height*channels; j+=channels)
				{
					CLOG(LERROR)<<"i*height*channels+j: " <<i*height*channels+j;
					xyz_p[0+j]=data[i*height*channels+j];
					xyz_p[1+j]=data[i*height*channels+j+1];
					xyz_p[2+j]=data[i*height*channels+j+2];
				}
			}
			CLOG(LERROR)<<"222";
			cv::FileStorage fs(newFileName, cv::FileStorage::WRITE);
			CLOG(LERROR)<<"333";
			fs << "img" << imageXYZRGB;
			CLOG(LERROR)<<"444";
			fs.release();
			CLOG(LERROR)<<"555";
		}
		else if(extension=="pcd")
		{
			CLOG(LERROR)<<"pcd ";
			string cloudType;
			if (tempFileName.find("xyzrgb") != std::string::npos)
			{
				cloudType="xyzrgb";
			}
			else if (tempFileName.find("xyzsift") != std::string::npos)
			{
				cloudType="xyzsift";
			}
			else if (tempFileName.find("xyzshot") != std::string::npos)
			{
				cloudType="xyzshot";
			}
			else if (tempFileName.find("xyz") != std::string::npos)
			{
				CLOG(LERROR)<<"xyz";
				cloudType="xyz";
			}
			else
			{
				CLOG(LERROR)<<"Don't know such PC!!!";
			}
			try
			{
				// TODO typ chmury odczytywac z pliku zapisanego w bazie a nie z nazwy pliku!!!
				//BSONObj obj = c->findOne(dbCollectionPath, Query(BSON("_id" << childOID)), fieldsToReturn, queryOptions);

				const BSONObj *fieldsToReturn = 0;
				int queryOptions = 0;
				/*
				if(cloudType=="xyz")
				{
					BSONObj obj = c->findOne(dbCollectionPath, Query(BSON("_id" << childOID)), fieldsToReturn, queryOptions);
					// read data to buffer
					int totalSize;
					float* buffer = (float*)obj[tempFileName].binData(totalSize);
					int bufferSize = totalSize/sizeof(float);
					CLOG(LERROR)<<"bufferSize: "<<bufferSize;
					float newBuffer[bufferSize];
					memcpy(newBuffer, buffer, totalSize);
					pcl::PointXYZ pt;
					for(int i=0; i<totalSize/(xyzPointSize*sizeof(float)); i++) // now it should be 10 iterations
					{
						pt.x=newBuffer[i*xyzPointSize];
						pt.y=newBuffer[i*xyzPointSize+1];
						pt.z=newBuffer[i*xyzPointSize+2];

						cloud_xyz->push_back(pt);
					}
					// save to file, only in test purposes
					pcl::io::savePCDFile(newFileName, *new_cloud_xyz, false);

				}
				else
					*/
				if(cloudType=="xyzrgb")
				{
					// read data to buffer
					int totalSize;
					float* buffer = (float*)obj[tempFileName].binData(totalSize);
					int bufferSize = totalSize/sizeof(float);
					CLOG(LERROR)<<"bufferSize: "<<bufferSize;
					float newBuffer[bufferSize];

					memcpy(newBuffer, buffer, totalSize);
					pcl::PointXYZRGB pt;
					for(int i=0; i<totalSize/(xyzrgbPointSize*sizeof(float)); i++) // now it should be 10 iterations
					{
						pt.x=newBuffer[i*xyzrgbPointSize];
						pt.y=newBuffer[i*xyzrgbPointSize+1];
						pt.z=newBuffer[i*xyzrgbPointSize+2];
						pt.rgb=newBuffer[i*xyzrgbPointSize+3];
						new_cloud_xyzrgb->push_back(pt);
					}
					pcl::io::savePCDFile(newFileName, *new_cloud_xyzrgb, false);
				}
				else if(cloudType=="xyzsift")
				{
					// read data to buffer
					int totalSize;
					float* buffer = (float*)obj[tempFileName].binData(totalSize);
					int bufferSize = totalSize/sizeof(float);
					CLOG(LERROR)<<"bufferSize: "<<bufferSize;
					float newBuffer[bufferSize];

					memcpy(newBuffer, buffer, totalSize);
					// for sift row size in float is equal 128+5 =133  floats
					PointXYZSIFT pt;
					for(int i=0; i<totalSize/(siftPointSize*sizeof(float)); i++) // now it should be 10 iterations
					{
						Eigen::Vector3f pointCoordinates;
						pointCoordinates[0]=newBuffer[i*siftPointSize];
						pointCoordinates[1]=newBuffer[i*siftPointSize+1];
						pointCoordinates[2]=newBuffer[i*siftPointSize+2];
						memcpy(&pt.multiplicity, &newBuffer[3+i*siftPointSize], sizeof(int)); // 4 bytes
						memcpy(&pt.pointId, &newBuffer[4+i*siftPointSize], sizeof(int));	// 4 bytes
						memcpy(&pt.descriptor, &newBuffer[5+i*siftPointSize], 128*sizeof(float)); // 128 * 4 bytes = 512 bytes

						pt.getVector3fMap() = pointCoordinates;
						new_cloud_xyzsift->push_back(pt);
					}
					pcl::io::savePCDFile(newFileName, *new_cloud_xyzsift, false);
				}
				CLOG(LERROR)<<"ViewReader::readFile: END";
			}catch(Exception &ex){CLOG(LERROR)<<ex.what();}
		}//pcd
		else if(extension=="txt")
		{
			// read from collection
			const BSONObj *fieldsToReturn = 0;
			int queryOptions = 0;
			// get bson object
			//BSONObj obj = c->findOne(dbCollectionPath, Query(BSON("_id" << childOID)), fieldsToReturn, queryOptions);
			const char *buffer;

			// get data to buffer
			buffer = obj[tempFileName].binData(size);

			for (int i=0; i<size;i++)
				CLOG(LERROR)<<buffer[i];

			CLOG(LERROR)<<*buffer;
			CLOG(LERROR)<<"size: "<<size;
			string fromDB(buffer,size-1);
			char *fileName = (char*)newFileName.c_str();
			std::ofstream outfile (fileName);
			outfile.write (buffer,size);
		}
	}
	else if(place=="grid"){
		GridFS fs(*c,collectionName);
		CLOG(LTRACE)<<"_id"<<childOID;
		GridFile file = fs.findFile(Query(BSON("_id" << childOID)));
		if (!file.exists())
			CLOG(LERROR) << "File not found in grid";
		else
			getFileFromGrid(file, modelOrViewName, nodeName, type);
	}
}

void MongoDBImporter::readFromMongoDB(string& nodeName, string& modelOrViewName, string& type)
{
	CLOG(LTRACE)<<"MongoDBImporter::readFromMongoDB";
	string name;
	try{
		int items=0;
		findDocumentInCollection(*c, dbCollectionPath, objectName, nodeName, cursorCollection, modelOrViewName, type, items);

		if(items>0)
		{
			CLOG(LINFO)<<"Founded some data";
			while (cursorCollection->more())
			{
				BSONObj obj = cursorCollection->next();
				CLOG(LTRACE)<<obj;
				vector<OID> childsVector;
				int items =  getChildOIDS(obj, "childOIDs", "childOID", childsVector);
				if(items>0)
				{
					CLOG(LTRACE)<<"There are childs "<<childsVector.size();
					for (unsigned int i = 0; i<childsVector.size(); i++)
					{
						childCursor =c->query(dbCollectionPath, BSON("_id"<<childsVector[i]),0,0,0);
						if(childCursor->more())
						{
							BSONObj childObj = childCursor->next();
							string childNodeName= childObj.getField("NodeName").str();
							if(childNodeName!="EOO")
							{
								if(isViewLastLeaf(nodeName) || isModelLastLeaf(nodeName))
								{
									CLOG(LTRACE)<<"LastLeaf"<<" childNodeName "<<childNodeName;
									readFile(modelOrViewName, nodeName, type, childsVector[i]);
								}
								else if(childNodeName=="View" || childNodeName=="Model")
								{
									string newName;

									setModelOrViewName(childNodeName, childObj, newName);
									type=childNodeName;
									readFromMongoDB(childNodeName, newName, type);
								}//TODO usunac tego elsa!!!
								else if(childNodeName=="View")
								{
									string newName;
								}
								else
									readFromMongoDB(childNodeName, modelOrViewName, type);
							}
						}//if(childNodeName!="EOO")
					}//for
				}//if
			}//while
		}//if
		else
		{
			CLOG(LTRACE)<<"10";
			if(nodeNameProp==nodeName)
				CLOG(LERROR)<<"Wrong name";
			CLOG(LTRACE)<<"No results";
		}
	}//try
	catch(DBException &e)
	{
		CLOG(LERROR) <<"ReadFromMongoDB(). Something goes wrong... :<";
		exit(1);
	}
}
} //: namespace MongoDBImporter
} //: namespace Processors
