/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */
#include "ModelReader.hpp"

#define siftPointSize 133
#define xyzPointSize 3
#define xyzrgbPointSize 4

namespace Processors {
namespace ModelReader  {
using namespace cv;
using namespace mongo;
using namespace boost::property_tree;

ModelReader::ModelReader(const std::string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", string("localhost")),
		objectName("objectName", string("GreenCup")),
		collectionName("collectionName", string("containers")),
		nodeNameProp("nodeName", string("Object")),
		viewOrModelName("viewOrModelName", string("lab012")),
		//type("type", string("")),
		modelType("modelType", string("SOM"))
//		modelType("modelType", string("SSOM"))
{
		registerProperty(mongoDBHost);
		registerProperty(objectName);
		registerProperty(collectionName);
		registerProperty(nodeNameProp);
		registerProperty(viewOrModelName);
		registerProperty(modelType);
        CLOG(LTRACE) << "Hello ModelReader";
        if(nodeNameProp=="Object")
        	type="";
        else
        	type="Model";
}

ModelReader::~ModelReader()
{
        CLOG(LTRACE) << "Good bye ModelReader";
}

void ModelReader::readfromDB()
{
	CLOG(LNOTICE) << "ModelReader::readfromDB";
	readFromMongoDB(nodeNameProp, viewOrModelName, type);
}
void ModelReader::prepareInterface() {
	CLOG(LTRACE) << "ModelReader::prepareInterface";
	h_readfromDB.setup(this, &ModelReader::readfromDB);
	registerHandler("Read", &h_readfromDB);

	registerStream("out_cloud_xyz", &out_cloud_xyz);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	registerStream("out_img", &out_img);
	registerStream("in_trigger", &in_trigger);
	registerHandler("onTriggeredReadAllFiles", boost::bind(&ModelReader::readAllFilesTriggered, this));
	addDependency("onTriggeredReadAllFiles", &in_trigger);
}

bool ModelReader::onInit()
{
	CLOG(LTRACE) << "ModelReader::initialize";
	if(collectionName=="containers")
		dbCollectionPath="images.containers";

	name_cloud_xyz="";
	name_cloud_xyzrgb="";
	name_cloud_xyzsift="";

	string hostname = mongoDBHost;
	connectToMongoDB(hostname);
	if(collectionName=="containers")
		MongoBase::dbCollectionPath=dbCollectionPath="images.containers";

	return true;
}

bool ModelReader::onFinish()
{
        CLOG(LTRACE) << "ModelReader::finish";
        return true;
}

bool ModelReader::onStep()
{
        CLOG(LTRACE) << "ModelReader::step";
        return true;
}

bool ModelReader::onStop()
{
        return true;
}

bool ModelReader::onStart()
{
        return true;
}

void ModelReader::addToAllChilds(std::vector<OID> & childsVector)
{
	CLOG(LTRACE)<<"ModelReader::addToAllChilds";
	allChildsVector+=childsVector;
}

void ModelReader::readAllFilesTriggered()
{

	CLOG(LTRACE)<<"ModelReader::readAllFilesTriggered";
	std::vector<AbstractObject*> models;
	for(std::vector<OID>::iterator it = allChildsVector.begin(); it != allChildsVector.end(); ++it)
	{
		//przerobic zeby czytalo tez bezposrednio z dokumentow...
		readFile(*it, models);
	}
	CLOG(LTRACE)<<"Send models to sink";
	out_cloud_xyzrgb.write(cloud_xyzrgb);
	out_models.write(models);
}

void ModelReader::ReadPCDCloudFromFile(const string& filename, const string& tempFile)
{
	CLOG(LTRACE) << "ModelReader::ReadPCDCloud";
	// Try to read the cloud of XYZRGB points.
	if(filename.find("xyzrgb")!=string::npos)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (tempFile, *cloud_xyzrgb) == -1){
			CLOG(LWARNING) <<"Cannot read PointXYZRGB cloud from "<<tempFile;
		}else{
			//out_cloud_xyzrgb.write(cloud_xyzrgb);
			CLOG(LINFO) <<"PointXYZRGB cloud loaded properly from "<<tempFile;
			//return;
		}// else
	}

	// Try to read the cloud of XYZSIFT points.


	 if(filename.find("xyzsift.pcd")!=string::npos)
	 {
		pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift (new pcl::PointCloud<PointXYZSIFT>);
		if (pcl::io::loadPCDFile<PointXYZSIFT> (tempFile, *cloud_xyzsift) == -1){
			CLOG(LWARNING) <<"Cannot read PointXYZSIFT cloud from "<<tempFile;
		}else{
			//out_cloud_xyzsift.write(cloud_xyzsift);
			CLOG(LINFO) <<"PointXYZSIFT cloud loaded properly from "<<tempFile;

			//return;
		}// else
	}

	else if(filename.find("xyz")!=string::npos)
		{
			// Try to read the cloud of XYZ points.
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
			if (pcl::io::loadPCDFile<pcl::PointXYZ> (tempFile, *cloud_xyz) == -1){
				CLOG(LWARNING) <<"Cannot read PointXYZ cloud from "<<tempFile;
			}else{
				//out_cloud_xyz.write(cloud_xyz);
				CLOG(LINFO) <<"PointXYZ cloud loaded properly from "<<tempFile;
				//return;
			}// else
		}
}

void ModelReader::loadModels(string& name_cloud, string& features_number, std::vector<AbstractObject*>& models) {
	CLOG(LTRACE) << "SOMJSONReader::loadModels()";

	model_name = nodeNameProp;
	if(name_cloud.find("xyzrgb")!=string::npos)
		name_cloud_xyzrgb = name_cloud;
	else if(name_cloud.find("xyzsift")!=string::npos)
	{
		istringstream (features_number) >> mean_viewpoint_features_number;
		name_cloud_xyzsift = name_cloud;
	}
	else if(name_cloud.find("xyz")!=string::npos)
		name_cloud_xyz = name_cloud;

	CLOG(LDEBUG) << "name_cloud_xyzrgb:" << name_cloud_xyzrgb;
	CLOG(LDEBUG) << "name_cloud_xyzsift:" << name_cloud_xyzsift;
	CLOG(LDEBUG) << "name_cloud_xyz:" << name_cloud_xyz;

	// Create SOModel and add it to list.
	if(name_cloud_xyzsift!="" && name_cloud_xyzrgb!="")
	{
		CLOG(LDEBUG) << "Create model";
		SIFTObjectModel* model;
		//TODO sprawdzic czy przypadkiem nie trzeba tworzyc modeli typu SSOM, to generuje tylko SOM
		model = dynamic_cast<SIFTObjectModel*>(produce());
		models.push_back(model);
		name_cloud_xyzsift="";
		name_cloud_xyzrgb="";
	}
}

void ModelReader::writeToSink(string& mime, string& tempFilename, string& fileName)
{
	if(mime=="image/png" || mime=="image/jpeg")
	{
		// read from disc
		cv::Mat image = imread(tempFilename, CV_LOAD_IMAGE_UNCHANGED);
		out_img.write(image);
	}
	else if(mime=="text/plain")
	{
		CLOG(LINFO)<<"mime==text/plain";
		CLOG(LINFO)<<"fileName.find(pcd): "<<fileName.find("pcd");
		if((fileName.find("pcd"))!=string::npos)
		{
			CLOG(LINFO)<<"pcd :)";
			ReadPCDCloudFromFile(fileName, tempFilename);
		}
		else if(fileName.find("txt")!=string::npos)
		{
			char *tempFileName = (char*)tempFilename.c_str();
		    std::ifstream ifs(tempFileName); // open a file

		    std::string str((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
		    //TODO przetestowac czy to dziala
		    cipFileOut.write(str);
		}
		else
			CLOG(LERROR)<<"Nie wiem co to za plik :/";
	}
}

void ModelReader::readFile(const OID& childOID, std::vector<AbstractObject*>& models)
{
	CLOG(LTRACE)<<"ModelReader::readFile";
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
	int size = obj.getField("size").Int();
	string tempFileName = obj.getField("fileName").str();
	CLOG(LERROR)<<"place: "<<place<<" size: "<<size<<", fileName: "<<tempFileName;

	if(place=="collection")
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
			out_img.write(image);

			// only in test purposes, it's to remove
			imwrite( "Gray_Image.jpg", image );
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
				/*if(cloudType=="xyz")
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
					pcl::io::savePCDFile("newCloudXYZ.pcd", *cloudXYZ, false);
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
					cloud_xyzrgb=new_cloud_xyzrgb->makeShared();
					// save to file, only in test purposes
					pcl::io::savePCDFile("newCloudXYZRGB.pcd", *cloud_xyzrgb, false);
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
					cloud_xyzsift=new_cloud_xyzsift->makeShared();
					// save to file, only in test purposes
					pcl::io::savePCDFile("newCloudSIFT.pcd", *cloud_xyzsift, false);
				}
				CLOG(LERROR)<<"cloud_xyzrgb->width: "<<cloud_xyzrgb->width;
				loadModels(tempFileName, featuresNumber, models);
				//out_cloud_xyzrgb.write(cloud_xyzrgb);
				CLOG(LERROR)<<"ViewReader::readFile: END";
			}catch(Exception &ex){CLOG(LERROR)<<ex.what();}
		}//pcd
		//TODO sprawdzic czemu czasami nie zapetla sie na samych plikach txt!!!
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
			CLOG(LERROR)<<"ReadedFile: \n"<<fromDB;
			CLOG(LERROR)<<"Save to sink";
			cipFileOut.write(fromDB);
		}

	}
	else
	{

		GridFS fs(*c,collectionName);
		CLOG(LTRACE)<<"_id"<<childOID;
		GridFile file = fs.findFile(Query(BSON("_id" << childOID)));
		if (!file.exists())
		{
			CLOG(LERROR) << "File not found in grid";
		}
		else
		{
			// get filename
			string filename = file.getFileField("filename").str();
			string featuresNumber;
			if((filename.find("sift"))!=string::npos)
				featuresNumber = file.getFileField("mean_viewpoint_features_number").str();
			// get mime from file
			string mime = file.getContentType();
			string tempFile = "tempFile";
			getFileFromGrid(file, tempFile);
			//TODO nazwa tej metody jest mylaca - poprawic!!!
			writeToSink(mime, tempFile, filename);
			CLOG(LTRACE)<<"Add to model";
			loadModels(filename, featuresNumber, models);
		}
	}
}

void ModelReader::readFromMongoDB(const string& nodeName, const string& modelOrViewName, const string& type)
{
	CLOG(LTRACE)<<"ModelReader::readFromMongoDB";
	string name;
	std::vector<AbstractObject*> models;
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
				// if node has a child
				if(items>0)
				{
					if(isViewLastLeaf(nodeName) || isModelLastLeaf(nodeName))
						addToAllChilds(childsVector);
					else
					{
						CLOG(LTRACE)<<"There are childs "<<childsVector.size();
						for (unsigned int i = 0; i<childsVector.size(); i++)
						{
							childCursor =c->query(dbCollectionPath, (Query(BSON("_id"<<childsVector[i]))));
							if(childCursor->more())
							{
								BSONObj childObj = childCursor->next();
								string childNodeName= childObj.getField("Type").str();
								CLOG(LTRACE)<<childNodeName;
								if(childNodeName!="EOO")
								{
									if(childNodeName=="Model")
									{
										CLOG(LTRACE)<<"setModelOrViewName";
										string newName;
										setModelOrViewName(childNodeName, childObj, newName);
										readFromMongoDB(childNodeName, newName, type);
									}
									else if(childNodeName=="View")
									{
										CLOG(LTRACE)<<"It's a view. Do nothing.";
									}
									else if(childNodeName=="SOM" || childNodeName=="SSOM")
									{
										if(modelType==childNodeName)
										{
											CLOG(LINFO)<<"modelType==childNodeName";
											readFromMongoDB(childNodeName, modelOrViewName, type);
										}
									}
									else
										readFromMongoDB(childNodeName, modelOrViewName, type);
								}
							}//if(childNodeName!="EOO")
						}//for
					}
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
		/*
		// stworz model i wyslij do sinka, jesli przegladanie drzewa jest zakonczone
		if(nodeNameProp==nodeName)
		{
			readAllFilesTriggered();
		}
		*/
	}//try
	catch(DBException &e)
	{
		CLOG(LERROR) <<"ReadFromMongoDB(). Something goes wrong... :<";
		e.what();
		exit(1);
	}
}
} //: namespace ModelReader
} //: namespace Processors
