/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

#include "ViewReader.hpp"


namespace Processors {
namespace ViewReader  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace boost::posix_time;

ViewReader::ViewReader(const std::string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", string("localhost")),
		objectName("objectName", string("GreenCup")),
		collectionName("collectionName", string("containers")),
		nodeTypeProp("nodeType", string("StereoLR")),
		viewOrModelName("viewName", string("lab012")),
		type("type", string("View"))
{
		registerProperty(mongoDBHost);
		registerProperty(objectName);
		registerProperty(collectionName);
		registerProperty(nodeTypeProp);
		registerProperty(viewOrModelName);
		registerProperty(type);
		this->position=0;
        CLOG(LTRACE) << "Hello ViewReader";
}

ViewReader::~ViewReader()
{
        CLOG(LTRACE) << "Good bye ViewReader";
}

void ViewReader::readfromDB()
{
	CLOG(LNOTICE) << "ViewReader::readfromDB";
	readFromMongoDB(nodeTypeProp, viewOrModelName, type);
}
void ViewReader::prepareInterface() {
        CLOG(LTRACE) << "ViewReader::prepareInterface";
        h_readfromDB.setup(this, &ViewReader::readfromDB);
        registerHandler("Read", &h_readfromDB);
        registerStream("out_cloud_xyz", &out_cloud_xyz);
        registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
        registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
        registerStream("out_img", &out_img);
        registerStream("in_trigger", &in_trigger);
        registerStream("cipFileOut", &cipFileOut);
		registerHandler("onTriggeredReadAllFiles", boost::bind(&ViewReader::readAllFilesTriggered, this));
		addDependency("onTriggeredReadAllFiles", &in_trigger);
}

bool ViewReader::onInit()
{
        CLOG(LTRACE) << "ViewReader::initialize";
        if(collectionName=="containers")
      		MongoBase::dbCollectionPath=dbCollectionPath="images.containers";
        string hostname = mongoDBHost;
        connectToMongoDB(hostname);
        return true;
}

bool ViewReader::onFinish()
{
        CLOG(LTRACE) << "ViewReader::finish";
        return true;
}

bool ViewReader::onStep()
{
        CLOG(LTRACE) << "ViewReader::step";
        return true;
}

bool ViewReader::onStop()
{
        return true;
}

bool ViewReader::onStart()
{
        return true;
}

void ViewReader::addToAllChilds(std::vector<OID> & childsVector)
{
	CLOG(LTRACE)<<"ViewReader::addToAllChilds";
	allChildsVector+=childsVector;
}

void ViewReader::cloudEncoding(OID& oid, string& tempFileName, string & cloudType)
{
	try{
		pcl::io::OctreePointCloudCompression<pcl::PointXYZ>* PointCloudDecoderXYZ;
		pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudDecoderXYZRGB;
		int queryOptions = 0;
		const BSONObj *fieldsToReturn = 0;
		CLOG(LERROR)<<"dbCollectionPath: "<<dbCollectionPath<<"\n\n";
		// get bson object from collection
		BSONObj obj = c->findOne(dbCollectionPath, QUERY("_id" << oid), fieldsToReturn, queryOptions);
		CLOG(LERROR)<<"2\n";
		// read data to buffer
		CLOG(LERROR)<<"3"<<"\n\n";
		int readSize;
		CLOG(LERROR)<<obj<<"\n";
		int size=21473;
		std::stringstream* newTable = new std::stringstream("jajko");

		newTable->str("jajko2");
		CLOG(LERROR)<<newTable->str();
		CLOG(LERROR)<<"obj[tempFileName]: "<<obj[tempFileName]<<"\n";;
		CLOG(LERROR)<<"obj[tempFileName].binData(readSize): "<< (std::stringstream*)obj[tempFileName].binData(readSize)<<"\n";
		const char* charPtr= (const char*)(obj[tempFileName].binData(readSize));
		char temp[readSize];
		memcpy(temp, charPtr, readSize);
		cout<<"Readed: "<<readSize<<" B.";
		cout<<"charPtr: "<<temp;
		stringstream * strPtr;
		strPtr= (stringstream*) temp;

		cout<<"strPtr: "<<strPtr;
		cout<<"temp[0]: "<<temp[0];

		cout<<*charPtr;

		// tu sie wypieprza, czemu? nie wiem... jest seg fault...
		cout<<*strPtr;
		/*
		cout<<strPtr->str();
		 */
		//memcpy (newTable, charPtr, readSize);
		// CLOG(LERROR)<<newTable[0];
		// CLOG(LERROR)<<*charPtr;
		// CLOG(LERROR)<<newTable->str();
		//cout<<"Readed: "<<readSize<<" B.";
		//cout<<"charPtr: "<<charPtr;
		//stringstream * strPtr = (stringstream*)charPtr;
		//cout<<"strPtr: "<<strPtr;
		//cout<<"charPtr[0]: "<<charPtr[0];
		//cout<<"strlen(charPtr): "<<strlen(charPtr);
		//cout<<*charPtr;
		//cout<<*strPtr;
		//CLOG(LERROR)<<(stringstream *)obj[tempFileName].binData(readSize)<<"\n\n";
		//stringstream* buffi = (stringstream *)obj[tempFileName].binData(readSize);

		//cout<<buffi[0];
		//cout<<*buffi;
		//cout<<buffi;
		if(cloudType=="xyz")
		{
			PointCloudDecoderXYZ=new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>();
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ (new pcl::PointCloud<pcl::PointXYZ>);
			//PointCloudDecoderXYZ->decodePointCloud (*strPtr, cloudXYZ);
		//	pcl::io::savePCDFile("newCloud2.pcd", *cloudXYZ, false);
		}
		else if(cloudType=="xyzrgb")
		{
			PointCloudDecoderXYZRGB=new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>();
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
		//	PointCloudDecoderXYZRGB->decodePointCloud (*buffi, cloudXYZRGB);
		//	pcl::io::savePCDFile("newCloud2.pcd", *cloudXYZRGB, false);
		}
		CLOG(LERROR)<<"ViewWriter::insertFileIntoCollection: END";
	}catch(Exception &ex){cout<<ex.what();}
}
void ViewReader::readAllFilesTriggered()
{
	CLOG(LTRACE)<<"ViewReader::readAllFiles";
	//in_trigger.read();
	//V1
	/*
    for(std::vector<OID>::iterator it = allChildsVector.begin(); it != allChildsVector.end(); ++it)
    {
    	readFile(*it);
    }
*/
    //V2
    readFile(allChildsVector[position]);
    if(position<allChildsVector.size())
    	++position;
    else
    	position=0;
}

void ViewReader::ReadPCDCloud(const string& filename, const string& tempFile)
{
	CLOG(LTRACE) << "ViewReader::ReadPCDCloud";
	// Try to read the cloud of XYZRGB points.
	if(filename.find("xyzrgb")!=string::npos)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (tempFile, *cloud_xyzrgb) == -1){
			CLOG(LWARNING) <<"Cannot read PointXYZRGB cloud from "<<tempFile;
		}else{
			out_cloud_xyzrgb.write(cloud_xyzrgb);
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
			out_cloud_xyzsift.write(cloud_xyzsift);
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
				out_cloud_xyz.write(cloud_xyz);
				CLOG(LINFO) <<"PointXYZ cloud loaded properly from "<<tempFile;
				//return;
			}// else
		}
}

void ViewReader::writeToSinkFromFile(string& mime, string& tempFilename, string& fileName)
{
	CLOG(LNOTICE)<<"ViewReader::writeToSink";
	if(mime=="image/png" || mime=="image/jpeg")
	{
		cv::Mat image = imread(tempFilename, CV_LOAD_IMAGE_UNCHANGED);
		out_img.write(image);
	}
	else if(mime=="text/plain")
	{
		CLOG(LINFO)<<"mime==text/plain";
		//CLOG(LINFO)<<"fileName.find(pcd): "<<fileName.find("pcd");
		if((fileName.find("pcd"))!=string::npos)
		{
			CLOG(LINFO)<<"pcd :)";
			ReadPCDCloud(fileName, tempFilename);
		}
		else if(fileName.find("txt")!=string::npos)
		{
			CLOG(LINFO)<<"txt :)";
			string CIPFile;
			char const* charFileName = tempFilename.c_str();
			CLOG(LINFO)<<tempFilename;
			std::ifstream t(charFileName);
			std::stringstream buffer;
			buffer << t.rdbuf();
			CIPFile = buffer.str();
			cipFileOut.write(CIPFile);
			CLOG(LINFO)<<CIPFile;
		}
		else
			CLOG(LERROR)<<"Nie wiem co to za plik :/";
	}
}
void ViewReader::readFile(OID& childOID)
{
	CLOG(LTRACE)<<"ViewReader::readFile";
	int queryOptions = 0;
	const BSONObj *fieldsToReturn = 0;

	// get bson object from collection
	BSONObj obj = c->findOne(dbCollectionPath, QUERY("_id" << childOID), fieldsToReturn, queryOptions);

	CLOG(LERROR)<<"obj: "<<obj<<", childOID: "<<childOID;
	string place = obj.getField("place").str();
	int size = obj.getField("size").Int();
	string tempFileName = obj.getField("fileName").str();
	CLOG(LERROR)<<"place: "<<place<<" size: "<<size<<", fileName: "<<tempFileName;

	//TODO 		read from collection

	if(place=="collection")
	{
		//readFromCollection();
		string extension = obj.getField("extension").str();
		if(extension=="jpg" || extension=="png")
		{
			CLOG(LERROR)<<"Read image\n";
			BSONObj obj = c->findOne(dbCollectionPath, QUERY("_id" << childOID), fieldsToReturn, queryOptions);

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

			// cloud encoding
			//CLOG(LERROR)<<" cloud encoding";
			//cloudEncoding(childOID, tempFileName, cloudType);
			try
			{
				if(cloudType=="xyz")
				{

				}
				else if(cloudType=="xyzrgb")
				{

				}
				else if(cloudType=="xyzsift")
				{

					const BSONObj *fieldsToReturn = 0;
					int queryOptions = 0;
					BSONObj obj = c->findOne(dbCollectionPath, QUERY("_id" << childOID), fieldsToReturn, queryOptions);

					// read data to buffer
					int totalSize;
					float* buffer = (float*)obj[tempFileName].binData(totalSize);
					int bufferSize = totalSize/sizeof(float);
					CLOG(LERROR)<<"bufferSize: "<<bufferSize;
					float newBuffer[bufferSize];
					memcpy(newBuffer, buffer, totalSize);
					// row size in float is equal 128+5 =133  floats
					int rowSize = 133;
					pcl::PointCloud<PointXYZSIFT>::Ptr cloudXYZSIFT (new pcl::PointCloud<PointXYZSIFT>);
					PointXYZSIFT pt;
					/*
					 * void ViewWriter::copyXYZSiftPointToFloatArray (const PointXYZSIFT &p, float * out) const
					{
					Eigen::Vector3f outCoordinates = p.getArray3fMap();
					out[0] = outCoordinates[0];	// 4 bytes
					out[1] = outCoordinates[1];	// 4 bytes
					out[2] = outCoordinates[2];	// 4 bytes
					//CLOG(LERROR)<<"out[0]: "<<out[0]<<"\t out[1]: "<<out[1]<<"\t out[2]: "<<out[2];
					memcpy(&out[3], &p.multiplicity, sizeof(int)); // 4 bytes
					memcpy(&out[4], &p.pointId, sizeof(int));	// 4 bytes
					memcpy(&out[5], &p.descriptor, 128*sizeof(float)); // 128 * 4 bytes = 512 bytes
					 */
					//cloudXYZSIFT->resize(bufferSize/rowSize);
					for(int i=0; i<totalSize/(rowSize*sizeof(float)); i++) // now it should be 10 iterations
					{
						Eigen::Vector3f pointCoordinates;
						pointCoordinates[0]=newBuffer[i*rowSize];
						pointCoordinates[1]=newBuffer[i*rowSize+1];
						pointCoordinates[2]=newBuffer[i*rowSize+2];
						memcpy(&pt.multiplicity, &newBuffer[3+i*rowSize], sizeof(int)); // 4 bytes
						memcpy(&pt.pointId, &newBuffer[4+i*rowSize], sizeof(int));	// 4 bytes
						memcpy(&pt.descriptor, &newBuffer[5+i*rowSize], 128*sizeof(float)); // 128 * 4 bytes = 512 bytes

						CLOG(LERROR)<<(pt.descriptor[27]);
						CLOG(LERROR)<<(pt.descriptor[28]);
						CLOG(LERROR)<<(pt.descriptor[29]);

						CLOG(LERROR)<<*(pt.descriptor+27)+1;
						CLOG(LERROR)<<*(pt.descriptor+27)+2;
						CLOG(LERROR)<<*(pt.descriptor+27)+3;
						CLOG(LERROR)<<*(pt.descriptor+27)+4;
						CLOG(LERROR)<<*(pt.descriptor+28)+1;
						CLOG(LERROR)<<*(pt.descriptor+28)+2;
						CLOG(LERROR)<<*(pt.descriptor+28)+3;
						CLOG(LERROR)<<*(pt.descriptor+28)+4;

						CLOG(LERROR)<<(newBuffer[27+5+i*rowSize]);
						CLOG(LERROR)<<(newBuffer[28+5+i*rowSize]);
						CLOG(LERROR)<<(newBuffer[29+5+i*rowSize]);

						pt.getVector3fMap() = pointCoordinates;
						CLOG(LERROR)<<"iteration: "<<i;
						CLOG(LERROR)<<"pointCoordinates[0]: "<<pointCoordinates[0]<<"\tpointCoordinates[1]: "<<pointCoordinates[1]<<"\tpointCoordinates[2]: "<<pointCoordinates[2];
						for(int i=0; i<128;i++)
							CLOG(LERROR)<<pt.descriptor[i]<<"\t"<<i;
						cloudXYZSIFT->push_back(pt);
					}
					// save to file, only in test purposes
					pcl::io::savePCDFile("newCloudSIFT.pcd", *cloudXYZSIFT, false);
				}
				CLOG(LERROR)<<"ViewWriter::insertFileIntoCollection: END";
			}catch(Exception &ex){CLOG(LERROR)<<ex.what();}
		}//pcd
		//TODO sprawdzic czemu czasami nie zapetla sie na samych plikach txt!!!
		else if(extension=="txt")
		{
			// read from collection
			const BSONObj *fieldsToReturn = 0;
			int queryOptions = 0;
			// get bson object
			BSONObj obj = c->findOne(dbCollectionPath, QUERY("_id" << childOID), fieldsToReturn, queryOptions);
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
		// if saved in grid
		GridFS fs(*c,collectionName);
		CLOG(LTRACE)<<"_id"<<childOID;
		GridFile file = fs.findFile(QUERY("_id" << childOID));

		if (!file.exists())
		{
			CLOG(LERROR) << "File not found in grid";
		}
		else
		{
			// get filename
			string filename = file.getFileField("filename").str();
			// get mime from file
			string mime = file.getContentType();
			string tempFile = "tempFile";
			getFileFromGrid(file, tempFile);
			writeToSinkFromFile(mime, tempFile, filename);
		}
	}
}

void ViewReader::readFromMongoDB(const string& nodeType, const string& modelOrViewName, const string& type)
{
	CLOG(LTRACE)<<"ViewReader::readFromMongoDB";
	string name;
	try{
		int items=0;
		findDocumentInCollection(*c, dbCollectionPath, objectName, nodeType, cursorCollection, modelOrViewName, type, items);
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
					if(isViewLastLeaf(nodeType) || isModelLastLeaf(nodeType))
						addToAllChilds(childsVector);
					else
					{
						CLOG(LTRACE)<<"There are childs "<<childsVector.size();
						for (unsigned int i = 0; i<childsVector.size(); i++)
						{
							childCursor =c->query(dbCollectionPath, (QUERY("_id"<<childsVector[i])));
							if(childCursor->more())
							{
								BSONObj childObj = childCursor->next();
								string childNodeName= childObj.getField("Type").str();
								if(childNodeName!="EOO")
								{
									if(childNodeName=="View" || childNodeName=="Model")
									{
										string newName;
										setModelOrViewName(childNodeName, childObj, newName);
										readFromMongoDB(childNodeName, newName, type);
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
			if(nodeTypeProp==nodeType)
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
} //: namespace ViewReader
} //: namespace Processors
