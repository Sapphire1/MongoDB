/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */
#include "ModelReader.hpp"


namespace Processors {
namespace ModelReader  {
using namespace cv;
using namespace mongo;
using namespace boost::property_tree;

ModelReader::ModelReader(const std::string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", string("localhost")),
		objectName("objectName", string("GreenCup")),
		collectionName("collectionName", string("containers")),
		nodeTypeProp("nodeType", string("Object")),
		viewOrModelName("viewOrModelName", string("lab012")),
		type("type", string("")),
		modelType("modelType", string("SOM"))
//		modelType("modelType", string("SSOM"))
{
		registerProperty(mongoDBHost);
		registerProperty(objectName);
		registerProperty(collectionName);
		registerProperty(nodeTypeProp);
		registerProperty(viewOrModelName);
		registerProperty(modelType);
        CLOG(LTRACE) << "Hello ModelReader";

        //base = new MongoBase::MongoBase();
}

ModelReader::~ModelReader()
{
        CLOG(LTRACE) << "Good bye ModelReader";
}

void ModelReader::readfromDB()
{
	CLOG(LNOTICE) << "ModelReader::readfromDB";
	readFromMongoDB(nodeTypeProp, viewOrModelName, type);
}
void ModelReader::prepareInterface() {
	CLOG(LTRACE) << "ModelReader::prepareInterface";
	h_readfromDB.setup(this, &ModelReader::readfromDB);
	registerHandler("Read", &h_readfromDB);

	registerStream("out_cloud_xyz", &out_cloud_xyz);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	registerStream("out_img", &out_img);
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

void ModelReader::ReadPCDCloud(const string& filename, const string& tempFile)
{
	CLOG(LTRACE) << "ViewReader::ReadPCDCloud";
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

	model_name = nodeTypeProp;
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

   // to wykonywac tylko jesli juz mamy caly model tylko jak to zrobic???
	// Create SOModel and add it to list.
	if(name_cloud_xyzsift!="" && name_cloud_xyzrgb!="")
	{
		CLOG(LDEBUG) << "Create model";
		SIFTObjectModel* model;
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
			ReadPCDCloud(fileName, tempFilename);
		}
		else if(fileName.find("txt")!=string::npos)
		{
			//TODO read text file
			;
		}
		else
			CLOG(LERROR)<<"Nie wiem co to za plik :/";
	}
}

void ModelReader::readFile(const string& modelOrViewName, const string& nodeType, const string& type, const OID& childOID, std::vector<AbstractObject*>& models)
{
	CLOG(LTRACE)<<"ModelReader::readFile";
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
		string featuresNumber;
		if((filename.find("sift"))!=string::npos)
			featuresNumber = file.getFileField("mean_viewpoint_features_number").str();
		// get mime from file
		string mime = file.getContentType();
		string tempFile = "tempFile";
		getFileFromGrid(file, modelOrViewName, nodeType, type, filename, mime, tempFile);
		writeToSink(mime, tempFile, filename);
		CLOG(LTRACE)<<"Add to model";
		loadModels(filename, featuresNumber, models);
	}
}

void ModelReader::readFromMongoDB(const string& nodeType, const string& modelOrViewName, const string& type)
{
	CLOG(LTRACE)<<"ModelReader::readFromMongoDB";
	string name;
	std::vector<AbstractObject*> models;
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
					CLOG(LTRACE)<<"There are childs "<<childsVector.size();
					for (unsigned int i = 0; i<childsVector.size(); i++)
					{
						childCursor =c->query(dbCollectionPath, (QUERY("_id"<<childsVector[i])));
						if(childCursor->more())
						{
							BSONObj childObj = childCursor->next();
							string childNodeName= childObj.getField("Type").str();
							CLOG(LTRACE)<<childNodeName;
							if(childNodeName!="EOO")
							{
								if(isViewLastLeaf(nodeType) || isModelLastLeaf(nodeType))
								{
									CLOG(LTRACE)<<"LastLeaf"<<" childNodeName "<<childNodeName;
									readFile(modelOrViewName, nodeType, type, childsVector[i], models);
								}
								else if(childNodeName=="Model")
								{
									CLOG(LTRACE)<<"setModelOrViewName";
									string newName;
									setModelOrViewName(childNodeName, childObj, newName);
									readFromMongoDB(childNodeName, newName, type);

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
		if(nodeTypeProp==nodeType)
		{
			CLOG(LINFO)<<"nodeTypeProp - nodeType: "<<nodeTypeProp<< " - " << nodeType;
			CLOG(LTRACE)<<"Send models to sink";
			out_models.write(models);
		}
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
