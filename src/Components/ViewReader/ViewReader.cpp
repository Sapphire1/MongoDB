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
using namespace boost::property_tree;

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

		registerHandler("onTriggeredReadAllFiles", boost::bind(&ViewReader::readAllFilesTriggered, this));
		addDependency("onTriggeredReadAllFiles", &in_trigger);
}

bool ViewReader::onInit()
{
        CLOG(LTRACE) << "ViewReader::initialize";
        if(collectionName=="containers")
        	dbCollectionPath="images.containers";
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

void ViewReader::writeToSink(string& mime, string& tempFilename, string& fileName)
{
	CLOG(LNOTICE)<<"ViewReader::writeToSink";
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
void ViewReader::readFile(const OID& childOID)
{
	CLOG(LTRACE)<<"ViewReader::readFile";
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
		writeToSink(mime, tempFile, filename);
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
