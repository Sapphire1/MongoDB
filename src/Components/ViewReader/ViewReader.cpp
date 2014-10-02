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
        CLOG(LTRACE) << "Hello ViewReader";

        base = new MongoBase::MongoBase();
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

//        registerStream("in_img", &in_img);
        registerStream("out_cloud_xyz", &out_cloud_xyz);
        registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
        registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
        registerStream("out_img", &out_img);
 //       addDependency("readfromDB", NULL);
}

bool ViewReader::onInit()
{
        CLOG(LTRACE) << "ViewReader::initialize";
        if(collectionName=="containers")
        	dbCollectionPath="images.containers";
        else if(collectionName=="food")
            dbCollectionPath="images.food";
        else if(collectionName=="dish")
            dbCollectionPath="images.dish";
        else if(collectionName=="other")
            dbCollectionPath="images.other";
        try
        {
      	  c.connect(mongoDBHost);
      	  //base = MongoBase::MongoBase(c,dbCollectionPath,objectName);
         }
         catch(DBException &e)
         {
        	 CLOG(LERROR) <<"Something goes wrong... :>";
        	 CLOG(LERROR) <<c.getLastError();
         }
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

void ViewReader::ReadPCDCloud(const string& filename)
{
	CLOG(LTRACE) << "ViewReader::ReadPCDCloud";
	string tempFile="tempFile";
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

void ViewReader::getFileFromGrid(const GridFile& file, const string& modelOrViewName, const string& nodeType, const string& type, const string& fileName, const string& mime)
{
	CLOG(LTRACE)<<"ViewReader::getFileFromGrid";
	CLOG(LINFO)<<"Filename: "<< fileName<< " Mime: "<<mime;
	stringstream ss;
	string str = ss.str();
	string fn = "tempFile";
	char *filename = (char*)fn.c_str();
	ofstream ofs(filename);
	gridfs_offset off = file.write(ofs);
	if (off != file.getContentLength())
	{
		CLOG(LERROR) << "Failed to read a file from mongoDB";
	}
	else
	{
		CLOG(LTRACE) << "Success read a file from mongoDB";
	}

	if(mime=="image/png" || mime=="image/jpeg")
	{
		// read from disc
		cv::Mat image = imread(filename, CV_LOAD_IMAGE_UNCHANGED);
		out_img.write(image);
	}
	else if(mime=="text/plain")
	{
		CLOG(LINFO)<<"mime==text/plain";
		CLOG(LINFO)<<"fileName.find(pcd): "<<fileName.find("pcd");
		if((fileName.find("pcd"))!=string::npos)
		{
			CLOG(LINFO)<<"pcd :)";
			ReadPCDCloud(fileName);
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

void ViewReader::setModelOrViewName(const string& childNodeName, const BSONObj& childObj)
{
	CLOG(LTRACE)<<"ViewReader::setModelOrViewName";
	string type = childNodeName;
	string modelOrViewName = childObj.getField(type+"Name").str();
	readFromMongoDB(childNodeName, modelOrViewName, type);
}
void ViewReader::readFile(const string& modelOrViewName, const string& nodeType, const string& type, const OID& childOID)
{
	CLOG(LTRACE)<<"ViewReader::readFile";
	GridFS fs(c,collectionName);
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
		getFileFromGrid(file, modelOrViewName, nodeType, type, filename, mime);
	}
}

void ViewReader::readFromMongoDB(const string& nodeType, const string& modelOrViewName, const string& type)
{
	CLOG(LTRACE)<<"ViewReader::readFromMongoDB";
	string name;
	try{
		int items=0;
		base->findDocumentInCollection(c, dbCollectionPath, objectName, nodeType, cursorCollection, modelOrViewName, type, items);
		if(items>0)
		{
			CLOG(LINFO)<<"Founded some data";
			while (cursorCollection->more())
			{
				BSONObj obj = cursorCollection->next();
				CLOG(LTRACE)<<obj;
				vector<OID> childsVector;
				int items =  base->getChildOIDS(obj, "childOIDs", "childOID", childsVector);
				if(items>0)
				{
					CLOG(LTRACE)<<"There are childs "<<childsVector.size();
					for (unsigned int i = 0; i<childsVector.size(); i++)
					{
						///TODO tutaj dodac jakies triggrowanie, albo na razie opoznienie czy cos
						childCursor =c.query(dbCollectionPath, (QUERY("_id"<<childsVector[i])));
						if(childCursor->more())
						{
							BSONObj childObj = childCursor->next();
							string childNodeName= childObj.getField("Type").str();
							if(childNodeName!="EOO")
							{
								if(base->isViewLastLeaf(nodeType) || base->isModelLastLeaf(nodeType))
								{
									CLOG(LTRACE)<<"LastLeaf"<<" childNodeName "<<childNodeName;
									readFile(modelOrViewName, nodeType, type, childsVector[i]);
								}
								else if(childNodeName=="View" || childNodeName=="Model")
								{
									setModelOrViewName(childNodeName, childObj);
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
	}//try
	catch(DBException &e)
	{
		CLOG(LERROR) <<"ReadFromMongoDB(). Something goes wrong... :<";
		exit(1);
	}
}
} //: namespace ViewReader
} //: namespace Processors
