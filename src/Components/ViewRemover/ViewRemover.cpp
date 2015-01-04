/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

#include "ViewRemover.hpp"


namespace Processors {
namespace ViewRemover  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace boost::posix_time;

ViewRemover::ViewRemover(const std::string & name) : Base::Component(name),
	mongoDBHost("mongoDBHost", string("localhost")),
	viewName("ViewName", string("lab012")),
	cameraInfoProp("file.cameraInfo.xml", false),
	xyzProp("image.xyz", false),
	rgbProp("image.rgb", false),
	depthProp("image.density", false),
	intensityProp("image.intensity", false),
	maskProp("image.mask", false),
	stereoProp("image.stereo", false),
	stereoTexturedProp("image.stereoTextured", false),
	pc_xyzProp("PC.xyz", false),
	pc_xyzrgbProp("PC.xyzrgb", false),
	pc_xyzsiftProp("PC.xyzsift", false),
	pc_xyzrgbsiftProp("PC.xyzrgbsift", false),
	pc_xyzshotProp("PC.xyzshot", false),
	pc_xyzrgbnormalProp("PC.xyzrgbnormal", false),
	removeAll("removeAll", false)
{
	registerProperty(mongoDBHost);
	registerProperty(viewName);
	registerProperty(cameraInfoProp);
	registerProperty(xyzProp);
	registerProperty(rgbProp);
	registerProperty(depthProp);
	registerProperty(intensityProp);
	registerProperty(maskProp);
	registerProperty(stereoProp);
	registerProperty(stereoTexturedProp);
	registerProperty(pc_xyzProp);
	registerProperty(pc_xyzrgbProp);
	registerProperty(pc_xyzsiftProp);
	registerProperty(pc_xyzrgbsiftProp);
	registerProperty(pc_xyzshotProp);
	registerProperty(pc_xyzrgbnormalProp);
	registerProperty(removeAll);

	hostname = mongoDBHost;
	CLOG(LTRACE) << "Hello ViewRemover";
}

ViewRemover::~ViewRemover()
{
        CLOG(LTRACE) << "Good bye ViewRemover";
}


void ViewRemover::prepareInterface() {
	CLOG(LTRACE) << "ViewRemover::prepareInterface";
	h_readfromDB.setup(this, &ViewRemover::readfromDB);
	registerHandler("Read", &h_readfromDB);

	//registerHandler("onTriggeredReadAllFiles", boost::bind(&ViewRemover::readAllFilesTriggered, this));
	//addDependency("onTriggeredReadAllFiles", &in_trigger);
}

bool ViewRemover::onInit()
{
        CLOG(LTRACE) << "ViewRemover::initialize";
        MongoProxy::MongoProxy::getSingleton(hostname);
        return true;
}

bool ViewRemover::onFinish()
{
        CLOG(LTRACE) << "ViewRemover::finish";
        return true;
}

bool ViewRemover::onStep()
{
        CLOG(LTRACE) << "ViewRemover::step";
        return true;
}

bool ViewRemover::onStop()
{
        return true;
}

bool ViewRemover::onStart()
{
        return true;
}

void ViewRemover::addToAllChilds(std::vector<OID> & childsVector)
{
	CLOG(LTRACE)<<"ViewRemover::addToAllChilds";
}

void ViewRemover::readAllFilesTriggered()
{
	CLOG(LTRACE)<<"ViewRemover::readAllFiles";
}


void ViewRemover::readfromDB()
{
	CLOG(LNOTICE) << "ViewRemover::readfromDB";

	string vn = string(viewName);
	viewPtr = boost::shared_ptr<View>(new View(vn,hostname));

	bool exist = viewPtr->checkIfExist();
	if(!exist)
	{
		CLOG(LERROR)<<"View doesn't exist in data base!!!, Change view name";
		return;
	}
	else
	{
		// get view document
		viewPtr->readViewDocument();

		// read all required types from GUI
		std::vector<fileTypes> requiredFileTypes;
		readRequiredData(requiredFileTypes);


		if(requiredFileTypes.size()==0)
		{
			CLOG(LERROR)<<"Please mark any checkbox";
			return;
		}
		// check if view contain all required types
		bool contain = viewPtr->checkIfContain(requiredFileTypes);

		if(!contain)
		{
			CLOG(LERROR)<<"View doesn't contain all required files! BYE!";
		}
		else
		{
			CLOG(LNOTICE)<<"Read files from View!";

			// read vector of files OIDs
			vector<OID> fileOIDSVector;
			viewPtr->getAllFilesOIDS(fileOIDSVector);

			// for full required files vector, read file document and check if its type is equal
			// one of requested file types

			if(removeAll)
			{
				// remove all files
				for(std::vector<OID>::iterator fileOIDIter = fileOIDSVector.begin(); fileOIDIter != fileOIDSVector.end(); ++fileOIDIter)
				{
					BSONObj query = BSON("_id" << *fileOIDIter);
					MongoProxy::MongoProxy::getSingleton(hostname).remove(*fileOIDIter);
				}

				// remove view document
				BSONObj obj = viewPtr->getDocument();
				BSONElement oi;
				obj.getObjectID(oi);
				OID viewOID = oi.__oid();
				MongoProxy::MongoProxy::getSingleton(hostname).remove(viewOID);
			}
			else
			{
				viewPtr->readFiles(fileOIDSVector, requiredFileTypes);

				//write to output
				int filesNr = viewPtr->getFilesSize();
				// remove only marked files
				for (int i=0; i<filesNr; i++)
				{
					viewPtr->getFile(i)->removeDocument();
				}
			}
		}
	}

}

void ViewRemover::readRequiredData(std::vector<fileTypes> & requiredFileTypes)
{
	CLOG(LNOTICE)<<"ViewWriter::checkProvidedData";
	bool cleanBuffers = false;

	if(cameraInfoProp==true)
	{
		requiredFileTypes.push_back(FileCameraInfo);
	}
	if(xyzProp==true)
	{
		requiredFileTypes.push_back(ImageXyz);
	}
	if(rgbProp==true)
	{
		requiredFileTypes.push_back(ImageRgb);
	}
	if(depthProp==true)
	{
		requiredFileTypes.push_back(ImageDepth);
	}
	if(intensityProp==true)
	{
		requiredFileTypes.push_back(ImageIntensity);
	}
	if(maskProp==true)
	{
		requiredFileTypes.push_back(ImageMask);
	}
	if(stereoProp==true)
	{
		requiredFileTypes.push_back(StereoLeft);
		requiredFileTypes.push_back(StereoRight);
	}
	if(stereoTexturedProp==true)
	{
		requiredFileTypes.push_back(StereoLeft);
		requiredFileTypes.push_back(StereoRight);
		requiredFileTypes.push_back(StereoLeftTextured);
		requiredFileTypes.push_back(StereoRightTextured);
	}
	if(pc_xyzProp==true)
	{
		requiredFileTypes.push_back(PCXyz);
	}
	if(pc_xyzrgbProp==true)
	{
		requiredFileTypes.push_back(PCXyzRgb);
	}
	if(pc_xyzsiftProp==true)
	{
		requiredFileTypes.push_back(PCXyzSift);
	}
	if(pc_xyzrgbsiftProp==true)
	{
		requiredFileTypes.push_back(PCXyzRgbSift);
	}
	if(pc_xyzshotProp==true)
	{
		requiredFileTypes.push_back(PCXyzShot);
	}
	if(pc_xyzrgbnormalProp==true)
	{
		requiredFileTypes.push_back(PCXyzRgbNormal);
	}
	CLOG(LNOTICE)<<"Size of required file types: "<<requiredFileTypes.size();
}
} //: namespace ViewRemover
} //: namespace Processors
