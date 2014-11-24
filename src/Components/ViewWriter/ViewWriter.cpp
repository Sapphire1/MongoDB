/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

#include "ViewWriter.hpp"
#include <pcl/point_representation.h>
#include <Eigen/Core>
#include <boost/archive/tmpdir.hpp>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/serialization.hpp>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/assume_abstract.hpp>



#define siftPointSize 133
#define xyzPointSize 3
#define xyzrgbPointSize 4

namespace Processors {
namespace ViewWriter  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace boost::posix_time;
using namespace Eigen;
using namespace MongoBase;

ViewWriter::ViewWriter(const string & name) : Base::Component(name),
	mongoDBHost("mongoDBHost", string("localhost")),
	description("description", string("My green coffe cup")),
	viewName("viewName", string("lab012")),
	fileName("fileName", string("tempFile")),
	SensorType("SensorType", string("Stereo")),
	objects("objects", string("object1;object2;object3")),
	xmlProp("files.xml", false),
	xyzProp("files.xyz", false),
	rgbProp("files.rgb", false),
	densityProp("files.density", false),
	intensityProp("files.intensity", false),
	maskProp("files.mask", false),
	stereoProp("files.stereo", false),
	stereoTexturedProp("files.stereoTextured", false),
	pc_xyzProp("PC.xyz", false),
	pc_xyzrgbProp("PC.xyzrgb", false),
	pc_xyzsiftProp("PC.xyzsift", false),
	pc_xyzrgbsiftProp("PC.xyzrgbsift", false),
	pc_xyzshotProp("PC.xyzshot", false),
	pc_xyzrgbnormalProp("PC.xyzrgbnormal", false)

	// for sift cloud:
	//mean_viewpoint_features_number("mean_viewpoint_features_number", int(12)),
{
	registerProperty(mongoDBHost);
	registerProperty(description);
	registerProperty(viewName);
	registerProperty(fileName);
	registerProperty(objects);
	registerProperty(SensorType);


	registerProperty(xmlProp);
	registerProperty(xyzProp);
	registerProperty(rgbProp);
	registerProperty(densityProp);
	registerProperty(intensityProp);
	registerProperty(maskProp);
	registerProperty(stereoTexturedProp);
	registerProperty(stereoProp);

	registerProperty(pc_xyzProp);
	registerProperty(pc_xyzrgbProp);
	registerProperty(pc_xyzsiftProp);
	registerProperty(pc_xyzrgbsiftProp);
	registerProperty(pc_xyzshotProp);
	registerProperty(pc_xyzrgbnormalProp);

	//registerProperty(mean_viewpoint_features_number);

	CLOG(LTRACE) << "Hello ViewWriter";
	//std::vector<keyTypes>* key_ptr = ;
	requiredTypes = boost::shared_ptr<std::vector<keyTypes> >(new std::vector<keyTypes>());
	string vn = string(viewName);
	string hostname = mongoDBHost;
	viewPtr = boost::shared_ptr<View>(new View(vn, hostname));
}

ViewWriter::~ViewWriter()
{
	CLOG(LTRACE) << "Good bye ViewWriter";
}

void ViewWriter::prepareInterface() {
	CLOG(LTRACE) << "ViewWriter::prepareInterface";
	registerHandler("writeXML2DB", boost::bind(&ViewWriter::writeData<xml>, this));
	registerHandler("writeXYZ2DB", boost::bind(&ViewWriter::writeData<xyz>, this));
	registerHandler("writeRGB2DB", boost::bind(&ViewWriter::writeData<rgb>, this));
	registerHandler("writeD2DB", boost::bind(&ViewWriter::writeData<density>, this));
	registerHandler("writeI2DB", boost::bind(&ViewWriter::writeData<intensity>, this));
	registerHandler("writeMask2DB", boost::bind(&ViewWriter::writeData<mask>, this));
	registerHandler("writeStereoL2DB", boost::bind(&ViewWriter::writeData<stereoL>, this));
	registerHandler("writeStereoR2DB", boost::bind(&ViewWriter::writeData<stereoR>, this));
	registerHandler("writeStereoLTextured2DB", boost::bind(&ViewWriter::writeData<stereoLTextured>, this));
	registerHandler("writeStereoRTextured2DB", boost::bind(&ViewWriter::writeData<stereoRTextured>, this));

	// PCL
	registerHandler("write_xyz", boost::bind(&ViewWriter::writeData<pc_xyz>, this));
	registerHandler("write_xyzrgb", boost::bind(&ViewWriter::writeData<pc_xyzrgb>, this));
	registerHandler("write_xyzsift", boost::bind(&ViewWriter::writeData<pc_xyzsift>, this));
	registerHandler("write_xyzrgbsift", boost::bind(&ViewWriter::writeData<pc_xyzrgbsift>, this));
	registerHandler("write_xyzshot", boost::bind(&ViewWriter::writeData<pc_xyzshot>, this));
	registerHandler("write_xyzrgbnormal", boost::bind(&ViewWriter::writeData<pc_xyzrgbnormal>, this));	//4 floats

	// streams registration
	registerStream("in_xml", &in_xml);
	registerStream("in_xyz", &in_xyz);
	registerStream("in_rgb", &in_rgb);
	registerStream("in_density", &in_density);
	registerStream("in_intensity", &in_intensity);
	registerStream("in_mask", &in_mask);
	registerStream("in_stereoL", &in_stereoL);
	registerStream("in_stereoR", &in_stereoR);
	registerStream("in_stereoLTextured", &in_stereoLTextured);
	registerStream("in_stereoRTextured", &in_stereoRTextured);

	// PCL
	registerStream("in_pc_xyz", &in_pc_xyz);
	registerStream("in_pc_xyzrgb", &in_pc_xyzrgb);
	registerStream("in_pc_xyzsift", &in_pc_xyzsift);
	registerStream("in_pc_xyzrgbsift", &in_pc_xyzrgbsift);
	registerStream("in_pc_xyzshot", &in_pc_xyzshot);
	registerStream("in_pc_xyzrgnormal", &in_pc_xyzrgbnormal);

	// adding dependency
	addDependency("writeXML2DB", &in_xml);
	addDependency("writeRGB2DB", &in_rgb);
	addDependency("writeXYZ2DB", &in_xyz);
	addDependency("writeXYZ2DB", &in_density);
	addDependency("writeD2DB", &in_density);
	addDependency("writeI2DB", &in_intensity);
	addDependency("writeMask2DB", &in_mask);
	addDependency("writeStereoL2DB", &in_stereoL);
	addDependency("writeStereoR2DB", &in_stereoR);
	addDependency("writeStereoLTextured2DB", &in_stereoLTextured);
	addDependency("writeStereoRTextured2DB", &in_stereoRTextured);
	addDependency("write_xyz", &in_pc_xyz);
	addDependency("write_xyzrgb", &in_pc_xyzrgb);
	addDependency("write_xyzsift", &in_pc_xyzsift);
	addDependency("write_xyzrgbsift", &in_pc_xyzrgbsift);
	addDependency("write_xyzshot", &in_pc_xyzshot);
	addDependency("write_xyzrgbnormal", &in_pc_xyzrgbnormal);
}

template <keyTypes keyType>
void ViewWriter::writeData()
{
	CLOG(LNOTICE) << "ViewWriter::writeData";

	bool exist = viewPtr->checkIfExist();
	if(!exist)
	{
		string objectList = objects;
		string sensor = SensorType;
		std::vector<std::string> splitedObjectNames;
		boost::split(splitedObjectNames, objectList, is_any_of(";"));
		viewPtr->setSensorType(sensor);
		viewPtr->setObjectNames(splitedObjectNames);
	}
	else
	{
		CLOG(LERROR)<<"View exist in data base!!!";
		exit(-1);
	}
	setInputFiles();
	viewPtr->setRequiredKeyTypes(requiredTypes);

	// read dataTypes from mongo when inserting add to readed from mongo
	// and then check if such type exist in view
	bool fileExist = viewPtr->checkIfFileExist(keyType);
	if(fileExist)
	{
		LOG(LERROR)<<"File exist in view. You can't write file. File type is: "<< keyType;
		exit(-1);
	}


	CLOG(LNOTICE)<<"keyType : "<< keyType;

	// read data from input
	string filename = (string)fileName;
	switch(keyType)
	{
		case xml:
		{
			string xmlData = in_xml.read();
			filename += ".xml";
			viewPtr->putStringToFile(xmlData, keyType, filename);
			break;
		}
		case rgb:
		{
			cv::Mat rgbData = in_rgb.read();
			filename += ".png";
			viewPtr->putMatToFile(rgbData, keyType, filename);
			break;
		}
		case density:
		{
			cv::Mat densityData = in_density.read();
			filename += ".png";
			viewPtr->putMatToFile(densityData, keyType, filename);
			break;
		}
		case intensity:
		{
			cv::Mat intensityData = in_intensity.read();
			viewPtr->putMatToFile(intensityData, keyType, filename);
			break;
		}
		case mask:
		{
			cv::Mat maskData = in_mask.read();
			filename += ".png";
			viewPtr->putMatToFile(maskData, keyType, filename);
			break;
		}
		case stereoL:
		{
			cv::Mat stereoLData = in_stereoL.read();
			filename += ".png";
			viewPtr->putMatToFile(stereoLData, keyType, filename);
			break;
		}
		case stereoR:
		{
			cv::Mat stereoRData = in_stereoR.read();
			filename += ".png";
			viewPtr->putMatToFile(stereoRData, keyType, filename);
			break;
		}
		case stereoLTextured:
		{
			cv::Mat stereoLTexturedData = in_stereoLTextured.read();
			filename += ".png";
			viewPtr->putMatToFile(stereoLTexturedData, keyType, filename);
			break;
		}
		case stereoRTextured:
		{
			cv::Mat stereoRTexturedData = in_stereoRTextured.read();
			filename += ".png";
			viewPtr->putMatToFile(stereoRTexturedData, keyType, filename);
			break;
		}
		case pc_xyz:
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr pc_xyz_Data = in_pc_xyz.read();
			filename += ".pcd";
			viewPtr->putPCxyzToFile(pc_xyz_Data, keyType, filename);
			break;
		}
		case pc_xyzrgb:
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_xyzrgb_Data = in_pc_xyzrgb.read();
			filename += ".pcd";
			viewPtr->putPCyxzrgbToFile(pc_xyzrgb_Data, keyType, filename);
			break;
		}
		case pc_xyzsift:
		{
			pcl::PointCloud<PointXYZSIFT>::Ptr pc_xyzsift_Data = in_pc_xyzsift.read();
			filename += ".pcd";
			viewPtr->putPCxyzsiftToFile(pc_xyzsift_Data, keyType, filename);
			break;
		}
		case pc_xyzrgbsift:
		{
			pcl::PointCloud<PointXYZRGBSIFT>::Ptr pc_xyzrgbsift_Data = in_pc_xyzrgbsift.read();
			filename += ".pcd";
			viewPtr->putPCxyzrgbsiftToFile(pc_xyzrgbsift_Data, keyType, filename);
			break;
		}
		case pc_xyzshot:
		{
			pcl::PointCloud<PointXYZSHOT>::Ptr pc_xyzshot_Data = in_pc_xyzshot.read();
			filename += ".pcd";
			viewPtr->putPCxyzshotToFile(pc_xyzshot_Data, keyType, filename);
			break;
		}
		case pc_xyzrgbnormal:
		{
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pc_xyzrgbnormal_Data = in_pc_xyzrgbnormal.read();
			filename += ".pcd";
			viewPtr->putPCxyzrgbNormalToFile(pc_xyzrgbnormal_Data, keyType, filename);
			break;
		}
	}
}
bool ViewWriter::onInit()
{
	CLOG(LTRACE) << "ViewWriter::initialize";

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

void ViewWriter::setInputFiles()
{
	requiredTypes->clear();
	if(xmlProp==true)
		requiredTypes->push_back(xml);
	if(xyzProp==true)
		requiredTypes->push_back(xyz);
	if(rgbProp==true)
		requiredTypes->push_back(rgb);
	if(densityProp==true)
		requiredTypes->push_back(density);
	if(intensityProp==true)
		requiredTypes->push_back(intensity);
	if(maskProp==true)
		requiredTypes->push_back(mask);
	if(stereoProp==true)
		requiredTypes->push_back(stereo);
	if(stereoTexturedProp==true)
		requiredTypes->push_back(stereoTextured);
	if(pc_xyzProp==true)
		requiredTypes->push_back(pc_xyz);
	if(pc_xyzrgbProp==true)
		requiredTypes->push_back(pc_xyzrgb);
	if(pc_xyzsiftProp==true)
		requiredTypes->push_back(pc_xyzsift);
	if(pc_xyzrgbsiftProp==true)
		requiredTypes->push_back(density);
	if(pc_xyzshotProp==true)
		requiredTypes->push_back(pc_xyzrgbsift);
	if(pc_xyzrgbnormalProp==true)
		requiredTypes->push_back(pc_xyzrgbnormal);

	LOG(LNOTICE)<<"Required nr: "<<requiredTypes->size();
}

} //: namespace ViewWriter
} //: namespace Processors
