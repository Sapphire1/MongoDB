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
using namespace MongoBase;

ModelWriter::ModelWriter(const string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", string("localhost")),
		description("description", string("My green coffe cup")),
		modelName("modelName", string("model1")),
		fileName("fileName", string("tempFile")),
		object("object", string("object1")),
		pc_xyzProp("PC.xyz", true),
		pc_xyzrgbProp("PC.xyzrgb", true),
		pc_xyzsiftProp("PC.xyzsift", false),
		pc_xyzrgbsiftProp("PC.xyzrgbsift", false),
		pc_xyzshotProp("PC.xyzshot", false),
		pc_xyzrgbnormalProp("PC.xyzrgbnormal", false)
{
	registerProperty(mongoDBHost);
	registerProperty(description);
	registerProperty(modelName);
	registerProperty(fileName);
	registerProperty(object);

	registerProperty(pc_xyzProp);
	registerProperty(pc_xyzrgbProp);
	registerProperty(pc_xyzsiftProp);
	registerProperty(pc_xyzrgbsiftProp);
	registerProperty(pc_xyzshotProp);
	registerProperty(pc_xyzrgbnormalProp);

	//registerProperty(mean_viewpoint_features_number);

	CLOG(LTRACE) << "Hello ModelWriter";
	requiredTypes = boost::shared_ptr<std::vector<keyTypes> >(new std::vector<keyTypes>());
	string vn = string (modelName);
	string hostname = mongoDBHost;

	modelPtr = boost::shared_ptr<Model>(new Model(vn, hostname));
}

ModelWriter::~ModelWriter()
{
	//delete(modelPtr);
    CLOG(LTRACE) << "Good bye ModelWriter";
}

void ModelWriter::prepareInterface() {
	CLOG(LTRACE) << "ModelWriter::prepareInterface";
	registerHandler("write_xyz", boost::bind(&ModelWriter::writeData<pc_xyz>, this));
	registerHandler("write_xyzrgb", boost::bind(&ModelWriter::writeData<pc_xyzrgb>, this));
	registerHandler("write_xyzsift", boost::bind(&ModelWriter::writeData<pc_xyzsift>, this));
	registerHandler("write_xyzrgbsift", boost::bind(&ModelWriter::writeData<pc_xyzrgbsift>, this));
	registerHandler("write_xyzshot", boost::bind(&ModelWriter::writeData<pc_xyzshot>, this));
	registerHandler("write_xyzrgbnormal", boost::bind(&ModelWriter::writeData<pc_xyzrgbnormal>, this));	//4 floats

	registerStream("in_pc_xyz", &in_pc_xyz);
	registerStream("in_pc_xyzrgb", &in_pc_xyzrgb);
	registerStream("in_pc_xyzsift", &in_pc_xyzsift);
	registerStream("in_pc_xyzrgbsift", &in_pc_xyzrgbsift);
	registerStream("in_pc_xyzshot", &in_pc_xyzshot);
	registerStream("in_pc_xyzrgnormal", &in_pc_xyzrgbnormal);

	// adding dependency
	addDependency("write_xyz", &in_pc_xyz);
	addDependency("write_xyzrgb", &in_pc_xyzrgb);
	addDependency("write_xyzsift", &in_pc_xyzsift);
	addDependency("write_xyzrgbsift", &in_pc_xyzrgbsift);
	addDependency("write_xyzshot", &in_pc_xyzshot);
	addDependency("write_xyzrgbnormal", &in_pc_xyzrgbnormal);
}

template <keyTypes keyType>
void ModelWriter::writeData()
{
	CLOG(LNOTICE) << "ModelWriter::writeData";

	bool exist = modelPtr->checkIfExist();
	if(!exist)
	{
		string objectList = object;
		modelPtr->setObjectNames(objectList);
	}
	else
	{
		CLOG(LERROR)<<"Model exist in data base!!!";
		exit(-1);
	}
	setInputFiles();
	modelPtr->setRequiredKeyTypes(requiredTypes);

	// read dataTypes from mongo when inserting add to readed from mongo
	// and then check if such type exist in view
	CLOG(LNOTICE)<<"bool fileExist = modelPtr>checkIfFileExist(keyType)";
	bool fileExist = modelPtr->checkIfFileExist(keyType);
	if(fileExist)
	{
		LOG(LERROR)<<"File exist in model. You can't write file. File type is: "<< keyType;
		exit(-1);
	}


	CLOG(LNOTICE)<<"keyType : "<< keyType;

	// read data from input
	string filename = (string)fileName;
	switch(keyType)
	{
		case pc_xyz:
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr pc_xyz_Data = in_pc_xyz.read();
			filename += ".pcd";
			modelPtr->putPCxyzToFile(pc_xyz_Data, keyType, filename);
			break;
		}
		case pc_xyzrgb:
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_xyzrgb_Data = in_pc_xyzrgb.read();
			filename += ".pcd";
			modelPtr->putPCyxzrgbToFile(pc_xyzrgb_Data, keyType, filename);
			break;
		}
		case pc_xyzsift:
		{
			pcl::PointCloud<PointXYZSIFT>::Ptr pc_xyzsift_Data = in_pc_xyzsift.read();
			filename += ".pcd";
			modelPtr->putPCxyzsiftToFile(pc_xyzsift_Data, keyType, filename);
			break;
		}
		case pc_xyzrgbsift:
		{
			pcl::PointCloud<PointXYZRGBSIFT>::Ptr pc_xyzrgbsift_Data = in_pc_xyzrgbsift.read();
			filename += ".pcd";
			modelPtr->putPCxyzrgbsiftToFile(pc_xyzrgbsift_Data, keyType, filename);
			break;
		}
		case pc_xyzshot:
		{
			pcl::PointCloud<PointXYZSHOT>::Ptr pc_xyzshot_Data = in_pc_xyzshot.read();
			filename += ".pcd";
			modelPtr->putPCxyzshotToFile(pc_xyzshot_Data, keyType, filename);
			break;
		}
		case pc_xyzrgbnormal:
		{
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pc_xyzrgbnormal_Data = in_pc_xyzrgbnormal.read();
			filename += ".pcd";
			modelPtr->putPCxyzrgbNormalToFile(pc_xyzrgbnormal_Data, keyType, filename);
			break;
		}
	}
}

bool ModelWriter::onInit()
{
	CLOG(LTRACE) << "ModelWriter::initialize";
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

void ModelWriter::setInputFiles()
{
	requiredTypes->clear();
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
} //: namespace ModelWriter
} //: namespace Processors
