
#ifndef  ModelWriter_H__
#define  ModelWriter_H__
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include <cstdlib>
#include <iostream>
#include <glob.h>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "Logger.hpp"
#include "mongo/client/dbclient.h"
#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include <dirent.h>
#include <Types/MongoBase.hpp>
#include <Types/PointXYZSIFT.hpp>

namespace Processors {
namespace ModelWriter {

using namespace cv;
using namespace mongo;
using namespace std;


class ModelWriter: public Base::Component, MongoBase::MongoBase
{
public:
        /*!
         * Constructor.
         */
	   ModelWriter(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~ModelWriter();

        /*!
         * Prepares communication interface.
         */
        virtual void prepareInterface();

protected:

        /*!
         * Connects source to given device.
         */
        bool onInit();

        /*!
         * Disconnect source from device, closes streams, etc.
         */
        bool onFinish();

        /*!
         * Retrieves data from device.
         */
        bool onStep();

        /*!
         * Start component
         */
        bool onStart();

        /*!
         * Stop component
         */
        bool onStop();


        /*!
         * Event handler function.
         */

        /// Event handler.



private:
	Base::Property<string> mongoDBHost;
	Base::Property<string> objectName;
	Base::Property<string> description;
	Base::Property<string> collectionName;
	Base::Property<string> modelNameProp;
	Base::Property<string> sceneNamesProp;
	Base::Property<string> fileName;
	Base::Property<string> remoteFileName;
	std::vector<std::string> splitedSceneNames;
	Base::Property<string> nodeNameProp;
	Base::Property<int> mean_viewpoint_features_number;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB;
	pcl::PointCloud<PointXYZSIFT>::Ptr cloudXYZSIFT;
	pcl::PointCloud<PointXYZRGBSIFT>::Ptr cloudXYZRGBSIFT;
	Base::Property<bool> binary;
	Base::Property<bool> suffix;
	string cloudType;
	cv::Mat tempImg;
	float sizeOfCloud;

	template <class PointT>
	void Write_cloud();

	void initObject();
	void writePCD2DB();
	void writeNode2MongoDB(const string &destination, const string &option, string, const string& fileType );
    void insert2MongoDB(const string &destination,  const string&,  const string&,  const string& fileType );
	void insertToModelOrView(const string &,const string &);
	float getFileSize(const string& fileType, string& tempFileName);
	void createModelOrView(const std::vector<string>::iterator, const string&, BSONArrayBuilder&);
	void insertFileIntoCollection(OID& oid, const string& fileType, string& tempFileName, int size);
    void copyXYZSiftPointToFloatArray (const PointXYZSIFT &p, float * out) const;
    void copyXYZPointToFloatArray (const pcl::PointXYZ &p, float * out) const;
    void copyXYZRGBPointToFloatArray (const pcl::PointXYZRGB &p, float * out) const;
    void insertFileIntoGrid(OID& oid, const string& fileType, string& tempFileName, int);

};
}//: namespace ModelWriter
}//: namespace Processors

REGISTER_COMPONENT("ModelWriter", Processors::ModelWriter::ModelWriter)

#endif /* ModelWriter_H__ */
