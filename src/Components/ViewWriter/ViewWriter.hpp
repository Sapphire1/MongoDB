
#ifndef  ViewWriter_H__
#define  ViewWriter_H__
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
namespace ViewWriter {

using namespace cv;
using namespace mongo;
using namespace std;


class ViewWriter: public Base::Component, MongoBase::MongoBase
{
public:
        /*!
         * Constructor.
         */
	   ViewWriter(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~ViewWriter();

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
        Base::Property<string> viewNameProp;
        Base::Property<string> sceneNamesProp;
        Base::Property<string> fileName;
        Base::Property<int> mean_viewpoint_features_number;
        Base::Property<string> remoteFileName;
        std::vector<std::string> splitedSceneNames;
        Base::Property<string> nodeTypeProp;
        Base::Property<bool> binary;
		Base::Property<bool> suffix;
		string cloudType;
        string dbCollectionPath;

		void Write_xyz();
		void Write_xyzrgb();
		void Write_xyzsift();
        void initObject();
        void writeNode2MongoDB(const string &destination, const string &option, string,  const string& fileType);
        void insert2MongoDB(const string &destination,  const string&,  const string&,  const string& fileType );
        void writeTXT2DB();
        void writeImage2DB();
        void writePCD2DB();
        void insertToModelOrView(const string &,const string &);
        void insertFileToGrid(OID&, const string& fileType);
        void createModelOrView(const std::vector<string>::iterator, const string&, BSONArrayBuilder&);
};
}//: namespace ViewWriter
}//: namespace Processors

REGISTER_COMPONENT("ViewWriter", Processors::ViewWriter::ViewWriter)

#endif /* ViewWriter_H__ */
