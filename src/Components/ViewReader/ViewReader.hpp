
#ifndef  ViewReader_H__
#define  ViewReader_H__

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <memory>
#include <cstring>
#include <cstdlib>
#include <glob.h>
#include <memory>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include "mongo/client/dbclient.h"
#include "mongo/bson/bson.h"
#include "Logger.hpp"
#include <Types/AddVector.hpp>
#include <Types/MongoBase.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZRGBSIFT.hpp>

namespace Processors {
namespace ViewReader {

using namespace cv;
using namespace mongo;


class ViewReader: public Base::Component, MongoBase::MongoBase
{
public:
        /*!
         * Constructor.
         */
		ViewReader(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~ViewReader();

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
        Base::EventHandler <ViewReader> h_readfromDB;

private:
        Base::Property<string> mongoDBHost;
        Base::Property<string> objectName;
        Base::Property<string> collectionName;
        Base::Property<string> nodeTypeProp;
        Base::Property<string> viewOrModelName;
        Base::Property<string> type;
        string dbCollectionPath;
        auto_ptr<DBClientCursor> cursorCollection;
        auto_ptr<DBClientCursor> childCursor;
        Base::DataStreamIn<Base::UnitType> in_trigger;

        // vector consisting all files OIDS
		std::vector<OID> allChildsVector;
        // position of allChildsVector
        int position;

        void readFromMongoDB(const string&, const string&, const string&);
        void ReadPCDCloud(const string&, const string&);
        void readfromDB();
        void writeToSinkFromFile(string& mime, string& filename, string& fileName);
        void readFile(OID& childOID);
        void readAllFilesTriggered();
        void addToAllChilds(std::vector<OID>&);
        void cloudEncoding(OID& oid, string& tempFileName, string & cloudType);


};
}//: namespace ViewReader
}//: namespace Processors

REGISTER_COMPONENT("ViewReader", Processors::ViewReader::ViewReader)

#endif /* ViewReader_H__ */
