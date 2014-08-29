
#ifndef  MONGODBREADER_H__
#define  MONGODBREADER_H__


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

#include "mongo/client/dbclient.h"
#include "mongo/bson/bson.h"
#include "Logger.hpp"

namespace Processors {
namespace MongoDBReader {

using namespace cv;
using namespace mongo;


class MongoDBReader: public Base::Component
{
public:
        /*!
         * Constructor.
         */
		MongoDBReader(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~MongoDBReader();

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
        void onNewImage();

        void connect2MongoDB();

        /// Event handler.
        Base::EventHandler <MongoDBReader> h_readfromDB;

        /// Input data stream
        Base::DataStreamIn <cv::Mat> in_img;

        /// Output data stream - processed image
        Base::DataStreamOut <Mat> out_img;

private:
        Base::Property<string> mongoDBHost;
        Base::Property<string> objectName;
        Base::Property<string> collectionName;
        Base::Property<string> nodeType;
        Base::Property<string> folderName;
        DBClientConnection c;
        string dbCollectionPath;

        auto_ptr<DBClientCursor>  findDocumentInCollection(string nodeName);
        vector<OID>  getchildOIDS(BSONObj &obj);
        void readFromMongoDB(string nodeName);
        void readfromDB();


        void run();
};
}//: namespace MongoDBReader
}//: namespace Processors

REGISTER_COMPONENT("MongoDBReader", Processors::MongoDBReader::MongoDBReader)

#endif /* MONGODBREADER_H__ */
