
#ifndef  ModelReader_H__
#define  ModelReader_H__

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
#include "MongoBase.hpp"

namespace Processors {
namespace ModelReader {

using namespace cv;
using namespace mongo;


class ModelReader: public Base::Component
{
public:
        /*!
         * Constructor.
         */
		ModelReader(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~ModelReader();

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
        Base::EventHandler <ModelReader> h_readfromDB;

        /// Input data stream
        Base::DataStreamIn <cv::Mat> in_img;

        /// Output data stream - processed image
        Base::DataStreamOut <Mat> out_img;

private:
        Base::Property<string> mongoDBHost;
        Base::Property<string> objectName;
        Base::Property<string> collectionName;
        Base::Property<string> nodeTypeProp;
        Base::Property<string> folderName;
        Base::Property<string> viewOrModelName;
        Base::Property<string> type;
        DBClientConnection c;
        string dbCollectionPath;
        auto_ptr<DBClientCursor> cursorCollection;
        auto_ptr<DBClientCursor> childCursor;
        MongoBase::MongoBase* base;

        void readFromMongoDB(const string&, const string&, const string&);
        void readfromDB();
        void getFileFromGrid(const GridFile &, const string &, const string &, const string &);
        void setModelOrViewName(const string&, const BSONObj&);
        void readFile(const string&, const string&, const string&, const OID&);
        void run();
};
}//: namespace ModelReader
}//: namespace Processors

REGISTER_COMPONENT("ModelReader", Processors::ModelReader::ModelReader)

#endif /* ModelReader_H__ */
