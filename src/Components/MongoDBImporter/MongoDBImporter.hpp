
#ifndef  MONGODBImporter_H__
#define  MONGODBImporter_H__

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
#include <Types/MongoBase.hpp>

namespace Processors {
namespace MongoDBImporter {

using namespace cv;
using namespace mongo;


class MongoDBImporter: public Base::Component, MongoBase::MongoBase
{
public:
        /*!
         * Constructor.
         */
		MongoDBImporter(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~MongoDBImporter();

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
        Base::EventHandler <MongoDBImporter> h_readfromDB;

private:
        Base::Property<string> mongoDBHost;
        Base::Property<string> objectName;
        Base::Property<string> collectionName;
        Base::Property<string> nodeTypeProp;
        Base::Property<string> folderName;
        Base::Property<string> viewOrModelName;
        Base::Property<string> type;
        string dbCollectionPath;
        auto_ptr<DBClientCursor> cursorCollection;
        auto_ptr<DBClientCursor> childCursor;

        void readFromMongoDB(const string&, const string&, const string&);
        void readfromDB();
        void readFile(const string&, const string&, const string&, const OID&);
        void getFileFromGrid(const GridFile& file, const string& modelOrViewName, const string& nodeType, const string& type);
        void run();
};
}//: namespace MongoDBImporter
}//: namespace Processors

REGISTER_COMPONENT("MongoDBImporter", Processors::MongoDBImporter::MongoDBImporter)

#endif /* MONGODBImporter_H__ */
