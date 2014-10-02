
#ifndef  SceneReader_H__
#define  SceneReader_H__

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
namespace SceneReader {

using namespace cv;
using namespace mongo;


class SceneReader: public Base::Component
{
public:
        /*!
         * Constructor.
         */
		SceneReader(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~SceneReader();

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
        Base::EventHandler <SceneReader> h_readfromDB;

        /// Input data stream
        Base::DataStreamIn <cv::Mat> in_img;

        /// Output data stream - processed image
        Base::DataStreamOut <Mat> out_img;

private:
        Base::Property<string> mongoDBHost;
        Base::Property<string> sceneName;
        Base::Property<string> objectName;
        Base::Property<string> collectionName;
        Base::Property<bool> getObjectsActive;
        Base::Property<bool> getScenesActive;
        Base::Property<bool> getObjectFromSceneActive;

        DBClientConnection c;
		string dbCollectionPath;
		auto_ptr<DBClientCursor> cursorCollection;
		auto_ptr<DBClientCursor> childCursor;
		MongoBase::MongoBase* base;

        void readfromDB();
        void getObjects();
        void getScenes();
        void getObjectFromScene();

};
}//: namespace SceneReader
}//: namespace Processors

REGISTER_COMPONENT("SceneReader", Processors::SceneReader::SceneReader)

#endif /* SceneReader_H__ */
