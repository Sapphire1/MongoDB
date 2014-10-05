
#ifndef  MONGODBExporter_H__
#define  MONGODBExporter_H__
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/algorithm/string.hpp>

#include <cstdlib>
#include <iostream>
#include <glob.h>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "Logger.hpp"
#include "mongo/client/dbclient.h"
#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include <dirent.h>
#include <Types/MongoBase.hpp>

namespace Processors {
namespace MongoDBExporter {

using namespace cv;
using namespace mongo;
using namespace std;


class MongoDBExporter: public Base::Component, MongoBase::MongoBase
{
public:
        /*!
         * Constructor.
         */
	   MongoDBExporter(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~MongoDBExporter();

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
        Base::EventHandler <MongoDBExporter> h_write2DB;

        /// Input data stream
        Base::DataStreamIn <cv::Mat> in_img;

        /// Output data stream - processed image
        Base::DataStreamOut <Mat> out_img;

private:
        Base::Property<string> mongoDBHost;
        Base::Property<string> objectName;
        Base::Property<string> description;
        Base::Property<string> collectionName;
        Base::Property<string> extensions;
        Base::Property<string> nodeTypeProp;
        Base::Property<string> folderName;
        Base::Property<string> viewNameProp;
        Base::Property<string> modelNameProp;
        Base::Property<string> sceneNamesProp;
        std::vector<std::string> fileExtensions;
        //string sceneName;
        std::vector<std::string> splitedSceneNames;



        string dbCollectionPath;
        //MongoBase::MongoBase* base;

        void run();
        void initObject();
        void writeNode2MongoDB(const string &source, const string &destination, const string &option, string );
        void insert2MongoDB(const string &destination,  const string&,  const string& );
        void write2DB();
        void insertToModelOrView(const string &,const string &);
        void initView(const string &, bool);
        void initModel(const string &, bool);
        void setModelOrViewName(const string&, const BSONObj&);
        void setMime(const std::vector<string>::iterator, string&);
        void insertFileToGrid(const std::vector<string>::iterator, const std::vector<string>::iterator, const string&, BSONArrayBuilder&);
        void addToObject(const Base::Property<string> & nodeTypeProp, const string &);
        void addScenes(BSONObj&);
        void createModelOrView(const std::vector<string>::iterator, const string&, BSONArrayBuilder&);
};
}//: namespace MongoDBExporter
}//: namespace Processors

REGISTER_COMPONENT("MongoDBExporter", Processors::MongoDBExporter::MongoDBExporter)

#endif /* MONGODBExporter_H__ */
