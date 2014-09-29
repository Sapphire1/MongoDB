
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

#include "Logger.hpp"
#include "mongo/client/dbclient.h"
#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include <dirent.h>
#include "MongoBase.hpp"

namespace Processors {
namespace ViewWriter {

using namespace cv;
using namespace mongo;
using namespace std;


class ViewWriter: public Base::Component
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
        Base::EventHandler <ViewWriter> h_write2DB;

        /// Input data stream
        Base::DataStreamIn <cv::Mat> in_img;

private:
        Base::Property<string> mongoDBHost;
        Base::Property<string> objectName;
        Base::Property<string> description;
        Base::Property<string> collectionName;
        Base::Property<string> viewNameProp;
        Base::Property<string> sceneNamesProp;
        Base::Property<string> extension;
        Base::Property<string> fileName;
        Base::Property<string> remoteFileName;
        std::vector<std::string> splitedSceneNames;
        Base::Property<string> nodeTypeProp;
        DBClientConnection c;
  	    vector<string>  docViewsNames;
  	    vector<string>  docModelsNames;
        string dbCollectionPath;
        MongoBase::MongoBase* base;

        void run();
        void initObject();
        void writeNode2MongoDB(const string &destination, const string &option, string );
        void insert2MongoDB(const string &destination,  const string&,  const string& );
        void write2DB();
        void insertToModelOrView(const string &,const string &);
        void initView(const string &, bool);
        void initModel(const string &, bool);
        void setModelOrViewName(const string&, const BSONObj&);
        void setMime(const string&, string&);
        void insertFileToGrid(BSONArrayBuilder&, OID&);
        void addToObject(const Base::Property<string> & nodeTypeProp, const string &);
        void addScenes(BSONObj&);
        void createModelOrView(const std::vector<string>::iterator, const string&, BSONArrayBuilder&);
};
}//: namespace ViewWriter
}//: namespace Processors

REGISTER_COMPONENT("ViewWriter", Processors::ViewWriter::ViewWriter)

#endif /* ViewWriter_H__ */
