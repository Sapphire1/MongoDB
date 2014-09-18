
#ifndef  MONGODBWRITER_H__
#define  MONGODBWRITER_H__
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


namespace Processors {
namespace MongoDBWriter {

using namespace cv;
using namespace mongo;
using namespace std;

class MongoDBWriter: public Base::Component
{
public:
        /*!
         * Constructor.
         */
	MongoDBWriter(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~MongoDBWriter();

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
        Base::EventHandler <MongoDBWriter> h_write2DB;

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
        std::vector<std::string> fileExtensions;

        DBClientConnection c;
  	    vector<string>  docViewsNames;
  	    vector<string>  docModelsNames;

        string dbCollectionPath;

        void run();
        vector<string> getAllFiles(const string& pattern);
        vector<string> getAllFolders(const string& pattern);
        vector<OID>  getChildOIDS(BSONObj &obj);
        auto_ptr<DBClientCursor>  findDocumentInCollection(string nodeName, string type, string name);
        void initObject();
        void writeNode2MongoDB(const string &source, const string &destination, const string &option, string );
        void insert2MongoDB(const string &destination,  const string&,  const string& );
        auto_ptr<DBClientCursor> findModelDocumentInCollection(string nodeName);
        auto_ptr<DBClientCursor> findViewDocumentInCollection(string nodeName);
        void write2DB();
        void insertToModelOrView(const string &,const string &);
        void initView(string &);
        void initModel(string &);

};
}//: namespace MongoDBWriter
}//: namespace Processors

REGISTER_COMPONENT("MongoDBWriter", Processors::MongoDBWriter::MongoDBWriter)

#endif /* MONGODBWRITER_H__ */
