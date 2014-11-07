
#ifndef  SceneWriter_H__
#define  SceneWriter_H__
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
namespace SceneWriter {

using namespace cv;
using namespace mongo;
using namespace std;


class SceneWriter: public Base::Component, MongoBase::MongoBase
{
public:
        /*!
         * Constructor.
         */
	   SceneWriter(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~SceneWriter();

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
        Base::EventHandler <SceneWriter> h_write2DB;

private:
        Base::Property<string> mongoDBHost;
        Base::Property<string> objectName;
        Base::Property<string> description;
        Base::Property<string> collectionName;
        Base::Property<string> nodeNameProp;
        Base::Property<string> sceneNamesProp;
        //string sceneName;
        std::vector<std::string> splitedSceneNames;
        string dbCollectionPath;


        void run();
        void initObject();
        void writeNode2MongoDB(const string &source, const string &destination, const string &option, string );
        void insert2MongoDB(const string &destination,  const string&,  const string& );
        void write2DB();
        void insertToModelOrView(const string &,const string &);
        void initView(const string &, bool);
        void initModel(const string &, bool);
        void insertFileToGrid(const std::vector<string>::iterator, const std::vector<string>::iterator, const string&, BSONArrayBuilder&);
        void addToObject(const Base::Property<string> & nodeNameProp, const string &);
        void addScenes(BSONObj&);
        void createModelOrView(const std::vector<string>::iterator, const string&, BSONArrayBuilder&);
};
}//: namespace SceneWriter
}//: namespace Processors

REGISTER_COMPONENT("SceneWriter", Processors::SceneWriter::SceneWriter)

#endif /* SceneWriter_H__ */
