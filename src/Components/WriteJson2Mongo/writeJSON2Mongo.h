
#ifndef WRITEJSON2MONGO_H_
#define WRITEJSON2MONGO_H_


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdlib>
#include <iostream>
#include "mongo/client/dbclient.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

namespace Processors {
namespace MongoDB {

using namespace cv;

class Json2MongoWriter_Processor: public Base::Component
{
public:
        /*!
         * Constructor.
         */
	Json2MongoWriter_Processor(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~Json2MongoWriter_Processor();

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
        Base::EventHandler <Json2MongoWriter_Processor> h_onNewImage;

        /// Input data stream
        Base::DataStreamIn <cv::Mat> in_img;

        /// Output data stream - processed image
        Base::DataStreamOut <Mat> out_img;

private:
        /// Type of the performed thresholding operation.
   //     Base::Property<int, ThresholdTranslator> m_type;

    //    Base::Property<double> m_thresh;
   //     Base::Property<double> m_maxval;
        Base::Property<std::string> mongoDBHost;

        void run();
};
}//: namespace MongoDB
}//: namespace Processors

REGISTER_COMPONENT("WriteJSON2Mongo", Processors::MongoDB::Json2MongoWriter_Processor)

#endif /* WRITEJSON2MONGO_H_ */
