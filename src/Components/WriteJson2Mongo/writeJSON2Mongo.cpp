/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

#include <cstdlib>
#include <iostream>
#include "mongo/client/dbclient.h"
#include "writeJSON2Mongo.h"
#include <memory>
#include <string>

#include "Logger.hpp"
namespace Processors {
namespace MongoDB  {
using namespace cv;

Json2MongoWriter_Processor::Json2MongoWriter_Processor(const std::string & name) : Base::Component(name),
                m_type("type", CV_THRESH_BINARY, "combo"),
                m_thresh("thresh", 128, "range"),
                m_maxval("maxval", 255, "range")
{
        LOG(LTRACE) << "Hello Json2MongoWriter_Processor\n";
        try
        {
        	mongo::DBClientConnection c;
        	c.connect("localhost");
        	std::cout << "connected ok" << std::endl;
         }
         catch( const mongo::DBException &e )
         {
             std::cout << "caught " << e.what() << std::endl;
         }
}

Json2MongoWriter_Processor::~Json2MongoWriter_Processor()
{
        LOG(LTRACE) << "Good bye Json2MongoWriter_Processor\n";
}


void Json2MongoWriter_Processor::prepareInterface() {
        CLOG(LTRACE) << "Json2MongoWriter_Processor::prepareInterface\n";

        h_onNewImage.setup(this, &Json2MongoWriter_Processor::onNewImage);
        registerHandler("onNewImage", &h_onNewImage);

        registerStream("in_img", &in_img);

        registerStream("out_img", &out_img);

        addDependency("onNewImage", &in_img);
}

bool Json2MongoWriter_Processor::onInit()
{
        LOG(LTRACE) << "Json2MongoWriter_Processor::initialize\n";

        return true;
}

bool Json2MongoWriter_Processor::onFinish()
{
        LOG(LTRACE) << "Json2MongoWriter_Processor::finish\n";

        return true;
}

bool Json2MongoWriter_Processor::onStep()
{
        LOG(LTRACE) << "Json2MongoWriter_Processor::step\n";
        return true;
}

bool Json2MongoWriter_Processor::onStop()
{
        return true;
}

bool Json2MongoWriter_Processor::onStart()
{
        return true;
}

void Json2MongoWriter_Processor::onNewImage()
{
        LOG(LNOTICE) << "Json2MongoWriter_Processor::onNewImage\n";
        try {
                cv::Mat img = in_img.read();
                cv::Mat out = img.clone();
                LOG(LTRACE) << "Threshold " << m_thresh;
                cv::threshold(img, out, m_thresh, m_maxval, m_type);
                out_img.write(out);
        } catch (...) {
                LOG(LERROR) << "Json2MongoWriter_Processor::onNewImage failed\n";
        }

}

} //: namespace MongoDB
} //: namespace Processors

 
