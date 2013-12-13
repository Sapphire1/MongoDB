#include <cstdlib>
#include <iostream>
#include "mongo/client/dbclient.h"
#include "writeJSON2Mongo.h"
#include <memory>
#include <string>

#include "Logger.hpp"
namespace Processors {
namespace TestDB {
using namespace cv;

TestDB_Processor::TestDB_Processor(const std::string & name) : Base::Component(name),
                m_type("type", CV_THRESH_BINARY, "combo"),
                m_thresh("thresh", 128, "range"),
                m_maxval("maxval", 255, "range")
{
        LOG(LTRACE) << "Hello BarCode_Processor\n";

        m_type.setToolTip("Thresholding type");
        m_type.addConstraint("BINARY");
        m_type.addConstraint("BINARY_INV");
        m_type.addConstraint("TRUNC");
        m_type.addConstraint("TOZERO");
        m_type.addConstraint("TOZERO_INV");

        m_thresh.setToolTip("Threshold level");
        m_thresh.addConstraint("0");
        m_thresh.addConstraint("255");

        m_maxval.setToolTip("Maximum value to use with BINARY and BINARY_INV thresholding types");
        m_maxval.addConstraint("0");
        m_maxval.addConstraint("255");

        // Register properties.
        registerProperty(m_type);
        registerProperty(m_thresh);
        registerProperty(m_maxval);
        try {
         mongo::DBClientConnection c;
                	c.connect("localhost");
                   std::cout << "connected ok" << std::endl;
                 } catch( const mongo::DBException &e ) {
                   std::cout << "caught " << e.what() << std::endl;
                 }
}

TestDB_Processor::~TestDB_Processor()
{
        LOG(LTRACE) << "Good bye BarCode_Processor\n";
}


void TestDB_Processor::prepareInterface() {
        CLOG(LTRACE) << "BarCode_Processor::prepareInterface\n";

        h_onNewImage.setup(this, &TestDB_Processor::onNewImage);
        registerHandler("onNewImage", &h_onNewImage);

        registerStream("in_img", &in_img);

        registerStream("out_img", &out_img);

        addDependency("onNewImage", &in_img);
}

bool TestDB_Processor::onInit()
{
        LOG(LTRACE) << "BarCode_Processor::initialize\n";

        return true;
}

bool TestDB_Processor::onFinish()
{
        LOG(LTRACE) << "TestDB_Processor_Processor::finish\n";

        return true;
}

bool TestDB_Processor::onStep()
{
        LOG(LTRACE) << "TestDB_Processor_Processor::step\n";
        return true;
}

bool TestDB_Processor::onStop()
{
        return true;
}

bool TestDB_Processor::onStart()
{
	std::cout<<"STRTTTTTTTTTTTTTTTTTTTTTT\n";
        return true;
}

void TestDB_Processor::onNewImage()
{
        LOG(LNOTICE) << "TestDB_Processor::onNewImage\n";
        try {
                cv::Mat img = in_img.read();
                cv::Mat out = img.clone();
                LOG(LTRACE) << "Threshold " << m_thresh;
                cv::threshold(img, out, m_thresh, m_maxval, m_type);
                out_img.write(out);
        } catch (...) {
                LOG(LERROR) << "TestDB_Processor::onNewImage failed\n";
        }

}

}//: namespace TestDB
} //: namespace Processors

 
