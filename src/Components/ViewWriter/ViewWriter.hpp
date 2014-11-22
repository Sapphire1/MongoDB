
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
#include <cstring>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include "Logger.hpp"
#include "mongo/client/dbclient.h"
#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include <dirent.h>
#include <Types/MongoBase.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZRGBSIFT.hpp>
#include <Types/PointXYZSHOT.hpp>
#include <Types/View.hpp>

namespace Processors {
namespace ViewWriter {

using namespace cv;
using namespace mongo;
using namespace std;
using namespace MongoBase;


class ViewWriter: public Base::Component, MongoBase::MongoBase
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

   	 /// Input camera_info
   	Base::DataStreamIn <cv::string> in_xml;

   	/// Input data stream -XYZ image
   	Base::DataStreamIn <cv::Mat> in_xyz;

   	/// Input RGB image
   	Base::DataStreamIn <cv::Mat> in_rgb;

  	/// Input density image
  	Base::DataStreamIn <cv::Mat> in_density;

  	/// Input intensity image
  	Base::DataStreamIn <cv::Mat> in_intensity;

   	/// Input mask image
   	Base::DataStreamIn <cv::Mat> in_mask;

  	/// Input density image
  	Base::DataStreamIn <cv::Mat> in_stereoL;

  	/// Input intensity image
  	Base::DataStreamIn <cv::Mat> in_stereoR;

  	/// Input density image
  	Base::DataStreamIn <cv::Mat> in_stereoLTextured;

  	/// Input intensity image
  	Base::DataStreamIn <cv::Mat> in_stereoRTextured;

  	//PCL - Point Clouds
   	/// Cloud containing points with Cartesian coordinates (XYZ).
   	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr > in_pc_xyz;

   	/// Cloud containing points with Cartesian coordinates and colour (XYZ + RGB).
   	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > in_pc_xyzrgb;

   	/// Cloud containing points with Cartesian coordinates and SIFT descriptor (XYZ + 128).
   	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr > in_pc_xyzsift;

   	/// Cloud containing points with Cartesian coordinates,  SIFT descriptor and color (XYZ + RGB + 128)
   	Base::DataStreamIn<pcl::PointCloud<PointXYZRGBSIFT>::Ptr > in_pc_xyzrgbsift;

   	/// Cloud containing points with Cartesian coordinates and SHOT descriptor  (XYZ + SHOT).
   	Base::DataStreamIn<pcl::PointCloud<PointXYZSHOT>::Ptr > in_pc_xyzshot;

   	/// Cloud containing points with Cartesian coordinates, colors and normals (XYZ + RGB + NORMAL).
   	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> in_pc_xyzrgbnormal;

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


private:
	Base::Property<string> mongoDBHost;
	Base::Property<string> objectName;
	Base::Property<string> description;
	Base::Property<string> collectionName;
	Base::Property<string> viewName;
	Base::Property<string> sceneNamesProp;
	Base::Property<string> fileName;
	Base::Property<int> mean_viewpoint_features_number;
	Base::Property<string> remoteFileName;
	std::vector<std::string> splitedSceneNames;
	Base::Property<string> nodeNameProp;
	Base::Property<bool> binary;
	Base::Property<bool> suffix;

	template <keyTypes key>
	void writeData();

	void Write_xyz();
	void Write_xyzrgb();
	void Write_xyzsift();

	void initObject();
	void writeNode2MongoDB(const string &destination, const string &option, string,  const string& fileType);
	void insert2MongoDB(const string &destination,  const string&,  const string&,  const string& fileType );
	void writeTXT2DB();
	void writeImage2DB();
	void writePCD2DB();
	void writeYAML2DB();
	float getFileSize(const string& fileType);
	void insertToModelOrView(const string &,const string &);
	void insertFileIntoGrid(OID&, const string&, int);
	void insertFileIntoCollection(OID& oid, const string& fileType, string& tempFileName, int);
	void createModelOrView(const std::vector<string>::iterator, const string&, BSONArrayBuilder&);
	void copyXYZSiftPointToFloatArray (const PointXYZSIFT &p, float * out) const;
	void copyXYZPointToFloatArray (const pcl::PointXYZ &p, float * out) const;
	void copyXYZRGBPointToFloatArray (const pcl::PointXYZRGB &p, float * out) const;
	char* serializeTable( int &length, 	std::stringstream& compressedData);
};
}//: namespace ViewWriter
}//: namespace Processors

REGISTER_COMPONENT("ViewWriter", Processors::ViewWriter::ViewWriter)

#endif /* ViewWriter_H__ */
