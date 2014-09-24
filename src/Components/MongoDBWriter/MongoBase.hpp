/*
 * MongoBase.hpp
 *
 *  Created on: Sep 23, 2014
 *      Author: lzmuda
 */

#ifndef MONGOBASE_HPP_
#define MONGOBASE_HPP_

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


namespace MongoBase {
using namespace cv;
using namespace mongo;
using namespace std;

class MongoBase {
public:
	MongoBase();
	virtual ~MongoBase();
	vector<string> getAllFiles(const string& pattern);
	vector<string> getAllFolders(const string& pattern);
	vector<OID>  getChildOIDS(BSONObj &obj, const string&,  const string&);
	bool isModelLastLeaf(const string&);
	bool isViewLastLeaf(const string&);
	void findDocumentInCollection(DBClientConnection&, string&, Base::Property<string> &, const string &, auto_ptr<DBClientCursor> &, const string &, const string & , int&);

};

} /* namespace MongoBase */
#endif /* MONGOBASE_HPP_ */
