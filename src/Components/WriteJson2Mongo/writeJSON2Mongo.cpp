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
using namespace mongo;
using namespace boost::property_tree;

Json2MongoWriter_Processor::Json2MongoWriter_Processor(const std::string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", std::string("localhost"))
{
        registerProperty(mongoDBHost);
        LOG(LTRACE) << "Hello Json2MongoWriter_Processor\n";
        try
        	 {
        	     	mongo::DBClientConnection db;

        	       	db.connect(mongoDBHost);
        	       	std::cout << "Connected to MongoDB\n";
        	       	// create ptree object
        	        std::stringstream ss, aa;
        	        // to jest jeden przykladowy obiekt przechowywany jako ptree
                    ss << "{ \"root\": { \"values\": [1, 2, 3, 4, 5 ] } }";

                   // ss << "{ \"root\":  [{ \"name\": Channel, \"default\": 1 }, {\"name\": Size, \"default\": 457 } ] }";


        	        boost::property_tree::ptree pt;
        	        boost::property_tree::read_json(ss, pt);
        	        pt.put("parent", "other4");
        	        pt.put("name", "kubek3");

        	        BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pt.get_child("root.values"))
        	        {
        	        	std::cout<"Read ptree\n";
        	        	assert(v.first.empty()); // array elements have no names
        	        	std::cout<< v.second.data() << std::endl;
        	        }
        	        // we have ptree object now w have to change to json object
        	        write_json(aa, pt);
        	        // extend lifetime to the lifetime of the reference
        	        const std::string& resultstr = aa.str();
        	        int sizeStr = resultstr.length();
        	        const char* cstr2 = resultstr.c_str();

        	        std::cout<<cstr2<<"\n";
        	        BSONObj bsonObj, bsonObj2;
        	        bsonObj = fromjson(cstr2,&sizeStr);
        	       /* std::string collectionName = "products.kubki";
        	        db.createCollection(collectionName);
        	        std::string collectionName2 = "products.talerze";
        	        db.createCollection(collectionName2);
        	        */
        	        db.insert("obiekty.dopicia.kubki", bsonObj);
        	        db.insert("obiekty.talerze", bsonObj);

        	        cout << "count:" << db.count("obiekty.dopicia.kubki") << endl;
        	        std:string str1 = "other";
        	        auto_ptr<DBClientCursor> cursor =
        	       // db.query("obiekty.kubki", QUERY("parent" << str1));
        	        db.query("obiekty.dopicia.kubki", BSONObj());
        	        //drukuje wszystko co jest w jednej kolekcji
        	        while (cursor->more())
        	           cout << cursor->next().toString() << endl;


        	        list<string> collNamespaces =
        	                db.getCollectionNames("obiekty");
        	        list<string>::iterator iter2 = collNamespaces.begin();
        	             while( iter2 != collNamespaces.end() )
        	             {
        	               // EACH ENTRY HAS THE FULL NAMESPACE ("database:collection").
        	               // Use this method to strip off the database name
        	               string collectionName = mongo::nsGetCollection(*iter2);
        	               std::cout << collectionName<<"\n";

        	               auto_ptr<DBClientCursor> cursor =
        	               db.query("obiekty."+collectionName, BSONObj());
        	               while (cursor->more())
        	            	   cout << cursor->next().toString() << endl;
        	               ++iter2;
        	             } // END WHILE iterate through collections

        	        /*
        	        db.insert("mydb.users",
        	          BSON(GENOID<<"a"<<1<<"b"<<1));
        	        // then:
        	        string err = db.getLastError();
        	        std::vector<mongo::BSONObj> BsonVec;
        	        */
        	   //     void findN(BsonVec, const string&ns, Query query, int 5)

        	 }

        	 catch( const mongo::DBException &e )
        	 {
        		 	LOG(LTRACE) << "Cannot connect!" << e.what() << "\n";
        	 }
       // connect2MongoDB();

}
void Json2MongoWriter_Processor::connect2MongoDB()
{
	 try
	 {
	     	mongo::DBClientConnection c;
	       	c.connect("localhost");
	       	CLOG(LTRACE) << "Connected with MongoDB\n";
	       	// create ptree object
	        std::stringstream ss;
	        ss << "{ \"root\": { \"values\": [1, 2, 3, 4, 5 ] } }";

	        boost::property_tree::ptree pt;
	        boost::property_tree::read_json(ss, pt);

	        BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pt.get_child("root.values"))
	        {
	        	std::cout<"Read ptree\n";
	        	assert(v.first.empty()); // array elements have no names
	        	CLOG(LTRACE) << v.second.data() << std::endl;
	        }
	 }
	 catch( const mongo::DBException &e )
	 {
		 	CLOG(LTRACE) << "Cannot connect!" << e.what() << "\n";
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
        /*
        try {
                cv::Mat img = in_img.read();
                cv::Mat out = img.clone();
                LOG(LTRACE) << "Threshold " << m_thresh;
                cv::threshold(img, out, m_thresh, m_maxval, m_type);
                out_img.write(out);
        } catch (...) {
                LOG(LERROR) << "Json2MongoWriter_Processor::onNewImage failed\n";
        }
        */

}

} //: namespace MongoDB
} //: namespace Processors

 
