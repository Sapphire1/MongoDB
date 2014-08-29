/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */


#include "MongoDBWriter.h"

namespace Processors {
namespace MongoDBWriter  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;

MongoDBWriter::MongoDBWriter(const string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", string("localhost")),
		objectName("objectName", string("GreenCup")),
		description("description", string("My green coffe cup")),
		collectionName("collectionName", string("containers")),
		extensions("extensions", string("*.png,*.jpg,*.txt")),
		nodeType("nodeType", string("Object")),
		folderName("folderName", string("/home/lzmuda/mongo_driver_tutorial/"))
		//folderName("folderName", string("./"))
{
        registerProperty(mongoDBHost);
        registerProperty(objectName);
        registerProperty(description);
        registerProperty(collectionName);
        registerProperty(extensions);
        registerProperty(nodeType);
        registerProperty(folderName);
        CLOG(LTRACE) << "Hello MongoDBWriter";
}
void MongoDBWriter::connect2MongoDB()
{
}

MongoDBWriter::~MongoDBWriter()
{
        CLOG(LTRACE) << "Good bye MongoDBWriter";
}
void MongoDBWriter::write2DB()
{
        CLOG(LNOTICE) << "MongoDBReader::write2DB";
        insert2MongoDB(nodeType);
}

void MongoDBWriter::prepareInterface() {
        CLOG(LTRACE) << "MongoDBWriter::prepareInterface";
        h_write2DB.setup(this, &MongoDBWriter::write2DB);
        registerHandler("write2DB", &h_write2DB);
	
       	//insert2MongoDB(c, fs); 
//      registerStream("in_img", &in_img);
//      registerStream("out_img", &out_img);

//	addDependency("insert2MongoDB", &in_img);
}

bool MongoDBWriter::onInit()
{
        CLOG(LTRACE) << "MongoDBWriter::initialize";
        try
        {
      string ext =extensions;
	  boost::split(fileExtensions, ext, is_any_of(","));
	  c.connect(mongoDBHost);
	  if(collectionName=="containers")
	    dbCollectionPath="images.containers";
	  else if(collectionName=="food")
	    dbCollectionPath="images.food";
	  else if(collectionName=="dish")
	    dbCollectionPath="images.dish";
	  else if(collectionName=="other")
	    dbCollectionPath="images.other";
         }
         catch(DBException &e)
         {
        	 CLOG(LERROR) <<"Something goes wrong... :>";
        	 CLOG(LERROR) <<c.getLastError();
         }

        return true;

}

bool MongoDBWriter::onFinish()
{
        CLOG(LTRACE) << "MongoDBWriter::finish";

        return true;
}

bool MongoDBWriter::onStep()
{
        CLOG(LTRACE) << "MongoDBWriter::step";
        return true;
}

bool MongoDBWriter::onStop()
{
        return true;
}

bool MongoDBWriter::onStart()
{
        return true;
}

void MongoDBWriter::onNewImage()
{
        CLOG(LNOTICE) << "MongoDBWriter::onNewImage";
}

vector<string> MongoDBWriter::getAllFiles(const string& pattern)
{
	glob_t glob_result;
	glob(pattern.c_str(),GLOB_TILDE,NULL,&glob_result);
	vector<string> files;
	for(unsigned int i=0;i<glob_result.gl_pathc;++i){
	    files.push_back(string(glob_result.gl_pathv[i]));
	}
	globfree(&glob_result);
	return files;

}

vector<OID>  MongoDBWriter::getChildOIDS(BSONObj &obj)
{
	  CLOG(LTRACE) << "Processing JSON document: " << obj.toString() << std::endl;
	  vector<BSONElement> v = obj.getField("childOIDs").Array();
	  vector<OID> oidsVector;

	  for (unsigned int i = 0; i<v.size(); i++)
	  {
		string readedOid =v[i]["childOID"].str();
		OID o = OID(readedOid);
		CLOG(LTRACE) <<"OID: "<< o.str() << endl;
		oidsVector.push_back(o);
	  }
	  return oidsVector;
}

auto_ptr<DBClientCursor>  MongoDBWriter::findDocumentInCollection(string nodeName)
{
      auto_ptr<DBClientCursor> cursorCollection;
      try{
	cursorCollection =c.query(dbCollectionPath, (QUERY("Type"<<nodeName<<"Name"<<objectName)));
      }
      catch(DBException &e)
      {
    	  CLOG(LERROR) <<"Something goes wrong... :>";
    	  CLOG(LERROR) <<c.getLastError();
      }
    return cursorCollection;

}

void MongoDBWriter::initObject()
	{
		  CLOG(LTRACE) <<"Create template of object";
	      try{
		      BSONObj object = BSONObjBuilder().genOID().append("Type", "Object").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, object);
		      BSONObj view = BSONObjBuilder().genOID().append("Type", "View").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, view);
		      BSONObj model = BSONObjBuilder().genOID().append("Type", "Model").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, model);

		      BSONObj stereo = BSONObjBuilder().genOID().append("Type", "Stereo").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, stereo);
		      BSONObj kinect = BSONObjBuilder().genOID().append("Type","Kinect").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, kinect);
		      BSONObj tof = BSONObjBuilder().genOID().append("Type", "ToF").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, tof);
		      BSONObj stereosilrx = BSONObjBuilder().genOID().append("Type", "StereoSiLR").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, stereosilrx);
		      BSONObj stereosirx = BSONObjBuilder().genOID().append("Type", "StereoSiRX").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, stereosirx);
		      BSONObj stereosirxm = BSONObjBuilder().genOID().append("Type", "StereoSiRXM").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, stereosirxm);

		      BSONObj kinectsilrx = BSONObjBuilder().genOID().append("Type", "KinectSiLR").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, kinectsilrx);
		      BSONObj kinectsirx = BSONObjBuilder().genOID().append("Type", "KinectSiRX").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, kinectsirx);
		      BSONObj kinectsirxm = BSONObjBuilder().genOID().append("Type", "KinectSiRXM").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, kinectsirxm);

		       BSONObj tofsilrx = BSONObjBuilder().genOID().append("Type", "ToFSiLR").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, tofsilrx);
		      BSONObj tofsirx = BSONObjBuilder().genOID().append("Type", "ToFSiRX").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, tofsirx);
		      BSONObj tofsirxm = BSONObjBuilder().genOID().append("Type", "ToFSiRXM").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, tofsirxm);

		      BSONObj som = BSONObjBuilder().genOID().append("Type", "Som").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, som);
		      BSONObj ssom = BSONObjBuilder().genOID().append("Type", "Ssom").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, ssom);
		      BSONObj somRGB = BSONObjBuilder().genOID().append("Type", "SomRgb").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, somRGB);
		      BSONObj somSIFT = BSONObjBuilder().genOID().append("Type", "SomSift").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, somSIFT);
		      BSONObj ssomRGB = BSONObjBuilder().genOID().append("Type", "SsomRgb").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, ssomRGB);
		      BSONObj ssomSIFT = BSONObjBuilder().genOID().append("Type", "SsomSift").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, ssomSIFT);
		      BSONObj ssomSHOT = BSONObjBuilder().genOID().append("Type", "SsomShot").append("Name", objectName).append("description", description).obj();
		      c.insert(dbCollectionPath, ssomSHOT);

		      // get object id for any bsonObj and update

		      BSONElement oi,oi2,oi3;
		      BSONArrayBuilder b,b1,b2,b3,b4,b5,b6,b7;
		      OID o, o2, o3;


		      model.getObjectID(oi);
		      view.getObjectID(oi2);
		      o=oi.__oid();
		      o2=oi2.__oid();

		      b.append(BSONObjBuilder().append("childOID", o.str()).obj());
		      b.append(BSONObjBuilder().append("childOID", o2.str()).obj());
		      BSONArray objectArr = b.arr();

		      stereo.getObjectID(oi);
		      kinect.getObjectID(oi2);
		      tof.getObjectID(oi3);
		      o=oi.__oid();
		      o2=oi2.__oid();
		      o3=oi3.__oid();

		      b1.append(BSONObjBuilder().append("childOID", o.str()).obj());
		      b1.append(BSONObjBuilder().append("childOID", o2.str()).obj());
		      b1.append(BSONObjBuilder().append("childOID", o3.str()).obj());
		      BSONArray viewArr = b1.arr();

		      stereosilrx.getObjectID(oi);
		      stereosirx.getObjectID(oi2);
		      stereosirxm.getObjectID(oi3);
		      o=oi.__oid();
		      o2=oi2.__oid();
		      o3=oi3.__oid();

		      b2.append(BSONObjBuilder().append("childOID", o.str()).obj());
		      b2.append(BSONObjBuilder().append("childOID", o2.str()).obj());
		      b2.append(BSONObjBuilder().append("childOID", o3.str()).obj());
		      BSONArray stereoArr = b2.arr();

		      kinectsilrx.getObjectID(oi);
		      kinectsirx.getObjectID(oi2);
		      kinectsirxm.getObjectID(oi3);
		      o=oi.__oid();
		      o2=oi2.__oid();
		      o3=oi3.__oid();

		      b6.append(BSONObjBuilder().append("childOID", o.str()).obj());
		      b6.append(BSONObjBuilder().append("childOID", o2.str()).obj());
		      b6.append(BSONObjBuilder().append("childOID", o3.str()).obj());
		      BSONArray kinectArr = b6.arr();

		      tofsilrx.getObjectID(oi);
		      tofsirx.getObjectID(oi2);
		      tofsirxm.getObjectID(oi3);
		      o=oi.__oid();
		      o2=oi2.__oid();
		      o3=oi3.__oid();

		      b7.append(BSONObjBuilder().append("childOID", o.str()).obj());
		      b7.append(BSONObjBuilder().append("childOID", o2.str()).obj());
		      b7.append(BSONObjBuilder().append("childOID", o3.str()).obj());
		      BSONArray tofArr = b7.arr();


		      som.getObjectID(oi);
		      ssom.getObjectID(oi2);
		      o=oi.__oid();
		      o2=oi2.__oid();

		      b3.append(BSONObjBuilder().append("childOID", o.str()).obj());
		      b3.append(BSONObjBuilder().append("childOID", o2.str()).obj());
		      BSONArray modelArr = b3.arr();


		      ssomRGB.getObjectID(oi);
		      ssomSIFT.getObjectID(oi2);
		      ssomSHOT.getObjectID(oi3);
		      o=oi.__oid();
		      o2=oi2.__oid();
		      o3=oi3.__oid();

		      b4.append(BSONObjBuilder().append("childOID", o.str()).obj());
		      b4.append(BSONObjBuilder().append("childOID", o2.str()).obj());
		      b4.append(BSONObjBuilder().append("childOID", o3.str()).obj());
		      BSONArray ssomArr = b4.arr();


		      somRGB.getObjectID(oi);
		      ssomSIFT.getObjectID(oi2);
		      o=oi.__oid();
		      o2=oi2.__oid();

		      b5.append(BSONObjBuilder().append("childOID", o.str()).obj());
		      b5.append(BSONObjBuilder().append("childOID", o2.str()).obj());
		      BSONArray somArr = b5.arr();

		      //update - add childs
		      c.update(dbCollectionPath, QUERY("Type"<<"ToF"<<"Name"<<objectName), BSON("$set"<<BSON("childOIDs"<<tofArr)), false, true);
		      c.update(dbCollectionPath, QUERY("Type"<<"Kinect"<<"Name"<<objectName), BSON("$set"<<BSON("childOIDs"<<kinectArr)), false, true);
		      c.update(dbCollectionPath, QUERY("Type"<<"Stereo"<<"Name"<<objectName), BSON("$set"<<BSON("childOIDs"<<stereoArr)), false, true);
		      c.update(dbCollectionPath, QUERY("Type"<<"Model"<<"Name"<<objectName), BSON("$set"<<BSON("childOIDs"<<modelArr)), false, true);
		      c.update(dbCollectionPath, QUERY("Type"<<"View"<<"Name"<<objectName), BSON("$set"<<BSON("childOIDs"<<viewArr)), false, true);
		      c.update(dbCollectionPath, QUERY("Type"<<"Object"<<"Name"<<objectName), BSON("$set"<<BSON("childOIDs"<<objectArr)), false, true);
		      c.update(dbCollectionPath, QUERY("Type"<<"Ssom"<<"Name"<<objectName), BSON("$set"<<BSON("childOIDs"<<ssomArr)), false, true);
		      c.update(dbCollectionPath, QUERY("Type"<<"Som"<<"Name"<<objectName), BSON("$set"<<BSON("childOIDs"<<somArr)), false, true);
	      }
	      catch(DBException &e)
	      {
	  		CLOG(LERROR) <<"Something goes wrong... :>";
	    	CLOG(LERROR) <<c.getLastError();
	      }
	}

void MongoDBWriter::writeNode2MongoDB(const string &source, const string &destination)
{
	BSONObj o;
	BSONElement bsonElement;
	BSONArrayBuilder bsonBuilder;
	OID oid;
	string mime;
	CLOG(LTRACE) <<"Source: " << source << " destination: "<< destination<<" dbCollectionPath: "<<dbCollectionPath;
    try{
    	for(std::vector<string>::iterator itExtension = fileExtensions.begin(); itExtension != fileExtensions.end(); ++itExtension) {
    		CLOG(LTRACE) <<"source+*itExtension "<<source+*itExtension;
			vector<string> files = getAllFiles(source+*itExtension);
			for(std::vector<string>::iterator it = files.begin(); it != files.end(); ++it) {
				CLOG(LTRACE) <<"fileName"<<*it;
				string fileName = *it;
				string newFileName;
				const size_t last_slash_idx = fileName.find_last_of("/");
				if (std::string::npos != last_slash_idx)
				{
					newFileName = fileName.erase(0, last_slash_idx + 1);
				}
				if (*itExtension=="*.png")
					mime="image/png";
				else if(*itExtension=="*.jpg")
					mime= "image/jpeg";
				else if(*itExtension=="*.txt" || *itExtension=="*.pcd")
					mime="text/plain";
				else
				{
					CLOG(LERROR) <<"I don't know such file extension! Please add extension to the code from http://www.sitepoint.com/web-foundations/mime-types-complete-list/";
					return;
				}
				GridFS fs(c, collectionName);
				o = fs.storeFile(*it, newFileName, mime);
				BSONObj b = BSONObjBuilder().appendElements(o).append("Name", objectName).obj();
				c.insert(dbCollectionPath, b);
				b.getObjectID(bsonElement);
				oid=bsonElement.__oid();
				bsonBuilder.append(BSONObjBuilder().append("childOID", oid.str()).obj());
			}
    	}
		BSONArray destArr = bsonBuilder.arr();
		c.update(dbCollectionPath, QUERY("Type"<<destination<<"Name"<<objectName), BSON("$set"<<BSON("childOIDs"<<destArr)), false, true);
		CLOG(LTRACE) <<"Files saved successfully";
    }
	catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :>";
		CLOG(LERROR) <<c.getLastError();
	}
}

void MongoDBWriter::insert2MongoDB(const string &destination)
{
	try{
		unsigned long long nr = c.count(dbCollectionPath, QUERY("Name"<<objectName<<"Type"<<"Object"));
		if(nr==0)
		{
			CLOG(LTRACE) <<"Object does not exists in "<< dbCollectionPath;
			initObject();
		}
		else
		{
			if(destination=="Object")
			{
				CLOG(LERROR) <<"Object "<<objectName<<" exist in "<< dbCollectionPath;
				return;
			}
		}
		if(destination=="StereoSiLR" || destination=="KinectSiLR" || destination=="ToFSiLR" || destination=="StereoSiRX" || destination=="KinectSiRX" ||  destination=="ToFSiRX" || destination=="StereoSiRXM" || destination=="KinectSiRXM" || destination=="ToFSiRXM" ||  destination=="SomRgb" || destination=="SomSift" ||  destination=="SsomRgb" || destination=="SsomSift" || destination=="SsomShot")
		{
			string source = (string)folderName+destination+"/";
			writeNode2MongoDB(source, destination);
		}
		else
		{
			auto_ptr<DBClientCursor> cursorCollection =findDocumentInCollection(destination);
			while (cursorCollection->more())
			{
				BSONObj obj = cursorCollection->next();
				vector<OID> childsVector =  getChildOIDS(obj);
				for (unsigned int i = 0; i<childsVector.size(); i++)
				{
					auto_ptr<DBClientCursor> childCursor =c.query(dbCollectionPath, (QUERY("_id"<<childsVector[i])));
					if( childCursor->more())
					{
						BSONObj childObj = childCursor->next();
						string childNodeName = childObj.getField("Type").str();
						// if node consists child read it
						if(childNodeName!="EOO")
							insert2MongoDB(childNodeName);
					}
				}
			}//while
		}//else
    }//try
	catch(DBException &e)
	{
		CLOG(LERROR) <<"Something goes wrong... :>";
		CLOG(LERROR) <<c.getLastError();
	}
}

} //: namespace MongoDBWriter
} //: namespace Processors

 
