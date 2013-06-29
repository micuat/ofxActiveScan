#include "testApp.h"

void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	rootDir = "../../../SharedData/";
	
	cv::FileStorage fs(ofToDataPath(rootDir + "/config.yml"), cv::FileStorage::READ);
	fs["proWidth"] >> options.projector_width;
	fs["proHeight"] >> options.projector_height;
	fs["camWidth"] >> cw;
	fs["camHeight"] >> ch;
	
	ofDirectory dir(ofToDataPath(rootDir + "img/", true));
	dir.listDir();
	
	std::vector<std::string> files;
	for(int i = 0; i < dir.numFiles(); i++) {
		files.push_back(dir.getPath(i));
	}
	
	CDecode decode(options);
	decode.Decode(files);
	decode.WriteMap(0,ofToDataPath(rootDir + "h.map", true));
	decode.WriteMap(1,ofToDataPath(rootDir + "v.map", true));
	decode.WriteMask(ofToDataPath(rootDir + "mask.bmp", true));
	decode.WriteReliable(ofToDataPath(rootDir + "reliable.bmp", true));	
}

void testApp::update() {
}

void testApp::draw() {
	ofBackground(0);
}

void testApp::keyPressed(int key) {
}
