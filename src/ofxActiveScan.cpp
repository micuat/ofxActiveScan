#include "ofxActiveScan.h"

namespace ofxActiveScan {

int Encode::init(Options op) {
	CEncode encode(op);
	width = op.projector_width;
	height = op.projector_height;
	
	size = encode.GetNumImages();
	for (int i = 0; i < size; i++) {
		Map2f field;
		encode.GetImage(i, field);
		
		patterns.push_back(toOf(field));
	}
	return 0;
}

int Encode::getSize() {
	return size;
}

int Encode::getWidth() {
	return width;
}

int Encode::getHeight() {
	return height;
}

ofImage& Encode::getPatternAt(int i) {
	return patterns[i];
}

int Decode::init(Options op, string path) {
	CDecode decode(op);
	
	ofDirectory dir(ofToDataPath(path, true));
	dir.listDir();
	
	std::vector<std::string> files;
	for(int i = 0; i < dir.numFiles(); i++) {
		files.push_back(dir.getPath(i));
	}
	
	ofLogNotice() << "Decode::init() start decoding";
	decode.Decode(files);
	
	mapH = decode.GetMap(0);
	mapV = decode.GetMap(1);
	mapMask = decode.GetMask();
	mapReliable = decode.GetReliable();
}

Map2f& Decode::getMapHorizontal() {
	return mapH;
}

Map2f& Decode::getMapVertical() {
	return mapV;
}

Map2f& Decode::getMask() {
	return mapMask;
}

Map2f& Decode::getReliable() {
	return mapReliable;
}

};
