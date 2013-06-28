#include "ofxActiveScan.h"

namespace ofxActiveScan {

int Encode::init(Options op) {
	CEncode encode(op);
	width = op.projector_width;
	height = op.projector_height;
	
	size = encode.GetNumImages();
	for (int i = 0; i < size; i++) {
		slib::Field<2,float> field;
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

}
