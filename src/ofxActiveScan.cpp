#include "ofxActiveScan.h"

namespace ofxActiveScan {

ofImage toOf(slib::Field<2,float> field) {
	ofImage img;
	int w = field.size(0);
	int h = field.size(1);
	img.allocate(w, h, OF_IMAGE_COLOR);
	
	for( int y = 0 ; y < h ; y++ ) {
		for( int x = 0 ; x < w ; x++ ) {
			img.setColor(x, y, field.cell(x, y) * 255);
		}
	}
	
	return img;
}

int Encode::init(char* fileName) {
	CEncode encode(fileName);
	
	size = encode.GetNumImages();
	for (int i = 0; i < size; i++) {
		slib::Field<2,float> field;
		encode.GetImage(i, field);
		
		width = field.size(0);
		height = field.size(1);
		
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
