#include "ofxActiveScanUtils.h"

namespace ofxActiveScan {

ofFloatImage toOfF(Map2f field) {
	ofFloatImage img;
	int w = field.size(0);
	int h = field.size(1);
	
	img.setFromPixels(field.ptr(), w, h, OF_IMAGE_GRAYSCALE);
	return img;
}

ofImage toOf(Map2f field) {
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

Map2f toAs(ofImage img) {
	int w = img.getWidth();
	int h = img.getHeight();
	Map2f m(w, h);
	
	for( int y = 0 ; y < h ; y++ ) {
		for( int x = 0 ; x < w ; x++ ) {
			m.cell(x, y) = img.getColor(x, y).getBrightness() / 255.0;
		}
	}
	
	return m;
}

};
